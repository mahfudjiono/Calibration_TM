import asyncio
import socket
import struct
import time
import techmanpy


class TMRobot:
    def __init__(self, ip, port_control=5890, port_position=5891, verbose=False):
        self.ip = ip
        self.port_control = port_control
        self.port_position = port_position
        self.verbose = verbose
        self._last_move_error = None
        self._last_move_ts = 0.0
        self._min_move_interval_sec = 0.8
        self._loop = None
        self._status_sock = None
        self._status_buffer = b''

        # cache last valid pose to survive temporary status dropouts
        self._last_valid_pose = None
        self._last_valid_pose_ts = 0.0
        self._max_pose_cache_age_sec = 1.0

    def _log(self, *args):
        if self.verbose:
            print(*args)

    def _parse_binary_coords(self, data_bytes):
        """
        Search the raw TM status stream for the latest coordinate field.
        """
        keys = [
            b'Coord_Base_Tool\x18\x00',
            b'Coord_Robot_Tool\x18\x00',
            b'Coord_Base_Flange\x18\x00',
            b'Coord_Robot_Flange\x18\x00',
        ]

        for key in keys:
            idx = data_bytes.rfind(key)
            if idx != -1:
                start = idx + len(key)
                end = start + 24
                if len(data_bytes) >= end:
                    try:
                        x, y, z, rx, ry, rz = struct.unpack('<6f', data_bytes[start:end])
                        pose = {
                            'x': float(x),
                            'y': float(y),
                            'z': float(z),
                            'rx': float(rx),
                            'ry': float(ry),
                            'rz': float(rz),
                        }

                        # reject bogus all-zero packets
                        if all(abs(pose[k]) < 1e-9 for k in pose):
                            return None

                        return pose
                    except Exception as exc:
                        self._log('DEBUG TM unpack failed:', repr(exc))
                        return None
        return None

    def _ensure_status_connection(self):
        if self._status_sock is None:
            try:
                self._status_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self._status_sock.settimeout(0.05)
                self._status_sock.connect((self.ip, self.port_position))
                self._status_buffer = b''
                self._log(f'Connected to TM status port at {self.ip}:{self.port_position}')
            except Exception as exc:
                self._log(f'Failed to connect to status port: {exc!r}')
                self._status_sock = None
        return self._status_sock

    def _get_loop(self):
        if self._loop is None or self._loop.is_closed():
            self._loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self._loop)
        return self._loop

    def get_position(self, timeout=0.3):
        """
        Fast pose read for runtime loops.
        - Return fresh pose immediately when available
        - Fall back to cached pose quickly if stream is briefly quiet
        """
        s = self._ensure_status_connection()
        now = time.time()

        if not s:
            if self._last_valid_pose is not None and (now - self._last_valid_pose_ts) < self._max_pose_cache_age_sec:
                self._log('DEBUG TM using cached last valid pose (no socket)')
                return self._last_valid_pose
            return None

        deadline = now + timeout
        empty_cycles = 0

        try:
            while time.time() < deadline:
                try:
                    chunk = s.recv(4096)
                    if not chunk:
                        self._log('DEBUG TM status socket closed by peer')
                        self.disconnect_status()
                        break

                    self._status_buffer += chunk

                    if len(self._status_buffer) > 131072:
                        self._status_buffer = self._status_buffer[-65536:]

                    pos = self._parse_binary_coords(self._status_buffer)
                    if pos is not None:
                        self._last_valid_pose = pos
                        self._last_valid_pose_ts = time.time()
                        self._log('Current Position (parsed):', pos)
                        return pos

                except socket.timeout:
                    empty_cycles += 1
                    if self._last_valid_pose is not None:
                        age = time.time() - self._last_valid_pose_ts
                        if age < self._max_pose_cache_age_sec:
                            self._log('DEBUG TM using cached last valid pose (fast fallback)')
                            return self._last_valid_pose
                    continue

        except Exception as e:
            self._log(f'get_position error: {e}')
            self.disconnect_status()

        if self._last_valid_pose is not None and (time.time() - self._last_valid_pose_ts) < self._max_pose_cache_age_sec:
            self._log('DEBUG TM using cached last valid pose')
            return self._last_valid_pose

        return None

    def disconnect_status(self):
        if self._status_sock:
            try:
                self._status_sock.close()
            except Exception:
                pass
            self._status_sock = None
        self._status_buffer = b''

    async def _move_async(self, target, speed_perc, acc_dur, precise):
        async with techmanpy.connect_sct(robot_ip=self.ip) as conn:
            await conn.move_to_point_ptp(
                tcp_point_goal=target,
                speed_perc=speed_perc,
                acceleration_duration=acc_dur,
                blending_perc=0.0,
                use_precise_positioning=precise,
                pose_goal=None,
            )

    def move(self, target_position, speed_perc=0.5, acceleration_duration=100,
             use_precise_positioning=False, retries=4):
        self._last_move_error = None

        dt = time.time() - self._last_move_ts
        if dt < self._min_move_interval_sec:
            time.sleep(self._min_move_interval_sec - dt)

        success = False
        for attempt in range(1, retries + 1):
            loop = self._get_loop()
            try:
                self._log(f'TM move attempt {attempt}, target (mm/deg): {target_position}')
                loop.run_until_complete(
                    self._move_async(
                        target_position,
                        speed_perc,
                        acceleration_duration,
                        use_precise_positioning
                    )
                )
                self._log(f'Move command sent to target position: {target_position}')
                success = True
                break
            except Exception as exc:
                self._last_move_error = str(exc)
                self._log(f'Failed to move on attempt {attempt}: {exc!r}')
                time.sleep(0.8)

        if success:
            self._last_move_ts = time.time()
            return True
        return False

    def get_last_move_error(self):
        return self._last_move_error

    def disconnect(self):
        self.disconnect_status()
        if self._loop and not self._loop.is_closed():
            self._loop.close()
        self._loop = None