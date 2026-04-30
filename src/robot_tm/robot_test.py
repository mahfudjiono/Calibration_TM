from FS100 import FS100
from robot_tm.TM_api import TMRobot
from techmanpy import TechmanException
import time
from typing import Any, Dict, Optional


class Robot:
    def __init__(
        self,
        robot_type: str = 'FS100',
        ip_address: str = '192.168.171.222',
        port_control: Optional[int] = None,
        port_position: Optional[int] = None,
        linear_tolerance: float = 0.05,
        angular_tolerance: float = 0.05,
        debug: bool = False,
    ):
        self.robot_type = robot_type
        self.robot_ip = ip_address
        self.port_control = port_control
        self.port_position = port_position
        self.linear_tolerance = linear_tolerance
        self.angular_tolerance = angular_tolerance
        self.debug = debug
        self.robot = None
        self.connect()

    def _log(self, *args):
        if self.debug:
            print(*args)

    def connect(self):
        if self.robot_type == 'FS100':
            self.robot = FS100(self.robot_ip)
            self.robot.switch_power(FS100.POWER_TYPE_SERVO, FS100.POWER_SWITCH_ON)
        elif self.robot_type == 'TM':
            self.robot = TMRobot(
                ip=self.robot_ip,
                port_control=self.port_control or 5890,
                port_position=self.port_position or 5891,
                verbose=self.debug,
            )
        else:
            raise ValueError(f'Unsupported robot_type: {self.robot_type}')

    def disconnect(self):
        if self.robot_type == 'TM' and self.robot is not None and hasattr(self.robot, 'disconnect'):
            try:
                self.robot.disconnect()
            except Exception as exc:
                self._log('TM disconnect warning:', repr(exc))

    def _fs100_mmdeg_to_raw(self, pose):
        if len(pose) not in (6, 7):
            raise ValueError(f'FS100 needs 6 or 7 values (x,y,z,Rx,Ry,Rz[,Re]), got {len(pose)}')
        p = list(pose) + [0] if len(pose) == 6 else list(pose)
        x, y, z, Rx, Ry, Rz, Re = p
        xi = int(round(x * 1000))
        yi = int(round(y * 1000))
        zi = int(round(z * 1000))
        rxi = int(round(Rx * 10000))
        ryi = int(round(Ry * 10000))
        rzi = int(round(Rz * 10000))
        rei = int(round(Re * 10000))
        return [xi, yi, zi, rxi, ryi, rzi, rei]

    @staticmethod
    def clamp_position(pos):
        return [max(-2147483648, min(2147483647, p)) for p in pos]

    def _fs100_speed_mmps_to_raw(self, mm_per_s):
        return int(round(mm_per_s * 10))

    def move(
        self,
        target_position=[0, 0, 0, 0, 0, 0],
        tool_no=8,
        speed=100,
        speed_perc=0.5,
        acceleration_duration=100,
        divide_1000=False,
        use_precise_positioning=False,
    ):
        if self.robot_type == 'FS100':
            raw_pos = self._fs100_mmdeg_to_raw(target_position)
            raw_speed = self._fs100_speed_mmps_to_raw(speed) if isinstance(speed, (int, float)) else speed
            return self.robot.mov(
                FS100.MOVE_TYPE_LINEAR_ABSOLUTE_POS,
                FS100.MOVE_COORDINATE_SYSTEM_ROBOT,
                FS100.MOVE_SPEED_CLASS_MILLIMETER,
                raw_speed,
                raw_pos,
                tool_no=tool_no,
            )

        if self.robot_type == 'TM':
            send_target_position = list(target_position)
            if divide_1000:
                # Only divide xyz if caller provided mm*1000 style raw units.
                send_target_position[0:3] = [coord / 1000.0 for coord in target_position[0:3]]
                send_target_position[3:6] = [angle / 10000.0 for angle in target_position[3:6]]

            self._log('TM move target (mm/deg):', send_target_position)
            ok = self.robot.move(
                send_target_position,
                speed_perc,
                acceleration_duration,
                use_precise_positioning=use_precise_positioning,
            )
            if not ok and hasattr(self.robot, 'get_last_move_error'):
                self._log('TM move error:', self.robot.get_last_move_error())
            return 0 if ok else 1

        raise ValueError(f'Unsupported robot_type: {self.robot_type}')

    def _tm_position_to_dict(self, position: Any) -> Optional[Dict[str, float]]:
        if position is None:
            return None
        if isinstance(position, dict):
            required = ['x', 'y', 'z', 'rx', 'ry', 'rz']
            if all(k in position for k in required):
                return {k: float(position[k]) for k in required}
            return None
        if isinstance(position, (list, tuple)) and len(position) >= 6:
            return {
                'x': float(position[0]),
                'y': float(position[1]),
                'z': float(position[2]),
                'rx': float(position[3]),
                'ry': float(position[4]),
                'rz': float(position[5]),
            }
        attrs = ['x', 'y', 'z', 'rx', 'ry', 'rz']
        if all(hasattr(position, k) for k in attrs):
            return {k: float(getattr(position, k)) for k in attrs}
        return None

    def _tm_try_get_position_once(self):
        if hasattr(self.robot, 'get_position'):
            return self.robot.get_position()
        return None

    def get_position(self, times_1000: bool = False, retries: int = 3, retry_delay: float = 0.1):
        if self.robot_type == 'FS100':
            info = {}
            status = self.robot.read_position(info)
            return status, info

        if self.robot_type == 'TM':
            last_raw = None
            last_exception = None
            for attempt in range(1, max(1, retries) + 1):
                try:
                    raw_position = self._tm_try_get_position_once()
                    self._log(f'DEBUG TM raw get_position() attempt {attempt}:', repr(raw_position))
                    last_raw = raw_position
                    position_dict = self._tm_position_to_dict(raw_position)
                    if position_dict is not None:
                        if times_1000:
                            position_dict = position_dict.copy()
                            position_dict['x'] *= 1000.0
                            position_dict['y'] *= 1000.0
                            position_dict['z'] *= 1000.0
                            position_dict['rx'] *= 10000.0
                            position_dict['ry'] *= 10000.0
                            position_dict['rz'] *= 10000.0
                        return 0, position_dict
                except (TechmanException, OSError, TimeoutError, ValueError) as exc:
                    last_exception = exc
                    self._log(f'DEBUG TM get_position exception attempt {attempt}:', repr(exc))
                except Exception as exc:
                    last_exception = exc
                    self._log(f'DEBUG TM unexpected get_position exception attempt {attempt}:', repr(exc))

                if attempt < retries:
                    time.sleep(retry_delay)

            if last_exception is not None:
                return -3, {'error': repr(last_exception), 'raw_position': last_raw}
            return -1, {'error': 'TM get_position returned None or unsupported format', 'raw_position': last_raw}

        raise ValueError(f'Unsupported robot_type: {self.robot_type}')

    def switch_power(self, power_type='servo', switch_status='on'):
        if self.robot_type == 'FS100':
            if power_type == 'servo':
                power_type_const = FS100.POWER_TYPE_SERVO
            elif power_type == 'hold':
                power_type_const = FS100.POWER_TYPE_HOLD
            elif power_type == 'hlock':
                power_type_const = FS100.POWER_TYPE_HLOCK
            else:
                raise ValueError('Invalid power type')

            if switch_status == 'on':
                switch_status_const = FS100.POWER_SWITCH_ON
            elif switch_status == 'off':
                switch_status_const = FS100.POWER_SWITCH_OFF
            else:
                raise ValueError('Invalid switch status')
            return self.robot.switch_power(power_type_const, switch_status_const)

        if self.robot_type == 'TM':
            return 0
        raise ValueError(f'Unsupported robot_type: {self.robot_type}')

    def reset_alarm(self, alarm_type=FS100.RESET_ALARM_TYPE_ERROR):
        if self.robot_type == 'FS100':
            return self.robot.reset_alarm(FS100.RESET_ALARM_TYPE_ALARM)
        if self.robot_type == 'TM':
            return 0
        raise ValueError(f'Unsupported robot_type: {self.robot_type}')


def _extract_pose_for_print(robot_type: str, raw_position: Any):
    if robot_type == 'FS100':
        pos = list(raw_position['pos'])[:6]
        return [
            pos[0] / 1000.0,
            pos[1] / 1000.0,
            pos[2] / 1000.0,
            pos[3] / 10000.0,
            pos[4] / 10000.0,
            pos[5] / 10000.0,
        ]
    if robot_type == 'TM':
        return [
            raw_position['x'] / 1000.0,
            raw_position['y'] / 1000.0,
            raw_position['z'] / 1000.0,
            raw_position['rx'] / 10000.0,
            raw_position['ry'] / 10000.0,
            raw_position['rz'] / 10000.0,
        ]
    raise ValueError(f'Unsupported robot_type: {robot_type}')


def _print_pose(label: str, pose_mmdeg):
    print(
        f"{label}: X={pose_mmdeg[0]:.3f}, Y={pose_mmdeg[1]:.3f}, Z={pose_mmdeg[2]:.3f} mm | "
        f"RX={pose_mmdeg[3]:.3f}, RY={pose_mmdeg[4]:.3f}, RZ={pose_mmdeg[5]:.3f} deg"
    )


def _wait_until_close(robot: Robot, robot_type: str, target_mmdeg, timeout_sec: float = 12.0,
                      pos_tol_mm: float = 2.0, ang_tol_deg: float = 2.0, poll_sec: float = 0.2):
    deadline = time.time() + timeout_sec
    last_pose = None
    while time.time() < deadline:
        status, raw = robot.get_position(times_1000=True, retries=3, retry_delay=0.1)
        if status == 0 and raw is not None:
            last_pose = _extract_pose_for_print(robot_type, raw)
            _print_pose('Current', last_pose)
            pos_ok = all(abs(last_pose[i] - target_mmdeg[i]) <= pos_tol_mm for i in range(3))
            ang_ok = all(abs(last_pose[i] - target_mmdeg[i]) <= ang_tol_deg for i in range(3, 6))
            if pos_ok and ang_ok:
                return True, last_pose
        else:
            print('Read position failed:', status, raw)
        time.sleep(poll_sec)
    return False, last_pose


if __name__ == '__main__':
    """
    Simple movement test.

    Edit ROBOT_TYPE / IP / test poses below, then run:
        python3 robot_command.py
    """
    ROBOT_TYPE = 'TM'   # 'TM' or 'FS100'
    IP_ADDRESS = '192.168.10.3' if ROBOT_TYPE == 'TM' else '172.16.0.1'
    DEBUG = True

    # Example poses in mm / deg
    if ROBOT_TYPE == 'TM':
        test_pose_1 = [320.0, -125.0, 270.0, 170.0, 0.01, 90.0]
        test_pose_2 = [320.0, -126.0, 330.0, 170.0, 0.01, 90.0]
    else:
        test_pose_1 = [538.755, 5.427, 80.534, 179.9834, 0.0524, 0.0883]
        test_pose_2 = [538.757, 5.363, -40.024, 179.9834, 0.0524, 0.0883]

    robot = Robot(robot_type=ROBOT_TYPE, ip_address=IP_ADDRESS, debug=DEBUG)

    try:
        print('=' * 70)
        print(f'STARTING {ROBOT_TYPE} MOVEMENT TEST')
        print('=' * 70)

        status, raw = robot.get_position(times_1000=True, retries=5, retry_delay=0.2)
        if status == 0 and raw is not None:
            _print_pose('Initial pose', _extract_pose_for_print(ROBOT_TYPE, raw))
        else:
            print('Could not read initial pose:', status, raw)

        for idx, target in enumerate([test_pose_1, test_pose_2, test_pose_1], start=1):
            print('\n' + '-' * 70)
            print(f'MOVE {idx}')
            _print_pose('Target', target)
            ret = robot.move(
                target_position=target,
                tool_no=8,
                speed=50,
                speed_perc=0.5,
                acceleration_duration=100,
                divide_1000=False,
                use_precise_positioning=False,
            )
            print('Move return:', ret)

            if ret != 0:
                print('Move command failed.')
                continue

            reached, final_pose = _wait_until_close(
                robot,
                ROBOT_TYPE,
                target,
                timeout_sec=12.0,
                pos_tol_mm=2.0,
                ang_tol_deg=2.0,
                poll_sec=0.2,
            )
            if reached:
                print('Reached target.')
            else:
                print('Did not confirm target within timeout.')
                if final_pose is not None:
                    _print_pose('Last pose', final_pose)

        print('\nDone.')
    finally:
        robot.disconnect()
