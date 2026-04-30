from FS100 import FS100
from TM_api import TMRobot
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
        return [
            int(round(x * 1000)),
            int(round(y * 1000)),
            int(round(z * 1000)),
            int(round(Rx * 10000)),
            int(round(Ry * 10000)),
            int(round(Rz * 10000)),
            int(round(Re * 10000)),
        ]

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
                send_target_position[0:3] = [coord / 1000.0 for coord in target_position[0:3]]
                send_target_position[3:6] = [angle / 10000.0 for angle in target_position[3:6]]

            self._log('TM move target:', send_target_position)
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
                if all(abs(float(position[k])) < 1e-9 for k in required):
                    self._log('DEBUG TM zero pose ignored')
                    return None
                return {k: float(position[k]) for k in required}
            return None

        if isinstance(position, (list, tuple)) and len(position) >= 6:
            vals = [float(v) for v in position[:6]]
            if all(abs(v) < 1e-9 for v in vals):
                self._log('DEBUG TM zero pose ignored')
                return None
            return {
                'x': vals[0],
                'y': vals[1],
                'z': vals[2],
                'rx': vals[3],
                'ry': vals[4],
                'rz': vals[5],
            }

        attrs = ['x', 'y', 'z', 'rx', 'ry', 'rz']
        if all(hasattr(position, k) for k in attrs):
            vals = [float(getattr(position, k)) for k in attrs]
            if all(abs(v) < 1e-9 for v in vals):
                self._log('DEBUG TM zero pose ignored')
                return None
            return {
                'x': vals[0],
                'y': vals[1],
                'z': vals[2],
                'rx': vals[3],
                'ry': vals[4],
                'rz': vals[5],
            }

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
                            # Only scale if the caller explicitly wants FS100-style raw units.
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


def _pose_dict_to_mmdeg(pos_dict):
    # TM get_position() already returns mm / deg here.
    return [
        pos_dict['x'],
        pos_dict['y'],
        pos_dict['z'],
        pos_dict['rx'],
        pos_dict['ry'],
        pos_dict['rz'],
    ]


def _is_close(current, target, pos_tol=2.0, ang_tol=2.0):
    pos_ok = all(abs(current[i] - target[i]) <= pos_tol for i in range(3))
    ang_ok = all(abs(current[i] - target[i]) <= ang_tol for i in range(3, 6))
    return pos_ok and ang_ok


def wait_until_target(robot, target, timeout_sec=20.0, pos_tol=2.0, ang_tol=2.0):
    start = time.time()
    while time.time() - start < timeout_sec:
        status, pos = robot.get_position(retries=3, retry_delay=0.1)
        print('Polling:', status, pos)
        if status == 0 and pos is not None:
            current = _pose_dict_to_mmdeg(pos)
            print('Current mm/deg:', current)
            if _is_close(current, target, pos_tol=pos_tol, ang_tol=ang_tol):
                print('Reached target:', current)
                return True
        time.sleep(0.2)

    print('Timeout waiting for target')
    return False


def main():
    robot_type = 'TM'

    if robot_type == 'TM':
        robot = Robot(robot_type='TM', ip_address='192.168.10.3', debug=True)
        pose1 = [320, -125, 270, 170.00, 0.01, 90]
        pose2 = [320, -126, 330, 170.00, 0.01, 90]

        speed_perc = 1
        try:
            print('Sending move command to pose1')
            ret = robot.move(target_position=pose1, speed_perc=speed_perc, acceleration_duration=10)
            print('Move return:', ret)
            if ret == 0:
                wait_until_target(robot, pose1, timeout_sec=20.0)

            print('Sending move command to pose2')
            ret = robot.move(target_position=pose2, speed_perc=speed_perc, acceleration_duration=10)
            print('Move return:', ret)
            if ret == 0:
                wait_until_target(robot, pose2, timeout_sec=20.0)

            print('Sending move command to pose1')
            ret = robot.move(target_position=pose1, speed_perc=speed_perc, acceleration_duration=10)
            print('Move return:', ret)
            if ret == 0:
                wait_until_target(robot, pose1, timeout_sec=20.0)
        finally:
            robot.disconnect()

    elif robot_type == 'FS100':
        robot = Robot(robot_type='FS100', ip_address='172.16.0.1', debug=True)
        pose1 = [502.586, 6.933, -240.251, 179.5431, 0.0804, 0.0945]
        pose2 = [502.586, 6.933, 30.352, 179.5431, 0.0804, 0.0945]

        print('Sending move command to pose1')
        ret = robot.move(target_position=pose1, speed_perc=0.5, acceleration_duration=100)
        print('Move return:', ret)

        print('Sending move command to pose2')
        ret = robot.move(target_position=pose2, speed_perc=0.5, acceleration_duration=100)
        print('Move return:', ret)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass