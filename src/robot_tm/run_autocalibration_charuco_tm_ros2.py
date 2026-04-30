from robot_command import Robot
from time import sleep, time
from threading import Event, Lock, Thread
import importlib


def _load_ros2_modules():
    """Lazily import ROS2 modules so editors without a sourced ROS env do not complain as much."""
    try:
        rclpy = importlib.import_module('rclpy')
        node_mod = importlib.import_module('rclpy.node')
        sensor_msg_mod = importlib.import_module('sensor_msgs.msg')
        cv_bridge_mod = importlib.import_module('cv_bridge')
        return rclpy, node_mod.Node, sensor_msg_mod.Image, cv_bridge_mod.CvBridge
    except Exception as exc:
        raise ImportError(
            'ROS2 image capture requires rclpy, sensor_msgs, and cv_bridge. '
            'Source your ROS2 environment first, then install cv_bridge for the same Python interpreter.'
        ) from exc


class ROS2TopicCameraCapture:
    def __init__(self, topic_name='/techman_image'):
        self._rclpy, NodeBase, ImageMsg, CvBridgeClass = _load_ros2_modules()
        self._ImageMsg = ImageMsg
        self._bridge = CvBridgeClass()
        self._lock = Lock()
        self._latest_frame = None
        self._latest_stamp = None
        self._frame_event = Event()
        self._topic_name = topic_name

        class _ROS2ImageSubscriber(NodeBase):
            def __init__(inner_self, outer):
                super().__init__('tmflow_image_capture_node')
                inner_self._outer = outer
                inner_self._subscription = inner_self.create_subscription(
                    outer._ImageMsg, topic_name, inner_self._image_callback, 10
                )

            def _image_callback(inner_self, msg):
                try:
                    frame = inner_self._outer._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                    with inner_self._outer._lock:
                        inner_self._outer._latest_frame = frame
                        inner_self._outer._latest_stamp = msg.header.stamp
                    inner_self._outer._frame_event.set()
                except Exception as exc:
                    inner_self.get_logger().warning(f'Failed to convert ROS image: {exc}')

        if not self._rclpy.ok():
            self._rclpy.init(args=None)
            self._owns_rclpy = True
        else:
            self._owns_rclpy = False

        self.node = _ROS2ImageSubscriber(self)
        self._stop_event = Event()
        self._spin_thread = Thread(target=self._spin, daemon=True)
        self._spin_thread.start()

        if not self.wait_for_first_frame(timeout_sec=8.0):
            raise RuntimeError(f'No image received from ROS2 topic: {topic_name}')

    def _spin(self):
        while not self._stop_event.is_set() and self._rclpy.ok():
            self._rclpy.spin_once(self.node, timeout_sec=0.1)

    def read_latest(self):
        with self._lock:
            if self._latest_frame is None:
                return False, None
            return True, self._latest_frame.copy()

    def wait_for_first_frame(self, timeout_sec=5.0):
        return self._frame_event.wait(timeout=timeout_sec)

    def read(self):
        return self.read_latest()

    def release(self):
        self._stop_event.set()
        if self._spin_thread.is_alive():
            self._spin_thread.join(timeout=1.0)
        try:
            self.node.destroy_node()
        except Exception:
            pass
        if self._owns_rclpy and self._rclpy.ok():
            try:
                self._rclpy.shutdown()
            except Exception:
                pass


import cv2
import numpy as np
import random
import pickle
import os
import copy


# =============================================================================
# Helper Functions
# =============================================================================

def pose_to_mmdeg(p):
    """
    Convert a pose to [x, y, z, rx, ry, rz] in [mm, mm, mm, deg, deg, deg].
    Accepts positions in meters or mm, and angles in radians or degrees.
    """
    x, y, z, rx, ry, rz = p

    if max(abs(x), abs(y), abs(z)) < 10.0:
        x, y, z = x * 1000.0, y * 1000.0, z * 1000.0

    if max(abs(rx), abs(ry), abs(rz)) < 6.5:
        rx, ry, rz = np.rad2deg(rx), np.rad2deg(ry), np.rad2deg(rz)

    return [float(x), float(y), float(z), float(rx), float(ry), float(rz)]


def raw_pose_to_mmdeg(pos_tuple):
    """
    Convert raw robot pose to [mm, mm, mm, deg, deg, deg].
    For both FS100 raw and TM get_position(times_1000=True), the convention is:
      x,y,z in mm*1000
      rx,ry,rz in deg*10000
    """
    raw6 = list(pos_tuple)[:6]
    return [
        raw6[0] / 1000.0,
        raw6[1] / 1000.0,
        raw6[2] / 1000.0,
        raw6[3] / 10000.0,
        raw6[4] / 10000.0,
        raw6[5] / 10000.0,
    ]


def extract_raw_pose(robot_type, raw_position):
    """Extract a raw 6-value pose list from robot.get_position(...)."""
    if robot_type == 'FS100':
        return list(raw_position['pos'])[:6]
    if robot_type == 'TM':
        return [raw_position[k] for k in ('x', 'y', 'z', 'rx', 'ry', 'rz')]
    raise ValueError(f'Unsupported robot_type: {robot_type}')


def normalize_angle_deg(angle):
    angle = angle % 360
    if angle > 180:
        angle -= 360
    return angle


def angle_diff_deg(a, b):
    diff = abs(normalize_angle_deg(a) - normalize_angle_deg(b))
    return min(diff, 360 - diff)


def is_at_target(current_mmdeg, target_mmdeg, pos_tol=2.0, ang_tol=2.0):
    pos_diff = np.abs(np.array(current_mmdeg[:3]) - np.array(target_mmdeg[:3]))
    ang_diff = np.array([
        angle_diff_deg(current_mmdeg[3], target_mmdeg[3]),
        angle_diff_deg(current_mmdeg[4], target_mmdeg[4]),
        angle_diff_deg(current_mmdeg[5], target_mmdeg[5]),
    ])
    return np.all(pos_diff <= pos_tol) and np.all(ang_diff <= ang_tol)


def check_pose_workspace(pose_mmdeg, robot_type='TM'):
    """Basic workspace sanity check in mm/deg."""
    x, y, z = pose_mmdeg[0], pose_mmdeg[1], pose_mmdeg[2]

    if robot_type == 'FS100':
        if x < 400 or x > 750:
            return False, f"X={x:.1f}mm out of range (400-750)"
        if y < -200 or y > 200:
            return False, f"Y={y:.1f}mm out of range (-200-200)"
        if z < 80 or z > 350:
            return False, f"Z={z:.1f}mm out of range (80-350)"
    else:
        if x < 250 or x > 450:
            return False, f"X={x:.1f}mm out of range (250-450)"
        if y < -250 or y > 50:
            return False, f"Y={y:.1f}mm out of range (-250-50)"
        if z < 200 or z > 380:
            return False, f"Z={z:.1f}mm out of range (200-380)"

    return True, 'OK'


def normalize_angle_rad(angle):
    while angle > np.pi:
        angle -= 2 * np.pi
    while angle < -np.pi:
        angle += 2 * np.pi
    return angle


# =============================================================================
# Pose Generation Functions
# =============================================================================

def get_robot_poses_with_variation(args, synthetic=False):
    """Generate robot poses with orientation variation for calibration."""
    lower_bound_m = [args['lower_bound'][0] / 1000.0, args['lower_bound'][1] / 1000.0, args['lower_bound'][2] / 1000.0]
    upper_bound_m = [args['upper_bound'][0] / 1000.0, args['upper_bound'][1] / 1000.0, args['upper_bound'][2] / 1000.0]

    base_rx = np.deg2rad(args['initial_pos1'][3])
    base_ry = np.deg2rad(args['initial_pos1'][4])
    base_rz = np.deg2rad(args['initial_pos1'][5])

    rx_variation_deg = 15
    ry_variation_deg = 15
    rz_variation_deg = 20

    print(f"Base orientation: RX={args['initial_pos1'][3]:.1f}, RY={args['initial_pos1'][4]:.1f}, RZ={args['initial_pos1'][5]:.1f} deg")
    print(f"Orientation variation: RX=±{rx_variation_deg}, RY=±{ry_variation_deg}, RZ=±{rz_variation_deg} deg")

    x_vals = np.arange(lower_bound_m[0], upper_bound_m[0] + 1e-9, args['distance_step'])
    y_vals = np.arange(lower_bound_m[1], upper_bound_m[1] + 1e-9, args['distance_step'])
    z_base = (lower_bound_m[2] + upper_bound_m[2]) / 2.0

    robot_poses = []

    for i, x in enumerate(x_vals):
        # Reverse Y direction every other X row to create a zigzag/snake pattern
        # this ensures smooth pose-to-pose flow without jumping back.
        current_y_vals = y_vals[::-1] if i % 2 == 1 else y_vals
        
        for y in current_y_vals:
            for z_offset in [-0.02, 0.0, 0.02]:
                z = z_base + z_offset
                if z < lower_bound_m[2] or z > upper_bound_m[2]:
                    continue

                rx_var = np.deg2rad(random.uniform(-rx_variation_deg, rx_variation_deg))
                ry_var = np.deg2rad(random.uniform(-ry_variation_deg, ry_variation_deg))
                rz_var = np.deg2rad(random.uniform(-rz_variation_deg, rz_variation_deg))

                pose = [
                    x,
                    y,
                    z,
                    normalize_angle_rad(base_rx + rx_var),
                    normalize_angle_rad(base_ry + ry_var),
                    normalize_angle_rad(base_rz + rz_var),
                ]
                robot_poses.append(pose)

    # Remove duplicates while PRESERVING the zigzag order
    seen = set()
    unique_poses = []
    for p in robot_poses:
        t = tuple(p)
        if t not in seen:
            unique_poses.append(p)
            seen.add(t)
    return unique_poses


def get_robot_poses_fixed_orientation(args, synthetic=False):
    """Generate robot poses with fixed orientation."""
    lower_bound_m = [args['lower_bound'][0] / 1000.0, args['lower_bound'][1] / 1000.0, args['lower_bound'][2] / 1000.0]
    upper_bound_m = [args['upper_bound'][0] / 1000.0, args['upper_bound'][1] / 1000.0, args['upper_bound'][2] / 1000.0]

    fixed_rx = np.deg2rad(args['initial_pos1'][3])
    fixed_ry = np.deg2rad(args['initial_pos1'][4])
    fixed_rz = np.deg2rad(args['initial_pos1'][5])

    print(f"Fixed orientation: RX={args['initial_pos1'][3]:.1f}, RY={args['initial_pos1'][4]:.1f}, RZ={args['initial_pos1'][5]:.1f} deg")

    x_vals = np.arange(lower_bound_m[0], upper_bound_m[0] + 1e-9, args['distance_step'])
    y_vals = np.arange(lower_bound_m[1], upper_bound_m[1] + 1e-9, args['distance_step'])
    z_val = (lower_bound_m[2] + upper_bound_m[2]) / 2.0

    robot_poses = []
    for x in x_vals:
        for y in y_vals:
            robot_poses.append([x, y, z_val, fixed_rx, fixed_ry, fixed_rz])

    z_variations = [z_val - 0.02, z_val, z_val + 0.02]
    for x in x_vals[::2]:
        for y in y_vals[::2]:
            for z_var in z_variations:
                if lower_bound_m[2] <= z_var <= upper_bound_m[2]:
                    robot_poses.append([x, y, z_var, fixed_rx, fixed_ry, fixed_rz])

    robot_poses = [list(t) for t in set(tuple(p) for p in robot_poses)]
    return robot_poses


def save_poses(path, robot_poses, initial_pos0, initial_pos1):
    data = {'robot_poses': robot_poses, 'initial_pos0': initial_pos0, 'initial_pos1': initial_pos1}
    with open(path, 'wb') as f:
        pickle.dump(data, f)


def visualize_robot_poses(robot_poses, initial_pos0, initial_pos1):
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

    np_poses = np.array(robot_poses)
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')

    ax.scatter(np_poses[:, 0] * 1000, np_poses[:, 1] * 1000, np_poses[:, 2] * 1000,
               marker='o', s=20, alpha=0.6, label='Robot poses')

    ax.scatter(initial_pos0[0], initial_pos0[1], initial_pos0[2], marker='s', s=100, label='Initial Pos0')
    ax.scatter(initial_pos1[0], initial_pos1[1], initial_pos1[2], marker='^', s=100, label='Initial Pos1')

    x_range = (np.max(np_poses[:, 0]) - np.min(np_poses[:, 0])) * 1000
    y_range = (np.max(np_poses[:, 1]) - np.min(np_poses[:, 1])) * 1000
    z_range = (np.max(np_poses[:, 2]) - np.min(np_poses[:, 2])) * 1000

    ax.set_xlabel('X (mm)')
    ax.set_ylabel('Y (mm)')
    ax.set_zlabel('Z (mm)')
    ax.set_title(f'Robot Poses - Movement Range: X={x_range:.0f}mm, Y={y_range:.0f}mm, Z={z_range:.0f}mm')
    ax.legend()
    plt.tight_layout()
    plt.show()


def warmup_position_read(robot, robot_type, max_tries=10, delay=0.2, retries=5, retry_delay=0.1):
    """Try a few reads before starting the main loop."""
    for attempt in range(1, max_tries + 1):
        status, raw_position = robot.get_position(times_1000=True, retries=retries, retry_delay=retry_delay)
        print(f'Warmup read {attempt}/{max_tries}: status={status}, raw_position={raw_position}')
        if status == 0 and raw_position is not None:
            return extract_raw_pose(robot_type, raw_position)
        sleep(delay)
    return None


def capture_charuco_sample(cap, aruco_dict, aruco_params, charuco_board, save_root, sample_idx, raw_position, max_attempts=20):
    """
    Capture a single frame and save it if Charuco board is detected.
    Always returns after max_attempts, even if detection fails.
    """
    for attempt in range(max_attempts):
        ret2, img2 = cap.read()
        if not ret2:
            sleep(0.05)
            continue

        gray2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)
        m_c2, m_ids2, _ = detect_markers(gray2)
        
        if m_ids2 is None or len(m_ids2) == 0:
            sleep(0.05)
            continue

        retval2, cu_corners2, cu_ids2, _, _ = detect_charuco(gray2, m_c2, m_ids2)
        
        # Require at least 4 Charuco corners for good calibration
        if retval2 is not None and retval2 >= 4 and cu_ids2 is not None and len(cu_ids2) >= 4:
            print(f'  ✓ Saved sample {sample_idx} with {retval2} Charuco corners')
            rgb_path = os.path.join(save_root, 'rgb', f'{sample_idx}.png')
            checker_path = os.path.join(save_root, 'checkerboard', f'{sample_idx}.png')
            cv2.imwrite(rgb_path, img2)

            rendered_dbg = img2.copy()
            try:
                cv2.aruco.drawDetectedMarkers(rendered_dbg, m_c2, m_ids2)
                cv2.aruco.drawDetectedCornersCharuco(rendered_dbg, cu_corners2, cu_ids2)
            except Exception:
                pass
            cv2.imwrite(checker_path, rendered_dbg)
            return True, list(raw_position)
        
        sleep(0.05)
    
    # Always return after max_attempts, even if failed
    print(f'  ✗ Charuco board not detected after {max_attempts} attempts')
    return False, None


# =============================================================================
# Main
# =============================================================================
if __name__ == '__main__':
    # ------------------------------
    # User settings
    # ------------------------------
    robot_type = 'TM'              # 'FS100' or 'TM'
    hand_eye_calibration = True
    create_pose = False           # True: generate poses only, False: collect data
    pose_mode = 'variation'        # 'fixed' or 'variation'
    synthetic = False
    robot_debug = True            # Set True if you want TM socket debug prints
    camera_source = 'ros2_topic'   # 'ids' or 'ros2_topic'
    ros_image_topic = '/techman_image'

    # Robot networking
    fs100_ip = '172.16.0.1'
    tm_ip = '192.168.10.3'         # Change if your TM controller is on another IP

    # Motion settings
    tool_no = 8
    robot_move_speed_yaskawa = 50  # mm/s
    robot_move_speed_tm = 1        # percent
    tm_acceleration_duration = 10

    # Read/arrival settings
    tm_position_retries = 5
    tm_position_retry_delay = 0.1
    pos_tol_mm = 2.0               # Tolerance for position arrival
    ang_tol_deg = 2.0              # Tolerance for angle arrival

    rgb_shape = np.array([480, 640, 3])

    if hand_eye_calibration:
        args = {
            # TM example in mm/deg - using known reachable positions
            'initial_pos0': [320, -125, 270, 170.00, 0.01, 90],
            'initial_pos1': [320, -126, 330, 170.00, 0.01, 90],
            # Reduced bounds to only reachable area
            'lower_bound': [290, -190, 250],
            'upper_bound': [350, -60, 310],

            'max_radius': 0.08,
            'max_height': 0.08,
            'distance_step': 0.025,  # Larger step = fewer poses
            'circumference_step': 0.035,
            'distance_noise_range': 0.01,
            'angular_noise_range': np.pi / 36,
            'rz_noise_range': np.pi / 8,
            'pitch_noise_range': np.pi / 36,
            'yaw_noise_range': np.pi / 36,
            'n_randomize_each_pose': 1,
            'img_shape': rgb_shape,
            'box_threshold': (1, 1),
            'shuffle': False,
            'check_if_object_exists': False,
        }
    else:
        args = {
            'initial_pos1': [320, -125, 270, 170.00, 0.01, 90],
            'initial_pos0': [320, -126, 330, 170.00, 0.01, 90],
            'lower_bound': [318, -123, 247],
            'upper_bound': [320, -121.68, 320],
            'max_radius': 0.12,
            'max_height': 0.08,
            'distance_step': 0.025,
            'circumference_step': 0.035,
            'distance_noise_range': 0.01,
            'angular_noise_range': np.pi / 36,
            'rz_noise_range': np.pi / 8,
            'pitch_noise_range': np.pi / 36,
            'yaw_noise_range': np.pi / 36,
            'n_randomize_each_pose': 1,
            'img_shape': rgb_shape,
            'box_threshold': (1, 1),
            'shuffle': False,
            'check_if_object_exists': False,
        }

    initial_pose_mmdeg = pose_to_mmdeg(args['initial_pos1'])
    save_root = '/home/hucenrotia/mahfud/Calibration_TM/src/local_data/robot_pose_dataset'
    os.makedirs(os.path.join(save_root, 'rgb'), exist_ok=True)
    os.makedirs(os.path.join(save_root, 'checkerboard'), exist_ok=True)

    try:
        ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000)
    except AttributeError:
        ARUCO_DICT = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_1000)
    print('Using Charuco board for detection.')

    try:
        CHARUCO_BOARD = cv2.aruco.CharucoBoard_create(10, 8, 0.020, 0.015, ARUCO_DICT)
    except AttributeError:
        CHARUCO_BOARD = cv2.aruco.CharucoBoard((10, 8), 0.020, 0.015, ARUCO_DICT)
    print('Charuco board created.')

    try:
        aruco_params = cv2.aruco.DetectorParameters_create()
    except AttributeError:
        aruco_params = cv2.aruco.DetectorParameters()
    print('Aruco detector parameters set.')

    # OpenCV ArUco / ChArUco compatibility helpers
    if hasattr(CHARUCO_BOARD, 'setLegacyPattern'):
        try:
            CHARUCO_BOARD.setLegacyPattern(True)
        except Exception:
            pass

    if hasattr(cv2.aruco, 'ArucoDetector'):
        aruco_detector = cv2.aruco.ArucoDetector(ARUCO_DICT, aruco_params)

        def detect_markers(gray_img):
            return aruco_detector.detectMarkers(gray_img)
    else:
        def detect_markers(gray_img):
            return cv2.aruco.detectMarkers(gray_img, ARUCO_DICT, parameters=aruco_params)

    if hasattr(cv2.aruco, 'CharucoDetector'):
        try:
            charuco_detector = cv2.aruco.CharucoDetector(CHARUCO_BOARD, detectorParams=aruco_params)
        except TypeError:
            charuco_detector = cv2.aruco.CharucoDetector(CHARUCO_BOARD)

        def detect_charuco(gray_img, marker_corners=None, marker_ids=None):
            if marker_corners is None or marker_ids is None or len(marker_corners) == 0:
                charuco_corners, charuco_ids, marker_corners, marker_ids = charuco_detector.detectBoard(gray_img)
            else:
                charuco_corners, charuco_ids, marker_corners, marker_ids = charuco_detector.detectBoard(
                    gray_img, markerCorners=marker_corners, markerIds=marker_ids
                )
            retval = 0 if charuco_ids is None else len(charuco_ids)
            return retval, charuco_corners, charuco_ids, marker_corners, marker_ids
    else:
        def detect_charuco(gray_img, marker_corners=None, marker_ids=None):
            retval, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
                marker_corners, marker_ids, gray_img, CHARUCO_BOARD
            )
            return retval, charuco_corners, charuco_ids, marker_corners, marker_ids

    if hand_eye_calibration:
        robot_poses_fname = '/home/hucenrotia/mahfud/Calibration_TM/src/local_data/robot_poses_hand_eye_calibration.pkl'
        synthetic = False
    else:
        robot_poses_fname = '/home/hucenrotia/mahfud/Calibration_TM/src/local_data/robot_poses.pkl'

    # ---------------------------------
    # Pose generation only
    # ---------------------------------
    if create_pose and not synthetic:
        print('\n' + '=' * 60)
        print('GENERATING ROBOT POSES')
        print(f'Mode: {pose_mode}')
        print('=' * 60)

        if pose_mode == 'variation':
            robot_poses = get_robot_poses_with_variation(args)
        else:
            robot_poses = get_robot_poses_fixed_orientation(args)

        print(f'Generated {len(robot_poses)} robot poses')

        poses_array = np.array(robot_poses)
        x_range = (np.max(poses_array[:, 0]) - np.min(poses_array[:, 0])) * 1000
        y_range = (np.max(poses_array[:, 1]) - np.min(poses_array[:, 1])) * 1000
        z_range = (np.max(poses_array[:, 2]) - np.min(poses_array[:, 2])) * 1000

        print('\nMovement range:')
        print(f'  X: {x_range:.1f} mm')
        print(f'  Y: {y_range:.1f} mm')
        print(f'  Z: {z_range:.1f} mm')

        rx_range = np.rad2deg(np.max(poses_array[:, 3]) - np.min(poses_array[:, 3]))
        ry_range = np.rad2deg(np.max(poses_array[:, 4]) - np.min(poses_array[:, 4]))
        rz_range = np.rad2deg(np.max(poses_array[:, 5]) - np.min(poses_array[:, 5]))
        print('\nRotation range:')
        print(f'  RX: {rx_range:.1f} deg')
        print(f'  RY: {ry_range:.1f} deg')
        print(f'  RZ: {rz_range:.1f} deg')

        print('\nChecking workspace feasibility...')
        all_ok = True
        for idx, pose in enumerate(robot_poses[:50]):
            pose_mmdeg = pose_to_mmdeg(pose)
            ok, msg = check_pose_workspace(pose_mmdeg, robot_type=robot_type)
            if not ok:
                print(f'  Pose {idx}: {msg}')
                all_ok = False

        if all_ok:
            print('✓ All poses within workspace!')
        else:
            print('\n⚠️ Some poses may be outside workspace. Adjust the bounds if needed.')

        save_poses(robot_poses_fname, robot_poses, args['initial_pos0'], args['initial_pos1'])
        print(f'\nSaved {len(robot_poses)} poses to: {robot_poses_fname}')
        visualize_robot_poses(robot_poses, args['initial_pos0'], args['initial_pos1'])
        print('\nNow set create_pose = False and run again to collect data')
        raise SystemExit

    # ---------------------------------
    # Data collection
    # ---------------------------------
    try:
        robot_poses_data = pickle.load(open(robot_poses_fname, 'rb'))
    except Exception:
        print('ERROR: Robot poses pickle file not found')
        print('Run with create_pose=True first to generate poses')
        raise SystemExit

    robot_poses = robot_poses_data['robot_poses']
    hand_eye_calibration_robot_poses = []
    print(f'Loaded {len(robot_poses)} robot poses')

    poses_array = np.array(robot_poses)
    x_range = (np.max(poses_array[:, 0]) - np.min(poses_array[:, 0])) * 1000
    y_range = (np.max(poses_array[:, 1]) - np.min(poses_array[:, 1])) * 1000
    print(f'Loaded poses movement range: X={x_range:.1f}mm, Y={y_range:.1f}mm')

    print('Robot poses loaded. Press enter to start the robot and image stream')
    x = input()
    if x == 'stop':
        raise SystemExit

    if robot_type == 'FS100':
        robot = Robot(robot_type='FS100', ip_address=fs100_ip, debug=robot_debug)
    elif robot_type == 'TM':
        robot = Robot(robot_type='TM', ip_address=tm_ip, debug=robot_debug)
    else:
        raise ValueError("robot_type must be 'FS100' or 'TM'")

    if camera_source == 'ids':
        from utils import IDSCameraCapture
        cap = IDSCameraCapture(0)
        print('Using IDS camera capture.')
    elif camera_source == 'ros2_topic':
        cap = ROS2TopicCameraCapture(topic_name=ros_image_topic)
        print(f'Using ROS2 image topic: {ros_image_topic}')
    else:
        raise ValueError("camera_source must be 'ids' or 'ros2_topic'")

    sleep(1)

    if robot_type == 'FS100':
        robot.switch_power(power_type='servo', switch_status='on')

    last_good_raw_pose = warmup_position_read(
        robot,
        robot_type,
        retries=tm_position_retries,
        retry_delay=tm_position_retry_delay,
    )
    if last_good_raw_pose is None:
        print('WARNING: could not read an initial robot position during warmup.')
        last_good_raw_pose = [0, 0, 0, 0, 0, 0]
    else:
        print('Initial robot position:', raw_pose_to_mmdeg(last_good_raw_pose))

    print('\n' + '=' * 60)
    print('CONTROLS:')
    print('  SPACE - Start auto data collection')
    print('  X     - Pause/resume data collection')
    print('  N     - Skip current pose')
    print('  R     - Manually capture at current pose')
    print('  Q/ESC - Quit')
    print('=' * 60 + '\n')

    i = 0
    start_time = None
    stuck_count = 0
    consecutive_read_failures = 0
    collecting_data = False
    moving = False
    running = True
    exit_gracefully = False
    target_position_mmdeg = pose_to_mmdeg(robot_poses[i])
    position_raw = copy.deepcopy(last_good_raw_pose)
    
    # Track which poses have been sampled
    sampled_poses = set()
    current_pose_sampled = False
    move_failed_count = 0

    try:
        while running:
            ret, img = cap.read()
            if not ret:
                print('Camera disconnected')
                raise RuntimeError('Camera disconnected')

            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            markers_corners, markers_ids, _ = detect_markers(gray)
            charuco_found = False
            charuco_corners, charuco_ids = None, None
            if markers_ids is not None and len(markers_ids) > 0:
                retval, charuco_corners, charuco_ids, _, _ = detect_charuco(gray, markers_corners, markers_ids)
                if retval is not None and retval >= 4 and charuco_ids is not None and len(charuco_ids) >= 4:
                    charuco_found = True

            rendered_rgb = img.copy()
            if charuco_found:
                try:
                    cv2.aruco.drawDetectedMarkers(rendered_rgb, markers_corners, markers_ids)
                    cv2.aruco.drawDetectedCornersCharuco(rendered_rgb, charuco_corners, charuco_ids)
                except Exception:
                    pass

            prev_raw = copy.deepcopy(position_raw)
            status, raw_position = robot.get_position(
                times_1000=True,
                retries=tm_position_retries,
                retry_delay=tm_position_retry_delay,
            )
            current_mmdeg = None
            read_success = (status == 0 and raw_position is not None)
            
            # Throttle the loop slightly to reduce CPU and network usage
            # 20Hz status updates are plenty for calibration
            sleep(0.05)

            if read_success:
                try:
                    position_raw = extract_raw_pose(robot_type, raw_position)
                    last_good_raw_pose = copy.deepcopy(position_raw)
                    current_mmdeg = raw_pose_to_mmdeg(position_raw)
                    consecutive_read_failures = 0
                except Exception as e:
                    print(f'Position parse error: {e}')
                    position_raw = copy.deepcopy(last_good_raw_pose)
                    current_mmdeg = raw_pose_to_mmdeg(position_raw)
            else:
                consecutive_read_failures += 1
                position_raw = copy.deepcopy(last_good_raw_pose)
                current_mmdeg = raw_pose_to_mmdeg(position_raw)
                if collecting_data and (consecutive_read_failures <= 5 or consecutive_read_failures % 20 == 0):
                    print(f"can't read position, status={status}, raw_position={raw_position}")
                sleep(0.05)

            if collecting_data:
                # Check if we've completed all poses
                if i >= len(robot_poses):
                    print('\n' + '=' * 60)
                    print('All poses completed! Data collection complete!')
                    print('=' * 60 + '\n')
                    running = False
                    break

                # Recovery: if move failed repeatedly, try to re-send or move to next
                if move_failed_count > 0 and not moving and collecting_data:
                    # Wait longer before retry to avoid Error 1
                    sleep(1.0) 

                # Check if we're at target position
                if is_at_target(current_mmdeg, target_position_mmdeg, pos_tol=pos_tol_mm, ang_tol=ang_tol_deg):
                    # Take sample if we haven't sampled this pose yet
                    if not current_pose_sampled and i not in sampled_poses:
                        print(f'\n✓ At target position for pose {i + 1}/{len(robot_poses)}')
                        print(f'  Target: X={target_position_mmdeg[0]:.1f}, Y={target_position_mmdeg[1]:.1f}, Z={target_position_mmdeg[2]:.1f} mm')
                        print(f'  Current: X={current_mmdeg[0]:.1f}, Y={current_mmdeg[1]:.1f}, Z={current_mmdeg[2]:.1f} mm')
                        
                        # Give camera time to stabilize
                        sleep(0.5)
                        
                        # Take ONE sample
                        ok, saved_raw = capture_charuco_sample(
                            cap=cap,
                            aruco_dict=ARUCO_DICT,
                            aruco_params=aruco_params,
                            charuco_board=CHARUCO_BOARD,
                            save_root=save_root,
                            sample_idx=len(hand_eye_calibration_robot_poses),
                            raw_position=position_raw,
                            max_attempts=20,
                        )
                        
                        if ok:
                            hand_eye_calibration_robot_poses.append(saved_raw)
                            sampled_poses.add(i)
                            print(f'  ✓ Sample {len(hand_eye_calibration_robot_poses)} saved for pose {i + 1}')
                        else:
                            print(f'  ✗ Failed to capture valid Charuco board at pose {i + 1}')
                            sampled_poses.add(i)  # Mark as attempted
                            
                        current_pose_sampled = True
                        sleep(0.3)
                    else:
                        print(f'  Pose {i + 1} already processed, moving to next...')
                    
                    # ALWAYS move to next pose after arrival (whether capture succeeded or not)
                    i += 1
                    current_pose_sampled = False
                    moving = False
                    stuck_count = 0
                    move_failed_count = 0
                    
                    if i < len(robot_poses):
                        target_position_mmdeg = pose_to_mmdeg(robot_poses[i])
                        print(f'\n--- Next pose: {i + 1}/{len(robot_poses)} ---')
                    sleep(0.2)
                    continue

                # Not at target yet - handle movement
                if not moving:
                    print(f'\n--- Moving to pose {i + 1}/{len(robot_poses)} ---')
                    print(f'  Target: X={target_position_mmdeg[0]:.1f}, Y={target_position_mmdeg[1]:.1f}, Z={target_position_mmdeg[2]:.1f} mm')
                    print(f'          RX={target_position_mmdeg[3]:.1f}, RY={target_position_mmdeg[4]:.1f}, RZ={target_position_mmdeg[5]:.1f} deg')

                    if robot_type == 'FS100':
                        ret_move = robot.move(
                            target_position=target_position_mmdeg,
                            tool_no=tool_no,
                            speed=robot_move_speed_yaskawa,
                        )
                    else:
                        ret_move = robot.move(
                            target_position=target_position_mmdeg,
                            speed_perc=robot_move_speed_tm,
                            acceleration_duration=tm_acceleration_duration,
                            divide_1000=False,
                            use_precise_positioning=True,  # Precision is better for calibration
                        )

                    # Handle move errors
                    if ret_move == 0:
                        moving = True
                        print('  Moving...')
                        move_failed_count = 0
                        # Grace period: let the robot start moving before checking for "stuck"
                        sleep(0.5)
                    else:
                        print(f'  ✗ Move command rejected (Error {ret_move}). Retrying...')
                        move_failed_count += 1
                        
                        # Skip this pose after too many failures
                        if move_failed_count >= 3:
                            print(f'  Skipping pose {i + 1} due to repeated movement errors')
                            sampled_poses.add(i)  # Mark as skipped
                            i += 1
                            current_pose_sampled = False
                            moving = False
                            move_failed_count = 0
                            
                            if i < len(robot_poses):
                                target_position_mmdeg = pose_to_mmdeg(robot_poses[i])
                                print(f'  Moving to next pose: {i + 1}/{len(robot_poses)}')
                            continue
                        
                        sleep(0.5)
                    stuck_count = 0

                else:
                    # Currently moving - check if arrived
                    if is_at_target(current_mmdeg, target_position_mmdeg, pos_tol=pos_tol_mm, ang_tol=ang_tol_deg):
                        print(f'  ✓ Arrived at target pose {i + 1}!')
                        moving = False
                        stuck_count = 0
                    elif prev_raw == position_raw:
                        stuck_count += 1
                        # 150 * 0.05s = 7.5 seconds. Enough time for TM to start moving.
                        if stuck_count > 150:
                            print(f'  ⚠️ Robot motion timeout at pose {i + 1}. Skipping...')
                            moving = False
                            sampled_poses.add(i)  # Mark as attempted
                            i += 1
                            if i < len(robot_poses):
                                target_position_mmdeg = pose_to_mmdeg(robot_poses[i])
                            stuck_count = 0
                            current_pose_sampled = False
                        else:
                            sleep(0.1)

            # Display camera feed
            cv2.imshow('Camera Frame', img)
            cv2.imshow('Rendered RGB', rendered_rgb)
            
            # Show progress on window title
            progress_text = f"Samples: {len(hand_eye_calibration_robot_poses)}/{len(robot_poses)}"
            cv2.setWindowTitle('Camera Frame', progress_text)

            k = cv2.waitKey(1)
            if k == ord(' '):
                if not collecting_data:
                    print('\n' + '=' * 60)
                    print('STARTING AUTO DATA COLLECTION')
                    print(f'Total poses to collect: {len(robot_poses)}')
                    print('=' * 60 + '\n')
                    collecting_data = True
                    current_pose_sampled = False
                    # Debounce to prevent multiple triggers
                    sleep(0.5)
                    if start_time is None:
                        start_time = time()
            elif k == ord('x') or k == ord('X'):
                collecting_data = not collecting_data
                state = 'RESUMED' if collecting_data else 'PAUSED'
                print(f'\n--- Data collection {state} ---')
                if collecting_data and start_time is None:
                    start_time = time()
            elif k == ord('n') or k == ord('N'):
                if i < len(robot_poses):
                    print(f'\n--- Skipping pose {i + 1} manually ---')
                    moving = False
                    sampled_poses.add(i)  # Mark as skipped
                    i += 1
                    if i < len(robot_poses):
                        target_position_mmdeg = pose_to_mmdeg(robot_poses[i])
                    stuck_count = 0
                    current_pose_sampled = False
                    move_failed_count = 0
            elif k == ord('r') or k == ord('R'):
                # Manual capture at current pose
                print('\n--- Manual capture triggered ---')
                ok, saved_raw = capture_charuco_sample(
                    cap=cap,
                    aruco_dict=ARUCO_DICT,
                    aruco_params=aruco_params,
                    charuco_board=CHARUCO_BOARD,
                    save_root=save_root,
                    sample_idx=len(hand_eye_calibration_robot_poses),
                    raw_position=position_raw,
                    max_attempts=20,
                )
                if ok:
                    hand_eye_calibration_robot_poses.append(saved_raw)
                    print(f'  ✓ Manual sample {len(hand_eye_calibration_robot_poses)} saved')
                else:
                    print('  ✗ Manual capture failed - Charuco board not detected')
            elif k == ord('s') or k == ord('S'):
                if robot_type == 'FS100':
                    robot.switch_power(power_type='servo', switch_status='on')
                    print('Servo ON')
            elif k == ord('q') or k == ord('Q') or k == 27:
                exit_gracefully = True
                break

    finally:
        cap.release()
        cv2.destroyAllWindows()

    if not synthetic and start_time is not None:
        elapsed = time() - start_time
        print(f'\nElapsed time: {elapsed:.1f} seconds ({elapsed/60:.1f} minutes)')

    if hand_eye_calibration and len(hand_eye_calibration_robot_poses) > 0:
        save_pose_path = os.path.join(save_root, 'robot_pose_camera_cal.npy')
        with open(save_pose_path, 'wb+') as f:
            np.save(f, np.array(hand_eye_calibration_robot_poses))
        print(f'\nSaved {len(hand_eye_calibration_robot_poses)} robot poses for hand-eye calibration')
        print(f'Saved robot pose array to: {save_pose_path}')
        print(f'Success rate: {len(hand_eye_calibration_robot_poses)}/{len(robot_poses)} poses captured')
    elif hand_eye_calibration:
        print('\nWARNING: No poses were captured successfully!')

    if exit_gracefully:
        print('Exited by user.')

    print('\nDONE')