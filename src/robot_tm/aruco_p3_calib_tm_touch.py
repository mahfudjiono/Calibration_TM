#!/usr/bin/env python3

import cv2
import numpy as np
import json
import time
import logging
import importlib
from time import sleep
from threading import Event, Lock, Thread

try:
    import serial
    import serial.tools.list_ports
except Exception:
    serial = None

from robot_command import Robot

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)
logger = logging.getLogger(__name__)

# ------------------------------------------------------------------------------
# ROS2 camera helper
# ------------------------------------------------------------------------------
def _load_ros2_modules():
    try:
        rclpy = importlib.import_module('rclpy')
        NodeBase = importlib.import_module('rclpy.node').Node
        ImageMsg = importlib.import_module('sensor_msgs.msg').Image
        CvBridgeClass = importlib.import_module('cv_bridge').CvBridge
        return rclpy, NodeBase, ImageMsg, CvBridgeClass
    except Exception as exc:
        raise ImportError(
            "ROS2 image capture requires rclpy, sensor_msgs, and cv_bridge. "
            "Source your ROS2 environment first."
        ) from exc


class ROS2TopicCameraCapture:
    def __init__(self, topic_name='/techman_image'):
        self.topic_name = topic_name
        self._rclpy, NodeBase, ImageMsg, CvBridgeClass = _load_ros2_modules()
        self._bridge = CvBridgeClass()
        self._latest_frame = None
        self._latest_stamp = None
        self._frame_event = Event()
        self._lock = Lock()

        if not self._rclpy.ok():
            self._rclpy.init(args=None)

        class _ImageNode(NodeBase):
            def __init__(inner_self, outer):
                super().__init__('aruco_p3_calib_tm_touch_camera')
                inner_self.outer = outer
                inner_self.create_subscription(ImageMsg, topic_name, inner_self._callback, 10)

            def _callback(inner_self, msg):
                try:
                    frame = inner_self.outer._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                    sec = getattr(msg.header.stamp, 'sec', 0)
                    nanosec = getattr(msg.header.stamp, 'nanosec', 0)
                    stamp = (sec, nanosec)
                    with inner_self.outer._lock:
                        inner_self.outer._latest_frame = frame.copy()
                        inner_self.outer._latest_stamp = stamp
                    inner_self.outer._frame_event.set()
                except Exception:
                    pass

        self._node = _ImageNode(self)
        self._spin_thread = Thread(target=self._spin, daemon=True)
        self._spin_thread.start()

    def _spin(self):
        try:
            while self._rclpy.ok():
                self._rclpy.spin_once(self._node, timeout_sec=0.1)
        except Exception:
            pass

    def wait_for_first_frame(self, timeout_sec=5.0):
        return self._frame_event.wait(timeout=timeout_sec)

    def read(self):
        with self._lock:
            if self._latest_frame is None:
                return False, None
            return True, self._latest_frame.copy()

    def release(self):
        try:
            if hasattr(self, '_node') and self._node is not None:
                self._node.destroy_node()
        except Exception:
            pass


# ------------------------------------------------------------------------------
# ESP32 touch sensor
# ------------------------------------------------------------------------------
class ESP32TouchSensor:
    def __init__(self, baudrate=115200, timeout=0.02, touch_min=1810, touch_max=1870):
        self.baudrate = baudrate
        self.timeout = timeout
        self.touch_min = touch_min
        self.touch_max = touch_max
        self.ser = None
        self.port = None

    @staticmethod
    def find_esp32():
        if serial is None:
            return None
        ports = serial.tools.list_ports.comports()
        for port in ports:
            desc = (port.description or '').upper()
            if 'USB' in desc or 'CP210' in desc or 'CH340' in desc:
                return port.device
        return None

    def connect(self):
        if serial is None:
            raise RuntimeError('pyserial is not installed')
        self.port = self.find_esp32()
        if self.port is None:
            raise RuntimeError('ESP32 not found')
        self.ser = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
        self.ser.setDTR(False)
        self.ser.setRTS(False)
        time.sleep(2.0)
        self.ser.reset_input_buffer()
        logger.info(f'Connected to ESP32 on {self.port}')

    def close(self):
        if self.ser is not None and self.ser.is_open:
            self.ser.close()

    def flush(self):
        if self.ser is not None:
            self.ser.reset_input_buffer()

    def is_touch(self, adc_value):
        return self.touch_min <= adc_value <= self.touch_max

    def read_once(self):
        if self.ser is None or not self.ser.is_open or not self.ser.in_waiting:
            return None, None
        try:
            line = self.ser.readline().decode('utf-8', errors='ignore').strip()
            if not line:
                return None, None
            if ',' in line:
                adc_str, _touch_str = line.split(',', 1)
                adc_value = int(adc_str.strip())
                touch = 1 if self.is_touch(adc_value) else 0
                return adc_value, touch
            adc_value = int(line)
            touch = 1 if self.is_touch(adc_value) else 0
            return adc_value, touch
        except Exception:
            return None, None

    def wait_for_touch(self, timeout=0.08, stable_count=2):
        touched = 0
        last_adc = None
        last_touch = None
        t0 = time.time()
        while time.time() - t0 < timeout:
            adc_value, touch = self.read_once()
            if adc_value is not None:
                last_adc = adc_value
            if touch is not None:
                last_touch = touch
                if touch == 1:
                    touched += 1
                    if touched >= stable_count:
                        return True, last_adc, last_touch
                else:
                    touched = 0
            time.sleep(0.005)
        return False, last_adc, last_touch


# ------------------------------------------------------------------------------
# Helper functions
# ------------------------------------------------------------------------------
def euler_to_rotation_matrix_zyx(rx, ry, rz):
    rx_m = np.array([
        [1, 0, 0],
        [0, np.cos(rx), -np.sin(rx)],
        [0, np.sin(rx), np.cos(rx)],
    ], dtype=np.float64)
    ry_m = np.array([
        [np.cos(ry), 0, np.sin(ry)],
        [0, 1, 0],
        [-np.sin(ry), 0, np.cos(ry)],
    ], dtype=np.float64)
    rz_m = np.array([
        [np.cos(rz), -np.sin(rz), 0],
        [np.sin(rz), np.cos(rz), 0],
        [0, 0, 1],
    ], dtype=np.float64)
    return rz_m @ ry_m @ rx_m


def rotation_matrix_to_euler_zyx_deg(R):
    sy = np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    if sy < 1e-6:
        rx = np.arctan2(-R[1, 2], R[1, 1])
        ry = np.arctan2(-R[2, 0], sy)
        rz = 0.0
    else:
        rx = np.arctan2(R[2, 1], R[2, 2])
        ry = np.arctan2(-R[2, 0], sy)
        rz = np.arctan2(R[1, 0], R[0, 0])
    return [np.rad2deg(rx), np.rad2deg(ry), np.rad2deg(rz)]


def robot_pose_mmdeg_to_transformation(pose_mmdeg):
    x = pose_mmdeg[0] / 1000.0
    y = pose_mmdeg[1] / 1000.0
    z = pose_mmdeg[2] / 1000.0
    rx = np.deg2rad(pose_mmdeg[3])
    ry = np.deg2rad(pose_mmdeg[4])
    rz = np.deg2rad(pose_mmdeg[5])
    T = np.eye(4)
    T[:3, :3] = euler_to_rotation_matrix_zyx(rx, ry, rz)
    T[:3, 3] = [x, y, z]
    return T


def make_transform(R=None, t=None):
    T = np.eye(4)
    if R is not None:
        T[:3, :3] = R
    if t is not None:
        T[:3, 3] = np.asarray(t, dtype=np.float64).reshape(3)
    return T


def transform_to_pose_mmdeg(T):
    pos_mm = T[:3, 3] * 1000.0
    rx_deg, ry_deg, rz_deg = rotation_matrix_to_euler_zyx_deg(T[:3, :3])
    return [pos_mm[0], pos_mm[1], pos_mm[2], rx_deg, ry_deg, rz_deg]


def build_local_offset_transform(offset_xyz_m, offset_rpy_deg):
    rx, ry, rz = np.deg2rad(offset_rpy_deg)
    R = euler_to_rotation_matrix_zyx(rx, ry, rz)
    return make_transform(R, offset_xyz_m)


def draw_axis(img, camera_matrix, dist_coeffs, rvec, tvec, length):
    axis_points = np.float32([
        [length, 0, 0],
        [0, length, 0],
        [0, 0, length],
        [0, 0, 0],
    ]).reshape(-1, 3)
    img_points, _ = cv2.projectPoints(axis_points, rvec, tvec, camera_matrix, dist_coeffs)
    img_points = img_points.reshape(-1, 2)
    origin = tuple(img_points[3].astype(int))
    cv2.line(img, origin, tuple(img_points[0].astype(int)), (0, 0, 255), 3)
    cv2.putText(img, 'X', tuple(img_points[0].astype(int)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
    cv2.line(img, origin, tuple(img_points[1].astype(int)), (0, 255, 0), 3)
    cv2.putText(img, 'Y', tuple(img_points[1].astype(int)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    cv2.line(img, origin, tuple(img_points[2].astype(int)), (255, 0, 0), 3)
    cv2.putText(img, 'Z', tuple(img_points[2].astype(int)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)


def detect_markers_aruco(gray, dictionary, params):
    if hasattr(cv2.aruco, 'ArucoDetector'):
        detector = cv2.aruco.ArucoDetector(dictionary, params)
        return detector.detectMarkers(gray)
    return cv2.aruco.detectMarkers(gray, dictionary, parameters=params)


def estimate_single_marker_pose(marker_corners, marker_size, camera_matrix, dist_coeffs):
    if hasattr(cv2.aruco, 'estimatePoseSingleMarkers'):
        rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
            [marker_corners], marker_size, camera_matrix, dist_coeffs
        )
        return rvec[0], tvec[0]

    half = marker_size / 2.0
    obj_points = np.array([
        [-half,  half, 0.0],
        [ half,  half, 0.0],
        [ half, -half, 0.0],
        [-half, -half, 0.0],
    ], dtype=np.float32)

    img_points = np.asarray(marker_corners, dtype=np.float32).reshape(4, 2)

    ok, rvec, tvec = cv2.solvePnP(
        obj_points,
        img_points,
        camera_matrix,
        dist_coeffs,
        flags=cv2.SOLVEPNP_IPPE_SQUARE if hasattr(cv2, 'SOLVEPNP_IPPE_SQUARE') else cv2.SOLVEPNP_ITERATIVE,
    )
    if not ok:
        raise RuntimeError('solvePnP failed for marker pose estimation')

    return rvec.reshape(1, 3), tvec.reshape(1, 3)


# ------------------------------------------------------------------------------
# 3-point calibration helpers
# ------------------------------------------------------------------------------
def three_point_calibration(robot_points, object_points):
    robot_positions = np.array([p[:3] for p in robot_points], dtype=np.float64)
    object_positions = np.array([p[:3] for p in object_points], dtype=np.float64)

    origin_obj = object_positions[0]
    x_axis_obj = object_positions[1] - object_positions[0]
    x_axis_obj = x_axis_obj / np.linalg.norm(x_axis_obj)
    xy_plane_obj = object_positions[2] - object_positions[0]
    y_axis_obj = xy_plane_obj - np.dot(xy_plane_obj, x_axis_obj) * x_axis_obj
    y_axis_obj = y_axis_obj / np.linalg.norm(y_axis_obj)
    z_axis_obj = np.cross(x_axis_obj, y_axis_obj)
    z_axis_obj = z_axis_obj / np.linalg.norm(z_axis_obj)
    y_axis_obj = np.cross(z_axis_obj, x_axis_obj)
    y_axis_obj = y_axis_obj / np.linalg.norm(y_axis_obj)
    R_obj = np.column_stack([x_axis_obj, y_axis_obj, z_axis_obj])

    origin_robot = robot_positions[0]
    x_axis_robot = robot_positions[1] - robot_positions[0]
    x_axis_robot = x_axis_robot / np.linalg.norm(x_axis_robot)
    xy_plane_robot = robot_positions[2] - robot_positions[0]
    y_axis_robot = xy_plane_robot - np.dot(xy_plane_robot, x_axis_robot) * x_axis_robot
    y_axis_robot = y_axis_robot / np.linalg.norm(y_axis_robot)
    z_axis_robot = np.cross(x_axis_robot, y_axis_robot)
    z_axis_robot = z_axis_robot / np.linalg.norm(z_axis_robot)
    y_axis_robot = np.cross(z_axis_robot, x_axis_robot)
    y_axis_robot = y_axis_robot / np.linalg.norm(y_axis_robot)
    R_robot = np.column_stack([x_axis_robot, y_axis_robot, z_axis_robot])

    R_object_in_robot = R_robot @ R_obj.T
    t_object_in_robot = origin_robot - R_object_in_robot @ origin_obj

    T = np.eye(4)
    T[:3, :3] = R_object_in_robot
    T[:3, 3] = t_object_in_robot
    return T, R_object_in_robot, t_object_in_robot


def transform_point(point_object, transformation_matrix):
    if len(point_object) == 6:
        pos_obj = np.array(point_object[:3], dtype=np.float64)
        pos_h = np.append(pos_obj, 1.0)
        pos_robot = (transformation_matrix @ pos_h)[:3]

        R_obj = euler_to_rotation_matrix_zyx(
            np.deg2rad(point_object[3]),
            np.deg2rad(point_object[4]),
            np.deg2rad(point_object[5]),
        )
        R_robot = transformation_matrix[:3, :3] @ R_obj
        ori_robot = rotation_matrix_to_euler_zyx_deg(R_robot)
        return np.array([pos_robot[0], pos_robot[1], pos_robot[2], ori_robot[0], ori_robot[1], ori_robot[2]])

    pos_obj = np.array(point_object[:3], dtype=np.float64)
    pos_h = np.append(pos_obj, 1.0)
    return (transformation_matrix @ pos_h)[:3]


def calculate_calibration_error(robot_points, object_points, transformation_matrix):
    errors = []
    lines = []
    for i, (robot_pt, object_pt) in enumerate(zip(robot_points, object_points)):
        transformed = transform_point(object_pt[:3], transformation_matrix)
        error = np.linalg.norm(transformed - np.array(robot_pt[:3], dtype=np.float64))
        errors.append(float(error))
        lines.append(
            f"Point {i+1} error: {error:.3f} mm | "
            f"Robot=({robot_pt[0]:.3f}, {robot_pt[1]:.3f}, {robot_pt[2]:.3f}) | "
            f"Transformed=({transformed[0]:.3f}, {transformed[1]:.3f}, {transformed[2]:.3f})"
        )
    return errors, lines


# ------------------------------------------------------------------------------
# Configuration (TM + touch)
# ------------------------------------------------------------------------------
ROBOT_TYPE = 'TM'
ROBOT_IP = '192.168.10.3'

CAMERA_SOURCE = 'ros2_topic'
ROS_IMAGE_TOPIC = '/techman_image'

INITIAL_POS = [297.0, -17.68, 355.0, 177.76, 0.28, 88.68]
TM_SPEED_PERC = 1.0
TM_ACCELERATION_DURATION = 10
TM_USE_PRECISE_POSITIONING = True

USE_SAFE_Z = True
SAFE_Z = 23.0  # keep same behavior as your current working TM file
MOVE_TO_CONTACT_POINTS = False

USE_XY_OFFSET = True
XY_OFFSET_X = 0.0
XY_OFFSET_Y = 0.0

INVERT_X_DIRECTION = False
INVERT_Y_DIRECTION = False
INVERT_Z_DIRECTION = False

FOLLOW_MARKER_ROTATION = True
MARKER_TO_TOOL_RPY_DEG = [-180.0, 0.0, 90.0]
MARKER_TO_TOOL_OFFSET_MM = [0.0, 0.0, 0.0]

# Touch acquisition settings
TOUCH_ACQUISITION_ENABLED = True
TOUCH_MIN = 1810
TOUCH_MAX = 1870
TOUCH_APPROACH_OFFSET_MM = 30.0
TOUCH_DESCENT_STEP_MM = 0.2
TOUCH_SEARCH_EXTRA_MM = 5.0
TOUCH_SETTLE_TIME = 0.05
TOUCH_STABLE_COUNT = 2

# Keep these at the same TM movement scale you already use successfully.
TOUCH_APPROACH_SPEED_PERC = TM_SPEED_PERC
TOUCH_DESCENT_SPEED_PERC = TM_SPEED_PERC
TOUCH_RETURN_SPEED_PERC = TM_SPEED_PERC

CALIBRATION_POINTS_CONFIG = [
    {
        'name': 'P1',
        'offset_xyz': [-0.060, 0.007, 0.020],
        'offset_rpy': [0.0, 0.0, 0.0],
        'description': 'Calibration contact point P1 (board origin)',
    },
    {
        'name': 'P2',
        'offset_xyz': [-0.060, -0.068, 0.020],
        'offset_rpy': [0.0, 0.0, 0.0],
        'description': 'Calibration contact point P2',
    },
    {
        'name': 'P3',
        'offset_xyz': [0.002, -0.069, 0.020],
        'offset_rpy': [0.0, 0.0, 0.0],
        'description': 'Calibration contact point P3',
    },
]

POINT_FINE_ADJUST_MM = {
    'P1': [0.0, 0.0, 0.0],
    'P2': [0.0, 0.0, 0.0],
    'P3': [0.0, 0.0, 0.0],
}
APPLY_POINT_FINE_ADJUST = True
APPLY_Z_TRIM_TO_APPROACH = False

CUBE_CORNERS_BOARD = [
    [128.23, -30.10, 47.233],
    [128.35, -80.09, 47.233],
    [178.80, -79.60, 47.233],
    [178.11, -29.59, 47.233],
]

EXPECTED_CORNERS_ROBOT = [
    [694.372, -53.389, -301.273],
    [692.282, -103.337, -301.273],
    [742.699, -105.084, -301.273],
    [744.227, -55.090, -301.273],
]

CUBE_APPROACH_OFFSET_MM = 20.0
DETECTION_RETRY_ATTEMPTS = 5
POSITION_VERIFICATION_TOLERANCE_MM = 5.0
ROBOT_ARRIVAL_TOLERANCE_MM = 2.0
ROBOT_ARRIVAL_TOLERANCE_DEG = 2.0
ROBOT_ARRIVAL_TIMEOUT = 30

ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
try:
    aruco_params = cv2.aruco.DetectorParameters_create()
except AttributeError:
    aruco_params = cv2.aruco.DetectorParameters()
aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
aruco_params.adaptiveThreshWinSizeMin = 3
aruco_params.adaptiveThreshWinSizeMax = 23
aruco_params.adaptiveThreshWinSizeStep = 10
aruco_params.minMarkerPerimeterRate = 0.02

TARGET_MARKER_ID = 36
MARKER_SIZE = 0.060

TRANSFORM_PATH = '/home/hucenrotia/mahfud/Calibration_TM/src/robot_tm/transformation_matrices_TM5.json'
CAMERA_INTRINSIC_PATH = '/home/hucenrotia/mahfud/Calibration_TM/src/robot_tm/camera_intrinsic_TM5.json'

with open(TRANSFORM_PATH, 'r') as f:
    transform_data = json.load(f)
T_cam_from_tool = np.array(transform_data['T1'], dtype=np.float64).reshape(4, 4)
T_tool_from_cam = np.linalg.inv(T_cam_from_tool)

with open(CAMERA_INTRINSIC_PATH, 'r') as f:
    cam_data = json.load(f)
camera_matrix = np.array(cam_data['camera_K'], dtype=np.float64).reshape(3, 3)
dist_coeffs = np.array(cam_data['dist_coef'], dtype=np.float64).reshape(-1, 1)

logger.info('=' * 60)
logger.info('LOADED HAND-EYE CALIBRATION RESULTS (TM + TOUCH)')
logger.info('=' * 60)
logger.info(f'T_cam_from_tool (loaded from T1):\n{T_cam_from_tool}')
logger.info(f'T_tool_from_cam (inverse used at runtime):\n{T_tool_from_cam}')
logger.info(f'\nCamera matrix:\n{camera_matrix}')
logger.info(f'Distortion coefficients: {dist_coeffs.ravel()}')
logger.info(f'Initial pose before all processes: {INITIAL_POS}')
logger.info(f'Marker->tool fixed RPY offset [deg]: {MARKER_TO_TOOL_RPY_DEG}')
logger.info(f'Marker->tool fixed XYZ offset [mm]: {MARKER_TO_TOOL_OFFSET_MM}')
logger.info(f'SAFE_Z for approach poses only: {SAFE_Z} mm')
logger.info(f'MOVE_TO_CONTACT_POINTS: {MOVE_TO_CONTACT_POINTS}')
logger.info(f'POINT_FINE_ADJUST_MM: {POINT_FINE_ADJUST_MM}')
logger.info(f'APPLY_Z_TRIM_TO_APPROACH: {APPLY_Z_TRIM_TO_APPROACH}')
logger.info(f'TOUCH_ACQUISITION_ENABLED: {TOUCH_ACQUISITION_ENABLED}')
logger.info(f'TOUCH_APPROACH_OFFSET_MM: {TOUCH_APPROACH_OFFSET_MM}')
logger.info(f'TOUCH_DESCENT_STEP_MM: {TOUCH_DESCENT_STEP_MM}')
logger.info(f'TOUCH_RANGE: [{TOUCH_MIN}, {TOUCH_MAX}]')


def build_marker_to_tool_desired_transform():
    rx, ry, rz = np.deg2rad(MARKER_TO_TOOL_RPY_DEG)
    R_marker_from_tool_des = euler_to_rotation_matrix_zyx(rx, ry, rz)
    t_marker_from_tool_des = np.array(MARKER_TO_TOOL_OFFSET_MM, dtype=np.float64) / 1000.0
    return make_transform(R_marker_from_tool_des, t_marker_from_tool_des)


T_marker_from_tool_des = build_marker_to_tool_desired_transform()


# ------------------------------------------------------------------------------
# Marker / board computation
# ------------------------------------------------------------------------------
def detect_aruco_marker(img, current_robot_pose=None):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = detect_markers_aruco(gray, ARUCO_DICT, aruco_params)
    if ids is None or TARGET_MARKER_ID not in ids.flatten():
        return False, None, None, None, None, None

    idx = np.where(ids.flatten() == TARGET_MARKER_ID)[0][0]
    marker_corners = corners[idx]

    rvec, tvec = estimate_single_marker_pose(
        marker_corners, MARKER_SIZE, camera_matrix, dist_coeffs
    )

    T_cam_from_marker = np.eye(4)
    R_cam_from_marker, _ = cv2.Rodrigues(rvec.reshape(3, 1))
    T_cam_from_marker[:3, :3] = R_cam_from_marker
    T_cam_from_marker[:3, 3] = tvec.reshape(3)

    robot_pose_info = None
    if current_robot_pose is not None and isinstance(current_robot_pose, np.ndarray) and current_robot_pose.shape == (4, 4):
        robot_position = current_robot_pose[:3, 3]
        robot_orientation_deg = rotation_matrix_to_euler_zyx_deg(current_robot_pose[:3, :3])
        robot_pose_info = {
            'position': {'x': robot_position[0], 'y': robot_position[1], 'z': robot_position[2]},
            'orientation_deg': {'rx': robot_orientation_deg[0], 'ry': robot_orientation_deg[1], 'rz': robot_orientation_deg[2]},
        }

    return True, T_cam_from_marker, rvec[0], tvec[0], [marker_corners], robot_pose_info


def compute_marker_pose_in_base(T_cam_from_marker, current_robot_pose_mmdeg):
    T_base_from_tool = robot_pose_mmdeg_to_transformation(current_robot_pose_mmdeg)
    return T_base_from_tool @ T_tool_from_cam @ T_cam_from_marker


def compute_center_tool_pose_from_marker(T_base_from_marker):
    return T_base_from_marker @ T_marker_from_tool_des


def _apply_xy_and_invert_to_pose(pose_mmdeg, apply_invert=True):
    pose = list(pose_mmdeg)
    if USE_XY_OFFSET:
        pose[0] += XY_OFFSET_X
        pose[1] += XY_OFFSET_Y
    if apply_invert and INVERT_X_DIRECTION:
        pose[0] = -pose[0]
    if apply_invert and INVERT_Y_DIRECTION:
        pose[1] = -pose[1]
    return pose


def build_approach_pose_from_contact(contact_pose_mmdeg):
    approach = list(contact_pose_mmdeg)
    if USE_SAFE_Z:
        approach[2] = SAFE_Z
    else:
        approach[2] = contact_pose_mmdeg[2] + 20.0
    return approach


def build_touch_approach_pose_from_contact(contact_pose_mmdeg):
    approach = list(contact_pose_mmdeg)
    approach[2] = contact_pose_mmdeg[2] + TOUCH_APPROACH_OFFSET_MM
    return approach


def apply_point_fine_adjustment(point_name, pose_mmdeg, apply_z=True):
    adjusted = list(pose_mmdeg)
    if not APPLY_POINT_FINE_ADJUST:
        return adjusted
    trim = POINT_FINE_ADJUST_MM.get(point_name, [0.0, 0.0, 0.0])
    adjusted[0] += float(trim[0])
    adjusted[1] += float(trim[1])
    if apply_z:
        adjusted[2] += float(trim[2])
    return adjusted


def compute_calibration_point_targets(T_base_from_tool_center):
    point_targets = []
    center_pose_contact_raw = transform_to_pose_mmdeg(T_base_from_tool_center)
    shared_contact_z = center_pose_contact_raw[2]

    for cfg in CALIBRATION_POINTS_CONFIG:
        T_center_from_point = build_local_offset_transform(cfg['offset_xyz'], cfg['offset_rpy'])
        T_base_from_point_contact = T_base_from_tool_center @ T_center_from_point
        pose_contact_raw = transform_to_pose_mmdeg(T_base_from_point_contact)
        pose_contact_raw[2] = shared_contact_z

        pose_contact = _apply_xy_and_invert_to_pose(pose_contact_raw, apply_invert=False)
        pose_contact = apply_point_fine_adjustment(cfg['name'], pose_contact, apply_z=True)

        pose_contact_move = _apply_xy_and_invert_to_pose(pose_contact, apply_invert=True)
        pose_approach_move = build_approach_pose_from_contact(pose_contact_move)
        pose_touch_approach_move = build_touch_approach_pose_from_contact(pose_contact_move)

        if APPLY_Z_TRIM_TO_APPROACH:
            pose_approach_move = apply_point_fine_adjustment(cfg['name'], pose_approach_move, apply_z=True)
            pose_touch_approach_move = apply_point_fine_adjustment(cfg['name'], pose_touch_approach_move, apply_z=True)
        else:
            pose_approach_move = apply_point_fine_adjustment(cfg['name'], pose_approach_move, apply_z=False)
            pose_touch_approach_move = apply_point_fine_adjustment(cfg['name'], pose_touch_approach_move, apply_z=False)

        point_targets.append({
            'name': cfg['name'],
            'description': cfg['description'],
            'transform_contact': T_base_from_point_contact,
            'pose_contact_raw': pose_contact_raw,
            'pose_contact': pose_contact,
            'pose_contact_move': pose_contact_move,
            'pose_approach_move': pose_approach_move,
            'pose_touch_approach_move': pose_touch_approach_move,
            'offset_xyz_m': cfg['offset_xyz'],
            'offset_rpy_deg': cfg['offset_rpy'],
            'shared_contact_z_mm': shared_contact_z,
            'fine_adjust_mm': POINT_FINE_ADJUST_MM.get(cfg['name'], [0.0, 0.0, 0.0]),
        })
    return point_targets


def point_targets_to_dict(point_targets):
    return {item['name']: item for item in point_targets}


def _normalize_cube_corners_board_to_mm(corners):
    corners_mm = []
    for corner in corners:
        xyz = []
        for value in corner[:3]:
            value_f = float(value)
            xyz.append(value_f * 1000.0 if abs(value_f) < 1.0 else value_f)
        corners_mm.append(xyz)
    return corners_mm


def build_three_point_calibration(point_targets, touched_contact_points=None):
    point_map = point_targets_to_dict(point_targets)
    touched_contact_points = touched_contact_points or {}
    p1 = touched_contact_points.get('P1', point_map['P1']['pose_contact'])
    p2 = touched_contact_points.get('P2', point_map['P2']['pose_contact'])
    p3 = touched_contact_points.get('P3', point_map['P3']['pose_contact'])

    p1_to_p2 = np.array(p2[:3], dtype=np.float64) - np.array(p1[:3], dtype=np.float64)
    p1_to_p3 = np.array(p3[:3], dtype=np.float64) - np.array(p1[:3], dtype=np.float64)
    p1_p2_distance = float(np.linalg.norm(p1_to_p2))
    p1_p3_distance = float(np.linalg.norm(p1_to_p3))

    robot_calibration_points = [p1, p2, p3]

    object_calibration_points = [
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, p1_p2_distance, 0.0, 0.0, 0.0, 0.0],
        [p1_p3_distance, 0.0, 0.0, 0.0, 0.0, 0.0],
    ]

    logger.info('Using MANUAL-STYLE object calibration points:')
    logger.info('  Obj P1: X=0.000, Y=0.000, Z=0.000 mm')
    logger.info(f'  Obj P2: X=0.000, Y={p1_p2_distance:.3f}, Z=0.000 mm')
    logger.info(f'  Obj P3: X={p1_p3_distance:.3f}, Y=0.000, Z=0.000 mm')

    T_board_to_robot, R_matrix, t_vector = three_point_calibration(robot_calibration_points, object_calibration_points)
    errors, error_lines = calculate_calibration_error(robot_calibration_points, object_calibration_points, T_board_to_robot)

    x_axis = (np.array(p3[:3]) - np.array(p1[:3])) / max(p1_p3_distance, 1e-9)
    y_axis = (np.array(p2[:3]) - np.array(p1[:3])) / max(p1_p2_distance, 1e-9)
    z_axis = np.cross(x_axis, y_axis)
    z_axis /= max(np.linalg.norm(z_axis), 1e-9)

    cube_corners_board_mm = _normalize_cube_corners_board_to_mm(CUBE_CORNERS_BOARD)

    cube_corners_robot = []
    cube_corner_errors = []
    cube_corners_robot_approach = []
    logger.info(
        f'Using exact P1 orientation for all cube corners: Rx={p1[3]:.3f}, Ry={p1[4]:.3f}, Rz={p1[5]:.3f}'
    )

    for i, corner in enumerate(cube_corners_board_mm):
        pos_robot = transform_point(corner[:3], T_board_to_robot)
        transformed_list = [
            float(pos_robot[0]),
            float(pos_robot[1]),
            float(pos_robot[2]),
            float(p1[3]),
            float(p1[4]),
            float(p1[5]),
        ]
        cube_corners_robot.append(transformed_list)

        approach = list(transformed_list)
        approach[2] = transformed_list[2] + CUBE_APPROACH_OFFSET_MM
        cube_corners_robot_approach.append(approach)

        if i < len(EXPECTED_CORNERS_ROBOT):
            expected = np.array(EXPECTED_CORNERS_ROBOT[i], dtype=np.float64)
            err = float(np.linalg.norm(np.array(transformed_list[:3]) - expected))
            cube_corner_errors.append(err)

    report = {
        'T_board_to_robot': T_board_to_robot,
        'R_board_to_robot': R_matrix,
        't_board_to_robot': t_vector,
        'robot_calibration_points': robot_calibration_points,
        'touched_contact_points': touched_contact_points,
        'object_calibration_points': object_calibration_points,
        'cube_corners_board_mm': cube_corners_board_mm,
        'p1_p2_distance': p1_p2_distance,
        'p1_p3_distance': p1_p3_distance,
        'point_errors_mm': errors,
        'point_error_lines': error_lines,
        'cube_corners_robot': cube_corners_robot,
        'cube_corners_robot_approach': cube_corners_robot_approach,
        'cube_corner_errors_mm': cube_corner_errors,
        'x_axis_robot': x_axis,
        'y_axis_robot': y_axis,
        'z_axis_robot': z_axis,
    }
    return report


# ------------------------------------------------------------------------------
# Robot movement helpers
# ------------------------------------------------------------------------------
def move_robot_to_pose(robot, target_pose_mmdeg, speed_perc=None, acceleration_duration=None):
    logger.info(
        f"Moving to: X={target_pose_mmdeg[0]:.2f}, Y={target_pose_mmdeg[1]:.2f}, Z={target_pose_mmdeg[2]:.2f} mm"
    )
    logger.info(
        f"  Orientation: Rx={target_pose_mmdeg[3]:.2f}, Ry={target_pose_mmdeg[4]:.2f}, Rz={target_pose_mmdeg[5]:.2f} deg"
    )

    if ROBOT_TYPE == 'TM':
        ret = robot.move(
            target_position=target_pose_mmdeg,
            speed_perc=TM_SPEED_PERC if speed_perc is None else speed_perc,
            acceleration_duration=TM_ACCELERATION_DURATION if acceleration_duration is None else acceleration_duration,
            use_precise_positioning=TM_USE_PRECISE_POSITIONING,
        )
        return ret == 0

    return False


def _get_tm_pose_mmdeg(robot, retries=1, retry_delay=0.05):
    status, position = robot.get_position(retries=retries, retry_delay=retry_delay)
    if status == 0 and position is not None:
        return status, [position['x'], position['y'], position['z'], position['rx'], position['ry'], position['rz']]
    return status, None


def wait_for_robot_arrival(robot, target_pose_mmdeg,
                           tolerance_mm=ROBOT_ARRIVAL_TOLERANCE_MM,
                           tolerance_deg=ROBOT_ARRIVAL_TOLERANCE_DEG,
                           max_wait=ROBOT_ARRIVAL_TIMEOUT):
    logger.info('Waiting for robot to arrive...')
    start_time = time.time()
    last_error_time = None
    while time.time() - start_time < max_wait:
        status, current_pose = _get_tm_pose_mmdeg(robot, retries=1, retry_delay=0.05)

        if status == 0 and current_pose is not None:
            pos_error = np.abs(np.array(current_pose[:3]) - np.array(target_pose_mmdeg[:3]))
            ang_error = np.abs(np.array(current_pose[3:6]) - np.array(target_pose_mmdeg[3:6]))
            if np.all(pos_error <= tolerance_mm) and np.all(ang_error <= tolerance_deg):
                logger.info(f'Arrived! Position error: {pos_error} mm, Angle error: {ang_error} deg')
                sleep(0.3)
                return True
            if last_error_time is None or time.time() - last_error_time > 5:
                logger.info(f'Still moving... Position error: {pos_error} mm, Angle error: {ang_error} deg')
                last_error_time = time.time()
        sleep(0.1)

    logger.warning(f'Timeout waiting for robot after {max_wait} seconds')
    return False


def verify_position(robot, expected_pose_mmdeg, tolerance_mm=POSITION_VERIFICATION_TOLERANCE_MM):
    status, current_pose = _get_tm_pose_mmdeg(robot, retries=3, retry_delay=0.1)
    if status != 0 or current_pose is None:
        return False

    pos_error = np.abs(np.array(current_pose[:3]) - np.array(expected_pose_mmdeg[:3]))
    ang_error = np.abs(np.array(current_pose[3:6]) - np.array(expected_pose_mmdeg[3:6]))
    if np.all(pos_error <= tolerance_mm):
        logger.info(f'✓ Position verified! Position error: {pos_error} mm, Angle error: {ang_error} deg')
        return True
    logger.warning(f'⚠ Position error: {pos_error} mm, Angle error: {ang_error} deg')
    return False


# ------------------------------------------------------------------------------
# Main class
# ------------------------------------------------------------------------------
class ArUcoBoardCalibrationGuide:
    def __init__(self):
        self.robot = None
        self.cap = None
        self.touch_sensor = None
        self.frame_count = 0
        self.selected_point_name = 'P1'
        self.touched_contact_points = {'P1': None, 'P2': None, 'P3': None}

    def initialize(self):
        logger.info('\nInitializing camera...')
        try:
            if CAMERA_SOURCE == 'ros2_topic':
                self.cap = ROS2TopicCameraCapture(ROS_IMAGE_TOPIC)
                if not self.cap.wait_for_first_frame(timeout_sec=5.0):
                    logger.error('No ROS2 image received from topic')
                    return False
                logger.info('ROS2 topic camera initialized successfully')
            else:
                raise ValueError(f'Unsupported CAMERA_SOURCE: {CAMERA_SOURCE}')
        except Exception as e:
            logger.error(f'Error initializing camera: {e}')
            return False

        logger.info('Initializing robot connection...')
        try:
            self.robot = Robot(robot_type=ROBOT_TYPE, ip_address=ROBOT_IP, debug=True)
            sleep(2)
            self.robot.switch_power(power_type='servo', switch_status='on')
            sleep(1)
            logger.info('Robot initialized successfully')
        except Exception as e:
            logger.error(f'Error initializing robot: {e}')
            return False

        logger.info('\nMoving to initial pose before all processes...')
        if not move_robot_to_pose(self.robot, INITIAL_POS):
            logger.error('Failed to start movement to initial pose')
            return False
        if not wait_for_robot_arrival(self.robot, INITIAL_POS):
            logger.error('Failed to reach initial pose')
            return False
        verify_position(self.robot, INITIAL_POS)
        logger.info('✓ Initial pose reached')

        logger.info('Initializing ESP32 touch sensor...')
        try:
            self.touch_sensor = ESP32TouchSensor(touch_min=TOUCH_MIN, touch_max=TOUCH_MAX)
            self.touch_sensor.connect()
            sleep(0.5)
            logger.info('ESP32 touch sensor initialized successfully')
        except Exception as e:
            logger.warning(f'ESP32 touch sensor not available: {e}')
            self.touch_sensor = None

        logger.info('Initialization complete!')
        return True

    def cleanup(self):
        logger.info('\nCleaning up...')
        if self.cap:
            try:
                self.cap.release()
                logger.info('Camera released')
            except Exception:
                pass
        if self.robot:
            try:
                self.robot.switch_power(power_type='servo', switch_status='off')
            except Exception:
                pass
            try:
                self.robot.disconnect()
            except Exception:
                pass
        if self.touch_sensor:
            try:
                self.touch_sensor.close()
                logger.info('ESP32 serial closed')
            except Exception:
                pass
        cv2.destroyAllWindows()
        logger.info('Cleanup complete')

    def get_current_robot_pose(self):
        status, pose = _get_tm_pose_mmdeg(self.robot, retries=1, retry_delay=0.05)
        if status == 0:
            return pose
        logger.error('Failed to get TM robot position')
        return None

    def get_selected_point_item(self, point_targets):
        for item in point_targets:
            if item['name'] == self.selected_point_name:
                return item
        return point_targets[0] if point_targets else None

    def get_touched_points_dict(self):
        return {k: v for k, v in self.touched_contact_points.items() if v is not None}

    def descend_until_touch(self, point_name, touch_approach_pose, nominal_contact_pose, already_at_touch_approach=False):
        if self.touch_sensor is None:
            logger.warning('Touch sensor not available, cannot run touch acquisition')
            return None

        logger.info(f"\n=== {point_name} touch acquisition ===")
        logger.info(f"Touch approach pose (~{TOUCH_APPROACH_OFFSET_MM:.1f} mm above): {touch_approach_pose}")
        logger.info(f"Nominal contact pose: {nominal_contact_pose}")

        if already_at_touch_approach:
            logger.info(f"{point_name}: already positioned about {TOUCH_APPROACH_OFFSET_MM:.1f} mm above this point. Starting descent.")
        else:
            logger.info(f"{point_name}: moving first to about {TOUCH_APPROACH_OFFSET_MM:.1f} mm above this point before descent")
            if not move_robot_to_pose(self.robot, touch_approach_pose, speed_perc=TOUCH_APPROACH_SPEED_PERC):
                return None
            if not wait_for_robot_arrival(self.robot, touch_approach_pose):
                return None
            verify_position(self.robot, touch_approach_pose)

        self.touch_sensor.flush()

        current_pose = list(touch_approach_pose)
        min_search_z = nominal_contact_pose[2] - TOUCH_SEARCH_EXTRA_MM
        logger.info(f'{point_name}: descending slowly until touch. Search until Z={min_search_z:.3f} mm')

        step_idx = 0
        while current_pose[2] > min_search_z:
            step_idx += 1
            current_pose[2] -= TOUCH_DESCENT_STEP_MM

            if not move_robot_to_pose(self.robot, current_pose, speed_perc=TOUCH_DESCENT_SPEED_PERC):
                return None
            if not wait_for_robot_arrival(self.robot, current_pose, max_wait=10):
                return None

            time.sleep(TOUCH_SETTLE_TIME)
            touched, adc_value, touch_state = self.touch_sensor.wait_for_touch(stable_count=TOUCH_STABLE_COUNT)

            logger.info(
                f"{point_name}: step {step_idx:02d} | Z={current_pose[2]:.3f} mm | "
                f"ADC={adc_value} | touch={touch_state}"
            )

            if touched:
                actual_pose = self.get_current_robot_pose()
                logger.info(f'{point_name}: TOUCH detected (ADC={adc_value})')
                logger.info(f'{point_name}: actual touched pose = {actual_pose}')
                return actual_pose

        logger.warning(f'{point_name}: no touch detected before reaching min search Z')
        return None

    def move_to_point_pre_approach(self, point_item):
        logger.info(
            f"{point_item['name']}: moving to pre-approach about {TOUCH_APPROACH_OFFSET_MM:.1f} mm above this point before touch search"
        )
        pre_pose = point_item['pose_touch_approach_move']
        if not move_robot_to_pose(self.robot, pre_pose, speed_perc=TOUCH_APPROACH_SPEED_PERC):
            return False
        if not wait_for_robot_arrival(self.robot, pre_pose):
            return False
        verify_position(self.robot, pre_pose)
        return True

    def acquire_touch_point(self, point_item, already_at_touch_approach=False):
        if not TOUCH_ACQUISITION_ENABLED:
            logger.warning('TOUCH_ACQUISITION_ENABLED=False')
            return False

        touched_pose = self.descend_until_touch(
            point_name=point_item['name'],
            touch_approach_pose=point_item['pose_touch_approach_move'],
            nominal_contact_pose=point_item['pose_contact_move'],
            already_at_touch_approach=already_at_touch_approach,
        )
        if touched_pose is None:
            return False

        self.touched_contact_points[point_item['name']] = touched_pose

        return_pose = list(touched_pose)
        return_pose[2] = touched_pose[2] + TOUCH_APPROACH_OFFSET_MM
        logger.info(f"{point_item['name']}: returning to about {TOUCH_APPROACH_OFFSET_MM:.1f} mm above touched point -> {return_pose}")

        if not move_robot_to_pose(self.robot, return_pose, speed_perc=TOUCH_RETURN_SPEED_PERC):
            return False
        if not wait_for_robot_arrival(self.robot, return_pose):
            return False
        verify_position(self.robot, return_pose)
        return True

    def acquire_all_touch_points(self, point_targets):
        for name in ['P1', 'P2', 'P3']:
            item = next((p for p in point_targets if p['name'] == name), None)
            if item is None:
                logger.warning(f'Missing point config for {name}')
                return False

            logger.info(f"\n{name}: pre-positioning above the next point before touch descent")
            if not self.move_to_point_pre_approach(item):
                logger.warning(f'{name}: failed to reach pre-approach pose')
                return False

            if not self.acquire_touch_point(item, already_at_touch_approach=True):
                logger.warning(f'Failed touch acquisition for {name}')
                return False

        logger.info('✓ Touch acquisition complete for P1/P2/P3')
        return True

    def move_to_initial_pose(self):
        logger.info('\nReturning to initial pose...')
        if move_robot_to_pose(self.robot, INITIAL_POS):
            if wait_for_robot_arrival(self.robot, INITIAL_POS):
                verify_position(self.robot, INITIAL_POS)
                logger.info('✓ Returned to initial pose')
                return True
        return False

    def move_calibration_point_sequence(self, point_item):
        logger.info(f"\n=== {point_item['name']} sequence ===")
        logger.info(f"Nominal contact pose: {point_item['pose_contact_move']}")
        logger.info(f"Touch approach pose (~{TOUCH_APPROACH_OFFSET_MM:.1f} mm above): {point_item['pose_touch_approach_move']}")

        if TOUCH_ACQUISITION_ENABLED and self.touch_sensor is not None:
            return self.acquire_touch_point(point_item)

        logger.warning('Touch sensor unavailable; falling back to nominal move sequence')
        if not move_robot_to_pose(self.robot, point_item['pose_touch_approach_move'], speed_perc=TOUCH_APPROACH_SPEED_PERC):
            return False
        if not wait_for_robot_arrival(self.robot, point_item['pose_touch_approach_move']):
            return False
        verify_position(self.robot, point_item['pose_touch_approach_move'])

        if not MOVE_TO_CONTACT_POINTS:
            logger.info('MOVE_TO_CONTACT_POINTS=False -> staying above point. No descent to board contact pose.')
            return True

        if not move_robot_to_pose(self.robot, point_item['pose_contact_move'], speed_perc=TOUCH_DESCENT_SPEED_PERC):
            return False
        if not wait_for_robot_arrival(self.robot, point_item['pose_contact_move']):
            return False
        verify_position(self.robot, point_item['pose_contact_move'])
        sleep(0.5)

        if not move_robot_to_pose(self.robot, point_item['pose_touch_approach_move'], speed_perc=TOUCH_RETURN_SPEED_PERC):
            return False
        if not wait_for_robot_arrival(self.robot, point_item['pose_touch_approach_move']):
            return False
        return True

    def move_cube_corners(self, cube_corners_robot, cube_corners_robot_approach):
        logger.info('\nRunning cube-edge evaluation movement...')
        self.move_to_initial_pose()

        if not cube_corners_robot or not cube_corners_robot_approach:
            logger.warning('No cube corners available')
            return False

        contact_sequence = list(cube_corners_robot) + [cube_corners_robot[0]]
        approach_sequence = list(cube_corners_robot_approach) + [cube_corners_robot_approach[0]]

        for i, (contact_pose, approach_pose) in enumerate(zip(contact_sequence, approach_sequence)):
            full_pose = list(contact_pose)
            approach = list(approach_pose)

            corner_label = i + 1
            if i == len(cube_corners_robot):
                corner_label = 1
                logger.info('\nReturning to cube corner 1 before going to initial pose...')

            logger.info(
                f"\nCube corner {corner_label}: contact=({full_pose[0]:.3f}, {full_pose[1]:.3f}, {full_pose[2]:.3f}) mm | "
                f"approach=({approach[0]:.3f}, {approach[1]:.3f}, {approach[2]:.3f}) mm"
            )

            if not move_robot_to_pose(self.robot, approach):
                return False
            if not wait_for_robot_arrival(self.robot, approach):
                return False

            if not move_robot_to_pose(self.robot, full_pose):
                return False
            if not wait_for_robot_arrival(self.robot, full_pose):
                return False
            verify_position(self.robot, full_pose)
            sleep(0.5)

            if not move_robot_to_pose(self.robot, approach):
                return False
            if not wait_for_robot_arrival(self.robot, approach):
                return False

        logger.info('✓ Cube-edge evaluation movement complete')
        self.move_to_initial_pose()
        return True

    def print_calibration_report(self, report):
        logger.info('\n' + '=' * 60)
        logger.info('3-POINT CALIBRATION REPORT')
        logger.info('=' * 60)
        used = report.get('touched_contact_points', {}) or {}
        logger.info(f"Calibration source: {'touch-acquired' if all(used.get(k) is not None for k in ['P1','P2','P3']) else 'marker-nominal / mixed'}")
        logger.info(f"P1->P2 distance: {report['p1_p2_distance']:.3f} mm")
        logger.info(f"P1->P3 distance: {report['p1_p3_distance']:.3f} mm")
        if 'x_axis_robot' in report:
            logger.info(f"Board X-axis in robot: {report['x_axis_robot']}")
        if 'y_axis_robot' in report:
            logger.info(f"Board Y-axis in robot: {report['y_axis_robot']}")
        if 'z_axis_robot' in report:
            logger.info(f"Board Z-axis in robot: {report['z_axis_robot']}")
        logger.info('T_board_to_robot:')
        logger.info('\n' + np.array2string(report['T_board_to_robot'], precision=6, suppress_small=False))
        for line in report['point_error_lines']:
            logger.info(line)
        logger.info(f"Mean 3-point error: {np.mean(report['point_errors_mm']):.3f} mm")
        logger.info(f"Max 3-point error: {np.max(report['point_errors_mm']):.3f} mm")
        logger.info('\nCube corners (board -> robot):')
        for i, corner in enumerate(report['cube_corners_robot']):
            logger.info(
                f"Corner {i+1}: X={corner[0]:.3f}, Y={corner[1]:.3f}, Z={corner[2]:.3f}, "
                f"Rx={corner[3]:.3f}, Ry={corner[4]:.3f}, Rz={corner[5]:.3f}"
            )
            if i < len(report['cube_corner_errors_mm']):
                logger.info(f"  Error vs expected: {report['cube_corner_errors_mm'][i]:.3f} mm")
        if report['cube_corner_errors_mm']:
            logger.info(f"Mean cube-corner error: {np.mean(report['cube_corner_errors_mm']):.3f} mm")
            logger.info(f"Max cube-corner error: {np.max(report['cube_corner_errors_mm']):.3f} mm")

    def display_overlay(self, frame, success, rvec=None, tvec=None, marker_pose_base=None,
                        center_pose=None, point_targets=None, calibration_report=None, touched_points=None):
        display = frame.copy()
        if success and rvec is not None and tvec is not None:
            corners, ids, _ = detect_markers_aruco(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), ARUCO_DICT, aruco_params)
            if ids is not None:
                cv2.aruco.drawDetectedMarkers(display, corners, ids)
            try:
                cv2.drawFrameAxes(display, camera_matrix, dist_coeffs, rvec, tvec, MARKER_SIZE * 0.5, 2)
            except Exception:
                draw_axis(display, camera_matrix, dist_coeffs, rvec, tvec, MARKER_SIZE * 0.5)

            pos_cam = tvec.flatten() * 1000.0
            cv2.putText(display, f'Marker in camera: ({pos_cam[0]:.1f}, {pos_cam[1]:.1f}, {pos_cam[2]:.1f}) mm',
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
            y = 60
            if marker_pose_base is not None:
                cv2.putText(display, f'Marker base: ({marker_pose_base[0]:.1f}, {marker_pose_base[1]:.1f}, {marker_pose_base[2]:.1f}) mm',
                            (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
                y += 25
                cv2.putText(display, f'Marker RPY: ({marker_pose_base[3]:.1f}, {marker_pose_base[4]:.1f}, {marker_pose_base[5]:.1f}) deg',
                            (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 2)
                y += 25
            if center_pose is not None:
                cv2.putText(display, f'Center TCP contact: ({center_pose[0]:.1f}, {center_pose[1]:.1f}, {center_pose[2]:.1f}) mm',
                            (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 180, 0), 2)
                y += 25
                cv2.putText(display, f'Approach SAFE_Z: {SAFE_Z:.1f} mm (movement only)',
                            (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 180, 0), 2)
                y += 25
            if point_targets:
                touched_points = touched_points or {}
                cv2.putText(display, f'Selected point: {self.selected_point_name}',
                            (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 160, 255), 2)
                y += 25
                for item in point_targets:
                    pose = item['pose_contact']
                    prefix = '>' if item['name'] == self.selected_point_name else ' '
                    cv2.putText(display, f"{prefix}{item['name']} N: ({pose[0]:.1f}, {pose[1]:.1f}, {pose[2]:.1f})",
                                (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (255, 160, 255), 1)
                    y += 18
                    touch_appr = item['pose_touch_approach_move']
                    cv2.putText(display, f"  {item['name']} TA: ({touch_appr[0]:.1f}, {touch_appr[1]:.1f}, {touch_appr[2]:.1f})",
                                (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (180, 180, 255), 1)
                    y += 18
                    touched = touched_points.get(item['name'])
                    if touched is not None:
                        cv2.putText(display, f"  {item['name']} T: ({touched[0]:.1f}, {touched[1]:.1f}, {touched[2]:.1f})",
                                    (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (120, 255, 120), 1)
                        y += 18
            if calibration_report is not None:
                text = 'Cube eval ready'
                if calibration_report['cube_corner_errors_mm']:
                    text = f"Cube eval mean err: {np.mean(calibration_report['cube_corner_errors_mm']):.2f} mm"
                cv2.putText(display, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        else:
            cv2.putText(display, f'Looking for marker ID {TARGET_MARKER_ID}... frame {self.frame_count}',
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        footer1 = '1/2/3 select P1/P2/P3 | SPACE touch-acquire selected point | a acquire all P1/P2/P3'
        footer2 = 'c compute 3-point eval | e run cube-edge movement | i return initial pose | q quit'
        cv2.putText(display, footer1, (10, display.shape[0] - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1)
        cv2.putText(display, footer2, (10, display.shape[0] - 18), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1)
        return display

    def run(self):
        try:
            if not self.initialize():
                return

            logger.info('\n' + '=' * 60)
            logger.info('MARKER-BASED 3-POINT BOARD CALIBRATION + CUBE EVALUATION (TM + TOUCH)')
            logger.info('=' * 60)

            last_success = False
            last_rvec = None
            last_tvec = None
            last_marker_pose_base = None
            last_center_pose = None
            last_point_targets = []
            last_calibration_report = None

            while True:
                ret, frame = self.cap.read()
                if not ret:
                    continue
                self.frame_count += 1

                current_pose = self.get_current_robot_pose()
                current_robot_transform = None if current_pose is None else robot_pose_mmdeg_to_transformation(current_pose)

                success, T_cam_from_marker, rvec, tvec, _corners, _robot_info = detect_aruco_marker(frame, current_robot_transform)
                if success and current_pose is not None:
                    last_success = True
                    last_rvec = rvec
                    last_tvec = tvec
                    T_base_from_marker = compute_marker_pose_in_base(T_cam_from_marker, current_pose)
                    T_base_from_tool_center = compute_center_tool_pose_from_marker(T_base_from_marker)
                    last_marker_pose_base = transform_to_pose_mmdeg(T_base_from_marker)
                    last_center_pose = transform_to_pose_mmdeg(T_base_from_tool_center)
                    last_point_targets = compute_calibration_point_targets(T_base_from_tool_center)

                display = self.display_overlay(
                    frame,
                    last_success,
                    rvec=last_rvec if last_success else None,
                    tvec=last_tvec if last_success else None,
                    marker_pose_base=last_marker_pose_base if last_success else None,
                    center_pose=last_center_pose if last_success else None,
                    point_targets=last_point_targets if last_success else None,
                    calibration_report=last_calibration_report,
                    touched_points=self.get_touched_points_dict(),
                )
                cv2.imshow('Marker 3-Point Calibration TM + Touch', display)
                key = cv2.waitKey(1) & 0xFF

                if key == ord('1'):
                    self.selected_point_name = 'P1'
                    logger.info('Selected point: P1')
                elif key == ord('2'):
                    self.selected_point_name = 'P2'
                    logger.info('Selected point: P2')
                elif key == ord('3'):
                    self.selected_point_name = 'P3'
                    logger.info('Selected point: P3')
                elif key == ord(' '):
                    if last_success and last_point_targets:
                        item = self.get_selected_point_item(last_point_targets)
                        if item is not None:
                            self.move_calibration_point_sequence(item)
                elif key == ord('a'):
                    if last_success and last_point_targets:
                        self.acquire_all_touch_points(last_point_targets)
                elif key == ord('c'):
                    if last_success and last_point_targets:
                        last_calibration_report = build_three_point_calibration(last_point_targets, self.get_touched_points_dict())
                        self.print_calibration_report(last_calibration_report)
                elif key == ord('e'):
                    if last_calibration_report is None and last_success and last_point_targets:
                        last_calibration_report = build_three_point_calibration(last_point_targets, self.get_touched_points_dict())
                        self.print_calibration_report(last_calibration_report)
                    if last_calibration_report is not None:
                        self.move_cube_corners(
                            last_calibration_report['cube_corners_robot'],
                            last_calibration_report['cube_corners_robot_approach'],
                        )
                elif key == ord('i'):
                    self.move_to_initial_pose()
                elif key == ord('q'):
                    logger.info('Quit requested by user')
                    break

        except KeyboardInterrupt:
            logger.info('\nInterrupted by user')
        finally:
            self.cleanup()


def main():
    print('=' * 60)
    print('MARKER-BASED 3-POINT BOARD CALIBRATION + CUBE EVALUATION (TM + TOUCH)')
    print('=' * 60)
    print('\nThis script:')
    print('  1) moves first to the given initial pose,')
    print('  2) detects the marker from ROS2 topic,')
    print('  3) builds pose_p1 / pose_p2 / pose_p3 on the real marker/table plane,')
    print('  4) uses touch acquisition to record actual touched poses for P1 / P2 / P3,')
    print('  5) runs the same 3-point calibration logic as the manual teaching workflow,')
    print('  6) evaluates the transformed cube corners.')
    print(f'\nInitial pose: {INITIAL_POS}')
    print(f'SAFE_Z for approach poses only: {SAFE_Z} mm')
    print(f'Hand-eye path: {TRANSFORM_PATH}')
    print(f'Camera intrinsic path: {CAMERA_INTRINSIC_PATH}')
    print(f'ROS image topic: {ROS_IMAGE_TOPIC}')
    print('\nKeys:')
    print('  1 / 2 / 3 : select P1 / P2 / P3')
    print('  SPACE     : touch-acquire selected point (approach -> slow descend -> record -> return above point)')
    print('  a         : touch-acquire all P1 / P2 / P3 sequentially')
    print('  c         : compute 3-point calibration + cube evaluation (uses touched points when available)')
    print('  e         : run cube-edge movement using transformed corners')
    print('  i         : return to initial pose')
    print('  q         : quit')
    print('\nPress ENTER to start...')
    input()
    guide = ArUcoBoardCalibrationGuide()
    guide.run()


if __name__ == '__main__':
    main()
