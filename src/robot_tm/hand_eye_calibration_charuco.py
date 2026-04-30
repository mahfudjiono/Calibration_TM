#!/usr/bin/env python3

import cv2
import json
import os
import numpy as np

# ------------------------------------------------------------------------------
# 1) Charuco board definition
# ------------------------------------------------------------------------------
CHARUCO_SQUARES_X = 10
CHARUCO_SQUARES_Y = 8
SQUARE_LENGTH = 0.020   # meters
MARKER_LENGTH = 0.015   # meters

try:
    CHARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000)
except AttributeError:
    CHARUCO_DICT = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_1000)

try:
    charuco_board = cv2.aruco.CharucoBoard_create(
        squaresX=CHARUCO_SQUARES_X,
        squaresY=CHARUCO_SQUARES_Y,
        squareLength=SQUARE_LENGTH,
        markerLength=MARKER_LENGTH,
        dictionary=CHARUCO_DICT,
    )
except AttributeError:
    charuco_board = cv2.aruco.CharucoBoard(
        (CHARUCO_SQUARES_X, CHARUCO_SQUARES_Y),
        SQUARE_LENGTH,
        MARKER_LENGTH,
        CHARUCO_DICT,
    )

try:
    aruco_params = cv2.aruco.DetectorParameters_create()
except AttributeError:
    aruco_params = cv2.aruco.DetectorParameters()

aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
aruco_params.adaptiveThreshWinSizeMin = 3
aruco_params.adaptiveThreshWinSizeMax = 23
aruco_params.adaptiveThreshWinSizeStep = 10
aruco_params.minMarkerPerimeterRate = 0.02

if hasattr(charuco_board, 'setLegacyPattern'):
    try:
        charuco_board.setLegacyPattern(True)
    except Exception:
        pass

# ------------------------------------------------------------------------------
# 2) Basic script parameters
# ------------------------------------------------------------------------------
# This script supports both older and newer OpenCV CharucoBoard APIs.
img_w = 1280
img_h = 960
optimal_new_K_alpha = 0
robot_type = 'TM'

# IMPORTANT:
# Your current TM collection pipeline saved poses from get_position(times_1000=True),
# so translations are effectively mm*1000 and rotations are deg*10000 in the .npy file.
TM_POSES_SAVED_AS_SCALED_RAW = True

SAVE_PRIMARY_CAMERA_K = 'raw'   # options: 'raw' or 'undistorted'

EULER_ORDERS_TO_TEST = ('xyz', 'xzy', 'yxz', 'yzx', 'zxy', 'zyx')
HAND_EYE_METHODS = {
    'TSAI': cv2.CALIB_HAND_EYE_TSAI,
    'PARK': cv2.CALIB_HAND_EYE_PARK,
    'HORAUD': cv2.CALIB_HAND_EYE_HORAUD,
    'DANIILIDIS': cv2.CALIB_HAND_EYE_DANIILIDIS,
}

# ------------------------------------------------------------------------------
# 3) Paths
# ------------------------------------------------------------------------------
dataset_root = '/home/hucenrotia/mahfud/Calibration_TM/src/local_data/robot_pose_dataset'
pose_file = os.path.join(dataset_root, 'robot_pose_camera_cal.npy')
rgb_dir = os.path.join(dataset_root, 'rgb')

transfom_path = '/home/hucenrotia/mahfud/Calibration_TM/src/robot_tm/transformation_matrices_TM5.json'
camera_intrinsic_path = '/home/hucenrotia/mahfud/Calibration_TM/src/robot_tm/camera_intrinsic_TM5.json'

# ------------------------------------------------------------------------------
# 4) Small helper functions
# ------------------------------------------------------------------------------
def rot_x(rad: float) -> np.ndarray:
    c, s = np.cos(rad), np.sin(rad)
    return np.array([[1.0, 0.0, 0.0],
                     [0.0, c, -s],
                     [0.0, s, c]], dtype=np.float64)


def rot_y(rad: float) -> np.ndarray:
    c, s = np.cos(rad), np.sin(rad)
    return np.array([[c, 0.0, s],
                     [0.0, 1.0, 0.0],
                     [-s, 0.0, c]], dtype=np.float64)


def rot_z(rad: float) -> np.ndarray:
    c, s = np.cos(rad), np.sin(rad)
    return np.array([[c, -s, 0.0],
                     [s, c, 0.0],
                     [0.0, 0.0, 1.0]], dtype=np.float64)


def euler_to_matrix_robot(angles_rad: np.ndarray, order: str = 'xyz') -> np.ndarray:
    rx, ry, rz = angles_rad
    mats = {'x': rot_x(rx), 'y': rot_y(ry), 'z': rot_z(rz)}
    R = np.eye(3, dtype=np.float64)
    for axis in order:
        R = R @ mats[axis]
    return R


def make_transform(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(t, dtype=np.float64).reshape(3)
    return T


def invert_transform(T: np.ndarray) -> np.ndarray:
    R = T[:3, :3]
    t = T[:3, 3]
    T_inv = np.eye(4, dtype=np.float64)
    T_inv[:3, :3] = R.T
    T_inv[:3, 3] = -R.T @ t
    return T_inv


def rotation_error_deg(R_a: np.ndarray, R_b: np.ndarray) -> float:
    R = R_a.T @ R_b
    value = (np.trace(R) - 1.0) / 2.0
    value = np.clip(value, -1.0, 1.0)
    return float(np.degrees(np.arccos(value)))


def project_to_so3(R: np.ndarray) -> np.ndarray:
    U, _, Vt = np.linalg.svd(R)
    R_proj = U @ Vt
    if np.linalg.det(R_proj) < 0:
        U[:, -1] *= -1
        R_proj = U @ Vt
    return R_proj


def evaluate_handeye_solution(H_g2b_list, H_t2c_list, H_c2g):
    H_b2t_list = []
    for H_g2b, H_t2c in zip(H_g2b_list, H_t2c_list):
        H_b2t_list.append(H_g2b @ H_c2g @ H_t2c)

    translations = np.array([H[:3, 3] for H in H_b2t_list], dtype=np.float64)
    t_std_mm = np.std(translations, axis=0) * 1000.0
    t_std_norm_mm = float(np.linalg.norm(t_std_mm))

    R_mean = project_to_so3(np.mean(np.array([H[:3, :3] for H in H_b2t_list]), axis=0))
    rot_errors_deg = np.array([rotation_error_deg(R_mean, H[:3, :3]) for H in H_b2t_list], dtype=np.float64)
    rot_mean_deg = float(np.mean(rot_errors_deg))

    score = t_std_norm_mm + 10.0 * rot_mean_deg
    return {
        'score': score,
        't_std_mm': t_std_mm,
        't_std_norm_mm': t_std_norm_mm,
        'rot_mean_deg': rot_mean_deg,
        'H_b2t_list': H_b2t_list,
    }


def detect_markers(gray_img):
    if hasattr(cv2.aruco, 'ArucoDetector'):
        detector = cv2.aruco.ArucoDetector(CHARUCO_DICT, aruco_params)
        return detector.detectMarkers(gray_img)
    return cv2.aruco.detectMarkers(gray_img, CHARUCO_DICT, parameters=aruco_params)


def interpolate_charuco(gray_img, marker_corners, marker_ids):
    if hasattr(cv2.aruco, 'CharucoDetector'):
        try:
            detector = cv2.aruco.CharucoDetector(charuco_board, detectorParams=aruco_params)
        except TypeError:
            detector = cv2.aruco.CharucoDetector(charuco_board)
        ch_corners, ch_ids, _, _ = detector.detectBoard(
            gray_img,
            markerCorners=marker_corners,
            markerIds=marker_ids,
        )
        retval = 0 if ch_ids is None else len(ch_ids)
        return retval, ch_corners, ch_ids
    return cv2.aruco.interpolateCornersCharuco(
        markerCorners=marker_corners,
        markerIds=marker_ids,
        image=gray_img,
        board=charuco_board,
    )


def load_robot_poses_for_tm(pose_path: str, scaled_raw: bool = True):
    robot_poses_raw = np.load(pose_path)
    print(f"Loaded {len(robot_poses_raw)} robot poses")
    print(f"First raw pose: {robot_poses_raw[0]}")

    if scaled_raw:
        translations_m = robot_poses_raw[:, :3].astype(np.float64) / 1_000_000.0
        angles_deg = robot_poses_raw[:, 3:6].astype(np.float64) / 10_000.0
    else:
        translations_m = robot_poses_raw[:, :3].astype(np.float64) / 1000.0
        angles_deg = robot_poses_raw[:, 3:6].astype(np.float64)

    angles_rad = np.deg2rad(angles_deg)

    print(f"First pose translation (m): {translations_m[0]}")
    print(f"First pose rotation (deg): {angles_deg[0]}")
    return robot_poses_raw, translations_m, angles_deg, angles_rad


# ------------------------------------------------------------------------------
# 5) Load robot poses
# ------------------------------------------------------------------------------
if not os.path.exists(pose_file):
    raise FileNotFoundError(f"Pose file not found: {pose_file}")

if robot_type != 'TM':
    raise RuntimeError("This rewritten script is TM-clean version. Set robot_type='TM'.")

robot_poses_raw, translations_m, angles_deg, angles_rad = load_robot_poses_for_tm(
    pose_file,
    scaled_raw=TM_POSES_SAVED_AS_SCALED_RAW,
)

# ------------------------------------------------------------------------------
# 6) Process images and collect valid correspondences
# ------------------------------------------------------------------------------
images = []
for i in range(len(robot_poses_raw)):
    fname = os.path.join(rgb_dir, f'{i}.png')
    if os.path.exists(fname):
        images.append((i, fname))

print(f"Found {len(images)} images")

objpoints = []
imgpoints = []
valid_robot_indices = []
min_corners_required = 20

for idx, (i, fname) in enumerate(images):
    print(f"[{idx + 1}/{len(images)}] Processing {fname}")
    img = cv2.imread(fname)
    if img is None:
        continue

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = detect_markers(gray)

    if ids is None or len(ids) == 0:
        continue

    valid = [(c, marker_id) for c, marker_id in zip(corners, ids) if c is not None and c.shape[1] == 4]
    if len(valid) < 4:
        continue

    valid_corners, valid_ids = zip(*valid)
    valid_ids = np.array(valid_ids, dtype=np.int32).reshape(-1, 1)

    try:
        retval, ch_corners, ch_ids = interpolate_charuco(gray, list(valid_corners), valid_ids)
    except cv2.error:
        continue

    if not retval or ch_corners is None or ch_ids is None:
        continue

    num_corners = len(ch_ids)
    if num_corners < min_corners_required:
        continue

    ch_ids_flat = ch_ids.reshape(-1).astype(int)

    if hasattr(charuco_board, 'getChessboardCorners'):
        corners3d = np.asarray(charuco_board.getChessboardCorners(), dtype=np.float32)
    else:
        corners3d = np.asarray(charuco_board.chessboardCorners, dtype=np.float32)
    if corners3d.ndim == 3 and corners3d.shape[1] == 1:
        corners3d = corners3d[:, 0, :]

    board_pts_3d = corners3d[ch_ids_flat]
    img_pts_2d = ch_corners.reshape(-1, 2).astype(np.float32)

    if board_pts_3d.shape[0] < min_corners_required:
        continue

    objpoints.append(board_pts_3d.astype(np.float32))
    imgpoints.append(img_pts_2d.astype(np.float32))
    valid_robot_indices.append(i)
    print(f"✓ Valid Charuco detection with {num_corners} corners")

print(f"\nCollected {len(objpoints)} valid images for calibration")
if len(objpoints) < 10:
    raise RuntimeError(f"Not enough valid images for calibration (only {len(objpoints)}). Need at least 10.")

# ------------------------------------------------------------------------------
# 7) Calibrate camera intrinsics
# ------------------------------------------------------------------------------
print("\nCalibrating camera...")
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
    objectPoints=objpoints,
    imagePoints=imgpoints,
    imageSize=(img_w, img_h),
    cameraMatrix=None,
    distCoeffs=None,
)
print("Camera calibration done.")
print("Camera matrix:\n", mtx)

newcameramtx, _ = cv2.getOptimalNewCameraMatrix(
    mtx, dist, (img_w, img_h), optimal_new_K_alpha, (img_w, img_h)
)

px_total_error = 0.0
for i in range(len(objpoints)):
    projected, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
    proj_2d = projected.reshape(-1, 2)
    err = cv2.norm(imgpoints[i], proj_2d, cv2.NORM_L2) / len(proj_2d)
    px_total_error += err
mean_reproj_error_px = px_total_error / len(objpoints)
print(f"Mean reprojection error (px): {mean_reproj_error_px:.4f} px")

# ------------------------------------------------------------------------------
# 8) Build target-to-camera transforms from ChArUco PnP results
# ------------------------------------------------------------------------------
R_target2cam = []
t_target2cam = []
H_target2cam = []

for rvec, tvec in zip(rvecs, tvecs):
    R_tc, _ = cv2.Rodrigues(rvec)
    t_tc = tvec.reshape(3, 1).astype(np.float64)
    R_target2cam.append(R_tc.astype(np.float64))
    t_target2cam.append(t_tc)
    H_target2cam.append(make_transform(R_tc, t_tc))

board_positions_cam = np.array([t.reshape(3) for t in t_target2cam], dtype=np.float64)
print("\nBoard positions in camera frame (m):")
print(f"  X: mean={np.mean(board_positions_cam[:, 0]):.3f}, std={np.std(board_positions_cam[:, 0]):.3f}")
print(f"  Y: mean={np.mean(board_positions_cam[:, 1]):.3f}, std={np.std(board_positions_cam[:, 1]):.3f}")
print(f"  Z: mean={np.mean(board_positions_cam[:, 2]):.3f}, std={np.std(board_positions_cam[:, 2]):.3f}")

# ------------------------------------------------------------------------------
# 9) Hand-eye calibration for TM
# ------------------------------------------------------------------------------
print("\n" + "=" * 60)
print(f"REAL HAND-EYE CALIBRATION FOR {robot_type}")
print("=" * 60)

best_result = None
summary_rows = []

for euler_order in EULER_ORDERS_TO_TEST:
    H_gripper2base = []
    R_gripper2base = []
    t_gripper2base = []

    for idx in valid_robot_indices:
        R_g2b = euler_to_matrix_robot(angles_rad[idx], order=euler_order)
        t_g2b = translations_m[idx].reshape(3, 1)
        H_g2b = make_transform(R_g2b, t_g2b)

        H_gripper2base.append(H_g2b)
        R_gripper2base.append(R_g2b)
        t_gripper2base.append(t_g2b)

    for method_name, method_flag in HAND_EYE_METHODS.items():
        try:
            R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
                R_gripper2base=R_gripper2base,
                t_gripper2base=t_gripper2base,
                R_target2cam=R_target2cam,
                t_target2cam=t_target2cam,
                method=method_flag,
            )
        except cv2.error as exc:
            summary_rows.append((euler_order, method_name, np.inf, np.inf, np.inf, f"failed: {exc}"))
            continue

        H_cam2gripper = make_transform(R_cam2gripper, t_cam2gripper)
        eval_result = evaluate_handeye_solution(H_gripper2base, H_target2cam, H_cam2gripper)

        summary_rows.append((
            euler_order,
            method_name,
            eval_result['score'],
            eval_result['t_std_norm_mm'],
            eval_result['rot_mean_deg'],
            'ok',
        ))

        if best_result is None or eval_result['score'] < best_result['score']:
            best_result = {
                'euler_order': euler_order,
                'method_name': method_name,
                'score': eval_result['score'],
                't_std_mm': eval_result['t_std_mm'],
                't_std_norm_mm': eval_result['t_std_norm_mm'],
                'rot_mean_deg': eval_result['rot_mean_deg'],
                'H_b2t_list': eval_result['H_b2t_list'],
                'H_cam2gripper': H_cam2gripper,
                'H_gripper2cam': invert_transform(H_cam2gripper),
            }

if best_result is None:
    raise RuntimeError('Hand-eye calibration failed for all tested TM Euler-order/method combinations.')

print("Candidate ranking (lower score is better):")
summary_rows = sorted(summary_rows, key=lambda row: row[2])
for row in summary_rows[:8]:
    order, method, score, t_std_norm_mm, rot_mean_deg, status = row
    print(
        f"  order={order:>3s} | method={method:<10s} | score={score:8.3f} | "
        f"target std={t_std_norm_mm:7.3f} mm | rot={rot_mean_deg:7.4f} deg | {status}"
    )

print("\nSelected TM interpretation:")
print(f"  Euler order: {best_result['euler_order']}")
print(f"  Hand-eye method: {best_result['method_name']}")
print(f"  Base-to-target consistency std norm: {best_result['t_std_norm_mm']:.3f} mm")
print(f"  Base-to-target consistency rotation: {best_result['rot_mean_deg']:.4f} deg")

T_cam2end = best_result['H_cam2gripper']
T_end2cam = best_result['H_gripper2cam']
mean_offset = T_end2cam[:3, 3]

print("\nCalculated camera transform:")
print(
    f"  End-effector -> camera translation [mm]: "
    f"X={mean_offset[0] * 1000:.1f}, Y={mean_offset[1] * 1000:.1f}, Z={mean_offset[2] * 1000:.1f}"
)
print(f"\nT_end2cam:\n{T_end2cam}")
print(f"\nT_cam2end:\n{T_cam2end}")

# ------------------------------------------------------------------------------
# 10) Compute mm-based reprojection error
# ------------------------------------------------------------------------------
internal_cols = CHARUCO_SQUARES_X - 1
internal_rows = CHARUCO_SQUARES_Y - 1
width_mm = internal_cols * SQUARE_LENGTH * 1000.0
height_mm = internal_rows * SQUARE_LENGTH * 1000.0
board_diagonal_mm = np.sqrt(width_mm ** 2 + height_mm ** 2)

mm_total_error = 0.0
mm_count = 0
for i in range(len(objpoints)):
    projected, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
    proj_2d = projected.reshape(-1, 2)
    l1_error = cv2.norm(imgpoints[i], proj_2d, cv2.NORM_L1) / len(proj_2d)
    x_min = np.min(proj_2d[:, 0])
    x_max = np.max(proj_2d[:, 0])
    y_min = np.min(proj_2d[:, 1])
    y_max = np.max(proj_2d[:, 1])
    diag_px = np.sqrt((x_max - x_min) ** 2 + (y_max - y_min) ** 2)
    if diag_px > 1e-9:
        error_mm = (board_diagonal_mm / diag_px) * l1_error
        mm_total_error += error_mm
        mm_count += 1

mean_error_mm = mm_total_error / mm_count if mm_count > 0 else 0.0
print(f"\nMean camera reprojection error (mm): {mean_error_mm:.4f} mm")

# ------------------------------------------------------------------------------
# 11) Save results
# ------------------------------------------------------------------------------
T1 = T_end2cam.flatten().tolist()
T2 = np.eye(4, dtype=np.float64).flatten().tolist()

with open(transfom_path, 'w') as f:
    json.dump({'T1': T1, 'T2': T2}, f, indent=2)

if SAVE_PRIMARY_CAMERA_K == 'undistorted':
    primary_camera_k = newcameramtx
    secondary_camera_k = mtx
else:
    primary_camera_k = mtx
    secondary_camera_k = newcameramtx

cam_json = {
    'camera_K': primary_camera_k.flatten().tolist(),
    'dist_coef': dist.flatten().tolist(),
    'old_camera_K': secondary_camera_k.flatten().tolist(),
}
with open(camera_intrinsic_path, 'w') as f:
    json.dump(cam_json, f, indent=2)

print("\n" + "=" * 60)
print("Hand-eye calibration completed successfully!")
print(f"Robot type: {robot_type}")
print(f"Selected Euler order: {best_result['euler_order']}")
print(f"Selected hand-eye method: {best_result['method_name']}")
print(f"Mean camera reprojection error: {mean_reproj_error_px:.4f} px")
print(f"Mean camera reprojection error: {mean_error_mm:.4f} mm")
print(
    f"Camera offset (end-effector to camera): "
    f"X={mean_offset[0] * 1000:.1f}, Y={mean_offset[1] * 1000:.1f}, Z={mean_offset[2] * 1000:.1f} mm"
)
print(
    f"Base-to-target consistency std [mm]: "
    f"X={best_result['t_std_mm'][0]:.3f}, "
    f"Y={best_result['t_std_mm'][1]:.3f}, "
    f"Z={best_result['t_std_mm'][2]:.3f}"
)
print(f"Saved to: {transfom_path}")
print(f"Saved to: {camera_intrinsic_path}")
print("=" * 60)
