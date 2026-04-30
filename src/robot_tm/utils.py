from threading import Thread
from time import sleep, time
from PyQt5.QtGui import QImage, QPixmap
#from yolox.utils.visualize import _COLORS
from pyueye import ueye

import cv2
import sys
import numpy as np
from vtk import vtkMatrix3x3,vtkMatrix4x4


def fill_holes(mask_255):
    """
    Fill internal holes in a 0/255 binary mask using OpenCV floodFill,
    without inverting before the fill operation.

    Standard steps:
    1) Flood-fill from (0,0) with 255 to fill the outside background.
    2) Invert that flood-filled image.
    3) Combine (OR) with the original to fill internal holes.
    """
    flood_filled = mask_255.copy().astype(np.uint8)
    h, w = flood_filled.shape[:2]
    flood_mask = np.zeros((h+2, w+2), np.uint8)

    # Flood-fill from (0,0) with 255 -> fill outside background
    cv2.floodFill(flood_filled, flood_mask, (0, 0), 255)

    # Invert flood-filled image -> holes become 255
    flood_filled_inv = cv2.bitwise_not(flood_filled)

    # OR with original -> holes filled
    filled = cv2.bitwise_or(mask_255, flood_filled_inv)
    return filled

def eulerAnglesToRotationMatrix(theta) :
    R_x = np.array([[1,         0,                  0                   ],
                    [0,         np.cos(theta[0]), -np.sin(theta[0]) ],
                    [0,         np.sin(theta[0]), np.cos(theta[0])  ]
                    ])
    R_y = np.array([[np.cos(theta[1]),    0,      np.sin(theta[1])  ],
                    [0,                     1,      0                   ],
                    [-np.sin(theta[1]),   0,      np.cos(theta[1])  ]
                    ])
    R_z = np.array([[np.cos(theta[2]),    -np.sin(theta[2]),    0],
                    [np.sin(theta[2]),    np.cos(theta[2]),     0],
                    [0,                     0,                      1]
                    ])
    R = np.dot(R_z, np.dot( R_y, R_x ))
    return R

def RotationMatrixToEulerAngles(R):
    sy = np.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
    singular = sy < 1e-6
    if  not singular :
        rx = np.arctan2(R[2,1] , R[2,2])
        ry = np.arctan2(-R[2,0], sy)
        rz = np.arctan2(R[1,0], R[0,0])
    else:
        rx = np.arctan2(-R[1,2], R[1,1])
        ry = np.arctan2(-R[2,0], sy)
        rz = 0
    return [rx, ry, rz]

def calculate_angle_with_axes(normal):
    """
    Calculate the angle between the normal vector and each axis (x, y, z).

    Parameters:
    normal (np.ndarray): The normal vector (shape: (3,)).

    Returns:
    dict: A dictionary with the angles in degrees between the normal vector and each axis.
    """
    # Normalize the normal vector
    normal = normal / np.linalg.norm(normal)
    
    # Define unit vectors along x, y, and z axes
    x_axis = np.array([1, 0, 0])
    y_axis = np.array([0, 1, 0])
    z_axis = np.array([0, 0, 1])
    
    # Calculate angles in radians
    angle_x = np.arccos(np.dot(normal, x_axis))
    angle_y = np.arccos(np.dot(normal, y_axis))
    angle_z = np.arccos(np.dot(normal, z_axis))
    
    # Convert angles to degrees
    angles = np.array([np.degrees(angle_x),np.degrees(angle_y),np.degrees(angle_z)])
    return angles


def inverse_R_t(R, t):
    R_inv = R.T
    t_inv = -R_inv @ t
    return R_inv, t_inv

def inverse_R_t_batch(R, t):
    R_inv = []
    t_inv = []
    for i in range(len(R)):
        rot_matrix_inv, t_matrix_inv = inverse_R_t(eulerAnglesToRotationMatrix(R[i, :]), t[i, :])
        R_inv.append(rot_matrix_inv)
        t_inv.append(t_matrix_inv)
    R_inv = np.array(R_inv)
    t = np.array(t_inv)
    return R_inv, t

def to_homogeneus(R, t):
    T = np.zeros((4, 4))
    T[:3, :3] = R
    T[:3, 3] = t.reshape((3,))
    T[3, 3] = 1
    return T

def convert_to_real_robot_pose(_pose):
    if _pose is None:
        return None
    pose = [0, 0, 0, 0, 0, 0]
    pose[0] = round(_pose[0] * 1000000)
    pose[1] = round(_pose[1] * 1000000)
    pose[2] = round(_pose[2] * 1000000)
    pose[3] = round(np.rad2deg(_pose[3]) * 10000)
    pose[4] = round(np.rad2deg(_pose[4]) * 10000)
    pose[5] = round(np.rad2deg(_pose[5]) * 10000)
    pose = [max(-2147483648, min(2147483647, p)) for p in pose]
    return pose

def convert_to_renderer_pose(_pose):
    if _pose is None:
        return None
    pose = [0, 0, 0, 0, 0, 0]
    pose[0] = _pose[0] / 1000000
    pose[1] = _pose[1] / 1000000
    pose[2] = _pose[2] / 1000000
    pose[3] = np.deg2rad(_pose[3]/10000)
    pose[4] = np.deg2rad(_pose[4]/10000)
    pose[5] = np.deg2rad(_pose[5]/10000)
    return pose


def get_T_end2base(robot_pose):
    R = eulerAnglesToRotationMatrix([robot_pose[3], robot_pose[4], robot_pose[5]])
    T_end2base = np.linalg.inv(to_homogeneus(R, np.array([robot_pose[0], robot_pose[1], robot_pose[2]])))
    return T_end2base

def convert_Te2b_to_position(T_end2base):
    T_base2end = np.linalg.inv(T_end2base)
    R = T_end2base[:3, :3]
    if R[0, 2] < 1:
        if R[0, 2] > -1:
            R_y = np.arcsin(R[0, 2])
            R_x = np.arctan2(-R[1, 2], R[2, 2])
            R_z = np.arctan2(-R[0, 1], R[0, 0])
        else:
            R_y = -np.pi/2
            R_x = -np.arctan2(R[1, 0], R[1, 1])
            R_z = 0
    else:
        R_y = np.pi/2
        R_x = np.arctan2(R[1, 0], R[1, 1])
        R_z = 0

    return [T_base2end[0, 3], T_base2end[1, 3], T_base2end[2, 3], -R_x, -R_y, -R_z]

def convert_Tb2e_to_position(T_base2end):
    # Extract the rotation matrix and translation vector
    R = T_base2end[:3, :3]
    position = T_base2end[:3, 3]

    # Calculate the Euler angles (R_x, R_y, R_z) from the rotation matrix
    if R[0, 2] < 1:
        if R[0, 2] > -1:
            R_y = np.arcsin(R[0, 2])
            R_x = np.arctan2(-R[1, 2], R[2, 2])
            R_z = np.arctan2(-R[0, 1], R[0, 0])
        else:
            R_y = -np.pi/2
            R_x = -np.arctan2(R[1, 0], R[1, 1])
            R_z = 0
    else:
        R_y = np.pi/2
        R_x = np.arctan2(R[1, 0], R[1, 1])
        R_z = 0

    # Return the position and orientation
    return [position[0], position[1], position[2], -R_x, -R_y, -R_z]

"""
#for TM
def is_arrived(prev, current, target):
    if prev != current:
        return False
    dist = np.abs(np.array(current[:6]) - np.array(target[:6]))
    if (dist > 500).any():
        return False
    return True

"""
def is_arrived(prev, current, target):
    prv = np.array(prev, dtype=np.int64)[:6]
    cur = np.array(current, dtype=np.int64)[:6]
    tgt = np.array(target, dtype=np.int64)[:6]

    # 1) robot has "stopped" (prev and current are close)
    stopped_tol = np.array([5, 5, 5, 10, 10, 10], dtype=np.int64)
    if np.any(np.abs(cur - prv) > stopped_tol):
        return False

    # 2) robot is close to target
    arrive_tol = np.array([500, 500, 500, 500, 500, 500], dtype=np.int64)
    if np.any(np.abs(cur - tgt) > arrive_tol):
        return False

    return True

class CameraCapture:
    def __init__(self, name):
        self.cap = cv2.VideoCapture(name)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
        self.frame = None
        self.running = False
        self.updating = False
        self.t = Thread(target=self._reader)
        self.t.start()
        ret = False
        start = time()
        while not ret:
            ret, frame = self.cap.read()
            self.update(frame)
            if time() - start > 2:
                raise RuntimeError('Cannot read image from webcam')

    def _reader(self):
        self.running = True
        while self.running:
            ret, frame = self.cap.read()
            if not ret:
                self.running = False
            else:
                self.update(frame)
                sleep(0.0001)

    def read(self):
        while self.updating:
            pass
        return self.running, self.frame
    
    def isOpened(self):
        return self.cap.isOpened()
    
    def release(self):
        self.running = False
        sleep(0.001)
        self.cap.release()
        self.t.join()
    
    def update(self, frame):
        self.updating = True
        self.frame = np.copy(frame)
        self.updating = False
        
class IDSCameraCapture:
    def __init__(self, name, img_size=(640, 480), config_filename='robot/ids_config.ini'):
        max_frame_width = 1280
        max_frame_height = 960
        self.img_size = img_size
        hCam = ueye.HIDS(name)             #0: first available camera;  1-254: The camera with the specified camera ID
        sInfo = ueye.SENSORINFO()
        cInfo = ueye.CAMINFO()
        self.pcImageMemory = ueye.c_mem_p()
        MemID = ueye.int()
        self.pitch = ueye.INT()

        nRet = ueye.is_InitCamera(hCam, None)
        assert nRet == ueye.IS_SUCCESS, "is_InitCamera ERROR"
        # Reads out the data hard-coded in the non-volatile camera memory and writes it to the data structure that cInfo points to
        nRet = ueye.is_GetCameraInfo(hCam, cInfo)
        assert nRet == ueye.IS_SUCCESS, "is_GetCameraInfo ERROR"
        # You can query additional information about the sensor type used in the camera
        nRet = ueye.is_GetSensorInfo(hCam, sInfo)
        assert nRet == ueye.IS_SUCCESS, "is_GetSensorInfo ERROR"
        nRet = ueye.is_ResetToDefault( hCam)
        assert nRet == ueye.IS_SUCCESS, "is_ResetToDefault ERROR"
        # Set display mode to DIB
        nRet = ueye.is_SetDisplayMode(hCam, ueye.IS_SET_DM_DIB)
        pParam = ueye.wchar_p()
        pParam.value = config_filename

        ueye.is_ParameterSet(hCam, ueye.IS_PARAMETERSET_CMD_LOAD_FILE, pParam, 0)

        m_nColorMode = ueye.IS_CM_BGR8_PACKED
        self.nBitsPerPixel = ueye.INT(24)
        self.bytes_per_pixel = int(self.nBitsPerPixel / 8)

        self.width = ueye.INT(max_frame_width)
        self.height = ueye.INT(max_frame_height)

        nRet = ueye.is_AllocImageMem(hCam, self.width, self.height, self.nBitsPerPixel, self.pcImageMemory, MemID)
        if nRet != ueye.IS_SUCCESS:
            print("is_AllocImageMem ERROR")
        else:
            # Makes the specified image memory the active memory
            nRet = ueye.is_SetImageMem(hCam, self.pcImageMemory, MemID)
            if nRet != ueye.IS_SUCCESS:
                print("is_SetImageMem ERROR")
            else:
                # Set the desired color mode
                nRet = ueye.is_SetColorMode(hCam, m_nColorMode)

        # Activates the camera's live video mode (free run mode)
        nRet = ueye.is_CaptureVideo(hCam, ueye.IS_DONT_WAIT)
        assert nRet == ueye.IS_SUCCESS, "is_CaptureVideo ERROR"

        # Enables the queue mode for existing image memory sequences
        nRet = ueye.is_InquireImageMem(hCam, self.pcImageMemory, MemID, self.width, self.height, self.nBitsPerPixel, self.pitch)
        assert nRet == ueye.IS_SUCCESS, "is_InquireImageMem ERROR"
        
        self.frame = None
        self.running = False
        self.updating = False

        array = ueye.get_data(self.pcImageMemory, self.width, self.height, self.nBitsPerPixel, self.pitch, copy=False)
        frame = np.reshape(array,(self.height.value, self.width.value, self.bytes_per_pixel))
        self.update(frame)
        self.t = Thread(target=self._reader)
        self.t.start()

    def _reader(self):
        self.running = True
        while self.running:
            array = ueye.get_data(self.pcImageMemory, self.width, self.height, self.nBitsPerPixel, self.pitch, copy=False)
            frame = np.reshape(array,(self.height.value, self.width.value, self.bytes_per_pixel))
            self.update(frame)
            sleep(0.0001)

    def read(self):
        s = time()
        while self.frame is None:
            if time() - s > 1:
                self.running = False
                break
        ret = [self.running, self.frame]
        self.frame = None
        return ret
    
    def release(self):
        self.running = False
        sleep(0.001)
        self.t.join()
    
    def update(self, frame):
        self.frame = cv2.resize(frame, self.img_size, interpolation=cv2.INTER_AREA)
    
def convertToQPixmap(img, img_shape=(640, 480)):
    try:
        img = cv2.resize(img, img_shape)
        img = QImage(img.data, img.shape[1], img.shape[0], img.strides[0], QImage.Format_RGB888)
    except:
        img = np.zeros((img_shape[1], img_shape[0], 3), dtype=np.uint8)
        img = QImage(img.data, img.shape[1], img.shape[0], img.strides[0], QImage.Format_RGB888)
    return QPixmap.fromImage(img)

def get_random_crop(image, crop_height, crop_width):
    max_x = image.shape[1] - crop_width
    max_y = image.shape[0] - crop_height
    x = np.random.randint(0, max_x)
    y = np.random.randint(0, max_y)
    return image[y: y + crop_height, x: x + crop_width]


def visualize_bboxes(img, boxes, class_names):
    for i in range(len(boxes)):
        box = boxes[i]
        x0 = np.clip(round(box[0]), 0, img.shape[1])
        y0 = np.clip(round(box[1]), 0, img.shape[0])
        x1 = np.clip(round(box[0] + box[2]), 0, img.shape[1])
        y1 = np.clip(round(box[1] + box[3]), 0, img.shape[0])

        color = (_COLORS[69] * 255).astype(np.uint8).tolist()
        text = '{}'.format(class_names[i])
        txt_color = (0, 0, 0) if np.mean(_COLORS[69]) > 0.5 else (255, 255, 255)

        txt_size = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.4, 1)[0]
        cv2.rectangle(img, (x0, y0), (x1, y1), color, 2)

        txt_bk_color = (_COLORS[69] * 255 * 0.7).astype(np.uint8).tolist()
        cv2.rectangle(
            img,
            (x0, y0 + 1),
            (x0 + txt_size[0] + 1, y0 + int(1.5*txt_size[1])),
            txt_bk_color,
            -1
        )
        cv2.putText(img, text, (x0, y0 + txt_size[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.4, txt_color, thickness=1)

