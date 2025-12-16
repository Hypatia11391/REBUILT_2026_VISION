import numpy as np
import cv2
import pupil_apriltags

def make_transform(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    """
    Construct a 4x4 homogeneous transform from rotation and translation.

    R: (3,3) rotation matrix
    t: (3,) or (3,1) translation vector
    """
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t.reshape(3)
    return T

def invert_transform(T: np.ndarray) -> np.ndarray:
    """
    Invert a homogeneous transform.
    """
    R = T[:3, :3]
    t = T[:3, 3]

    T_inv = np.eye(4)
    T_inv[:3, :3] = R.T
    T_inv[:3, 3] = -R.T @ t
    return T_inv