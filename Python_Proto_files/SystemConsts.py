import numpy as np

# Approximate camera intrinsics for 1080p image [PLACEHOLDER]
fx = 1000.0
fy = 1000.0
cx = 960.0   # 1920 / 2
cy = 540.0   # 1080 / 2

cameraMatrix = np.array([
    [fx,  0, cx],
    [0,  fy, cy],
    [0,   0,  1]
], dtype=np.float32)

# Distortion coefficients [PLACEHOLDER]
distCoeffs = np.array([0, 0, 0, 0, 0])

tag_size = 0.1651  # meters
half = tag_size / 2.0

objPoints = np.array([
    [-half,  half, 0],
    [ half,  half, 0],
    [ half, -half, 0],
    [-half, -half, 0]
], dtype=np.float32)

# Poses of all the tags in the arena by ID
tagPosesGlobal = {0: np.array([0, 0, 0, 0, 0, 0], dtype = np.float32)} # [PLACEHOLDER]

# Pose of the camera with respect to the robot
cameraPoseRobot = np.array([1, 1, 1, 0, 0, 0], dtype = np.float32) # [PLACEHOLDER]