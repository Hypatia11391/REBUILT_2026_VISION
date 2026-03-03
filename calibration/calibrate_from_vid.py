import numpy as np
import cv2

video_path = "./calibration0.mp4"

cap = cv2.VideoCapture(video_path) # Edit this to be the name of the calibration video.

if not cap.isOpened():
    print("Error while opening the video file. Check video file is at" + video_path)

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((7*7,3), np.float32)
objp[:,:2] = np.mgrid[0:7,0:7].T.reshape(-1,2)

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

i = 0

while cap.isOpened():
    if not i % 10 == 0:
        i += 1
        continue

    ret, frame = cap.read()

    if not ret:
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, (7,7), None)

    # If found, add object points, image points (after refining them)
    if ret:
        objpoints.append(objp)

        corners2 = cv2.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners2)

        # Draw and display the corners
        cv2.drawChessboardCorners(frame, (7,7), corners2, ret)
    
    cv2.imshow('Frame', frame)

    cv2.waitKey(1)

    i += 1

cv2.destroyAllWindows()

print(f"{i/10} frames captured")

print("[INFO] Points captured, calibrating ...")

ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

if ret is not None:
    print("Calibration successful!")
    print(" ")

    print(f"retVal: {ret}")
    
    print("Distortion matrix:")
    print(mtx)

    print("Distortion coeficients:")
    print(dist)

else:
    print("Calibration failed")