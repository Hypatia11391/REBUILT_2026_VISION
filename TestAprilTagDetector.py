import numpy as np
import pupil_apriltags as apriltag
import cv2
import argparse

# Construct arguement parser and parse arguements
ap = argparse.ArgumentParser()
ap.add_argument("-i", "--image", required = True, help = "Path to the ArpilTag image.")
args = vars(ap.parse_args())

# Read the image and convert to grayscale.
print("[INFO] Loading image...")
image = cv2.imread(args["image"])
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# Define the AprilTags detector options and then detect the AprilTags in the input image
print("[INFO] detecting AprilTags...")
detector = apriltag.Detector()
results = detector.detect(gray)
print(f"[INFO] {len(results)} total AprilTags detected")

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

distCoeffs = np.zeros(5)

tag_size = 0.1651  # meters
half = tag_size / 2.0

objPoints = np.array([
    [-half,  half, 0],
    [ half,  half, 0],
    [ half, -half, 0],
    [-half, -half, 0]
], dtype=np.float32)

# Loop over the AprilTag detection results
for r in results:
	# Extract the bounding box (x, y)-coordinates for the AprilTag and convert each of the (x, y)-coordinate pairs to integers
	(ptA, ptB, ptC, ptD) = r.corners
	ptB = (int(ptB[0]), int(ptB[1]))
	ptC = (int(ptC[0]), int(ptC[1]))
	ptD = (int(ptD[0]), int(ptD[1]))
	ptA = (int(ptA[0]), int(ptA[1]))

	# Draw the bounding box of the AprilTag detection
	cv2.line(image, ptA, ptB, (0, 255, 0), 2)
	cv2.line(image, ptB, ptC, (0, 255, 0), 2)
	cv2.line(image, ptC, ptD, (0, 255, 0), 2)
	cv2.line(image, ptD, ptA, (0, 255, 0), 2)

	# Draw the center (x, y)-coordinates of the AprilTag
	(cX, cY) = (int(r.center[0]), int(r.center[1]))
	cv2.circle(image, (cX, cY), 5, (0, 0, 255), -1)
	
	# Draw the tag family on the image
	tagFamily = r.tag_family.decode("utf-8")
	tagID = r.tag_id
	print(tagID, type(tagID))
	cv2.putText(image, str(tagID), (ptA[0], ptA[1] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
	print(f"[INFO] tag family: {tagFamily}")

	# Estimate pose
	imgPoints = np.array(r.corners, dtype=np.float32)
	success, rvec, tvec = cv2.solvePnP(objPoints, imgPoints, cameraMatrix, distCoeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE)

	if success:
		print("Tag:", r.tag_id)
		print(" rvec:", rvec.ravel())
		print(" tvec:", tvec.ravel())

        # Convert to rotation matrix if needed
		R, _ = cv2.Rodrigues(rvec)
		print(" Rotation matrix:\n", R)
	else:
		print("Pose solve failed for tag:", r.tag_id)

# Show the output image after AprilTag detection
cv2.imshow("Image", image)
cv2.waitKey(0)