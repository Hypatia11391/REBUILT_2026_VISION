import numpy as np
import cv2
import pupil_apriltags as aptag
import argparse
import VisionUtills
import SystemConsts

ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video", help = "Path to the optional video file")
ap.add_argument("-t", "--test", )
args = vars(ap.parse_args())

# Set up video feed from the camera, or from passed video
if args["video"] is not None:
    videoCapture = cv2.VideoCapture(args["video"])

    if not videoCapture.isOpened():
        print(f"Error: could not open video file at {args['video']}")
        exit
else:
    pass

# Set up apriltag detector
detector = aptag.Detector()

# Start looping over frames.
# Loop through the video frames
while True:
    # Read a frame from the video
    ret, frame = videoCapture.read()

    # Break the loop if the frame was not read (end of video)
    if not ret:
        print("End of video or error reading frame.")
        continue

    # Break the loop if 'q' is pressed
    if cv2.waitKey(25) & 0xFF == ord('q'):
        break

    # Convert to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect apriltags in the image
    Tags = detector.detect(gray, estimate_tag_pose = True,
                           camera_params = [SystemConsts.fx, SystemConsts.fy, SystemConsts.cx, SystemConsts.cy],
                           tag_size = SystemConsts.tag_size
                           )
    
    if args["test"] is not None:
        for tag in Tags:
            # Extract the bounding box (x, y)-coordinates for the AprilTag and convert each of the (x, y)-coordinate pairs to integers
            (ptA, ptB, ptC, ptD) = r.corners
            ptB = (int(ptB[0]), int(ptB[1]))
            ptC = (int(ptC[0]), int(ptC[1]))
            ptD = (int(ptD[0]), int(ptD[1]))
            ptA = (int(ptA[0]), int(ptA[1]))

            # Draw the bounding box of the AprilTag detection
            cv2.line(frame, ptA, ptB, (0, 255, 0), 2)
            cv2.line(frame, ptB, ptC, (0, 255, 0), 2)
            cv2.line(frame, ptC, ptD, (0, 255, 0), 2)
            cv2.line(frame, ptD, ptA, (0, 255, 0), 2)

            # Draw the center (x, y)-coordinates of the AprilTag
            (cX, cY) = (int(r.center[0]), int(r.center[1]))
            cv2.circle(frame, (cX, cY), 5, (0, 0, 255), -1)

            # Draw the tag family on the image
            tagFamily = tag.tag_family.decode("utf-8")
            tagID = tag.tag_id
            cv2.putText(frame, "ID:" + str(tagID), (ptA[0], ptA[1] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cv2.putText(frame, "Pose_R" + str(tag.pose_R), (ptA[0], ptA[1] + 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cv2.putText(frame, "Pose_t" + str(tag.pose_t), (ptA[0], ptA[1] + 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            print(f"[INFO] tag family: {tagFamily}")

    # Convert each apriltag pose to a robot pose in global frame. (Use the apriltag poses by id stored in SystemConsts.tagPoses)

    # Reject ridiculous poses (outside the arena, in the air, etc.)

    # Send to RoboRio

    # Some method to break the loop.

# Clean up Feed.