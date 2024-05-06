#!/usr/bin/env python3
"""
Handles the embedded camera. Used to have accurate precision of the solar panels.
"""

import rospy
import cv2
import numpy as np
from std_msgs.msg import Bool, Int8

NEUTRAL = 0
BLUE = 1
YELLOW = -1
BOTH = 2

start = False


def det_angle(img: np.ndarray, detector: cv2.aruco.ArucoDetector):
    """
    Determines the angle of the solar panel
    """
    corners, ids, _ = detector.detectMarkers(img)

    if ids is None:
        return None

    if len(ids) > 1 or ids[0] != 47:
        return None

    # Get the corners of the marker
    x1, y1 = corners[0][0][0]
    x2, y2 = corners[0][0][1]

    rotation = np.arctan2(y2 - y1, x2 - x1) % (2 * np.pi)

    # add pi
    rotation = (rotation + np.pi) % (2 * np.pi)

    return rotation


def get_winner(img: np.ndarray, detector: cv2.aruco.ArucoDetector):
    """
    Estimates the team that captured the solar panel
    """
    angle = det_angle(img, detector)

    if angle is None:
        return None

    if 4.61 < angle < 4.81:  # -pi/2
        return NEUTRAL
    if 4.81 >= angle >= 1.77:  # -pi/2 to pi/2 -
        return YELLOW
    elif 1.77 > angle > 1.37:  # pi/2 - to pi/2 +
        return BOTH
    else:
        return BLUE
    
def enable_camera(data):
    global cam_enabled
    cam_enabled = data.data
    if cam_enabled:
        rospy.loginfo("Camera back is enabled")
    else:
        rospy.loginfo("Camera back is disabled")


def run(data):
    global start
    start = data
    rospy.loginfo(f"{rospy.get_name()} received {start} from running_phase")


if __name__ == "__main__":
    try:
        # Initialization
        rospy.init_node("camera_back")
        rospy.loginfo("[START] Camera back node has started.")

        # Load configuration parameters
        frequency = rospy.get_param("/frequency")
        queue_size = rospy.get_param("/queue_size")

        # Initialize camera
        cam = cv2.VideoCapture(0)
        arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        params = cv2.aruco.DetectorParameters()

        # Setup detector
        params.detectInvertedMarker = True
        aruco_detector = cv2.aruco.ArucoDetector(arucoDict, params)

        # Subscribers and publishers
        rate = rospy.Rate(frequency)
        cam_enabled = False
        rospy.Subscriber("running_phase", Bool, run)
        rospy.Subscriber("solar_mode", Bool, enable_camera)
        cam_back_pub = rospy.Publisher("solar_aruco", Int8, queue_size=queue_size)

        while not start:
            rate.sleep()

        while not rospy.is_shutdown() and cam_enabled:
            ret, frame = cam.read()
            if not ret:
                continue

            winner = get_winner(frame, aruco_detector)
            if winner is not None:
                cam_back_pub.publish(Int8(data=winner))
                rospy.loginfo("(CAMERA_BACK) winner : " + str(winner))
            rate.sleep()

        cam.release()
    finally:
        rospy.loginfo("[STOP] Camera back node has stopped.")
