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

def det_angle(img : np.ndarray, detector : cv2.aruco.ArucoDetector) :
    """
    Determines the angle of the solar panel
    """

    corners, ids, _ = detector.detectMarkers(img)

    if ids is None :
        return None

    if len(ids) > 1 or ids[0] != 47 :
        return None

    # Get the corners of the marker
    x1, y1 = corners[0][0][0]
    x2, y2 = corners[0][0][1]

    rotation = np.arctan2(y2 - y1, x2 - x1) % (2 * np.pi)

    # add pi
    rotation = (rotation + np.pi) % (2 * np.pi)

    return rotation

def get_winner(img : np.ndarray, detector : cv2.aruco.ArucoDetector) :
    """
    Estimates the team that captured the solar panel
    """

    angle = det_angle(img, detector)

    if angle is None :
        return None
    
    if angle > 4.61 and angle < 4.81 : # -pi/2
        return NEUTRAL
    if angle <= 4.81 and angle >= 1.77 : # -pi/2 to pi/2 -
        return YELLOW
    elif angle < 1.77 and angle > 1.37 : # pi/2 - to pi/2 +
        return BOTH
    else :
        return BLUE
    
def run(data) :
    global start
    start = data
    rospy.loginfo(f"{rospy.get_name()} received {start} from runningPhase")



if __name__ == "__main__" :
    # Initialization
    rospy.init_node("camera_front")
    rospy.loginfo("[START] Camera front node has started.")

    # Load configuration parameters
    frequency = rospy.get_param("/frequency")
    queue_size = rospy.get_param("/queue_size")

    # Initialize camera
    cam = cv2.VideoCapture(0)
    arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    params = cv2.aruco.DetectorParameters()
    # Make detector more lenient
    params.detectInvertedMarker = True
    detector = cv2.aruco.ArucoDetector(arucoDict, params)

    rate = rospy.Rate(frequency)
    rospy.Subscriber("runningPhase", Bool, run)

    while not start :
        rate.sleep()

    cam_front_pub = rospy.Publisher("camera_front", Int8, queue_size=queue_size)

    while not rospy.is_shutdown() :
        ret, frame = cam.read()
        if not ret :
            continue

        winner = get_winner(frame, detector)
        if winner is not None :
            cam_front_pub.publish(Int8(data=winner))
            rospy.loginfo("[CAMERA_FRONT] winner : " + str(winner))
        rate.sleep()

    cam.release()