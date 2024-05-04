#!/usr/bin/env python3
"""
Manages the data of the lidar.
"""

import math

import numpy as np
import rospy
import serial
from geometry_msgs.msg import PoseArray, Pose, Pose2D
from sklearn.cluster import DBSCAN
from std_msgs.msg import Bool

from hokuyo.driver import hokuyo
from hokuyo.tools import serial_port

uart_port = rospy.get_param("/arduino/arduino_serial_ports/Lidar")
uart_speed = 19200
laser = None


def get_position(data: Pose2D):
    global x_robot, y_robot, theta_robot
    x_robot = data.x
    y_robot = data.y
    theta_robot = data.theta


def run(data: Bool):
    global start
    start = data.data


if __name__ == '__main__':
    rospy.init_node("Lidar", anonymous=True)
    rospy.loginfo("[START] Lidar node has started.")
    pub_data = rospy.Publisher("lidar_data", PoseArray, queue_size=1)
    rospy.Subscriber("odometry", Pose2D, get_position)

    laser_serial = serial.Serial(port=uart_port, baudrate=uart_speed, timeout=0.5)
    port = serial_port.SerialPort(laser_serial)

    laser = hokuyo.Hokuyo(port)
    x = []
    y = []

    x_robot = 0.0
    y_robot = 0.0
    theta_robot = 0.0
    start = False
    rospy.Subscriber("runningPhase", Bool, run)
    while not start:
        rospy.sleep(0.1)
    laser.laser_off()
    laser.laser_on()

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        dictionary = laser.get_scan()
        x = []
        y = []
        for i in range(0, len(dictionary[0])):
            if dictionary[1][i] > 150:
                x_temp = (dictionary[1][i] / 1000 * math.sin(math.radians(dictionary[0][i]) - theta_robot + math.pi / 2)
                          + x_robot)
                y_temp = (dictionary[1][i] / 1000 * math.cos(math.radians(dictionary[0][i]) - theta_robot + math.pi / 2)
                          + y_robot)
                if 0 < x_temp < 3 and 0 < y_temp < 2:
                    x.append(x_temp)
                    y.append(y_temp)

        points = np.array([x, y]).T

        # Appliquer DBSCAN sur les points. eps est la distance maximale entre deux échantillons pour qu'ils soient
        # considérés comme dans le même voisinage.
        try:
            db = DBSCAN(eps=0.06, min_samples=3).fit(points)

            labels = db.labels_

            # Pour chaque groupe, calculer le point moyen et l'ajouter à la liste des groupes.
            groups = []
            for group_id in set(labels):
                if group_id != -1:  # Ignorer le bruit
                    group_points = points[labels == group_id]
                    group_mean = group_points.mean(axis=0)
                    groups.append(group_mean)
        except Exception as e:
            print(e)
            groups = []

        PoseArray_msg = PoseArray()
        PoseArray_msg.header.stamp = rospy.Time.now()
        for group in groups:
            Pose_msg = Pose()
            Pose_msg.position.x = group[0]
            Pose_msg.position.y = group[1]
            PoseArray_msg.poses.append(Pose_msg)
        pub_data.publish(PoseArray_msg)
        rate.sleep()
