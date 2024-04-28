#!/usr/bin/env python3
"""
Manages the data of the 8 ultrasound sensors.
"""
import rospy
import serial
import math
from std_msgs.msg import Bool, Float32MultiArray
from geometry_msgs.msg import Pose2D
from typing import Tuple


def clamp_sensor_data(raw_data: float, sensor_position: tuple) -> Tuple[float, float]:
    # Extract the robot's yaw angle from its orientation
    robot_yaw = current_pose.theta

    # Calculate the sensor's absolute orientation by adding its relative angle to the robot's yaw
    sensor_absolute_angle = robot_yaw + sensor_position[3]

    # Calculate the sensor's absolute position on the map
    sensor_x_absolute = (current_pose.x + sensor_position[0] * math.cos(sensor_absolute_angle) - sensor_position[1] *
                         math.sin(sensor_absolute_angle))
    sensor_y_absolute = (current_pose.y + sensor_position[0] * math.sin(sensor_absolute_angle) + sensor_position[1] *
                         math.cos(sensor_absolute_angle))

    # Calculate the x, y coordinates of the intersection between the sensor's line of sight and the map boundaries
    if sensor_absolute_angle == 0 or sensor_absolute_angle == math.pi:
        x_intersection = sensor_x_absolute
        y_intersection = map_boundaries[2] if sensor_absolute_angle == 0 else map_boundaries[0]
    else:
        slope = math.tan(sensor_absolute_angle)
        x_intersection = (map_boundaries[1] - sensor_y_absolute + slope * sensor_x_absolute) / slope
        y_intersection = slope * x_intersection + sensor_y_absolute

    # Calculate the distance between the sensor and the intersection point
    distance = math.sqrt((sensor_x_absolute - x_intersection) ** 2 + (sensor_y_absolute - y_intersection) ** 2)
    if distance < raw_data:
        return -1, -1

    # Else return the absolute coordinates of the raw data
    if sensor_absolute_angle == 0:
        x_obstacle = sensor_x_absolute + raw_data
        y_obstacle = sensor_y_absolute  # No change in y
    elif sensor_absolute_angle == math.pi:
        x_obstacle = sensor_x_absolute - raw_data
        y_obstacle = sensor_y_absolute
    else:
        x_obstacle = sensor_x_absolute + raw_data * math.cos(sensor_absolute_angle)
        y_obstacle = sensor_y_absolute + raw_data * math.sin(sensor_absolute_angle)
    return x_obstacle, y_obstacle


def read_and_publish_sensor_data():
    while not rospy.is_shutdown():
        if serial_port.in_waiting:  # If there is data to read
            raw_data = serial_port.readline()  # Read
            try:
                sensor_readings = [float(x) for x in
                                   raw_data.decode('utf-8').replace('\r\n', '').replace('b', '')
                                   .replace("'", '').strip('[]').split('; ')]  # Parse
                clamped_readings = [
                    clamp_sensor_data(reading, pos) for reading, pos in
                    zip(sensor_readings, sensor_positions)  # Clamp
                ]
                # Flatten the list of tuples
                sensor_data_pub.publish(Float32MultiArray(data=[item for sublist in clamped_readings
                                                                for item in sublist]))
                rospy.loginfo(f" US DATA : {[item for sublist in clamped_readings for item in sublist]}")
            except ValueError:
                rospy.logwarn(raw_data)
                rospy.logwarn('{rospy.get_name()} received malformed data from US Arduino.')
        rate.sleep()


def run(data):
    global start
    start = data
    rospy.loginfo(f"{rospy.get_name()} received: {data.data} from RunningPhase")


def pose_callback(pose_msg: Pose2D):
    global current_pose
    current_pose = pose_msg
    # rospy.loginfo(f"{rospy.get_name()} received {current_pose} from Pose")


if __name__ == '__main__':
    # Initialization
    rospy.init_node('ultrasound_sensor_manager')
    rospy.loginfo("[START] Ultrasound Sensor node has started.")

    start = False
    # Wait for the runningPhase True signal
    rospy.Subscriber('runningPhase', Bool, run)
    while not start:
        rospy.sleep(0.1)

    # Manage robot's pose
    current_pose = Pose2D()

    # Load configuration parameters
    frequency = rospy.get_param('/frequency')
    queue_size = rospy.get_param('/queue_size')
    baudrate = rospy.get_param('/arduino/baudrate')
    map_boundaries = rospy.get_param('/map_boundaries')  # (x_min, y_min, x_max, y_max)
    rate = rospy.Rate(frequency)

    # Load sensor parameters
    sensor_positions = rospy.get_param('/sensor_positions')  # [(x, y, z, angle), ...]. Angle is in radians
    serial_port_param = rospy.get_param(f'/arduino/arduino_serial_ports/US')
    serial_port = serial.Serial(serial_port_param, baudrate, timeout=1)
    #serial_port = None

    # Publisher and Subscriber
    sensor_data_pub = rospy.Publisher('ultrasound_sensor_data', Float32MultiArray, queue_size=queue_size)
    rospy.Subscriber('odometry', Pose2D, pose_callback)

    try:
        read_and_publish_sensor_data()
        pass
    except rospy.ROSInterruptException:
        pass
    finally:
        if serial_port is not None:
            serial_port.close()
