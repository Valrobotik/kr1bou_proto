#!/usr/bin/env python3
"""
Manages the data of the 8 ultrasound sensors.
"""
import rospy
import serial
import math
from std_msgs.msg import Float32MultiArray, Bool
from geometry_msgs.msg import Pose2D


def clamp_sensor_data(raw_data: float, sensor_position: tuple, map_boundaries: tuple):
    if current_pose is None:
        rospy.logwarn("Current pose not yet received.")
        return raw_data  # Return the unmodified data if no pose received
    
    # Extract the robot's yaw angle from its orientation
    robot_yaw = current_pose.theta
    
    # Calculate the sensor's absolute orientation by adding its relative angle to the robot's yaw
    sensor_absolute_angle = robot_yaw + sensor_position[3]

    # Calculate the sensor's absolute position on the map
    sensor_x_absolute = current_pose.x + sensor_position[0] * math.cos(sensor_absolute_angle) - sensor_position[1] * math.sin(sensor_absolute_angle)
    sensor_y_absolute = current_pose.y + sensor_position[0] * math.sin(sensor_absolute_angle) + sensor_position[1] * math.cos(sensor_absolute_angle)

    # Calculate the x, y coordinates of the intersection between the sensor's line of sight and the map boundaries
    if sensor_absolute_angle == 0 or sensor_absolute_angle == math.pi:
        x_intersection = sensor_x_absolute
        y_intersection = map_boundaries[2] if sensor_absolute_angle == 0 else map_boundaries[0]
    else:
        slope = math.tan(sensor_absolute_angle)
        x_intersection = (map_boundaries[1] - sensor_y_absolute + slope * sensor_x_absolute) / slope
        y_intersection = slope * x_intersection + sensor_y_absolute

    # Calculate the distance between the sensor and the intersection point
    distance = math.sqrt((sensor_x_absolute - x_intersection)**2 + (sensor_y_absolute - y_intersection)**2)
    return min(raw_data, distance)    


def read_and_publish_sensor_data():
    rate = rospy.Rate(frequency)
    while not rospy.is_shutdown():
        if serial_port.in_waiting:  # If there is data to read
            raw_data = serial_port.readline()   # Read
            try:
                sensor_readings = [float(x) for x in raw_data.decode('utf-8').strip('[]\n').split('; ')]    # Parse
                clamped_readings = [
                    clamp_sensor_data(reading, pos, map_boundaries) for reading, pos in zip(sensor_readings, sensor_positions) # Clamp
                ]
                sensor_data_pub.publish(Float32MultiArray(data=clamped_readings)) # Publish
            except ValueError:
                rospy.logwarn('Received malformed data from Arduino.')
        rate.sleep()


start = False
def run(data):
    global start
    start = data
    rospy.loginfo(f"Received {start} from runningPhase")

# Manage robot's pose
current_pose = None
def pose_callback(pose_msg:Pose2D):
    global current_pose
    current_pose = pose_msg
    rospy.loginfo(f"Received {current_pose} from Pose")


if __name__ == '__main__':
    # Initialization
    rospy.init_node('ultrasound_sensor_manager')
    rospy.loginfo("[START] Ultrasound Sensor node has started.")

    # Load configuration parameters
    frequency = rospy.get_param('/frequency')
    queue_size = rospy.get_param('/queue_size')
    baudrate = rospy.get_param('/arduino/baudrate')
    map_boundaries = rospy.get_param('/map_boundaries')  # (x_min, y_min, x_max, y_max)

    rate = rospy.Rate(frequency)
    # Wait for the runningPhase True signal
    rospy.Subscriber('runningPhase', Bool, run)
    while not start:
        rate.sleep()
    
    # Load sensor parameters
    sensor_positions = rospy.get_param('/sensor_positions/uS')  # [(x, y, z, angle), ...]. Angle is in radians
    serial_port_param = rospy.get_param(f'/arduino/arduino_serial_ports/uS')
    serial_port = serial.Serial(serial_port_param, baudrate, timeout=1)

    # Publisher and Subscriber
    sensor_data_pub = rospy.Publisher('ultrasound_sensor_data', Float32MultiArray, queue_size=queue_size)
    rospy.Subscriber('pose', Pose2D, pose_callback)

    try:
        read_and_publish_sensor_data()
    except rospy.ROSInterruptException:
        pass
    finally:
        serial_port.close()
