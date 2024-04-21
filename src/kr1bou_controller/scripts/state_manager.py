#!/usr/bin/env python3
"""
Manages different states of the robot (e.g., running, start, stop, emergency).
"""

import rospy
import serial.tools.list_ports
from std_msgs.msg import Bool


# Identify connected Arduinos
def identify_arduino_ports(known_sensors):
    ports = serial.tools.list_ports.comports()
    identified_ports = {}
    for port, desc, hwid in sorted(ports):
        ser = serial.Serial(port)  # Open the port
        ser.write('identify\n'.encode())  # Send command to get sensor ID response
        line = ser.readline()
        sensor_id = line.decode().strip()
        if sensor_id in known_sensors:
            identified_ports[sensor_id] = port
        ser.close()
    return identified_ports


def config_callback(msg):
    if msg.data:  # True when the starter key/button is pressed
        rospy.loginfo("Config phase started")

        known_sensors = rospy.get_param('/arduino/known_sensors')
        rospy.loginfo("Identifying connected Arduino sensors...")
        identified_ports = identify_arduino_ports(known_sensors)

        # Dump identified ports to ROS parameters
        for sensor_id, port in identified_ports.items():
            rospy.set_param(f'arduino/arduino_serial_ports/uS_{sensor_id}', port)

        rospy.loginfo("Configuration phase completed. Identified ports dumped to ROS parameters.")
        # Transition to the next state if necessary, e.g., publish on a 'state_change' topic

if __name__ == '__main__':
    rospy.init_node('state_manager')

    rospy.Subscriber('configPhase', Bool, config_callback)

    rospy.loginfo("State manager node initialized, awaiting config phase.")
    rospy.spin()
