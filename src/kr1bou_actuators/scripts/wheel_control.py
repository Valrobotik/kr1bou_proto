#!/usr/bin/env python3
"""
Handles the actuators logic. Sends orders to each wheel.
"""

import rospy
import serial
from geometry_msgs.msg import Vector3, Pose2D
from std_msgs.msg import Bool, String
from math import pi


class WheelController():
    def __init__(self):
        # Retrieve the serial port parameter for the Arduino controlling the wheels
        self.serial_port_param = rospy.get_param(f'/arduino/arduino_serial_ports/Motor')
        self.baudrate = rospy.get_param('/arduino/baudrate')

        # Initialize the serial port for communication with Arduino
        self.serial_port = serial.Serial(self.serial_port_param, self.baudrate, timeout=1)
        
        # Subscribe to the motor_speed topic
        rospy.Subscriber('motor_speed', Vector3, self.motor_speed_callback)
        rospy.Subscriber('odom_corrected', Pose2D, self.correct_odometry)
        rospy.Subscriber('stop', String, self.stop)

    def motor_speed_callback(self, data):
        # Format : Vg.gg;Vd.ddR g for gauche, d for droite in m/s
        command = f"V{data.x};{data.y}R"
        self.serial_port.write(command.encode())

    def correct_odometry(self, data: Pose2D):
        # Format : Ox.xx;y.yy;t.ttR x and y in cm, t in rad
        if data.theta < 0:
            data.theta += pi + pi
        try:
            self.serial_port.write(f"O{format(data.x/100, '.2f')};{format(data.y/100, '.2f')};{format(data.theta, '.2f')}R\n".encode())
        except:
            rospy.logwarn("Error while sending correction")

    def stop(self, data):
        # Format : SR
        self.serial_port.write("SR".encode())
        

    def run(self):
        rospy.spin()

    def __del__(self):
        self.serial_port.close()


if __name__ == "__main__":
    try:
        # Initialization
        rospy.init_node('wheel_controller')
        # Wait for the runningPhase True signal
        start = rospy.wait_for_message('runningPhase', Bool)
        while not start.data:
            start = rospy.wait_for_message('runningPhase', Bool)
        wheel_controller = WheelController()
        rospy.loginfo("Wheel Controller node has started.")

        wheel_controller.run()
    except rospy.ROSInterruptException as e:
        rospy.logerr(e)
    finally:
        del wheel_controller
        rospy.loginfo("Wheel Controller node has stopped.")