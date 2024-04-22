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
        self.rate = rospy.Rate(30)

        # Initialize the serial port for communication with Arduino
        self.serial_port = serial.Serial(self.serial_port_param, self.baudrate, timeout=1)

        # Publish the received odometry
        self.publisher_odometry = rospy.Publisher('odometry', Pose2D, queue_size=1)

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
        rospy.loginfo(f"O{format(data.x/100, '.2f')};{format(data.y/100, '.2f')};{format(data.theta, '.2f')}R\n")
        try:
            self.serial_port.write(f"O{format(data.x/100, '.2f')};{format(data.y/100, '.2f')};{format(data.theta, '.2f')}R\n".encode())
        except:
            rospy.logwarn("Error while sending correction")

    def receive_odometry(self):  
        if self.serial_port.in_waiting > 0:
            data = str(self.serial_port.read_until(b'R')).replace('b', '').replace("'", '').replace('\\r\\n', '').replace('R', '')
            self.serial_port.reset_input_buffer()
            data = data.split(';')
            rospy.loginfo(data)
            position = Pose2D(float(data[0]), float(data[1]), float(data[2]))
            self.publisher_odometry.publish(position)
            # rospy.loginfo(f"Received ({position}) from arduino")


    def stop(self, data):
        # Format : SR
        self.serial_port.write("SR".encode())
        

    def run(self):
        while not rospy.is_shutdown():
            self.receive_odometry()
            self.rate.sleep()

    def close(self):
        self.serial_port.close()



start = False
def run(data:Bool):
    global start
    start = data.data
    rospy.loginfo(f"Received {start} from runningPhase")

if __name__ == "__main__":
    try:
        # Initialization
        rospy.init_node('wheel_controller')
        rospy.loginfo("[START] Wheel Controller node has started.")
        # Wait for the runningPhase True signal
        rate = rospy.Rate(rospy.get_param('/frequency'))
        rospy.Subscriber('runningPhase', Bool, run)
        while not start:
            rate.sleep()
        wheel_controller = WheelController()
        wheel_controller.run()
    except rospy.ROSInterruptException as e:
        pass
    finally:
        wheel_controller.close()
        rospy.loginfo("Wheel Controller node has stopped.")