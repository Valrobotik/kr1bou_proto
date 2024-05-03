#!/usr/bin/env python3
"""
Handles the actuators logic. Sends orders to each wheel.
"""

import rospy
import serial
from geometry_msgs.msg import Vector3, Pose2D
from std_msgs.msg import Bool, String


class WheelController:
    def __init__(self):
        # Retrieve the serial port parameter for the Arduino controlling the wheels

        self.serial_port_param = rospy.get_param(f'/arduino/arduino_serial_ports/Motor')
        self.baudrate = rospy.get_param('/arduino/baudrate')
        self.rate = rospy.Rate(30)
        self.list_command_to_send = []

        # Initialize the serial port for communication with Arduino
        self.serial_port = serial.Serial(self.serial_port_param, self.baudrate, timeout=1)
        rospy.sleep(0.1)

        # Publish the received odometry
        self.publisher_odometry = rospy.Publisher('odometry', Pose2D, queue_size=1)

        # Subscribe to the motor_speed topic
        rospy.Subscriber('motor_speed', Vector3, self.motor_speed_callback)
        rospy.Subscriber('odom_corrected', Pose2D, self.correct_odometry)
        rospy.Subscriber('stop', String, self.stop)

    def motor_speed_callback(self, data):
        # Format : Vg.gg;Vd.ddR g for 'gauche', d for 'droite' in m/s
        self.list_command_to_send.append(f"V{format(data.x, '.2f')};{format(data.y, '.2f')}R")
        # rospy.loginfo(f"speed cmd receive : {self.list_command_to_send}")

    def correct_odometry(self, data: Pose2D):
        # Format : Ox.xx;y.yy;t.ttR x and y in cm, t in rad
        rospy.loginfo(f"(WHEEL CONTROL) O{format(data.x / 100, '.2f')};{format(data.y / 100, '.2f')};{format(data.theta, '.2f')}R\n")
        self.list_command_to_send.append(
            f"O{format(data.x / 100, '.2f')};{format(data.y / 100, '.2f')};{format(data.theta, '.2f')}R\n".encode())

    def receive_odometry(self):
        if self.serial_port.in_waiting > 0:
            data = str(self.serial_port.read_until(b'R')).replace('b', '').replace("'", '').replace('\\r\\n',
                                                                                                    '').replace('R', '')
            self.serial_port.reset_input_buffer()
            data = data.split(';')
            # rospy.loginfo(data)
            position = Pose2D(float(data[0]), float(data[1]), float(data[2]))
            self.publisher_odometry.publish(position)
            # rospy.loginfo(f"{rospy.get_name()} received ({position}) from arduino")

    def stop(self):
        # Format : SR
        self.list_command_to_send.append("SR")

    def run(self):
        while not rospy.is_shutdown():
            self.receive_odometry()
            while len(self.list_command_to_send) > 0:
                try:
                    str_ = ""
                    for i in self.list_command_to_send:
                        str_ += i
                        self.list_command_to_send.pop(self.list_command_to_send.index(i))
                    self.serial_port.write(str_.encode())
                    # rospy.loginfo(f"(WHEEL CONTROL) send_data : {str}")
                except serial.SerialException as s:
                    rospy.logwarn("(WHEEL CONTROL) Error while sending correction to Arduino.")
                    rospy.logwarn(s)
            self.rate.sleep()

    def close(self):
        self.serial_port.close()
        rospy.loginfo("(WHEEL CONTROL) Serial port closed.")


def run(data: Bool):
    global start
    start = data.data
    rospy.loginfo(f"{rospy.get_name()} received: {data.data} from RunningPhase")


if __name__ == "__main__":
    start = False
    wheel_controller = None
    try:
        # Initialization
        rospy.init_node('wheel_controller')
        rospy.loginfo("[START] Wheel Controller node has started.")
        # Wait for the runningPhase True signal
        rospy.sleep(2)
        wheel_controller = WheelController()
        rate = rospy.Rate(rospy.get_param('/frequency'))
        wheel_controller.run()
    finally:
        if wheel_controller is not None:
            wheel_controller.close()
        rospy.loginfo("Wheel Controller node has stopped.")
