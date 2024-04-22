#!/usr/bin/env python3
"""
Manages different states of the robot (e.g., running, start, stop, emergency).
"""
import rospy
import serial.tools.list_ports
from std_msgs.msg import Bool

queue_size = rospy.get_param('/queue_size')


# Identify connected Arduinos
def identify_arduino_ports(known_sensors):
    ports = serial.tools.list_ports.comports()
    identified_ports = {}
    list_port = [port for port, desc, hwid in sorted(ports)]
    rospy.loginfo(f"list of ports : {list_port}")
    for port, desc, hwid in sorted(ports):
        # rospy.loginfo("coucou444")
        rospy.loginfo(port)
        if 'ACM' in port or 'USB' in port:
            try :
                ser = serial.Serial(port, 115200)  # Open the port
            except Exception as e:
                rospy.loginfo(e)
            ser.write('NR\n'.encode())  # Send command to get sensor ID response
            rospy.loginfo("coucou")
            line = ser.readline()
            sensor_id = line.decode().strip()
            for known_sensor in known_sensors:
                if sensor_id in known_sensor:
                    rospy.loginfo(f"sensor {sensor_id} connected on {port}")
                    identified_ports[sensor_id] = port
            ser.close()
            rospy.sleep(0.1)
        elif len(known_sensors) == len(identified_ports):
            break
    if not identified_ports:
        rospy.logerr("No port detected")
    return identified_ports


def config_callback(msg: Bool):
    if msg.data:  # We want the key to be released
        rospy.loginfo("Config phase started")

        known_sensors = rospy.get_param('/arduino/known_sensors')
        rospy.loginfo("Identifying connected Arduino sensors...")
        identified_ports = identify_arduino_ports(known_sensors)

        # Dump identified ports to ROS parameters
        for sensor_id, port in identified_ports.items():
            rospy.set_param(f'/arduino/arduino_serial_ports/{sensor_id}', port)
            rospy.loginfo(f'/arduino/arduino_serial_ports/{sensor_id} : {port}')

        rospy.loginfo("Configuration phase completed. Identified ports dumped to ROS parameters.")
        # Transition to the next state via publisher
        pub = rospy.Publisher('configPhase', Bool, queue_size=queue_size)
        rospy.sleep(0.2)
        pub.publish(False)
        # pub.unregister()
        pubrunning = rospy.Publisher('runningPhase', Bool, queue_size=queue_size)
        rospy.sleep(0.2)
        pubrunning.publish(True)
        # pubrunning.unregister()
        rospy.loginfo("Published on runningPhase topic")


def running_callback(msg):
    # TODO : Implement the running phase logic, if any
    pass


def stop_callback(msg):
    # TODO : Implement the stop logic. Send stop data to the actuator nodes ?
    pass


def emergency_callback(msg):
    # TODO : Implement the emergency stop logic. Send emergency stop data to the actuator nodes ? / Stop all nodes ?
    pass


if __name__ == '__main__':
    rospy.init_node('state_manager')
    rospy.loginfo("[START] State Manager node has started.")

    rospy.Subscriber('configPhase', Bool, config_callback)
    rospy.Subscriber('runningPhase', Bool, running_callback)
    rospy.Subscriber('stop', Bool, stop_callback)
    rospy.Subscriber('emergencyStop', Bool, emergency_callback)

    rospy.spin()
