#!/usr/bin/env python3
"""
Manages different states of the robot (e.g., running, start, stop, emergency).
"""
import rospy
# import serial.tools.list_ports
from std_msgs.msg import Bool

queue_size = rospy.get_param('/queue_size')


# Identify connected Arduino sensors
def identify_arduino_ports(known_sensors):
    identified_ports = {known_sensors[0]: "/dev/ttyACM0", known_sensors[1]: "/dev/ttyUSB1"}
    return identified_ports
    # ports = serial.tools.list_ports.comports()
    # list_port = [port for port, desc, hwid in sorted(ports)]
    # rospy.loginfo(f"list of ports : {list_port}")
    # for port, desc, hwid in sorted(ports):
    #     # rospy.loginfo("coucou444")
    #     rospy.loginfo(port)
    #     if ('USB' in port or 'ACM' in port):
    #         try :
    #             ser = serial.Serial(port, 115200, timeout=1)  # Open the port
    #             rospy.sleep(0.2)
    #         except Exception as e:
    #             rospy.loginfo(e)
    #         ser.write(b'NR\n')  # Send command to get sensor ID response
    #         rospy.loginfo("coucou")
    #         while(ser.in_waiting < 1) : pass
    #         line = ser.readline()
    #         rospy.loginfo(line)
    #         sensor_id = line.decode().strip()
    #         if sensor_id == "": 
    #             rospy.loginfo(f"no data on port : {port}")
    #         else:
    #             for known_sensor in known_sensors:
    #                 if sensor_id in known_sensor:
    #                     rospy.loginfo(f"sensor {sensor_id} connected on {port}")
    #                     identified_ports[sensor_id] = port
    #             ser.close()
    #             rospy.sleep(0.1)
    #         if len(known_sensors) == len(identified_ports):
    #             break
    # if not identified_ports:
    #     rospy.logerr("No port detected")
    # return identified_ports


def config_callback(msg: Bool):
    if msg.data:  # We want the key to be released
        rospy.loginfo("(STATE MANAGER) Config phase started")

        rospy.loginfo("(STATE MANAGER) Configuration phase completed. Identified ports dumped to ROS parameters.")
        # Transition to the next state via publisher
        pub = rospy.Publisher('configPhase', Bool, queue_size=queue_size)
        rospy.sleep(0.2)
        pub.publish(False)
        # pub.unregister()
        pub_running = rospy.Publisher('runningPhase', Bool, queue_size=queue_size)
        rospy.sleep(0.2)
        pub_running.publish(True)
        # pub_running.unregister()
        rospy.loginfo("(STATE MANAGER) Published on runningPhase topic")


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
    # rospy.init_node('state_manager')
    # rospy.loginfo("[START] State Manager node has started.")

    # rospy.Subscriber('configPhase', Bool, config_callback)
    # rospy.Subscriber('runningPhase', Bool, running_callback)
    # rospy.Subscriber('stop', Bool, stop_callback)
    # rospy.Subscriber('emergencyStop', Bool, emergency_callback)


    # known_sensors = rospy.get_param('/arduino/known_sensors')
    # rospy.loginfo("(STATE MANAGER) Identifying connected Arduino sensors...")
    # identified_ports = identify_arduino_ports(known_sensors)

    # # Dump identified ports to ROS parameters
    # for sensor_id, port in identified_ports.items():
    #     rospy.set_param(f'/arduino/arduino_serial_ports/{sensor_id}', port)
    #     rospy.loginfo(f'/arduino/arduino_serial_ports/{sensor_id} : {port}')
    # rospy.spin()
    pass