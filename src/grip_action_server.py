#!/usr/bin/env python

import rospy
from edo.gripper_action import GripperActionServer
def start_server():
    rospy.init_node("grip_action_server")
    rospy.loginfo("Initializing grip action server...")
    server = GripperActionServer()

    rospy.loginfo("Running Grip Action Server. Ctrl-c to quit")
    server.spin()

def main():
    start_server()


if __name__ == "__main__":
    main()

