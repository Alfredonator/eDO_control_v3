#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Basic joint states publisher for real eDo robot
# The source node usually publishes at 90Hz, hence this node

import rospy
from sensor_msgs.msg import JointState
from edo_core_msgs.msg import JointStateArray
from edo.states import ordered_joint_names
from edo.gripper_states import grip_joints, transform_joint_7_to_grip_states
from rospy import ROSException

joints_names = ordered_joint_names[:-1] + grip_joints
js = JointState(name=joints_names)
js_publisher = rospy.Publisher('joint_states', JointState, queue_size=5)

def jsa_to_js(jsa):
    # arm positions and velocity
    position = [joint.position * 0.01745 for joint in jsa.joints][:-1] # Transform Deg to Rad
    velocity = [joint.velocity * 0.01745 for joint in jsa.joints][:-1] # Transform Deg to Rad
    effort = [joint.current for joint in jsa.joints][:-1]              # TODO Approximate conversion motor_current => effort?
    # grip position and velocity
    jnt_7 = [joint for joint in jsa.joints][-1:]
    jnt_7 = jnt_7[0]
    (base_p, tip_p) = transform_joint_7_to_grip_states(jnt_7.position * 0.01745) 

    ## add position and velocity to griper base and tip joints.
    position.extend([base_p, base_p, tip_p, tip_p])
    velocity.extend([0, 0, 0, 0])
    effort.extend([0, 0, 0, 0])
    return (position, velocity, effort)
    
            

def js_callback(jsa):
    if jsa.joints_mask == 127:
        js.header.stamp = rospy.Time.now()
        (position, velocity, effort) = jsa_to_js(jsa)
        js.position = position 
        js.velocity = velocity
        js.effort = effort
        try:
            js_publisher.publish(js)
        except ROSException:
            pass
    elif jsa.joints_mask < 9999999999999:  # Huge number pops when robot isn't ready
        raise NotImplementedError("Joint State publisher for real robot does not know edo joints mask {}".format(jsa.joints_mask))

rospy.init_node('joint_state_publisher')

rospy.Subscriber("usb_jnt_state", JointStateArray, js_callback)
rospy.loginfo("Starting eDo joint state publisher for real robot...")
rospy.spin()
