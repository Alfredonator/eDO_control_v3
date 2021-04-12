#! /usr/bin/env python
# -*- coding: utf-8 -*-

from sensor_msgs.msg import JointState
import math

# Calculates the angles of the gripper giving the span in mm.
# This can transform the position of the grasp to the fake urdf. edo_joint_7 gives the position between 0 closed 1 open. And we know that the maximum span is 60mm.
def _joints_position_rad(span):
    def clamp(n):
        return max(min(0.5, n), -0.5)

    base_angle = -0.7428 * span + 29.72
    tip_angle = base_angle * (-1)
    tip_angle = clamp(tip_angle / 180 * math.pi)
    base_angle = clamp(base_angle / 180 * math.pi)
    # if span is 0 it can create a radian bigger than abs(0.5) and will create a collision on moveit. We force maximum angles to be 0.5
    return (base_angle, tip_angle)

def transform_joint_7_to_grip_states(grip_state):
    open_span = 60 #grip open 60mm when fully open, grip_state == 1.
    current_grip_span_mm = grip_state * open_span
    (base_rad, tip_rad) = _joints_position_rad(current_grip_span_mm)
    return (base_rad, tip_rad)   
  
  
grip_joints = ["edo_gripper_left_base_joint", "edo_gripper_right_base_joint", "edo_gripper_left_finger_joint", "edo_gripper_right_finger_joint"]

