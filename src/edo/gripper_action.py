#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
from math import fabs
from edo.states import EdoStates

from control_msgs.msg import (
    GripperCommandAction,
    GripperCommandFeedback,
    GripperCommandResult,
)


class GripperActionServer(object):
    def __init__(self):
        self._name_space = 'grip/gripper_action'
        # Start Action Server.
        self._server = actionlib.SimpleActionServer(
            self._name_space,
            GripperCommandAction,
            execute_cb=self._on_gripper_action,
            auto_start=False
        )
        self._action_name = rospy.get_name()
        self._server.start()
        # initialize a edo states in the action server
        self.states = EdoStates()
        # Action Feedback/Result
        self._feedback = GripperCommandFeedback()
        self._result = GripperCommandResult()
        self._timeout = 5.0
        self._action_name = rospy.get_name()
        self._update_rate_spinner = rospy.Rate(20)

    def spin(self):
        while not rospy.is_shutdown():
            self.states.update()
            self._update_rate_spinner.sleep()

    def _check_state(self, position):
        return fabs(self.states.gripper_position - position) < 5

    def _command_gripper(self, position):
        if position >= 100.0:
            self.states.change_gripper_state(True)
        else:
            self.states.change_gripper_state(False)

    def _update_feedback(self, position):
        self._feedback.position = self.states.gripper_position
        # True when the grip reach 5mm to its destination.
        self._feedback.reached_goal = (fabs(self.states.gripper_position - position) < 5)
        self._result = self._feedback
        self._server.publish_feedback(self._feedback)

    def _on_gripper_action(self, goal):
        position = goal.command.position
        control_rate = rospy.Rate(20.0)
        self._update_feedback(position)
        start_time = rospy.get_time()

        def now_from_start(start):
            return rospy.get_time() - start

        while ((now_from_start(start_time) < self._timeout or self._timeout < 0.0 and not rospy.is_shutdown())):
            if self._check_state(position):
                self._server.set_suceeded(self._result)
                return
            self._command_gripper(position)
            control_rate.sleep()
        if not rospy.is_shutdown():
            rospy.logerr("%s: Gripper Command Not Achieved in Allotted Time" %
                         (self._action_name,))
        self._update_feedback(position)
        self._server.set_aborted(self._result)
