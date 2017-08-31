#!/usr/bin/env python

import rospy


class StopStates(object):
    NORMAL = 0
    FULL_STOP = 1
    IGNORE_STOP_SIGNS = 2


class StopSign(object):

    def __init__(self):
        self.state = StopStates.NORMAL
        self.stopDuration = 2
        self.ignoreDuration = 4

    def stopSignDetected(self):
        self.state = StopStates.FULL_STOP
        timer = rospy.Timer(rospy.Duration(self.stopDuration),
                            self.stepStateMachine,
                            oneshot=True)

    def stepStateMachine(self, event):
        if self.state is StopStates.NORMAL:
            self.state = StopStates.FULL_STOP

        elif self.state is StopStates.FULL_STOP:
            self.state = StopStates.IGNORE_STOP_SIGNS
            timer = rospy.Timer(rospy.Duration(self.ignoreDuration),
                                self.stepStateMachine,
                                oneshot=True)

        elif self.state is StopStates.IGNORE_STOP_SIGNS:
            self.state = StopStates.NORMAL
