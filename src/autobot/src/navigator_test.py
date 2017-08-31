#!/usr/bin/env python
import unittest
import mock
from autobot.msg import detected_object
from autobot.msg import wall_dist
from navigator import *


def fake_stopCar():
    return True


def fake_togglePathFinder(state):
    return


def fake_setWallDist(dist, wall):
    return


class NavigatorTest(unittest.TestCase):
    @mock.patch('navigator.setWallDist',
                side_effect=fake_setWallDist)
    @mock.patch('navigator.togglePathFinder',
                side_effect=fake_togglePathFinder)
    @mock.patch('navigator.stopCar', side_effect=fake_stopCar)
    def testPersonInFront(self, fake_stopCar,
                          fake_togglePathFinder,
                          fake_setWallDist):
        global OBJECT_MAP
        global PATH_STATE
        OBJECT_MAP.addToMap('person', 10, 50, 1.2)
        OBJECT_MAP.addToMap('cat', 10, 50, 60)
        OBJECT_MAP.addToMap('bat', 10, 50, 65)
        PATH_STATE.enabled = True
        onDecisionInterval(None)
        fake_setWallDist.assert_not_called()
        fake_stopCar.assert_called()

    @mock.patch('navigator.setWallDist',
                side_effect=fake_setWallDist)
    @mock.patch('navigator.togglePathFinder',
                side_effect=fake_togglePathFinder)
    @mock.patch('navigator.stopCar', side_effect=fake_stopCar)
    def testMoveCarAwayFromDoor(self, fake_stopCar,
                                fake_togglePathFinder,
                                fake_setWallDist):
        global OBJECT_MAP
        global PATH_STATE
        # OBJECT_MAP.addToMap('person', 10, 50, 1.2)
        OBJECT_MAP.addToMap('door', 10, 50, 20)
        OBJECT_MAP.addToMap('door', 10, 50, 15)
        PATH_STATE.enabled = True
        PATH_STATE.wallToWatch = wall_dist.WALL_LEFT
        onDecisionInterval(None)
        fake_setWallDist.assert_called_with(2.5, wall_dist.WALL_LEFT)
        fake_stopCar.assert_not_called()

    @mock.patch('navigator.setWallDist', side_effect=fake_setWallDist)
    @mock.patch('navigator.togglePathFinder',
                side_effect=fake_togglePathFinder)
    @mock.patch('navigator.stopCar', side_effect=fake_stopCar)
    def testStayOnWall(self, fake_stopCar,
                       fake_togglePathFinder,
                       fake_setWallDist):
        global OBJECT_MAP
        global PATH_STATE
        OBJECT_MAP.addToMap('door', 10, 50, 20)
        OBJECT_MAP.addToMap('door', 10, 50, 15)
        PATH_STATE.enabled = True
        PATH_STATE.wallToWatch = wall_dist.WALL_RIGHT
        onDecisionInterval(None)
        fake_setWallDist.assert_called_with(PATH_STATE.desiredTrajectory,
                                            PATH_STATE.wallToWatch)
        fake_stopCar.assert_not_called()

    @mock.patch('navigator.setWallDist', side_effect=fake_setWallDist)
    @mock.patch('navigator.togglePathFinder',
                side_effect=fake_togglePathFinder)
    @mock.patch('navigator.stopCar', side_effect=fake_stopCar)
    @mock.patch('rospy.Timer')
    def testStopSign(self,
                     mock_timer,
                     fake_stopCar,
                     fake_togglePathFinder,
                     fake_setWallDist):
        global OBJECT_MAP
        global PATH_STATE
        global STOP_LOGIC
        PATH_STATE.enabled = True
        PATH_STATE.wallToWatch = wall_dist.WALL_RIGHT

        # Stop sign detected. Stop.
        OBJECT_MAP.addToMap('stop sign', 10, 50, 0.8)
        OBJECT_MAP.addToMap('door', 10, 50, 15)
        fake_stopCar.reset_mock()
        fake_togglePathFinder.reset_mock()
        self.assertEqual(STOP_LOGIC.state, StopStates.NORMAL)
        onDecisionInterval(None)
        fake_stopCar.assert_called()

        # Stay stopped for a bit.
        OBJECT_MAP.addToMap('stop sign', 10, 50, 0.8)
        OBJECT_MAP.addToMap('door', 10, 50, 15)
        fake_stopCar.reset_mock()
        fake_togglePathFinder.reset_mock()
        self.assertEqual(STOP_LOGIC.state, StopStates.FULL_STOP)
        onDecisionInterval(None)
        fake_togglePathFinder.assert_not_called()

        # Continue moving and ignore stop signs
        print 'continue moving and ignore stop signs'
        OBJECT_MAP.addToMap('stop sign', 10, 50, 0.8)
        OBJECT_MAP.addToMap('door', 10, 50, 15)
        STOP_LOGIC.state = StopStates.IGNORE_STOP_SIGNS
        fake_stopCar.reset_mock()
        self.assertEqual(STOP_LOGIC.state, StopStates.IGNORE_STOP_SIGNS)
        onDecisionInterval(None)
        fake_stopCar.assert_not_called()

        # Continue ignoring stop signs
        OBJECT_MAP.addToMap('stop sign', 10, 50, 0.8)
        OBJECT_MAP.addToMap('door', 10, 50, 15)
        self.assertEqual(STOP_LOGIC.state, StopStates.IGNORE_STOP_SIGNS)
        onDecisionInterval(None)
        fake_stopCar.assert_not_called()

        # Pretend ignore interval done. Stop at signs.
        OBJECT_MAP.addToMap('stop sign', 10, 50, 0.8)
        OBJECT_MAP.addToMap('door', 10, 50, 15)
        STOP_LOGIC.state = StopStates.NORMAL
        fake_stopCar.reset_mock()
        fake_togglePathFinder.reset_mock()
        self.assertEqual(STOP_LOGIC.state, StopStates.NORMAL)
        onDecisionInterval(None)
        fake_stopCar.assert_called()
