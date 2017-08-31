#!/usr/bin/env python

import unittest
from udpRemote import parseCommand

"""
TODO
- [ ] Convert to ROS test format: http://wiki.ros.org/unittest
- [ ] Mock publisher
- [ ] Node integration test
"""


class MockDriveParam:
    velocity = 0.0
    angle = 0.0


class UdpRemoteTest(unittest.TestCase):
    def testValidParse(self):
        p = MockDriveParam()
        p, state = parseCommand("V44.4", p)
        self.assertTrue(state)
        self.assertEqual(p.velocity, 44.4)
        self.assertEqual(p.angle, 0.0)

        p, state = parseCommand("A81.3", p)
        self.assertTrue(state)
        self.assertEqual(p.velocity, 44.4)
        self.assertEqual(p.angle, 81.3)

    def testInvalidParse(self):
        p = MockDriveParam()
        p, state = parseCommand("X44.4", p)
        self.assertFalse(state)
        self.assertEqual(p.velocity, 0.0)
        self.assertEqual(p.angle, 0.0)

        p = MockDriveParam()
        p, state = parseCommand("V0F.4", p)
        self.assertFalse(state)
        self.assertEqual(p.velocity, 0.0)
        self.assertEqual(p.angle, 0.0)
