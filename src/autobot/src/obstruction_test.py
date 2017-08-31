#!/usr/bin/env python

import unittest
from obstruction import *


class ObstructionTest(unittest.TestCase):
    def testAddingObjects(self):
        o = ObstructionMap()
        o.addToMap('dog', 10, 50, 60)
        o.addToMap('cat', 10, 50, 60)
        o.addToMap('bat', 10, 50, 60)
        self.assertEqual(len(o.obstructions[ObstructionMap.LEFT]), 3)

    def testGetClosest(self):
        o = ObstructionMap()
        o.addToMap('dog', 10, 50, 80)
        o.addToMap('cat', 10, 50, 60)
        o.addToMap('bat', 10, 50, 65)
        c = o.getClosest()
        self.assertEqual(len(o.obstructions[ObstructionMap.LEFT]), 3)
        self.assertEqual(c.className, 'cat')
