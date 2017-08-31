#!/usr/bin/env python
from collections import defaultdict


class ObstructionInfo(object):

    def __init__(self):
        self.distance = 999
        self.coord = (0, 0)
        self.className = ""


class ObstructionMap(object):
    """ TODO: Write down class information
    Store objects into a dict?
    Or a numpy type of array? Numpy can get things by distance
    This class could be our decision maker also. Probably more efficient
    to make decisions as things are added

    Obstructions
    LEFT, RIGHT
    """
    LEFT = 'LEFT'
    RIGHT = 'RIGHT'
    TOP = 'TOP'
    BOTTOM = 'BOTTOM'
    CENTER = 'CENTER'
    HIGHPRIORITIES = ['person', 'chair', 'stop sign']

    def __init__(self):
        self.obstructions = defaultdict(list)
        self.highprios = []

    def clearMap(self):
        self.obstructions.clear()
        self.highprios = []

    def addToMap(self, className, x, y, distance):
        """NOTE
        if (x > threshold) gives the location of object in terms of left/right
        # Do objects closest to the car get priority?
        self.obstructions[k] = (className, distance)
        """
        # side = ObstructionMap.LEFT
        side = self.xToSide(x)

        obs = ObstructionInfo()
        obs.className = className
        obs.distance = distance
        obs.coord = (x, y)

        self.obstructions[side].append(obs)
        if obs.className in self.HIGHPRIORITIES:
            self.highprios.append(obs)

    def xToSide(self, xCoord):
        """TODO: Get proper coordinates for left/right/center"""
        if xCoord < 40:
            return ObstructionMap.LEFT
        elif xCoord > 80:
            return ObstructionMap.RIGHT
        else:
            return ObstructionMap.CENTER

    def getClosest(self):
        """NOTE
        Loop through obstruction list and make decision on which object
        is the closest

        Returns ObstructionInfo object
        """
        if len(self.obstructions) is 0:
            return None

        closestObject = ObstructionInfo()
        for side in self.obstructions:
            for obs in self.obstructions[side]:
                if obs.distance < closestObject.distance:
                    closestObject = obs

        return closestObject

    def getClosestOnSide(self, side):
        if len(self.obstructions) is 0:
            return None

        if side not in self.obstructions:
            return None

        closestObject = ObstructionInfo()
        for obs in self.obstructions[side]:
            if obs.distance < closestObject.distance:
                closestObject = obs

        return closestObject

    def getHighPriorities(self):
        """
        TODO: Return a list of high priority objects?
        High priorities include things like chairs (what else?).

        E.g. If currently hugging the right wall closely and a closed door is
        coming up, but there is a chair somewhat to the left of the car...
        do NOT attempt to drive away from the wall to avoid the door. Instead
        either stick to that wall or maybe swing the car all the way to the
        left wall to avoid the chair.
        """
        return self.highprios
