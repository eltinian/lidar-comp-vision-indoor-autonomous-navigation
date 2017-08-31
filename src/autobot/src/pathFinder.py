#!/usr/bin/env python
import rospy
import math
import autobot
from autobot.msg import drive_param
from sensor_msgs.msg import LaserScan
from autobot.msg import pid_input
from autobot.msg import wall_dist
from autobot.msg import pathFinderState
from autobot.srv import *

"""
TODO:
- [x] Decide if you want to hug right/left/or closest wall
    - Right wall hugged for now to simulate right side of road
    - Update: wall decision to be made by image processing node
- [x] Send error left to hug left wall
- [ ] Use a command line argument to enable/disable debug msgs
"""


class PathConfig(object):
    __slots__ = ('wallToWatch', 'desiredTrajectory', 'velocity', 'pubRate',
                 'minFrontDist', 'enabled')
    """
    wallToWatch: Set which wall to hug
    options: autobot.msg.wall_dist.WALL_LEFT
             autobot.msg.wall_dist.WALL_RIGHT
             autobot.msg.wall_dist.WALL_FRONT  #< probably won't be used
    """
    def __init__(self):
        self.wallToWatch = autobot.msg.wall_dist.WALL_RIGHT
        self.desiredTrajectory = 0.5  # desired distance from the wall
        self.minFrontDist = 2.2       # minimum required distance in front of car
        self.velocity = 8.5           # velocity of drive
        self.pubRate = 0              # publish rate of node
        self.enabled = False          # enable/disable state of wall hugging

PATH_CONFIG = PathConfig()
errorPub = rospy.Publisher('error', pid_input, queue_size=10)
motorPub = rospy.Publisher('drive_parameters', drive_param, queue_size=10)
statePub = rospy.Publisher('pathFinderStatus', pathFinderState, queue_size=10)


def HandleTogglePathFinderService(req):
    """ Handler for enabling/disabling path finder
    Responds with ack msg (bool)
    """
    global PATH_CONFIG
    PATH_CONFIG.enabled = req.state
    return TogglePathFinderResponse(True)


def HandleAdjustWallDist(req):
    """ Handler for adjusting wall hugging parameters

    Responds with wall_dist msg and a bool to verify that the
    service command has been accepted
    """
    global PATH_CONFIG

    # print " wall {}".format(req.cmd.wall)
    # print " dist {}\n".format(req.cmd.dist)

    resp = wall_dist()
    isValid = req.cmd.dist >= 0

    if isValid is True and req.cmd.wall != autobot.msg.wall_dist.WALL_FRONT:
        """ only accept WALL_LEFT or WALL_RIGHT
        Service client can send an invalid wall or distance
        query current settings
        """
        if req.cmd.wall is not wall_dist.WALL_UNDEF:
            PATH_CONFIG.wallToWatch = req.cmd.wall

        PATH_CONFIG.desiredTrajectory = req.cmd.dist
    else:
        isValid = False

    resp.wall = PATH_CONFIG.wallToWatch
    resp.dist = PATH_CONFIG.desiredTrajectory
    return AdjustWallDistResponse(resp, isValid)


def publishCurrentState(event):
    global PATH_CONFIG

    msg = pathFinderState()
    msg.velocity = PATH_CONFIG.velocity
    msg.hug.wall = PATH_CONFIG.wallToWatch
    msg.hug.dist = PATH_CONFIG.desiredTrajectory
    msg.enabled = PATH_CONFIG.enabled
    statePub.publish(msg)


def getRange(data, theta):
    """ Find the index of the array that corresponds to angle theta.
    Return the lidar scan value at that index
    Do some error checking for NaN and absurd values
    data: the LidarScan data
    theta: the angle to return the distance for
    """
    car_theta = math.radians(theta) - math.pi / 2
    if car_theta > 3 * math.pi / 4:
        car_theta = 3 * math.pi / 4
    elif car_theta < -3 * math.pi / 4:
        car_theta = -3 * math.pi / 4

    float_index = (car_theta + 3 * math.pi / 4) / data.angle_increment
    index = int(float_index)
    return data.ranges[index]


def callback(data):
    global PATH_CONFIG

    # Do not attempt to hug wall if disabled
    if PATH_CONFIG.enabled is False:
        return

    frontDistance = getRange(data, 90)

    theta = 50  # PICK THIS ANGLE TO BE BETWEEN 0 AND 70 DEGREES

    thetaDistRight = getRange(data, theta)  # a
    rightDist = getRange(data, 0)  # b

    thetaDistLeft = getRange(data, 180-theta)  # aL
    leftDist = getRange(data, 180)  # bL

    if frontDistance < PATH_CONFIG.minFrontDist:
        # TURN
        print "Blocked!"
        driveParam = drive_param()
        if rightDist > leftDist:
            driveParam.angle = 90
            print "Turning Right"
        else:
            driveParam.angle = -90
            print "Turning Left"
        driveParam.velocity = PATH_CONFIG.velocity
        motorPub.publish(driveParam)
        return

    thetaRadsRight = math.radians(theta)  # aRads
    thetaRadsLeft = math.radians(130)     # bRads

    # alpha right
    carAngleRight = math.atan2(thetaDistRight * math.cos(thetaRadsRight) - rightDist,
                               thetaDistRight * math.sin(thetaRadsRight))

    # alpha left
    carAngleLeft = math.atan2(thetaDistLeft * math.cos(thetaRadsRight) - leftDist,
                              thetaDistLeft * math.sin(thetaRadsRight))

    carToWallRight = rightDist * math.cos(carAngleRight)  # AB
    carToWallLeft = leftDist * math.cos(carAngleLeft)     # ABL

    distTraveled = 1.0  # AC MAY NEED TO EDIT THIS VALUE

    # CD
    projectedDistRight = carToWallRight + distTraveled * math.sin(carAngleRight)

    # CDL
    projectedDistLeft = carToWallLeft + distTraveled * math.sin(carAngleLeft)

    """
    If too far from the wall:
        Turn right to get closer to it
        errorRight will be positive
    if too close to wall:
        Turn left to get further
        errorRight will be negative

    The error_ values are differences between projected future position
    and the distance to the wall
    """
    # ARE WE PROCESSING THIS ERROR CORRECTLY? GETS SENT TO PIDCONTROL.PY
    errorRight = projectedDistRight - PATH_CONFIG.desiredTrajectory
    errorLeft = projectedDistLeft - PATH_CONFIG.desiredTrajectory
    errorLeft *= -1

    dbg = ("----------------------------------\n"
            "             | LEFT     | RIGHT\n"
            "------------ + -------- + --------\n"
            "    carAngle | {:6.3f}   | {:6.3f}\n"
            "   thetaDist | {:6.3f}   | {:6.3f}\n"
            "    distance | {:6.3f}   | {:6.3f}\n"
            "   carToWall | {:6.3f}   | {:6.3f}\n"
            "   projected | {:6.3f}   | {:6.3f}\n"
            "       error | {:6.3f}   | {:6.3f}\n"
            "----------------------------------\n").format(
                carAngleLeft, carAngleRight,
                thetaDistLeft, thetaDistRight,
                leftDist, rightDist,
                carToWallLeft, carToWallRight,
                projectedDistLeft, projectedDistRight,
                errorLeft, errorRight)

    print dbg

    msg = pid_input()
    if PATH_CONFIG.wallToWatch == autobot.msg.wall_dist.WALL_LEFT:
        msg.pid_error = errorLeft
    else:
        msg.pid_error = errorRight

    msg.pid_vel = PATH_CONFIG.velocity

    PATH_CONFIG.pubRate += 1
    if (PATH_CONFIG.pubRate % 10 == 0):
        PATH_CONFIG.pubRate = 0
        errorPub.publish(msg)

if __name__ == '__main__':
    print("Path finding node started")
    rospy.Service('adjustWallDist', AdjustWallDist, HandleAdjustWallDist)
    rospy.Service('togglePathFinder', TogglePathFinder,
                  HandleTogglePathFinderService)
    rospy.init_node('pathFinder', anonymous=True)
    rospy.Subscriber("scan", LaserScan, callback)
    rospy.Timer(rospy.Duration(0.5), callback=publishCurrentState)
    rospy.spin()
