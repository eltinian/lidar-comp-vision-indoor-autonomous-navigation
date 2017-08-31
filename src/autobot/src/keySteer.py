#!/usr/bin/env python

import rospy
from autobot.msg import drive_param
from autobot.msg import wall_dist
from autobot.msg import pathFinderState
from autobot.srv import *
import curses

"""
TODO:
- [ ] Update UI to show wall hug status
- [ ] Create menu boxes
"""

velocity = 0
velocityMultiplier = 10
steerAngle = 0
steerIncrement = 15
steerMultiplier = 1.5

stdscr = curses.initscr()
curses.cbreak()
curses.noecho()
stdscr.keypad(1)

stdscr.refresh()


def drawGreeter():
    global stdscr
    stdscr.addstr(0, 5, "AUTOBOT CONSOLE")

    stdscr.addstr(2, 5, "MOTOR STATUS ")
    stdscr.addstr(3, 5, "----------------")

    stdscr.addstr(4, 5, "VELOCITY")
    stdscr.addstr(5, 5, "ANGLE   ")

    stdscr.addstr(2, 30, "  KEYBINDINGS")
    stdscr.addstr(3, 30, "+ ---------------------------------------------")
    stdscr.addstr(4, 30, "| WASD  - steer. (hold shift to increase ratio)")
    stdscr.addstr(5, 30, "| Space - stop (disables pathfinder)")
    stdscr.addstr(6, 30, "| c     - center wheels")
    stdscr.addstr(7, 30, "| l     - hug left wall")
    stdscr.addstr(8, 30, "| r     - hug right wall")
    stdscr.addstr(9, 30, "| t     - turn on pathFinder")
    stdscr.addstr(10, 30, "| q     - quit")


def srvTogglePathFinder(state):
    try:
        rospy.wait_for_service('togglePathFinder', timeout=0.2)
        srv = rospy.ServiceProxy('togglePathFinder', TogglePathFinder)
        srv(state)  # ignore ACK response
    except rospy.ROSException, e:
        # print "Service called failed: %s" % e
        pass


def modVelocity(incr):
    global velocity
    velocity = velocity + incr
    stdscr.addstr(4, 16, '%.2f' % velocity)


def zeroVelocity():
    global velocity
    srvTogglePathFinder(False)
    velocity = 0
    stdscr.addstr(4, 16, '%.2f' % velocity)


def modAngle(incr):
    global steerAngle
    if (steerAngle == 60 and incr > 0) or (steerAngle == -60 and incr < 0):
        return  
    else:
        steerAngle = steerAngle + incr
    stdscr.addstr(5, 16, '%.2f' % steerAngle)


def zeroAngle():
    global steerAngle
    steerAngle = 0
    stdscr.addstr(5, 16, '%.2f' % steerAngle)


def setWallDist(wall, dist):
    try:
        rospy.wait_for_service('adjustWallDist')
        adjustWall = rospy.ServiceProxy('adjustWallDist', AdjustWallDist)
        cmd = wall_dist()
        cmd.wall = wall
        cmd.dist = dist
        resp = adjustWall(cmd)
        return resp
    except rospy.ROSException, e:
        # print "Service called failed: %s" % e
        pass


def convertWallToString(wall):
    # WALL_LEFT=0
    # WALL_FRONT=1
    # WALL_RIGHT=2
    if (wall is wall_dist.WALL_LEFT):
        return "Left"
    elif (wall is wall_dist.WALL_RIGHT):
        return "Right"
    elif (wall is wall_dist.WALL_FRONT):
        return "Front"
    else:
        return "Unknown"


def driveParamsUpdated(driveParam):
    global stdscr
    global velocity
    global steerAngle
    velocity = driveParam.velocity
    steerAngle = driveParam.angle
    stdscr.addstr(4, 5, "VELOCITY : {:3.2}".format(driveParam.velocity))
    stdscr.addstr(5, 5, "ANGLE    : {:3.2}".format(driveParam.angle))


def pathFinderUpdated(status):
    global stdscr
    stdscr.addstr(4, 5, "VELOCITY :")
    stdscr.addstr(5, 5, "ANGLE    :")

    stdscr.addstr(7, 5, "PathFinder State")
    stdscr.addstr(8, 5, "----------------")
    stdscr.addstr(9, 5, "ENABLED  : {}  ".format(status.enabled))
    stdscr.addstr(10, 5, "WALL     : {}  ".format(
            convertWallToString(status.hug.wall)
        ))
    stdscr.addstr(11, 5, "WALLDIST : {:3.1}  ".format(status.hug.dist))
    stdscr.refresh()


def main():
    rospy.init_node('keyboardSteer', anonymous=True)
    drivePub = rospy.Publisher('drive_parameters', drive_param, queue_size=10)
    drawGreeter()
    key = ''
    while key != ord('q'):
        keyPressed = False
        key = stdscr.getch()
        stdscr.refresh()

        if key == curses.KEY_UP or key == ord('w'):
            modVelocity(0.1 * velocityMultiplier)
            keyPressed = True
        elif key == ord('W'):
            modVelocity(0.1 * velocityMultiplier)
            keyPressed = True
        elif key == curses.KEY_DOWN or key == ord('s'):
            modVelocity(-0.1 * velocityMultiplier)
            keyPressed = True
        elif key == ord('S'):
            modVelocity(-0.1 * velocityMultiplier)
            keyPressed = True

        if key == curses.KEY_LEFT or key == ord('a'):
            modAngle(-1 * steerIncrement)
            keyPressed = True
        elif key == ord('A'):
            modAngle(-1 * steerIncrement * steerMultiplier)
            keyPressed = True
        elif key == curses.KEY_RIGHT or key == ord('d'):
            modAngle(steerIncrement)
            keyPressed = True
        elif key == ord('D'):
            modAngle(steerIncrement * steerMultiplier)
            keyPressed = True
        elif key == ord(' '):
            zeroVelocity()
            keyPressed = True
        elif key == ord('c'):
            zeroAngle()
            keyPressed = True

        if keyPressed is True:
            msg = drive_param()
            msg.velocity = velocity
            msg.angle = steerAngle
            drivePub.publish(msg)

        if key == ord('r'):
            setWallDist(autobot.msg.wall_dist.WALL_RIGHT, 0.5)
        elif key == ord('l'):
            setWallDist(autobot.msg.wall_dist.WALL_LEFT, 0.5)
        elif key == ord('t'):
            srvTogglePathFinder(True)

    curses.endwin()


if __name__ == '__main__':
    rospy.Subscriber("pathFinderStatus", pathFinderState, pathFinderUpdated)
    rospy.Subscriber("drive_parameters", drive_param, driveParamsUpdated)
    main()
