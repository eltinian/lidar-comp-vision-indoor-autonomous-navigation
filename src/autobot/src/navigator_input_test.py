#!/usr/bin/env python
"""
This node is intended to stream in fake messages of 
objects being detected to the object_detector topic
"""

import rospy
from autobot.msg import drive_param
from autobot.msg import wall_dist
from autobot.msg import detected_object
from autobot.msg import pathFinderState
from sensor_msgs.msg import Image
import curses
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

stdscr = curses.initscr()
curses.cbreak()
curses.noecho()
stdscr.keypad(1)

stdscr.refresh()

PUB_OBJ = rospy.Publisher('detected_object', detected_object, queue_size=10)


def drawGreeter():
    global stdscr
    stdscr.addstr(0, 5, "FAKE OBJECT PUBLISHER")

    stdscr.addstr(2, 5, "MOTOR STATUS ")
    stdscr.addstr(3, 5, "----------------")

    stdscr.addstr(4, 5, "VELOCITY")
    stdscr.addstr(5, 5, "ANGLE   ")

    stdscr.addstr(2, 30, "  KEYBINDINGS")
    stdscr.addstr(3, 30, "+ ---------------------------------------------")
    stdscr.addstr(4, 30, "| P - signal a person is in view (close)")
    stdscr.addstr(5, 30, "| p - signal a person is in view (far)")
    stdscr.addstr(6, 30, "| S - signal that a stop sign is in view (far)")
    stdscr.addstr(7, 30, "| s - signal that a stop sign is in view (close)")
    stdscr.addstr(8, 30, "| d - signal door on left")
    stdscr.addstr(9, 30, "| D - signal door on right")
    stdscr.addstr(10, 30, "| q     - quit")


def srvTogglePathFinder(state):
    try:
        rospy.wait_for_service('togglePathFinder', timeout=0.2)
        srv = rospy.ServiceProxy('togglePathFinder', TogglePathFinder)
        srv(state)  # ignore ACK response
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


def createObjectMessage(name, color, x, y, height, width):
    msg = detected_object()
    msg.className = name

    img = np.zeros([200, 200, 3], dtype=np.uint8)
    img.fill(color)

    bridge = CvBridge()
    imgMsg = bridge.cv2_to_imgmsg(img, "bgr8")
    msg.depthImg = imgMsg
    msg.box.origin_x = x
    msg.box.origin_y = y
    msg.box.height = height
    msg.box.width = width
    return msg


def main():
    rospy.init_node('navigator_input_test', anonymous=True)
    rospy.Subscriber("pathFinderStatus", pathFinderState, pathFinderUpdated)
    drivePub = rospy.Publisher('drive_parameters', drive_param, queue_size=10)
    drawGreeter()
    key = ''
    while key != ord('q'):
        keyPressed = False
        key = stdscr.getch()
        stdscr.refresh()

        if key == curses.KEY_UP or key == ord('w'):
            pass
        if key is ord('P'):
            msg = createObjectMessage(name='person',
                                      color=255,
                                      x=10,
                                      y=10,
                                      height=100,
                                      width=25)
            PUB_OBJ.publish(msg)
        elif key is ord('p'):
            msg = createObjectMessage(name='person',
                                      color=10,
                                      x=10,
                                      y=10,
                                      height=100,
                                      width=25)
            PUB_OBJ.publish(msg)
        elif key is ord('S'):
            msg = createObjectMessage(name='stop sign',
                                      color=255,
                                      x=10,
                                      y=10,
                                      height=100,
                                      width=25)
            PUB_OBJ.publish(msg)
        elif key is ord('s'):
            msg = createObjectMessage(name='stop sign',
                                      color=20,
                                      x=10,
                                      y=10,
                                      height=100,
                                      width=25)
            PUB_OBJ.publish(msg)
        elif key is ord('d'):
            msg = createObjectMessage(name='door',
                                      color=255,
                                      x=10,
                                      y=10,
                                      height=100,
                                      width=25)
            PUB_OBJ.publish(msg)
        elif key is ord('D'):
            msg = createObjectMessage(name='door',
                                      color=255,
                                      x=95,
                                      y=10,
                                      height=100,
                                      width=25)
            PUB_OBJ.publish(msg)
    curses.endwin()


if __name__ == '__main__':
    main()
