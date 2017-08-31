#!/usr/bin/env python
import rospy
from autobot.msg import drive_param
from autobot.msg import pid_input
from std_msgs.msg import String
import math


kp = 14.0 * 3
kd = 0.09 * 10 	## handling how fast
servo_offset = 18.5
prev_error = 0.0 
vel_input = 0.0
mode = 'wall'

motorPub = rospy.Publisher('drive_parameters', drive_param, queue_size=10)


def control(data):

    global kp
    global kd
    global servo_offset
    global prev_error
    global vel_input
    global mode

    driveParam = drive_param()
    driveParam.velocity = data.pid_vel

    if mode == 'wall':
        pid_error = data.pid_error
        error = pid_error * kp
        errordot = kd * (pid_error - prev_error)

        angle = error + errordot

        if angle > 100:
            angle = 100
        elif angle < -100:
            angle = -100

        prev_error = pid_error
        print 'pid_error {}\nangle {}'.format(pid_error, angle)

        driveParam.angle = angle

    elif mode == 'corner':
        print 'corner mode, angle 100'
        driveParam.angle = 100

    motorPub.publish(driveParam)


def update_mode(_mode):
    global mode
    mode = _mode.data


if __name__ == '__main__':
    print("Listening to error for PID")
    rospy.init_node('pid_controller', anonymous=True)
    rospy.Subscriber("error", pid_input, control)
    rospy.Subscriber("mode", String, update_mode)

    rospy.spin()

