#!/usr/bin/env python

"""
This node is responsible for translating drive_param.msg (velocity/angle)
messages to drive_value (pwm) for the teensyboard

Subscribes to:
    drive_parameters:   drive_param

Publishes to:
    drive_pwm:          drive_values
    eStop:              Bool
"""

import rospy
from autobot.msg import drive_values
from autobot.msg import drive_param
from std_msgs.msg import Bool

pub = rospy.Publisher('drive_pwm', drive_values, queue_size=10)
em_pub = rospy.Publisher('eStop', Bool, queue_size=10)


# function to map from one range to another, similar to arduino
def arduino_map(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) // (in_max - in_min) + out_min


# callback function on occurance of drive parameters(angle & velocity)
def callback(data):
    velocity = data.velocity
    angle = data.angle
    print("Velocity: ", velocity, "Angle: ", angle)
    # Do the computation
    pwm1 = arduino_map(velocity, -100, 100, 6554, 13108)
    pwm2 = arduino_map(angle, -100, 100, 6554, 13108)
    msg = drive_values()
    msg.pwm_drive = pwm1
    msg.pwm_angle = pwm2
    pub.publish(msg)


def talker():
    rospy.init_node('motorControl', anonymous=True)
    em_pub.publish(False)
    rospy.Subscriber("drive_parameters", drive_param, callback)
    rospy.spin()


if __name__ == '__main__':
    print("Motor control initialized")
    talker()
