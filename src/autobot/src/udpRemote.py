#!/usr/bin/env python

import rospy
import socket
from autobot.msg import drive_param
import sys

"""
Warning: This code has not been tested at all

Protocol could be comma delimited
Vaa.aa;Abb.bb

TODO
- [ ] Unit test
- [ ] Define protocol
- [ ] Use select() for non-blocking operation
- [ ] Use a timeout for setting drive/angle to '0' (safety)
"""


def parseCommand(cmd, driveParam):
    """
    pass in Paa.aa where P is the ID of the command

    returns a tuple (driveParam, valid)
    """
    val = 0.0
    validState = True

    # first test to see if able to parse the value from substring
    try:
        val = float(cmd[1:])
    except ValueError:
        validState = False
        return driveParam, validState  # unable to parse, bail

    if cmd[0] == 'V':
        driveParam.velocity = val
        validState = True
    elif cmd[0] == 'A':
        driveParam.angle = val
        validState = True
    else:
        # Unknown command byte
        validState = False

    return driveParam, validState  # valid drive parameter parsed


def parseMessage(msg, pub):
    """
        Attempts to parse a message for a proper command string
        If the command string is valid, a drive parameter will be
        published
    """
    driveParam = drive_param()
    if ";" in msg:
        arr = msg.split(";")
        okState = True
        for cmd in arr:
            driveParam, parseState = parseCommand(cmd, driveParam)
            okState &= parseState

        # Only publish driveParam if all parsed parameters are OK
        if okState is True:
            pub.publish(driveParam)
    else:
        pass


def main():
    #UDP_IP = "127.0.0.1"    # loopback
    UDP_IP = "192.168.0.100"
    UDP_PORT = 11156
    UDP_ADDRESS = (UDP_IP, UDP_PORT)

    rospy.init_node("udpRemote", anonymous=True)
    pub = rospy.Publisher("drive_parameters", drive_param, queue_size=10)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))

    print 'starting up on %s port %s' % UDP_ADDRESS
    
    while True:
        data, addr = sock.recvfrom(1024)
        print 'received %s bytes from %s' % (len(data), addr)
        print data

        #parseMessage(data.decode("utf-8"), pub)
        parseMessage(data.decode("unicode_escape"), pub)

if __name__ == "__main__":
    main()
