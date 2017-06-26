#! /usr/bin/env python

#copyright 2016-2017 INRIA. Licensed under the Apache License, Version 2.0.

#script to log some robot data while moving
#broker ip is the ip of the robot, port is the port number of naoqi on the robot

ALMEMORY_KEYS = [
    "Device/SubDeviceList/HeadPitch/Position/Sensor/Value",
    "Device/SubDeviceList/HeadYaw/Position/Sensor/Value",
    "Device/SubDeviceList/LElbowRoll/Position/Sensor/Value",
    "Device/SubDeviceList/LElbowYaw/Position/Sensor/Value",
    "Device/SubDeviceList/RElbowRoll/Position/Sensor/Value",
    "Device/SubDeviceList/RElbowYaw/Position/Sensor/Value",
    "Device/SubDeviceList/LWristYaw/Position/Sensor/Value",
    "Device/SubDeviceList/RWristYaw/Position/Sensor/Value",
    "Device/SubDeviceList/LShoulderPitch/Position/Sensor/Value",
    "Device/SubDeviceList/LShoulderRoll/Position/Sensor/Value",
    "Device/SubDeviceList/RShoulderPitch/Position/Sensor/Value",
    "Device/SubDeviceList/RShoulderRoll/Position/Sensor/Value",
    "Device/SubDeviceList/HipPitch/Position/Sensor/Value",
    "Device/SubDeviceList/HipRoll/Position/Sensor/Value",
    "Device/SubDeviceList/KneePitch/Position/Sensor/Value",
    "Device/SubDeviceList/HeadPitch/Position/Actuator/Value",
    "Device/SubDeviceList/HeadYaw/Position/Actuator/Value",
    "Device/SubDeviceList/LElbowRoll/Position/Actuator/Value",
    "Device/SubDeviceList/LElbowYaw/Position/Actuator/Value",
    "Device/SubDeviceList/RElbowRoll/Position/Actuator/Value",
    "Device/SubDeviceList/RElbowYaw/Position/Actuator/Value",
    "Device/SubDeviceList/LWristYaw/Position/Actuator/Value",
    "Device/SubDeviceList/RWristYaw/Position/Actuator/Value",
    "Device/SubDeviceList/LShoulderPitch/Position/Actuator/Value",
    "Device/SubDeviceList/LShoulderRoll/Position/Actuator/Value",
    "Device/SubDeviceList/RShoulderPitch/Position/Actuator/Value",
    "Device/SubDeviceList/RShoulderRoll/Position/Actuator/Value",
    "Device/SubDeviceList/HipPitch/Position/Actuator/Value",
    "Device/SubDeviceList/HipRoll/Position/Actuator/Value",
    "Device/SubDeviceList/KneePitch/Position/Actuator/Value",
    "Device/SubDeviceList/HeadPitch/Hardness/Actuator/Value",
    "Device/SubDeviceList/HeadYaw/Hardness/Actuator/Value",
    "Device/SubDeviceList/LElbowRoll/Hardness/Actuator/Value",
    "Device/SubDeviceList/LElbowYaw/Hardness/Actuator/Value",
    "Device/SubDeviceList/RElbowRoll/Hardness/Actuator/Value",
    "Device/SubDeviceList/RElbowYaw/Hardness/Actuator/Value",
    "Device/SubDeviceList/LWristYaw/Hardness/Actuator/Value",
    "Device/SubDeviceList/RWristYaw/Hardness/Actuator/Value",
    "Device/SubDeviceList/LShoulderPitch/Hardness/Actuator/Value",
    "Device/SubDeviceList/LShoulderRoll/Hardness/Actuator/Value",
    "Device/SubDeviceList/RShoulderPitch/Hardness/Actuator/Value",
    "Device/SubDeviceList/RShoulderRoll/Hardness/Actuator/Value",
    "Device/SubDeviceList/HipPitch/Hardness/Actuator/Value",
    "Device/SubDeviceList/HipRoll/Hardness/Actuator/Value",
    "Device/SubDeviceList/KneePitch/Hardness/Actuator/Value"
]

import os
import sys
import time
import threading

from naoqi    import ALProxy, ALModule
from optparse import OptionParser

class LoggingThread(threading.Thread):
    """
       Class representing thread which logs data
       as the robot is moving
    """

    def __init__(self, nao_ip, nao_port, filename):
        threading.Thread.__init__(self)
        self.nao_ip_   = nao_ip
        self.nao_port_ = nao_port
        self.filename_ = filename

    def run(self):
        """
            Thread entry point
        """

        memory_proxy = ALProxy("ALMemory", self.nao_ip_, self.nao_port_)
        with open(self.filename_, "w") as f:
            f.write("%one row of the matrix: " + "\n")
            for key in ALMEMORY_KEYS:
                f.write("%" + key + "\n")
            f.write("data" + " = " + "[ " + "\n")
            while not e.is_set():
                for key in ALMEMORY_KEYS:
                    f.write(str(memory_proxy.getData(key)) + " ")
                f.write(" ;" + "\n")
            f.write(" ];" + "\n")

if __name__ == "__main__":
    parser = OptionParser()
    parser.add_option("-b", "--broker-ip",   action="store", type="string", dest="IP",     default="127.0.0.1")
    parser.add_option("-p", "--broker-port", action="store", type="int",    dest="PORT",   default=9559)
    parser.add_option("-n", "--no-arms",     action="store_true", dest="NOARMS")
    (options, args) = parser.parse_args();

    print("----- Started")

    try:
        navigation_proxy = ALProxy("ALNavigation", options.IP, options.PORT)
        motion_proxy     = ALProxy("ALMotion",     options.IP, options.PORT)
    except Exception, e:
        print("Could not create proxies to ALNavigation or ALMotion")
        print(str(e))
        exit(1)

    if options.NOARMS:
        print("----- Moving with arms disabled")
        motion_proxy.setMoveArmsEnabled(False, False)

    # get an event
    e = threading.Event()

    # spawn looging thread
    thread = LoggingThread(options.IP, options.PORT, "log.m")
    thread.start()

    # move 2 meters ahead
    navigation_proxy.navigateTo(2., 0.)
    e.set()
    thread.join()

    print("----- Finished")
