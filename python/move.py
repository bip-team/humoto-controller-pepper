#! /usr/bin/env python

#author Jan Michalczyk 
#copyright 2016-2017 INRIA. Licensed under the Apache License, Version 2.0.

#script to call peppercontroller module on the robot
#broker ip is the ip of the robot, port is the port number of naoqi on the robot
#config is a name of the config file

#ex. ./move.py -c mpc-mp-move-straight-medium.yaml

import os
import sys
import time

from naoqi    import ALProxy
from optparse import OptionParser
    
# initial pose
wakeup_configuration = [-0.200000000000011,     1.21242176360421e-16, -0.52345698999998,     -1.2217007,
                         0.52345698999998,      1.2217007,             2.09663041746472e-16, -2.09879700531888e-16,
                         1.57000000000001,      0.11999999999998,      1.57000000000001,     -0.119999999999982,   -0.0399999999999646,
                        -6.14750622559617e-15, -0.0100000000000153,    0.0,                   0.0,                  0.0]

# initial pose with low CoM
low_CoM_wakeup_configuration = [0.06850114982656237, 0.0, -0.52345699,        -1.2217007, 0.52345699, 1.2217007, 0.0, 0.0,   
                                1.639454123287812,   0.12, 1.639454123287804, -0.12,      0.4160023626190753,    0.0,
                               -0.1975011835036236,  0.0,  0.0,                0.0]
# rest pose
rest_configuration   = [0.743980646133,  -0.0153398513794, -0.0122718811035, -0.489339828491,
                        0.00613594055176, 0.490873813629,  -0.785449981689,   0.782298088074,  1.14588367939,
                       -0.0536892414093,  1.14434981346,   -0.122718572617,  -1.04617476463,  -0.246970891953,
                        0.52001953125,    0.0,              0.0,              0.0]

if __name__ == "__main__":

    parser = OptionParser()
    parser.add_option("-b", "--broker-ip",   action="store", type="string", dest="IP",     default="10.42.0.61")
    parser.add_option("-p", "--broker-port", action="store", type="int",    dest="PORT",   default=9559)
    parser.add_option("-c", "--config",      action="store", type="string", dest="CONFIG", default="")
    (options, args) = parser.parse_args();

    print("----- Started")

    try:
        peppercontroller_proxy = ALProxy("PepperController", options.IP, options.PORT)
    except Exception, e:
        print("Error when creating proxy")
        print(str(e))
        exit(1)

    print("----- Proxy was created")
    
    print("----- Killing ALMotion")
    peppercontroller_proxy.killALMotionModule()
    
    # call goInitialPose() only when controller is off
    peppercontroller_proxy.setActuatorsStiffness(2000, 0.6)
    #peppercontroller_proxy.setUpperJointsStiffness(2000, 1.0)
    peppercontroller_proxy.goInitialPose(8000)
    
    print("----- Starting controller")
    peppercontroller_proxy.startControl()

    time.sleep(5)
    
    print("----- Send new MPC motion parameters")
    # set mpc motion parameters only when controller is in idle state

    peppercontroller_proxy.setMPCMotionParameters(options.CONFIG)
    
    try:
        while True:
            pass
    except KeyboardInterrupt:
        print("----- Terminating motion")
    
    print("----- Print root pose")
    peppercontroller_proxy.printRootPose()
    
    peppercontroller_proxy.stopControl()
    
    time.sleep(5)
    
    # call goRestPose() only when controller is off
    #peppercontroller_proxy.goRestPose(8000)
    peppercontroller_proxy.setWheelsStiffness(2000, 0.0)
    #peppercontroller_proxy.setActuatorsStiffness(2000, 0.0)
    
    print("----- Finished")
