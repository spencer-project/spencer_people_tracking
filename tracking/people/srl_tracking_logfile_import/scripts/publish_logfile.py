#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from srl_tracking_logfile_import import LogfilePublisher


if __name__ == '__main__':
    arguments = rospy.myargv()

    rospy.init_node("publish_logfile")

    if len(arguments) < 2:
        rospy.logerr("Must specify log filename as argument!")
    else:
        logfilePublisher = LogfilePublisher()
        logfilePublisher.read(filename=arguments[1])
