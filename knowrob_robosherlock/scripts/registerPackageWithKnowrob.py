#!/usr/bin/env python

import roslib; roslib.load_manifest('json_prolog')

import rospy
from json_prolog import json_prolog

if __name__ == '__main__':
    rospy.init_node('test_json_prolog')
    prolog = json_prolog.Prolog()
    query = prolog.query("register_ros_package(knowrob_robosherlock)")
