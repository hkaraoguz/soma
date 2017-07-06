#!/usr/bin/env python

import rospy
import argparse
import sys

from soma_roi_manager.soma_roi import SOMAROIManager

if __name__=="__main__":

    # TODO: add list command

    parser = argparse.ArgumentParser(prog='soma_roi.py')
    parser.add_argument(
        "conf", nargs=1, help='Name of the roi configuration. Put [] with "," to launch multiple ROI configuration instances. For example: "[config1,config2]"'
    )
    parser.add_argument('-t', metavar='config-file',help="Config file path for reading the room types")

    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])

    rospy.init_node("soma_roi_manager")
    configs = args.conf[0].replace(" ", "")
    configs = configs.replace("[", "")
    configs = configs.replace("]", "")
    configs = configs.split(",")
    for config in configs:
        SOMAROIManager(config,args.t)
    rospy.spin()
