#!/usr/bin/env python

import rospy
import argparse
import sys

from soma_manager.object_manager import SOMAObjectManager



if __name__=="__main__":

    # TODO: add list command

    parser = argparse.ArgumentParser()
    #parser.add_argument("map", nargs=1, help='Name of the used 2D map')
    parser.add_argument("conf", nargs=1, help='Name of the object configuration')
    parser.add_argument('-t', metavar='config-file', help="Configuration file path for reading object types")

    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])

    SOMAObjectManager(args.conf[0],args.t)
