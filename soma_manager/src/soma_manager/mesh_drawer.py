#!/usr/bin/env python

import roslib; roslib.load_manifest("soma_manager")
import rospy
from rospkg import RosPack
import json
import argparse
import random
import copy
import sys

from threading import Timer

from mongodb_store.message_store import MessageStoreProxy
from visualization_msgs.msg import Marker, MarkerArray, InteractiveMarker
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Polygon
from geometry_msgs.msg import PoseArray

from soma_manager.srv import SOMADrawMesh, SOMADrawMeshResponse


def trapezoidal_shaped_func(a, b, c, d, x):
    min_val = min(min((x - a)/(b - a), float(1.0)), (d - x)/(d - c))
    return max(min_val, float(0.0))


def r_func(x):
    a = -0.125
    b =  0.125
    c =  0.375
    d =  0.625

    x = 1.0 - x

    value = trapezoidal_shaped_func(a,b,c,d,x)
    return value

def g_func(x):
    a =  0.125
    b =  0.375
    c =  0.625
    d =  0.875

    x = 1.0 - x

    value = trapezoidal_shaped_func(a,b,c,d,x)
    return value


def b_func(x):
    a =  0.375
    b =  0.625
    c =  0.875
    d =  1.125

    x = 1.0 - x

    value = trapezoidal_shaped_func(a,b,c,d,x)
    return value


class SOMAMeshDrawer():

    def __init__(self):

        # default file
        rp = RosPack()

        ''' Set the default color as blue '''
        self.rgb = [0.0,0.0,1.0]
        self.markerarray = MarkerArray()
        self.marker_index = 0
        #self.rgb[0] = 0.0
        #self.rgb[1] = 0.0
        #self.rgb[2] = 1.0

        #self._msg_store=MessageStoreProxy(database="somadata",collection="roi")

        s = rospy.Service('soma/draw_mesh', SOMADrawMesh, self.handle_draw_mesh)

       # Publisher vis_pub = node_handle.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
        self.markerpub = rospy.Publisher("soma_object_meshes", MarkerArray, queue_size=1)

        rospy.spin()

    def handle_draw_mesh(self,msg):

        self.markerarray = MarkerArray()
        self.markerpub.publish(self.markerarray)
        self.marker_index = 0

        for i,path in enumerate(msg.mesh_paths):
            text_marker,mesh_marker = self.create_object_marker(msg.object_ids[i],msg.object_types[i],path,msg.object_poses[i])
            self.markerarray.markers.append(text_marker)
            self.markerarray.markers.append(mesh_marker)

        self.markerpub.publish(self.markerarray)

        return SOMADrawMeshResponse(True)

        return True

    def create_object_marker(self, obj_id, obj_type, mesh_path, mesh_pose):

        random.seed(obj_type)
        val = random.random()


        text_marker = Marker()
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.header.frame_id = "map"
        text_marker.color.r = r_func(val)
        text_marker.color.g = g_func(val)
        text_marker.color.b = b_func(val)
        text_marker.color.a = 1.0
        text_marker.scale.x = 0.25
        text_marker.scale.y = 0.25
        text_marker.scale.z = 0.25

        #int_marker.name = obj_type
        text_marker.id = self.marker_index
        self.marker_index +=1
        text_marker.text = obj_id + "_" + obj_type
        text_marker.pose = mesh_pose
        text_marker.pose.position.z += 0.25
        #text_marker.pose.position.z = 0.05

        mesh_marker = Marker()
        mesh_marker.header.frame_id = "map"
        mesh_marker.type = Marker.MESH_RESOURCE
        mesh_marker.scale.x = 1
        mesh_marker.scale.y = 1
        mesh_marker.scale.z = 1
        mesh_marker.id = self.marker_index
        self.marker_index += 1


        mesh_marker.color.r = r_func(val)
        mesh_marker.color.g = g_func(val)
        mesh_marker.color.b = b_func(val)
        mesh_marker.color.a = 1.0
        mesh_marker.pose = mesh_pose
        mesh_marker.mesh_resource = mesh_path

        return text_marker,mesh_marker

if __name__=="__main__":


    rospy.init_node("soma_mesh_drawer")
    rospy.loginfo("Running SOMA Mesh Drawer")
    SOMAMeshDrawer()
