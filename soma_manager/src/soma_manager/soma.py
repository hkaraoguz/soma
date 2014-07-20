#!/usr/bin/env python

import roslib; roslib.load_manifest("soma_manager")
import rospy
from rospkg import RosPack
import json
import argparse
import random
import copy

from threading import Timer

from ros_datacentre.message_store import MessageStoreProxy
from visualization_msgs.msg import Marker, InteractiveMarkerControl
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from geometry_msgs.msg import Pose

from soma_msgs.msg import SOMAObject
from bson.objectid import ObjectId

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


class SOMAManager():

    def __init__(self, soma_map, soma_conf, config_file=None):

        self.soma_map = soma_map
        self.soma_conf = soma_conf
        if config_file:
            self._config_file = config_file
        else:
            # default file
            rp = RosPack()
            path = rp.get_path('soma_objects') + '/config/'
            filename = 'default.json'
            self._config_file=path+filename
        self._soma_obj_ids = dict()
        self._soma_obj_msg = dict()

        self._interactive = True

        self._msg_store=MessageStoreProxy(collection="soma")
        
        self._server = InteractiveMarkerServer("soma")

        self._init_types()

        self._init_menu()
        
        self.load_objects()

        rospy.spin()


    def _init_types(self):
        # read from config in soma_objects 
        
        with open(self._config_file) as config_file:
            config = json.load(config_file)

            self.mesh = dict()
            for k, v in config['mesh'].iteritems():
                self.mesh[k] = v

    def _init_menu(self):

        self.menu_handler = MenuHandler()
        add_entry = self.menu_handler.insert( "Add object" )

        self.menu_item = dict()
        for i, k in enumerate(self.mesh):
            entry =  self.menu_handler.insert(k, parent=add_entry, callback=self._add_cb)
            self.menu_item[entry] = k
            
        del_entry =  self.menu_handler.insert( "Delete object", callback=self._del_cb)

        enable_entry = self.menu_handler.insert( "Movement control", callback=self._enable_cb )

        self.menu_handler.setCheckState( enable_entry, MenuHandler.CHECKED )

    def _add_cb(self, feedback):
        rospy.loginfo("Add marker: %s", self.menu_item[feedback.menu_entry_id])
        self.add_object(self.menu_item[feedback.menu_entry_id], feedback.pose)

    def _del_cb(self, feedback):
        rospy.loginfo("Delete marker: %s", feedback.marker_name)
        self.delete_object(feedback.marker_name)        

    def _update_cb(self, feedback):
        p = feedback.pose.position
        print "Marker " + feedback.marker_name + " is now at " + str(p.x) + ", " + str(p.y)
        
        if hasattr(self, "vp_timer_"+feedback.marker_name):
            getattr(self, "vp_timer_"+feedback.marker_name).cancel()        
        setattr(self, "vp_timer_"+feedback.marker_name,
                Timer(3, self.update_object, [feedback]))
        getattr(self, "vp_timer_"+feedback.marker_name).start()        
        
    def _enable_cb(self, feedback):
        handle = feedback.menu_entry_id
        state = self.menu_handler.getCheckState( handle )

        if state == MenuHandler.CHECKED:
            self.menu_handler.setCheckState( handle, MenuHandler.UNCHECKED )
            self._interactive = False
        else:
            self.menu_handler.setCheckState( handle, MenuHandler.CHECKED )
            self._interactive = True

        self.menu_handler.reApply( self._server )

        self.load_objects()
        
        self._server.applyChanges()
        
    def _next_id(self):
        self._soma_id += 1
        return self._soma_id

    def _retrieve_objects(self):

        objs = self._msg_store.query(SOMAObject._type, message_query={"map": self.soma_map,
                                                                      "config": self.soma_conf})

        max_id = 0
        for o,om in objs:
            if int(o.id) > max_id:
                max_id = int(o.id)
        self._soma_id = max_id
        
        return objs
    
    def load_objects(self):

        objs = self._retrieve_objects()

        # if collection is empty insert initial object
        if not objs:
            pose = Pose()
            self.add_object('Table', pose)
            return

        # otherwise, load all object from collection
        for o, om  in objs:
            self._soma_obj_ids[o.id] = om['_id']
            self._soma_obj_msg[o.id] = o
            self.load_object(o.id, o.type, o.pose)


    def load_object(self, soma_id, soma_type, pose):

        int_marker = self.create_object_marker(soma_id, soma_type, pose)
        
        self._server.insert(int_marker, self._update_cb)

        self.menu_handler.apply( self._server, soma_id )

        self._server.applyChanges()

    def add_object(self, soma_type, pose):
        # todo: add to mongodb
        
        soma_id = self._next_id()

        soma_obj = SOMAObject()
        soma_obj.id = str(soma_id)
        soma_obj.map = str(self.soma_map)
        soma_obj.config = str(self.soma_conf)
        soma_obj.type = soma_type
        soma_obj.pose = pose
        soma_obj.frame = '/map'
        soma_obj.mesh = self.mesh[soma_type]

        _id = self._msg_store.insert(soma_obj)
        self._soma_obj_ids[soma_obj.id] = _id
        self._soma_obj_msg[soma_obj.id] = soma_obj
        
        self.load_object(str(soma_id), soma_type, pose)


    def delete_object(self, soma_id):
        # todo: delete from mongodb

        _id = self._soma_obj_ids[str(soma_id)]
        self._msg_store.delete(str(_id))
        
        self._server.erase(soma_id)
        self._server.applyChanges()
        
    def update_object(self, feedback):
        print "Updated marker " + feedback.marker_name

        _id = self._soma_obj_ids[feedback.marker_name]
        msg = self._soma_obj_msg[feedback.marker_name]

        new_msg = copy.deepcopy(msg)
        new_msg.pose = feedback.pose
        
        self._msg_store.update_id(_id, new_msg)  
        

    def create_object_marker(self, soma_obj, soma_type, pose):
        # create an interactive marker for our server
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "/map"
        int_marker.name = soma_obj
        int_marker.description = "id" + soma_obj
        int_marker.pose = pose
        
        mesh_marker = Marker()
        mesh_marker.type = Marker.MESH_RESOURCE
        mesh_marker.scale.x = 1
        mesh_marker.scale.y = 1
        mesh_marker.scale.z = 1

        random.seed(soma_type)
        val = random.random()
        mesh_marker.color.r = r_func(val)
        mesh_marker.color.g = g_func(val)
        mesh_marker.color.b = b_func(val)
        mesh_marker.color.a = 1.0
        #mesh_marker.pose = pose
        mesh_marker.mesh_resource = self.mesh[soma_type]

        # create a control which will move the box
        # this control does not contain any markers,
        # which will cause RViz to insert two arrows
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.interaction_mode = InteractiveMarkerControl.MOVE_ROTATE

        if self._interactive:
            int_marker.controls.append(copy.deepcopy(control))
            # add the control to the interactive marker
            int_marker.controls.append(control);

        # add menu control
        menu_control = InteractiveMarkerControl()

        menu_control.interaction_mode = InteractiveMarkerControl.BUTTON
        menu_control.always_visible = True
        
        menu_control.markers.append( mesh_marker) #makeBox(int_marker) )
        int_marker.controls.append(menu_control)

        return int_marker
        

if __name__=="__main__":

    # TODO: add list command
    
    parser = argparse.ArgumentParser(prog='soma.py')
    parser.add_argument("map", nargs=1, help='Name of the used 2D map')
    parser.add_argument("conf", nargs=1, help='Name of the object configuration')
    parser.add_argument('-t', metavar='config-file')
                        
    args = parser.parse_args()
    
    rospy.init_node("soma")
    rospy.loginfo("Running SOMA (map: %s, conf: %s, types: %s)", args.map[0], args.conf[0], args.t)
    SOMAManager(args.map[0], args.conf[0],args.t)
    


