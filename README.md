SOMa
====

Semantic Object Map (SOMa) package. SOMa is a framework designed for storing perceptual information at different abstraction levels. SOMa can store objects, regions of interest (ROI) and trajectories. The framework is based on ROS and MongoDB. The extracted data is stored with spatial and temporal information. As such it can be later queried with combination of spatio-temporal constraints.


Prerequisites
-------------

- MongoDB (>=2.6)
- ROS mongodb_store package
- ROS navigation stack (only map server)
- Qt5 (sudo apt-get install qtbase5-dev)


Getting started (general steps)
-------------------------------
1. Start the roscore:

    ```
   $ roscore
    ```
2. Launch the ROS datacentre:

    ```
    $ roslaunch mongodb_store mongodb_store.launch db_path:=<path_to_db>
    ```
By default, the SOMa data are stored in `somadata` database. The collections under this database are `object` for SOMa objects, `roi` for SOMa rois and `map` for 2D occupancy maps.

SOMa map manager
----------------
SOMa is based on the assumption that all the data are with respect to 2D global map frame. So it is **mandatory to publish a 2D map using SOMa map manager** before using any other component of SOMa. This node is used for storing, reading and publishing 2D map:
```
$ rosrun soma_map_manager soma_map_manager_node.py --mapname <map_name>
```
If there are any stored 2D occupancy maps in the datacenter, the name of the map could be inputted as an argument to the map manager. Alternatively, user can choose the map to be published from the outputted list. If there are no stored maps, it will wait for a 2D map to be published from map_server. In order to add a map to SOMa, run the map_server with a 2D map:
  ```
  $ rosrun map_server map_server <map.yaml>
  ```
where `map.yaml` specifies the map you want to load. After running the `map_server`, you should save the published map using the `SOMa map manager`.

If you want to check the published map, start RVIZ, add a Map display type and subscribe to the `soma/map` topic:

  ```
  $ rosrun rviz rviz
  ```

Running the basic SOMa framework
-------------------------
The easiest way to run the basic SOMa system is to use the provided launch file:
```
$ roslaunch soma_manager soma_local.launch map_name:=<map_name>
```
where `map_name` should be one of the stored maps in the SOMa framework.

SOMa Extensions
-----------------------
SOMa Framework has different extensions for handling and visualizing data.

1. For creating and editing convex regions [SOMa ROI manager](../indigo-devel/soma_roi_manager/)

2. For adding and editing virtual objects  [SOMa Object manager](../indigo-devel/soma_object_manager/)

3. For visualizing and querying the objects and regions [SOMa visualizer](../indigo-devel/soma_visualizer/)

4. For low-level perceptual data interface [SOMa llsd](../indigo-devel/soma_llsd/)

5. For high-level data interface [SOMa manager](../indigo-devel/soma_manager/)

6. For editing and visualizing human trajectories  [SOMa trajectory](../indigo-devel/soma_trajectory/)
