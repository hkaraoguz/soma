SOMa Visualizer
====

SOMa Visualizer is a GUI for querying and visualizing high-level SOMa objects.

This visual interface enables the user to easily create and execute advanced spatio-temporal queries while inspecting the high-level SOMa data. The returned SOMa objects are displayed in rviz as either point clouds or mesh models.

Prerequisites
-------------

- MongoDB (>=2.6)
- mongodb_store
- Qt5


Getting started (general steps)
-------------------------------
1. Start the ros core:

    ```
   $ roscore
    ```
2. Launch the ROS datacentre:

    ```
    $ roslaunch mongodb_store mongodb_store.launch db_path:=<path_to_db>

    ```

3. Launch the basic SOMa framework:
```
$ roslaunch soma_manager soma_local.launch map_name:=<map_name>
```

SOMa Visualizer
---------------
You can run the visualizer by calling
```
$ roslaunch soma_visualizer soma_visualizer.launch
```

1. Add a MarkerArray display type in RVIZ and subscribe to the `/soma_rois` topic for visualizing the selected ROI in the user interface. Subscribe to ```/soma_visualizer/soma_object_clouds``` and ```/soma_object_meshes``` to visualize the fetched SOMa objects.


2. Run rviz to visually inspect the results of the queries. You can go back and forth between time intervals using the slider. You can choose the time interval to be inspected in terms of days,hours and minutes. You can also execute more complex queries by setting date, time, region constraints. These constraints can be enabled by selecting the checkboxes. The query is executed when `Query` button is pressed. When you make changes in query, make sure to press `Query` button for fetching the results. You can also export the executed query in JSON format using the `Export JSON` button. You can reset the interface using `Reload` button. The returned objects are also displayed in the table view. You can double click on any of the rows to see the detailed information and images of that object. If there are multiple images, you can go through them by pressing to left and right buttons.

**If a query returns more than 50 objects, only the first 50 of them are fetched with point clouds and images in order to decrease the query execution time**.

![marker](https://github.com/hkaraoguz/soma/blob/visualizeraddons/soma_visualizer/doc/soma_visualizer.png)
