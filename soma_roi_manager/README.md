SOMA ROI manager
----------------
SOMa ROI manager is useful for creating and storing convex regions on top of a 2D map which are called as SOMa ROIs. User can create different types of regions in arbitrary convex shapes.

1. If you want to create SOMA ROIs, run the SOMA ROI manager:

    ```
    $ rosrun soma_roi_manager soma_roi_node.py <config_name>
    ```
where `config_name` denotes a configuration name. By default, the configuration file `soma_roi_manager/config/default.json` is used to initialize the list of available ROI types. Alternatively, the following command can be used to use a different configuration file:

    ```
    $ rosrun soma_roi_manager soma_roi_node.py <config_name> -t /path/to/config/file <config>
    ```
2. In RVIZ, add an InteractiveMarker display type, and subscribe to the `/soma_roi/<config_name>/update` topic:

3. Add, delete, modify ROIs in RVIZ using the interactive marker and the context menu (right-mouse-click)

4. You can even run multiple ROI managers with different configurations using the command:

    ```
    $ rosrun soma_roi_manager soma_roi_node.py "[config_name1, config_name2]"
    ```
The ROIs for each configuration can be modified by subscribing the associated topic in RVIZ.

![marker](https://raw.githubusercontent.com/kunzel/soma/master/doc/images/soma_roi.png)
