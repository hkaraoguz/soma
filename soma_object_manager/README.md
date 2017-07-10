SOMa object manager
-------------------
SOMa object manager can be used for virtually creating a scene on a 2D map using object models.

5. For running the SOMa object manager:

    ```
    $ rosrun soma_object_manager soma_object_maner_node.py <config_name>
    ```
where `config_name` denotes an object configuration within this map. By default, the configuration file `soma_objects/config/default.json` is used to initialize the list of available object types. Alternatively, the following command can be used to use a different configuration file:

    ```
    $ rosrun soma_object_manager soma_object_maner_node.py -t /path/to/config/file <map> <config_name>
    ```
6. In RVIZ, add an InteractiveMarker display type, and subsribe to the `/soma_objects/update` topic:
7. Add, delete, move, and rotate objects in RVIZ using the interactive marker and the context menu (right-mouse-click)

![marker](https://raw.githubusercontent.com/kunzel/soma/master/doc/images/soma_manager.png)
