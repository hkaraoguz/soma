SOMa manager
------------
SOMa manager acts as an interface at high-level semantic layer. The user can interact with the data using provided ROS services. The high-level data that is stored in SOMa is more complete object models which are usually the outputs of task/user-specific reasoning approaches.

ROS Services
--------
The SOMa services are enabled by running the associated nodes.
## SOMa data manager
This node provides services for inserting, modifying and deleting high-level semantic data.
1. Run the soma data manager:
```
$ rosrun soma_manager data_manager_node.py --db_name <db_name>
--object_collection_name <object_collection_name> --roi_collection_name <roi__collection_name>
```
The parameters `db_name` and `x_collection_name` are optional which can be used to define the database and collection name for data storage.

## SOMa query manager
This node provides the spatio-temporal query interface for high-level semantic layer.
1. Run the soma query manager:
```
$ rosrun soma_query_manager query_manager_node
<object_db_name> <object_collection_name> <roi_db_name> <roi_collection_name>
```

By default the high-level data is stored under default db and collections :

|                 |  object  |    ROI   | trajectory |   map   |
|:---------------:|:--------:|:--------:|:----------:|:--------:|
|     db_name     | somadata | somadata |  somadata  | somadata |
| collection_name |  object  |    roi   | trajectory |     map   |


### Object insertion
One or multiple SOMa objects can be inserted using the SOMa service call `/soma/insert_objects`. The unique mongodb ids and a boolean value are returned. The boolean return value indicates whether the request was successfully completed or not.
### Object deletion
One or multiple SOMa objects can be deleted using the SOMa service call `/soma/delete_objects`. The SOMa object ids and configurations are used for deletion. The boolean return value determines whether the request was successfully completed or not.
### Object update
A SOMa object can be updated using the SOMa service call `/soma/update_object`. The boolean return value determines whether the request was successfully completed or not.
### Object query
SOMa objects could be queried using SOMa service call `/soma/query_objects`. The query request should be filled according to the spatio-temporal constraints. The results are returned based on the query type and constraints.

### ROI insertion
One or multiple SOMa rois can be inserted using the SOMa service call `/soma/insert_rois`. The unique mongodb ids and a boolean value are returned. The boolean return value indicates whether the request was successfully completed or not.
### ROI deletion
One or multiple SOMa ROIs can be deleted using the SOMa service call `/soma/delete_rois`. The SOMA ROI ids and configurations are used for deletion. The boolean return value indicates whether the request was successfully completed or not.
### ROI update
A SOMa ROI can be updated using the SOMa service call `/soma/update_roi`. The boolean return value indicates whether the request was successfully completed or not.
### ROI query
SOMa ROIs could be queried using SOMa service call `/soma/query_rois`. The query request should be filled according to the spatio-temporal constraints. The results are returned based on the query type and constraints.
