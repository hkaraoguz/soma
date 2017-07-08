#include <QThread>
#include <ros/ros.h>
#include <QVector>
#include <QCollator>
#include <mongodb_store/MongoFind.h>
#include <mongodb_store/message_store.h>
#include <mongo/bson/bson.h>
#include <soma_msgs/SOMAObject.h>
#include <soma_map_manager/MapInfo.h>
#include <soma_msgs/SOMAROIObject.h>
#include <soma_manager/SOMAQueryObjs.h>
#include <soma_roi_manager/DrawROI.h>
#include <soma_manager/SOMADrawMesh.h>
#include <geometry_msgs/Pose.h>
#include <algorithm>    // std::unique, std::distance
#include <vector>       // std::vector

struct SOMAROINameIDConfig
{

    std::string id;
    std::string name;
    std::string config;

};

struct SOMATimeLimits
{


    long mintimestamp;
    long maxtimestamp;

};


class RosThread:public QObject
{
    Q_OBJECT

public:

    RosThread();

    sensor_msgs::PointCloud2 worldstate;

     // Shutdown the node
     void shutdownROS();


     void visualizeWorldState(const std::vector<soma_msgs::SOMAObject>& somaobjects);




     // Return the name of the current map
     std::string getMapName();

     // Service call for drawing roi with id
     void drawROIwithID(std::string id, std::string config);



     // Get the SOMA ROI with id
     soma_msgs::SOMAROIObject getSOMAROIwithIDConfig(int id, std::string config);


     // Query the SOMA objects
     std::vector<soma_msgs::SOMAObject> querySOMAObjects(soma_manager::SOMAQueryObjs &somaquery);

     void fetchDataFromDB();


     // Get the time limits of SOMa objects
     SOMATimeLimits getSOMACollectionMinMaxTimelimits();


private:
     bool shutdown;
     ros::NodeHandle n;
     ros::Publisher pcp;
     ros::ServiceClient roi_query_client;
     ros::ServiceClient object_query_client;
     ros::ServiceClient roi_draw_client;
     ros::ServiceClient mesh_draw_client;


     SOMATimeLimits limits;

     // Returns the meshes of soma objects
     soma_manager::SOMADrawMesh getObjectMeshes(const std::vector<soma_msgs::SOMAObject>& somaobjects);

     // Returns the combined cloud of soma objects
     sensor_msgs::PointCloud2 getCombinedObjectCloud(const std::vector<soma_msgs::SOMAObject>& somaobjects);

     // Publish SOMA combined Object Cloud
     void publishSOMAObjectCloud(sensor_msgs::PointCloud2 msg);

     // Get the ROIs from db
     void fetchSOMAROIs();

     // Get the object types,ids and configs from db
     void fetchSOMAObjectTypesIDsConfigs();

     void drawMesh(soma_manager::SOMADrawMesh drawmesh);

     std::string map_name;
     std::string map_unique_id;
     std::vector<SOMAROINameIDConfig> roinameidconfigs;
     std::vector<std::string> labelnames;
     std::vector <soma_msgs::SOMAROIObject> roiarray;

signals:

   void  rosStarted();
   void  rosStartFailed();
   void  rosFinished();
   void  mapinfoReceived();
   void  SOMAObjectTypes(std::vector<std::string>);
   void  SOMAObjectIDs(std::vector<std::string>);
   void  SOMAROINames(std::vector<SOMAROINameIDConfig>);

public slots:

   void loop();

};
Q_DECLARE_METATYPE (std::vector<std::string>)
Q_DECLARE_METATYPE (std::vector<SOMAROINameIDConfig>)
