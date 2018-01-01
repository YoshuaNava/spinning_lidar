
#ifndef LOOP_CLOSURE_HANDLER_H
#define LOOP_CLOSURE_HANDLER_H

#include "utils/geometric_utils.h"

#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include "g2o/core/sparse_optimizer.h"

class LoopClosureHandler
{
private:
    typedef ros::Time TimeStamp;
    typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

    uint curr_key_;
    std::map<u_int, TimeStamp> graph_stamps_;
    std::map<u_int, PointCloud> graph_scans_;
    std::map<u_int, Pose6DOF> graph_poses_;
    g2o::SparseOptimizer optimizer;

    ros::NodeHandle nh_;
    ros::Publisher pose_graph_pub;
    std::string namespace_, pose_graph_topic;

    std_msgs::ColorRGBA vertex_color_;
    std_msgs::ColorRGBA odom_edge_color_;
    std_msgs::ColorRGBA closure_edge_color_;
    std_msgs::ColorRGBA kf_edge_color_;
    geometry_msgs::Vector3 markers_scale_;


public:

    LoopClosureHandler(ros::NodeHandle nh);

    void init();

    void setGraphMarkersProperties();

    void advertisePublishers();

    void addNewKeyscan(PointCloud new_cloud, TimeStamp stamp, uint *key);

    void addNewPose(Pose6DOF pose, TimeStamp stamp, uint *key);

    bool checkLoopClosure();

    void publishPoseGraphMarkers();

};

#endif