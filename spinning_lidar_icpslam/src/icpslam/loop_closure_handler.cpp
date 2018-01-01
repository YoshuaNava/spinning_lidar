
#include "utils/geometric_utils.h"
#include "utils/messaging_utils.h"
#include "icpslam/loop_closure_handler.h"

#include <random>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <pcl_ros/impl/transforms.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/filters/voxel_grid.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
#include "g2o/types/icp/types_icp.h"


LoopClosureHandler::LoopClosureHandler(ros::NodeHandle nh) :
    nh_(nh)
{
    init();
}

void LoopClosureHandler::init()
{
    curr_key_ = 0;
    graph_stamps_.clear();
    graph_scans_.clear();
    graph_poses_.clear();
    optimizer.setVerbose(true);

    namespace_ = "icpslam";
    advertisePublishers();
}

void LoopClosureHandler::setGraphMarkersProperties()
{
    vertex_color_.r = 0;
    vertex_color_.g = 0;
    vertex_color_.b = 1;
    odom_edge_color_.r = 1;
    odom_edge_color_.g = 0;
    odom_edge_color_.b = 0;
    closure_edge_color_.r = 1;
    closure_edge_color_.g = 1;
    closure_edge_color_.b = 1;
    kf_edge_color_.r = 0;
    kf_edge_color_.g = 1;
    kf_edge_color_.b = 0;

    markers_scale_.x = 0.1;
    markers_scale_.y = 0.1;
    markers_scale_.z = 0.1;
}

void LoopClosureHandler::advertisePublishers()
{
    pose_graph_pub = nh_.advertise<visualization_msgs::Marker>(pose_graph_topic, 1);
    setGraphMarkersProperties();
}

void LoopClosureHandler::addNewKeyscan(PointCloud new_cloud, TimeStamp stamp, uint *key)
{
    *key = curr_key_;
    graph_stamps_.insert(std::pair<unsigned int, ros::Time>(*key, stamp));
    graph_scans_.insert(std::pair<unsigned int, PointCloud>(*key, new_cloud));
    curr_key_++;
}

void LoopClosureHandler::addNewPose(Pose6DOF pose, TimeStamp stamp, uint *key)
{
    *key = curr_key_;
    graph_stamps_.insert(std::pair<unsigned int, ros::Time>(*key, stamp));
    graph_poses_.insert(std::pair<unsigned int, Pose6DOF>(*key, pose));
    curr_key_++;
}

bool LoopClosureHandler::checkLoopClosure()
{
    // g2o::SparseOptimizer optimizer;
    // optimizer.setVerbose(true);
    // optimizer.setVerbose(false);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(
        g2o::make_unique<g2o::BlockSolverX>(g2o::make_unique<g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>>()));

    uint vertex_id = 0;

    // set up two poses
    for (size_t i=0; i<2; ++i)
    {
        // set up rotation and translation for this node
        Eigen::Vector3d t(0,0,i);
        Eigen::Quaterniond q;
        q.setIdentity();

        Eigen::Isometry3d cam; // camera pose
        cam = q;
        cam.translation() = t;

        // set up node
        g2o::VertexSE3 *vc = new g2o::VertexSE3();
        vc->setEstimate(cam);

        vc->setId(vertex_id);      // vertex id

        std::cerr << t.transpose() << " | " << q.coeffs().transpose() << std::endl;

        // set first cam pose fixed
        if (i==0)
            vc->setFixed(true);

        // add to optimizer
        optimizer.addVertex(vc);

        vertex_id++;             
    }
}


void LoopClosureHandler::publishPoseGraphMarkers()
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "odom";
    marker.header.stamp = ros::Time().now();
    marker.ns = namespace_;
    marker.id = 0;
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.color = vertex_color_;
    marker.scale = markers_scale_;


}