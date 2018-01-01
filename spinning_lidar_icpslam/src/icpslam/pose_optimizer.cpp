
#include "utils/geometric_utils.h"
#include "utils/messaging_utils.h"
#include "icpslam/pose_optimizer.h"

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
#include "g2o/types/slam3d/types_slam3d.h"


PoseOptimizer::PoseOptimizer(ros::NodeHandle nh) :
    nh_(nh)
{
    init();
}

void PoseOptimizer::init()
{
    curr_vertex_key_ = 0;
    curr_edge_key_ = 0;
    graph_stamps_.clear();
    graph_scans_.clear();
    graph_poses_.clear();

    optimizer_ = new g2o::SparseOptimizer();
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(
        g2o::make_unique<g2o::BlockSolverX>(g2o::make_unique<g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>>()));
    optimizer_->setAlgorithm(solver);
    optimizer_->setVerbose(true);

    namespace_ = "icpslam";
    advertisePublishers();
}

void PoseOptimizer::setGraphMarkersProperties()
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

void PoseOptimizer::advertisePublishers()
{
    pose_graph_pub = nh_.advertise<visualization_msgs::Marker>(pose_graph_topic, 1);
    setGraphMarkersProperties();
}

/* Inspired on mrsmap by Jurg Stuckler */
void PoseOptimizer::addNewVertex(PointCloud new_cloud, Pose6DOF pose, TimeStamp stamp, bool is_keyframe, uint *key)
{
    *key = curr_vertex_key_;
    graph_stamps_.insert(std::pair<unsigned int, ros::Time>(*key, stamp));

    if(is_keyframe)
        graph_scans_.insert(std::pair<unsigned int, PointCloud>(*key, new_cloud));

    g2o::VertexSE3* v = new g2o::VertexSE3();
    v->setId(*key);
    g2o::SE3Quat se3_pose(pose.rot, pose.pos);

    if(curr_vertex_key_ == 0)
    {
        v->setEstimate(g2o::SE3Quat());
        // v->setEstimate( se3_pose );
        v->setFixed(true);
    }
    else
    {
        v->setEstimate( se3_pose );
    }
    optimizer_->addVertex( v );

    curr_vertex_key_++;
}

void PoseOptimizer::addNewEdge(Pose6DOF pose, TimeStamp stamp, uint vertex1_key, uint vertex2_key, uint *key)
{
    *key = curr_edge_key_;
    graph_stamps_.insert(std::pair<unsigned int, ros::Time>(*key, stamp));
    graph_poses_.insert(std::pair<unsigned int, Pose6DOF>(*key, pose));

    g2o::SE3Quat se3_pose(pose.rot, pose.pos);
	Eigen::Matrix< double, 6, 6 > meas_info = pose.cov.inverse();

	g2o::VertexSE3* vertex1 = dynamic_cast< g2o::VertexSE3* >( optimizer_->vertex( vertex1_key ) );
	g2o::VertexSE3* vertex2 = dynamic_cast< g2o::VertexSE3* >( optimizer_->vertex( vertex2_key ) );

	g2o::EdgeSE3* edge = new g2o::EdgeSE3();
	edge->vertices()[0] = vertex1;
	edge->vertices()[1] = vertex2;
	edge->setMeasurement( se3_pose );
	edge->setInformation( meas_info );
    optimizer_->addEdge( edge );

    curr_edge_key_++;
}

bool PoseOptimizer::checkLoopClosure()
{

}


void PoseOptimizer::publishPoseGraphMarkers()
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