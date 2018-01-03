
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
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"
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

    // g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(
    //     g2o::make_unique<g2o::BlockSolverX>(g2o::make_unique<g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>>()));

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(
        g2o::make_unique<g2o::BlockSolver_6_3>(g2o::make_unique<g2o::LinearSolverCholmod<g2o::BlockSolver_6_3::PoseMatrixType>>()));

    optimizer_->setAlgorithm(solver);
    optimizer_->setVerbose(true);

    namespace_ = "icpslam";
    pose_opt_iters = 10;

    loadParameters();
    advertisePublishers();
}

void PoseOptimizer::loadParameters()
{
    nh_.param("verbosity_level_", verbosity_level_, 1);

	nh_.param("map_frame", map_frame_, std::string("map"));
	nh_.param("odom_frame", odom_frame_, std::string("odom"));
	nh_.param("robot_frame", robot_frame_, std::string("base_link"));
	nh_.param("laser_frame", laser_frame_, std::string("laser"));

	nh_.param("graph_edges_topic", graph_edges_topic_, std::string("icpslam/graph_edges"));
    nh_.param("graph_vertices_topic", graph_vertices_topic_, std::string("icpslam/graph_vertices"));
    nh_.param("graph_keyframes_topic", graph_keyframes_topic_, std::string("icpslam/graph_keyframes"));
}

void PoseOptimizer::advertisePublishers()
{
    tf::TransformBroadcaster tf_broadcaster;
	tf_broadcaster_ptr_ = &tf_broadcaster;
    graph_edges_pub_ = nh_.advertise<visualization_msgs::Marker>(graph_edges_topic_, 1);
    graph_vertices_pub_ = nh_.advertise<visualization_msgs::Marker>(graph_vertices_topic_, 1);
    graph_keyframes_pub_ = nh_.advertise<visualization_msgs::Marker>(graph_keyframes_topic_, 1);
    increment_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(increment_cloud_topic_, 1);

    ros::Timer map_transform_timer_ = nh_.createTimer(ros::Duration(0.05), &PoseOptimizer::mapTransformCallback, this);

    setGraphMarkersProperties();
}

void PoseOptimizer::setGraphMarkersProperties()
{
    vertex_color_.r = 0;
    vertex_color_.g = 0;
    vertex_color_.b = 1;
    vertex_color_.a = 1;
    odom_edge_color_.r = 1;
    odom_edge_color_.g = 0;
    odom_edge_color_.b = 0;
    odom_edge_color_.a = 1;
    closure_edge_color_.r = 1;
    closure_edge_color_.g = 1;
    closure_edge_color_.b = 1;
    closure_edge_color_.a = 1;
    keyframes_color_.r = 0;
    keyframes_color_.g = 1;
    keyframes_color_.b = 0;
    keyframes_color_.a = 1;

    keyframes_scale_.x = 0.15;
    keyframes_scale_.y = 0.15;
    keyframes_scale_.z = 0.15;
    vertex_scale_.x = 0.05;
    vertex_scale_.y = 0.05;
    vertex_scale_.z = 0.05;
    closure_edge_scale_.x = 0.05;
    odom_edge_scale_.x = 0.05;
}

/* Inspired on mrsmap by Jurg Stuckler */
void PoseOptimizer::addNewVertex(PointCloud new_cloud, Pose6DOF pose, bool is_keyframe, uint *key)
{
    *key = curr_vertex_key_;

    g2o::VertexSE3* v = new g2o::VertexSE3();
    v->setId(*key);
    g2o::SE3Quat se3_pose(pose.rot, pose.pos);
    std::cout << "      se3_pose:\n" << se3_pose << std::endl;

    if(is_keyframe)
    {
        graph_scans_.insert(std::pair<uint, PointCloud>(*key, new_cloud));
        // v->setMarginalized(true);
    }

    if(curr_vertex_key_ == 0)
    {
        // v->setEstimate(g2o::SE3Quat());
        v->setEstimate( se3_pose );
        v->setFixed(true);
        // v->setMarginalized(true);
    }
    else
    {
        v->setEstimate( se3_pose );
    }

    optimizer_->addVertex( v );

    graph_stamps_.insert(std::pair<uint, ros::Time>(*key, pose.time_stamp));
    graph_poses_.insert(std::pair<uint, Pose6DOF>(*key, pose));

    curr_vertex_key_++;
}

void PoseOptimizer::addNewEdge(Pose6DOF pose, uint vertex1_key, uint vertex2_key, uint *key)
{
    *key = curr_edge_key_;
    // graph_poses_.insert(std::pair<uint, Pose6DOF>(*key, pose));

    g2o::SE3Quat se3_pose(pose.rot, pose.pos);
	Eigen::Matrix< double, 6, 6 > meas_info = pose.cov.inverse();
    std::cout << "      se3_pose:\n" << se3_pose << std::endl;
    // std::cout << "Covariance:\n" << pose.cov << std::endl;

	g2o::VertexSE3* vertex1 = dynamic_cast< g2o::VertexSE3* >( optimizer_->vertex( vertex1_key ) );
	g2o::VertexSE3* vertex2 = dynamic_cast< g2o::VertexSE3* >( optimizer_->vertex( vertex2_key ) );
    // std::cout << "Vertex1:\n" << vertex1 << std::endl;
    // std::cout << "Vertex2:\n" << vertex2 << std::endl;

	g2o::EdgeSE3* edge = new g2o::EdgeSE3();
    edge->setId(*key);
    edge->vertices()[0] = vertex1;
	edge->vertices()[1] = vertex2;
	edge->setMeasurement( se3_pose );
	edge->setInformation( meas_info );

    bool success = optimizer_->addEdge( edge );
    
    if(!success)
    {
        ROS_ERROR("Error adding edge between vertices %d and %d", vertex1_key, vertex2_key);
        return;
    }

    graph_stamps_.insert(std::pair<uint, ros::Time>(*key, pose.time_stamp));
    std::pair<uint, uint> edge_keys(vertex1_key, vertex2_key);
    graph_edges_.insert(std::pair<uint, std::pair<uint, uint>>(*key, edge_keys));

    curr_edge_key_++;
}

bool PoseOptimizer::optimizeGraph()
{
    optimizer_->save("icpslam_posegraph_before.g2o");
    optimizer_->initializeOptimization();
    optimizer_->computeActiveErrors();
    optimizer_->setVerbose(true);

    ROS_ERROR("Optimization iterating");
    int iters = optimizer_->optimize(pose_opt_iters);
    
    if (iters == 0)
    {
        ROS_ERROR("Pose graph optimization failed!");
        return false;
    }

    ROS_INFO("Pose graph optimization finished after %i iterations.", iters);
    // std::cout << "Results: " << optimizer_->vertices().size() << " nodes, " << optimizer_->edges().size() << " edges, " << "chi2: " << optimizer_->chi2() << "\n";
    
    optimizer_->save("icpslam_posegraph_after.g2o");

    return true;
}

void PoseOptimizer::refinePoseGraph()
{
    refineVertices();
    refineEdges();
}

void PoseOptimizer::refineVertices()
{
    ROS_INFO("Refining vertices");
    for(size_t v_key=0; v_key<optimizer_->vertices().size() ;v_key++)
    {
        g2o::VertexSE3* v = dynamic_cast< g2o::VertexSE3* >( optimizer_->vertex( v_key ) );
        Eigen::Matrix4d v_T = v->estimate().matrix();
        Pose6DOF v_pose(v_T);
        v_pose.pos = -1.0 * v_pose.pos;
        graph_poses_.find(v_key)->second = v_pose;
    }
}

void PoseOptimizer::refineEdges()
{
    ROS_INFO("Refining edges");
    uint edge_key = 0;
    for( g2o::HyperGraph::EdgeSet::iterator edge_ptr = optimizer_->edges().begin(); edge_ptr != optimizer_->edges().end(); edge_ptr++)
    {
        g2o::EdgeSE3* edge = dynamic_cast< g2o::EdgeSE3* >( *edge_ptr );
        uint vertex1_key = edge->vertices()[0]->id();
        uint vertex2_key = edge->vertices()[1]->id();
        g2o::VertexSE3* vertex1 = dynamic_cast< g2o::VertexSE3* >(optimizer_->vertex( vertex1_key ));
	    g2o::VertexSE3* vertex2 = dynamic_cast< g2o::VertexSE3* >(optimizer_->vertex( vertex2_key ));
        Eigen::Matrix4d e_T = (vertex1->estimate().inverse() * vertex2->estimate()).matrix();
        Pose6DOF new_pose;
    
        Pose6DOF e_pose(e_T);
        e_pose.pos = -1.0 * e_pose.pos;
        g2o::SE3Quat se3_pose(e_pose.rot, e_pose.pos);
        // edge->setMeasurement(se3_pose);

        edge_key++;
    }
}

Pose6DOF PoseOptimizer::getLatestPose()
{
    return graph_poses_.find(curr_vertex_key_-1)->second;
}

bool PoseOptimizer::checkLoopClosure()
{
    return false;
}

void PoseOptimizer::publishPoseGraphMarkers()
{
    visualization_msgs::Marker edges_marker;
    edges_marker.header.frame_id = "odom";
    edges_marker.header.stamp = ros::Time().now();
    edges_marker.ns = namespace_;
    edges_marker.id = 0;
    edges_marker.action = visualization_msgs::Marker::ADD;
    edges_marker.type = visualization_msgs::Marker::LINE_LIST;
    edges_marker.color = odom_edge_color_;
    edges_marker.scale = odom_edge_scale_;
    edges_marker.pose.position.x = 0;
    edges_marker.pose.position.y = 0;
    edges_marker.pose.position.z = 0;
    edges_marker.pose.orientation.x = 0;
    edges_marker.pose.orientation.y = 0;
    edges_marker.pose.orientation.z = 0;
    edges_marker.pose.orientation.w = 1;


    for (size_t i = 0; i < curr_edge_key_; ++i)
    {
        uint vertex1_key = graph_edges_.at(i).first;
        uint vertex2_key = graph_edges_.at(i).second;

        geometry_msgs::Point point1, point2;
        point1 = getROSPointFromPose6DOF(graph_poses_.at(vertex1_key));
        point2 = getROSPointFromPose6DOF(graph_poses_.at(vertex2_key));

        // ROS_INFO("Vis of Edge %d", i);
        // Pose6DOF pose1 = graph_poses_.at(vertex1_key);
        // Pose6DOF pose2 = graph_poses_.at(vertex2_key);
        // std::cout << "  vertex1: " << pose1.pos << "\n";
        // std::cout << "  vertex2: " << pose2.pos << "\n";

        edges_marker.points.push_back(point1);
        edges_marker.points.push_back(point2);
    }
    graph_edges_pub_.publish(edges_marker);


    visualization_msgs::Marker vertices_marker;
    vertices_marker.header.frame_id = "odom";
    vertices_marker.header.stamp = ros::Time().now();
    vertices_marker.ns = namespace_;
    vertices_marker.id = 1;
    vertices_marker.action = visualization_msgs::Marker::ADD;
    vertices_marker.type = visualization_msgs::Marker::SPHERE_LIST;
    vertices_marker.color = vertex_color_;
    vertices_marker.scale = vertex_scale_;
    vertices_marker.pose.position.x = 0;
    vertices_marker.pose.position.y = 0;
    vertices_marker.pose.position.z = 0;
    vertices_marker.pose.orientation.x = 0;
    vertices_marker.pose.orientation.y = 0;
    vertices_marker.pose.orientation.z = 0;
    vertices_marker.pose.orientation.w = 1;

    visualization_msgs::Marker keyframes_marker;
    keyframes_marker.header.frame_id = "odom";
    keyframes_marker.header.stamp = ros::Time().now();
    keyframes_marker.ns = namespace_;
    keyframes_marker.id = 2;
    keyframes_marker.action = visualization_msgs::Marker::ADD;
    keyframes_marker.type = visualization_msgs::Marker::SPHERE_LIST;
    keyframes_marker.color = keyframes_color_;
    keyframes_marker.scale = keyframes_scale_;
    keyframes_marker.pose.position.x = 0;
    keyframes_marker.pose.position.y = 0;
    keyframes_marker.pose.position.z = 0;
    keyframes_marker.pose.orientation.x = 0;
    keyframes_marker.pose.orientation.y = 0;
    keyframes_marker.pose.orientation.z = 0;
    keyframes_marker.pose.orientation.w = 1;

    for (size_t i = 0; i < curr_vertex_key_; ++i)
    {
        geometry_msgs::Point point = getROSPointFromPose6DOF(graph_poses_.at(i));

        // ROS_INFO("Vis of Vertex  %d", i);
        // Pose6DOF pose = graph_poses_.at(i);
        // std::cout << "  vertex: " << pose.pos;

        if(graph_scans_.count(i) == 1)
            keyframes_marker.points.push_back(point);
        else
            vertices_marker.points.push_back(point);
    }

    if(sizeof(vertices_marker.points) > 0)
        graph_vertices_pub_.publish(vertices_marker);

    if(sizeof(keyframes_marker.points) > 0)
        graph_keyframes_pub_.publish(keyframes_marker);
    
}

void PoseOptimizer::mapTransformCallback(const ros::TimerEvent&)
{
    tf::Transform transform;
    if(graph_poses_.size() > 0)
    {
        Pose6DOF pose = getLatestPose();

        // Publish transform between odom and map
        transform.setOrigin( tf::Vector3(pose.pos(0), pose.pos(1), pose.pos(2)) );
        transform.setRotation( tf::Quaternion(pose.rot.x(), pose.rot.y(), pose.rot.z(), pose.rot.w()) );
    }
    else
    {
        transform.setOrigin( tf::Vector3(0, 0, 0) );
        transform.setRotation( tf::Quaternion(0, 0, 0, 1) );
    }

    tf_broadcaster_ptr_->sendTransform(tf::StampedTransform(transform.inverse(), ros::Time::now(), odom_frame_, map_frame_));
}

void PoseOptimizer::publishRefinedMap()
{
    for (size_t i = 0; i < curr_vertex_key_; ++i)
    {
        if(graph_scans_.count(i) == 1)
        {
            Pose6DOF pose = graph_poses_.at(i);
            PointCloud::Ptr cloud(&graph_scans_.at(i));
            PointCloud::Ptr cloud_out(new PointCloud());

            tf::StampedTransform transform;
            transform.setOrigin(tf::Vector3(pose.pos(0), pose.pos(1), pose.pos(2)));
            transform.setRotation(tf::Quaternion(pose.rot.x(), pose.rot.y(), pose.rot.z(), pose.rot.w()));
            pcl_ros::transformPointCloud(*cloud, *cloud_out, transform);

            publishPointCloud(cloud_out, map_frame_, ros::Time().now(), &increment_cloud_pub_);
        }
    }   
}

