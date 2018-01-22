
#include "icpslam/pose_optimizer_gtsam.h"

#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>

#include "utils/pose6DOF.h"
#include "utils/geometric_utils.h"
#include "utils/messaging_utils.h"


PoseOptimizerGTSAM::PoseOptimizerGTSAM(ros::NodeHandle nh) :
    PoseOptimizer(nh)
{
    init();
}

void PoseOptimizerGTSAM::init()
{
    curr_vertex_key_ = 0;
    curr_edge_key_ = 0;
    graph_scans_.clear();
    graph_poses_.clear();

    gtsam::ISAM2Params parameters;
    parameters.relinearizeSkip = true;
    parameters.relinearizeThreshold = 0.01;
    isam_.reset(new gtsam::ISAM2(parameters));

    namespace_ = "icpslam";
    pose_opt_iters = 30;

    loadParameters();
    advertisePublishers();
    ROS_INFO("Pose optimizer initialized. Backend = GTSAM");
}

gtsam::Pose3 toGtsamPose3(Pose6DOF &pose)
{
  gtsam::Vector3 translation(pose.pos(0), pose.pos(1), pose.pos(2));
  gtsam::Rot3 rotation(gtsam::Rot3(pose.rot));
  return gtsam::Pose3(rotation, translation);
}

Pose6DOF toPose6DOF(const gtsam::Pose3& pose3)
{
  Pose6DOF out;
  out.pos(0) = pose3.translation().x();
  out.pos(1) = pose3.translation().y();
  out.pos(2) = pose3.translation().z();
  out.rot = pose3.rotation().toQuaternion();
  return out;
}

gtsam::noiseModel::Gaussian::shared_ptr toGtsamGaussian(const Eigen::MatrixXd &cov)
{
  gtsam::Matrix66 gtsam_cov;

  for (int i = 0; i < 6; ++i)
    for (int j = 0; j < 6; ++j)
      gtsam_cov(i, j) = cov(i, j);

  return gtsam::noiseModel::Gaussian::Covariance(gtsam_cov);
}

void PoseOptimizerGTSAM::setInitialPose(Pose6DOF &initial_pose)
{
    ROS_INFO("  Setting initial pose");

    gtsam::Pose3 pose3 = toGtsamPose3(initial_pose);
    gtsam::Vector6 noise;
    noise << 0.001, 0.001, 0.001, 0.005, 0.005, 0.005;
    gtsam::noiseModel::Diagonal::shared_ptr covariance(gtsam::noiseModel::Diagonal::Sigmas(noise));

    gtsam::NonlinearFactorGraph new_factor;
    gtsam::Values new_value;
    new_factor.add(gtsam::PriorFactor<gtsam::Pose3>(curr_vertex_key_, pose3, covariance));
    new_value.insert(curr_vertex_key_, pose3);
    
    isam_->update(new_factor, new_value);
    graph_values_ = isam_->calculateEstimate();

    std::cout << "Vertex " << curr_vertex_key_ << "\n" << initial_pose;

    graph_poses_.insert(std::pair<uint, Pose6DOF>(curr_vertex_key_, initial_pose));
    curr_vertex_key_++;
}


void PoseOptimizerGTSAM::extendGraph(Pose6DOF &transform, Pose6DOF &pose, bool is_keyframe)
{
    gtsam::Pose3 delta = toGtsamPose3(transform);
    gtsam::Pose3 pose3 = toGtsamPose3(pose);
    // gtsam::noiseModel::Gaussian::shared_ptr covariance = toGtsamGaussian(transform.cov);

    gtsam::Vector6 noise;
    if(is_keyframe)
        noise << 0.5, 0.5, 0.5, 1.0, 1.0, 1.0;
    else
        noise << 1.0, 1.0, 1.0, 2.0, 2.0, 2.0;
    gtsam::noiseModel::Diagonal::shared_ptr covariance(gtsam::noiseModel::Diagonal::Sigmas(noise));


    gtsam::NonlinearFactorGraph new_factor;
    gtsam::Values new_value;
    new_factor.add(gtsam::BetweenFactor<gtsam::Pose3>(curr_vertex_key_-1, curr_vertex_key_, delta, covariance));
    new_value.insert(curr_vertex_key_, pose3);
    isam_->update(new_factor, new_value);
    graph_values_ = isam_->calculateEstimate();

    std::cout << "Vertex " << curr_vertex_key_ << "\n Pose: \n" << pose;
    std::cout << "Transform: \n" << transform;

    // graph_values_.print("Current pose graph:\n");
}


void PoseOptimizerGTSAM::addNewFactor(PointCloud::Ptr *new_cloud_ptr, Pose6DOF transform, Pose6DOF pose, uint *key, bool is_keyframe)
{
    *key = curr_vertex_key_;
    ROS_INFO("  New factor");
    this->extendGraph(transform, pose, is_keyframe);

    std::pair<uint, uint> edge_keys(curr_vertex_key_-1, curr_vertex_key_);
    graph_edges_.insert(std::pair<uint, std::pair<uint, uint>>(curr_edge_key_, edge_keys));
    graph_poses_.insert(std::pair<uint, Pose6DOF>(curr_vertex_key_, pose));
    if(is_keyframe)
      graph_scans_.insert(std::pair<uint, PointCloud::Ptr>(*key, *new_cloud_ptr));

    curr_vertex_key_++;
    curr_edge_key_++;

    this->refinePoseGraph();
}



bool PoseOptimizerGTSAM::optimizeGraph()
{
    // graph_values_.print("Optimized pose graph:\n");
    // graph_values_ = isam_->calculateEstimate();

    return true;
}

void PoseOptimizerGTSAM::refinePoseGraph()
{
    refineVertices();
}

void PoseOptimizerGTSAM::refineVertices()
{
    ROS_ERROR("Refining vertices");
    for (const auto& value : graph_values_)
    {
        const unsigned int key = value.key;
        Pose6DOF v_pose = toPose6DOF(graph_values_.at<gtsam::Pose3>(key));
        graph_poses_.find(key)->second.pos = v_pose.pos;
        graph_poses_.find(key)->second.rot = v_pose.rot;
        Pose6DOF new_pose = graph_poses_.find(key)->second;
        std::cout << "Vertex " << key << "\n" << new_pose;

        // if(v_key == optimizer_->vertices().size()-1)
        //     latest_pose = v_pose;
    }
    ROS_ERROR("Vertices refined");
}

void PoseOptimizerGTSAM::refineEdges()
{
    ROS_INFO("Refining edges");
    // uint edge_key = 0;
    // for( g2o::HyperGraph::EdgeSet::iterator edge_ptr = optimizer_->edges().begin(); edge_ptr != optimizer_->edges().end(); edge_ptr++)
    // {
        // g2o::EdgeSE3* edge = dynamic_cast< g2o::EdgeSE3* >( *edge_ptr );
        // uint vertex1_key = edge->vertices()[0]->id();
        // uint vertex2_key = edge->vertices()[1]->id();
        // g2o::VertexSE3* vertex1 = dynamic_cast< g2o::VertexSE3* >(optimizer_->vertex( vertex1_key ));
	    // g2o::VertexSE3* vertex2 = dynamic_cast< g2o::VertexSE3* >(optimizer_->vertex( vertex2_key ));
        // Eigen::Matrix4d e_T = (vertex1->estimate() * vertex2->estimate().inverse()).matrix();
        // Pose6DOF e_pose(e_T);
        // g2o::SE3Quat se3_pose(e_pose.rot, e_pose.pos);
        // edge->setMeasurement(se3_pose);

        // edge_key++;
    // }
}


bool PoseOptimizerGTSAM::checkLoopClosure()
{
    return false;
}

