
#include "icpslam/pose_optimizer_gtsam.h"

#include <gtsam/slam/dataset.h>
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
#include "utils/matplotlibcpp.h"


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

gtsam::noiseModel::Gaussian::shared_ptr toGtsamGaussian(const Eigen::VectorXd &cov)
{
  gtsam::Matrix66 gtsam_cov;

  for (int i = 0; i < 6; ++i)
      gtsam_cov(i, i) = cov(i);

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
    // transform.rot = transform.rot.inverse();
    gtsam::Pose3 delta = toGtsamPose3(transform);
    gtsam::Pose3 pose3 = toGtsamPose3(pose);
    // gtsam::noiseModel::Gaussian::shared_ptr covariance = toGtsamGaussian(transform.cov.diagonal());
    // std::cout << "Transform covariance = " << transform.cov.diagonal().transpose() << std::endl;

    gtsam::Vector6 noise;
    if(is_keyframe)
        noise << 100.0, 100.0, 100.0, 1.0, 1.0, 1.0;
    else
        noise << 1.0, 1.0, 1.0, 50.0, 50.0, 50.0;
        // noise = transform.cov.diagonal();

    gtsam::noiseModel::Diagonal::shared_ptr covariance(gtsam::noiseModel::Diagonal::Sigmas(noise));

    gtsam::NonlinearFactorGraph new_factor;
    gtsam::Values new_value;
    new_factor.add(gtsam::BetweenFactor<gtsam::Pose3>(curr_vertex_key_-1, curr_vertex_key_, delta, covariance));
    new_value.insert(curr_vertex_key_, pose3);
    isam_->update(new_factor, new_value);
    isam_->printStats();
    graph_values_ = isam_->calculateEstimate();

    const std::string output_file = "/home/alfredoso/factor_graph.txt";
    gtsam::writeG2o(isam_->getFactorsUnsafe(), graph_values_, output_file);

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
 
         // if(v_key == optimizer_->vertices().size()-1)
        //     latest_pose = v_pose;
    }
    ROS_ERROR("Vertices refined\n\n");
}

void PoseOptimizerGTSAM::refineEdges()
{
    // ROS_INFO("Refining edges");
}


bool PoseOptimizerGTSAM::checkLoopClosure()
{
    return false;
}

