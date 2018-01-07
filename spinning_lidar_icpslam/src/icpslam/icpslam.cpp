
#include "icpslam/icp_odometer.h"
#include "icpslam/pose_optimizer.h"
#include "icpslam/octree_mapper.h"


const double KFS_DIST_THRESH = 0.5;
const double VERTEX_DIST_THRESH = 0.1;



int main(int argc, char** argv)
{
	ros::init(argc, argv, "icpslam");
	ros::NodeHandle nh("~");


	ROS_INFO("#####       ICPSLAM         #####");	
	ICPOdometer icp_odometer(nh);
	PoseOptimizer pose_optimizer(nh);
	OctreeMapper octree_mapper(nh);

	bool run_pose_optimization = false;
	int keyframes_window = 3;

	uint curr_vertex_key, prev_vertex_key, last_keyframe_key, edge_key;
	prev_vertex_key = 0;
	curr_vertex_key = 0;
	last_keyframe_key = 0;
	bool last_keyframe_linked = true;

	Pose6DOF robot_odom_pose, prev_robot_odom_pose, icp_odom_pose, prev_keyframe_pose, icp_latest_transform;

	long num_keyframes = 0, num_vertices = 0;
	int iter = 0;
	while(ros::ok())
    {
		if(icp_odometer.isOdomReady())
		{
			// ROS_INFO("Iteration %d", iter);
			bool new_transform = false;
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

			icp_odometer.getLatestCloud(&cloud, &icp_latest_transform, &icp_odom_pose, &robot_odom_pose, &new_transform);

			if(((Pose6DOF::distanceEuclidean(icp_odom_pose, prev_keyframe_pose) > KFS_DIST_THRESH) || (num_keyframes == 0)) && (cloud->points.size()>0))
			{
				pose_optimizer.addNewKeyframeVertex(&cloud, icp_latest_transform, icp_odom_pose, &curr_vertex_key);
				num_keyframes++;
				num_vertices++;
				prev_keyframe_pose = icp_odom_pose;
				prev_robot_odom_pose = robot_odom_pose;
				last_keyframe_key = curr_vertex_key;
				last_keyframe_linked = false;

				ROS_INFO("##### Number of keyframes = %lu", num_keyframes);
				// ROS_INFO("	Keyframe inserted! ID %d", curr_vertex_key);
				
				if((curr_vertex_key > prev_vertex_key) && (prev_vertex_key > 0))
				{
					Eigen::Matrix<double,6,6> covariance = Eigen::MatrixXd::Zero(6,6);
					for (int i = 0; i < 3; ++i)
						covariance(i, i) = 0.001;
					for (int i = 3; i < 6; ++i)
						covariance(i, i) = 0.002;
					pose_optimizer.addNewEdge(covariance, prev_vertex_key, curr_vertex_key, &edge_key);
					// ROS_INFO("	Edge inserted! ID %d", edge_key);
					// ROS_INFO("		Between vertices	%d	 and   %d   ", prev_vertex_key, curr_vertex_key);
				}

				if(num_keyframes % keyframes_window == 0)
					run_pose_optimization = true;
			}

			if ((Pose6DOF::distanceEuclidean(robot_odom_pose, prev_robot_odom_pose) > VERTEX_DIST_THRESH) && (num_keyframes > 0))
			{
				prev_robot_odom_pose = robot_odom_pose;
				num_vertices++;
				pose_optimizer.addNewOdometryVertex(&cloud, robot_odom_pose, &curr_vertex_key);
				ROS_INFO("	Odometry vertex inserted! ID %d", curr_vertex_key);


				Eigen::Matrix<double,6,6> covariance = Eigen::MatrixXd::Zero(6,6);
				for (int i = 0; i < 3; ++i)
						covariance(i, i) = 100;
				for (int i = 3; i < 6; ++i)
						covariance(i, i) = 200;
				pose_optimizer.addNewEdge(covariance, prev_vertex_key, curr_vertex_key, &edge_key);

				if(!last_keyframe_linked)
				{
					for (int i = 0; i < 3; ++i)
						covariance(i, i) = 0.01;
					for (int i = 3; i < 6; ++i)
						covariance(i, i) = 0.02;
					pose_optimizer.addNewEdge(covariance, last_keyframe_key, curr_vertex_key, &edge_key);
					last_keyframe_linked = true;
				}
				
				// ROS_INFO("	Edge inserted! ID %d", edge_key);
				// ROS_INFO("		Between vertices	%d	 and   %d   ", prev_vertex_key, curr_vertex_key);

				prev_vertex_key = curr_vertex_key;
			}
	


			if(run_pose_optimization)
			{
				bool success = pose_optimizer.optimizeGraph();

				// if(success)
				{
					pose_optimizer.refinePoseGraph();
					pose_optimizer.publishRefinedMap();
				}
				run_pose_optimization = false;
				ROS_INFO("***** Graph pose optimization");
			}

			pose_optimizer.publishPoseGraphMarkers();

			iter++;
		}
        ros::spinOnce();
    }

	return EXIT_SUCCESS;
}