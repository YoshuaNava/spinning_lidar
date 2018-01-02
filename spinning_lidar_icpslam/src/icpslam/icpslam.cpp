

#include "icpslam/icp_odometer.h"
#include "icpslam/pose_optimizer.h"


const double KFS_DIST_THRESH = 0.5;
const double VERTEX_DIST_THRESH = 0.2;


int main(int argc, char** argv)
{
	ros::init(argc, argv, "icpslam");
	ros::NodeHandle nh;

	ROS_INFO("#####       ICPSLAM         #####");	
	ICPOdometer icp_odometer(nh);
	PoseOptimizer pose_optimizer(nh);

	int keyframes_window = 3;
	uint curr_vertex_key, prev_vertex_key, edge_key;
	Pose6DOF robot_odom_pose, prev_edge_pose, icp_odom_pose, prev_keyframe_pose;
	prev_edge_pose.pos = Eigen::Vector3d(0,0,0);
	prev_edge_pose.rot = Eigen::Quaterniond(0,0,0,1);
	prev_keyframe_pose.pos = Eigen::Vector3d(0,0,0);
	prev_keyframe_pose.rot = Eigen::Quaterniond(0,0,0, 1);

	long num_keyframes = 0;
	while(ros::ok())
    {
		if(icp_odometer.isOdomReady())
		{
			bool is_keyframe = false;
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

			icp_odometer.getLatestPoseRobotOdometry(&robot_odom_pose);
			icp_odometer.getLatestCloud(cloud, &icp_odom_pose);

			Eigen::Vector3d edge_pose_diff = robot_odom_pose.pos - prev_edge_pose.pos;
			Eigen::Vector3d keyframes_pose_diff = icp_odom_pose.pos - prev_keyframe_pose.pos;
			
			if((keyframes_pose_diff.norm() > KFS_DIST_THRESH) || (num_keyframes == 0))
			{
				ROS_INFO("Keyframe inserted! Number %lu", num_keyframes+1);
				prev_keyframe_pose = robot_odom_pose;
				num_keyframes++;
				is_keyframe = true;
				pose_optimizer.addNewVertex(*cloud, robot_odom_pose, is_keyframe, &curr_vertex_key);
			}
			else if (edge_pose_diff.norm() > VERTEX_DIST_THRESH)
			{
				pose_optimizer.addNewVertex(*cloud, robot_odom_pose, is_keyframe, &curr_vertex_key);
				ROS_INFO("Vertex inserted! Number %d", curr_vertex_key+1);

				if ((curr_vertex_key > 1) && (prev_vertex_key < curr_vertex_key))
				{
					prev_edge_pose = robot_odom_pose;
					pose_optimizer.addNewEdge(robot_odom_pose, prev_vertex_key, curr_vertex_key, &edge_key);
				}
			}

			if(num_keyframes % keyframes_window == 0)
			{
				bool result = pose_optimizer.optimizeGraph();
			}

			prev_vertex_key = curr_vertex_key;
			pose_optimizer.publishPoseGraphMarkers();
		}
        ros::spinOnce();
    }

	return EXIT_SUCCESS;
}