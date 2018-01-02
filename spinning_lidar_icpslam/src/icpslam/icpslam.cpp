

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

	bool run_pose_optimization = false;
	int keyframes_window = 3;
	uint curr_vertex_key, prev_vertex_key, edge_key;
	prev_vertex_key = 0;
	curr_vertex_key = 0;

	Pose6DOF robot_odom_pose, prev_keyframe_pose, icp_odom_pose, prev_vertex_pose;

	long num_keyframes = 0, num_vertices = 0;
	int iter = 0;
	while(ros::ok())
    {
		if(icp_odometer.isOdomReady())
		{
			// ROS_INFO("Iteration %d", iter);
			bool is_keyframe = false;
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

			icp_odometer.getLatestPoseRobotOdometry(&robot_odom_pose);
			icp_odometer.getLatestCloud(cloud, &icp_odom_pose);

			Eigen::Vector3d keyframe_pose_diff = icp_odom_pose.pos - prev_keyframe_pose.pos;
			Eigen::Vector3d vertex_pose_diff = robot_odom_pose.pos - prev_vertex_pose.pos;
			
			if((keyframe_pose_diff.norm() > KFS_DIST_THRESH) || (num_keyframes == 0))
			{
				prev_keyframe_pose = icp_odom_pose;
				prev_vertex_pose = robot_odom_pose;
				num_keyframes++;
				num_vertices++;
				is_keyframe = true;
				pose_optimizer.addNewVertex(*cloud, robot_odom_pose, is_keyframe, &curr_vertex_key);
				ROS_INFO("##### Number of keyframes = %lu", num_keyframes);
				ROS_INFO("	Keyframe inserted! ID %d", curr_vertex_key);
				
				if(num_keyframes % keyframes_window == 0)
					run_pose_optimization = true;
			}
			else if (vertex_pose_diff.norm() > VERTEX_DIST_THRESH)
			{
				prev_vertex_pose = robot_odom_pose;
				num_vertices++;
				pose_optimizer.addNewVertex(*cloud, robot_odom_pose, is_keyframe, &curr_vertex_key);
				ROS_INFO("	Vertex inserted! ID %d", curr_vertex_key);
			}
	
			if ((num_vertices > 1) && (prev_vertex_key < curr_vertex_key))
			{
				pose_optimizer.addNewEdge(robot_odom_pose, prev_vertex_key, curr_vertex_key, &edge_key);
				ROS_INFO("	Edge inserted! ID %d", edge_key);
				ROS_INFO("		Between vertices	%d	 and   %d   ", prev_vertex_key, curr_vertex_key);
			}

			if(run_pose_optimization)
			{
				ROS_INFO("***** Graph pose optimization");
				bool success = pose_optimizer.optimizeGraph();

				if(success)
				{
					pose_optimizer.refineVertices();
				}
				run_pose_optimization = false;
			}

			prev_vertex_key = curr_vertex_key;
			pose_optimizer.publishPoseGraphMarkers();

			// std::cout << std::endl;

			iter++;
		}
        ros::spinOnce();
    }

	return EXIT_SUCCESS;
}