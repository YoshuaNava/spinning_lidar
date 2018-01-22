
#include "icpslam/icp_odometer.h"
#include "icpslam/octree_mapper.h"
#include "icpslam/pose_optimizer_g2o.h"
#include "icpslam/pose_optimizer_gtsam.h"


const double KFS_DIST_THRESH = 0.3;
const double VERTEX_DIST_THRESH = 0.2;



int main(int argc, char** argv)
{
	ros::init(argc, argv, "icpslam");
	ros::NodeHandle nh("~");

	ROS_INFO("#####       ICPSLAM         #####");	
	ICPOdometer icp_odometer(nh);
	OctreeMapper octree_mapper(nh);
	PoseOptimizerGTSAM* pose_optimizer = new PoseOptimizerGTSAM(nh);

	int keyframes_window = 3;
	unsigned int curr_vertex_key=0,
		 prev_vertex_key=0, 
		 last_keyframe_key=0, 
		 edge_key=0;
	bool last_keyframe_linked = true,
		 run_pose_optimization = false;
	long num_keyframes = 0,
		 num_vertices = 0,
		 iter = 0;

	Pose6DOF robot_odom_pose, 
			 prev_robot_odom_pose, 
			 icp_odom_pose, 
			 prev_keyframe_pose, 
			 icp_transform;

	while(ros::ok())
    {
		if(icp_odometer.isOdomReady())
		{
			// ROS_INFO("Iteration %d", iter);
			bool new_transform = false;
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

			icp_odometer.getLatestCloud(&cloud, &icp_transform, &icp_odom_pose, &robot_odom_pose, &new_transform);
			Pose6DOF robot_odom_transform = Pose6DOF::subtract(prev_robot_odom_pose, robot_odom_pose);

			if(num_vertices == 0)
			{
				ROS_INFO("	Initial pose inserted");
				pose_optimizer->setInitialPose(robot_odom_pose);
				num_vertices++;
				prev_keyframe_pose = icp_odom_pose;
				prev_robot_odom_pose = robot_odom_pose;
				continue;
			}

			if((Pose6DOF::distanceEuclidean(icp_odom_pose, prev_keyframe_pose) > KFS_DIST_THRESH) && (cloud->points.size()>0))
			{
				ROS_INFO("##### Number of keyframes = %lu", num_keyframes+1);
				ROS_INFO("	Keyframe inserted! ID %d", curr_vertex_key+1);
				pose_optimizer->addNewFactor(&cloud, icp_transform, icp_odom_pose, &curr_vertex_key, true);
				num_keyframes++;
				num_vertices++;
				prev_keyframe_pose = icp_odom_pose;
				prev_robot_odom_pose = robot_odom_pose;

				if(num_keyframes % keyframes_window == 0)
					run_pose_optimization = true;

			}
			else if ((Pose6DOF::distanceEuclidean(robot_odom_pose, prev_robot_odom_pose) > VERTEX_DIST_THRESH) && (num_keyframes > 0))
			{
				ROS_INFO("	Odometry vertex inserted! ID %d", curr_vertex_key+1);
				pose_optimizer->addNewFactor(&cloud, robot_odom_transform, robot_odom_pose, &curr_vertex_key, false);
				num_vertices++;
				prev_robot_odom_pose = robot_odom_pose;
			}

			// pose_optimizer->refinePoseGraph();

			if(run_pose_optimization)
			{
				// pose_optimizer->refinePoseGraph();
				pose_optimizer->publishRefinedMap();
				run_pose_optimization = false;	
			}

			pose_optimizer->publishPoseGraphMarkers();
			

			prev_vertex_key = curr_vertex_key;

			iter++;
		}
        ros::spinOnce();
    }

	return EXIT_SUCCESS;
}