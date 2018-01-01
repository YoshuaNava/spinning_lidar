

#include "icpslam/icp_odometer.h"
#include "icpslam/pose_optimizer.h"


int main(int argc, char** argv)
{
	ros::init(argc, argv, "icpslam");
	ros::NodeHandle nh;

	ROS_INFO("#####       ICPSLAM         #####");	
	ICPOdometer icp_odometer(nh);
	PoseOptimizer pose_optimizer(nh);

	while(ros::ok())
    {
		if(icp_odometer.isOdomReady())
		{
			Pose6DOF odom_pose = icp_odometer.getLatestPoseRobotOdometry();
		}
        ros::spinOnce();
    }

	return EXIT_SUCCESS;
}