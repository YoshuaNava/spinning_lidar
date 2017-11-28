

#include <ros/ros.h>
#include <laser_assembler/AssembleScans.h>
#include <laser_assembler/AssembleScans2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Empty.h>

ros::ServiceClient assemble_client;
laser_assembler::AssembleScans2 assemble_srv;
ros::Publisher point_cloud_pub;

void irInterruptCallback(const std_msgs::Empty::ConstPtr& msg)
{
	assemble_srv.request.end = ros::Time::now();
	if (assemble_client.call(assemble_srv))
	{
		point_cloud_pub.publish(assemble_srv.response.cloud);
		// ROS_INFO("Got cloud with %i points", assemble_srv.response.cloud.points.size());
	}
	else
	{
		ROS_INFO("Service call failed");
	}

	assemble_srv.request.begin = assemble_srv.request.end;
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "test_continuous_assemblers_node");
	ros::NodeHandle nh;

	std::string point_cloud_topic = "spinning_lidar/assembled_cloud";
	point_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>(point_cloud_topic, 1);
	ros::Subscriber ir_interrupt_sub = nh.subscribe("spinning_lidar/ir_interrupt", 1, irInterruptCallback);

	// ros::Duration(0.1).sleep();

	ros::service::waitForService("assemble_scans2");
	assemble_client = nh.serviceClient<laser_assembler::AssembleScans2>("assemble_scans2");
		
	ros::spin();

	return EXIT_SUCCESS;
}
