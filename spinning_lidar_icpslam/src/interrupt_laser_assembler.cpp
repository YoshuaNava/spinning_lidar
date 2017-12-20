

#include <ros/ros.h>
#include <laser_assembler/AssembleScans.h>
#include <laser_assembler/AssembleScans2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Empty.h>


std::string ir_interrupt_topic, assembled_cloud_topic, assemble_service;
ros::ServiceClient assemble_client;
laser_assembler::AssembleScans2 assemble_srv;
ros::Publisher point_cloud_pub;
int num_points_thesh = 5000;

void irInterruptCallback(const std_msgs::Empty::ConstPtr& msg)
{
	assemble_srv.request.end = ros::Time::now();
	if (assemble_client.call(assemble_srv))
	{
		if(assemble_srv.response.cloud.width > num_points_thesh)
		{
			point_cloud_pub.publish(assemble_srv.response.cloud);
			ROS_INFO("Got cloud with %i points", assemble_srv.response.cloud.width);
		}
	}
	else
	{
		ROS_INFO("Service call failed");
	}

	assemble_srv.request.begin = assemble_srv.request.end;
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "interrupt_laser_assembler");
	ros::NodeHandle nh;

	nh.param("ir_interrupt_topic", ir_interrupt_topic, std::string("spinning_lidar/ir_interrupt"));
	nh.param("assembled_cloud_topic", assembled_cloud_topic, std::string("spinning_lidar/assembled_cloud"));
	nh.param("assemble_service", assemble_service, std::string("assemble_scans2"));

	point_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>(assembled_cloud_topic, 1);
	ros::Subscriber ir_interrupt_sub = nh.subscribe(ir_interrupt_topic, 1, irInterruptCallback);

	ros::service::waitForService(assemble_service);
	assemble_client = nh.serviceClient<laser_assembler::AssembleScans2>(assemble_service);
		
	ros::spin();

	return EXIT_SUCCESS;
}
