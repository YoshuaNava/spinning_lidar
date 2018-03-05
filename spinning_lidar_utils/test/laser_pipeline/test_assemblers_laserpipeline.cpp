

#include <ros/ros.h>
#include <laser_assembler/AssembleScans.h>
#include <laser_assembler/AssembleScans2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>

// using namespace laser_assembler;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_assembler_laserpipeline");
  ros::NodeHandle nh;

  std::string point_cloud_topic = "spinning_lidar/assembled_cloud";
  ros::Publisher point_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>(point_cloud_topic, 1);

  ros::service::waitForService("assemble_scans2");
  ros::ServiceClient client = nh.serviceClient<laser_assembler::AssembleScans2>("assemble_scans2");
  laser_assembler::AssembleScans2 assemble_srv;
  assemble_srv.request.begin = ros::Time(0,0);
  ros::Duration(1.0).sleep();
  assemble_srv.request.end   = ros::Time::now();
  if (client.call(assemble_srv))
  {
    point_cloud_pub.publish(assemble_srv.response.cloud);
    // ROS_INFO("Got cloud with %i points", assemble_srv.response.cloud.points.size());
  }
  else
  {
    ROS_INFO("Service call failed");
  }

  ros::Duration(1.0).sleep();
  ros::spinOnce();

  return EXIT_SUCCESS;
}
