#include "feature_extractor.hpp"





int main(int argc, char** argv)
{
	ros::init(argc, argv, "loam_feature_extractor");
	ros::NodeHandle nh;

	std::string laser_scan_topic, laser_link, base_link;
	nh.param("base_link", base_link, std::string("laser_mount"));
	nh.param("laser_link", laser_link, std::string("laser"));
	nh.param("laser_scan_topic", laser_scan_topic, std::string("spinning_lidar/scan"));

	ROS_INFO("Starting LOAM feature extractor");
	FeatureExtractorLOAM feature_extractor(nh, laser_scan_topic, laser_link, base_link);

	ros::spin();

	return EXIT_SUCCESS;
}
