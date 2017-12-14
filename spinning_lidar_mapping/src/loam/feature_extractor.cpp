#include "loam/feature_extractor.hpp"





int main(int argc, char** argv)
{
	ros::init(argc, argv, "loam_feature_extractor");
	ros::NodeHandle nh;

	std::string laser_scan_topic, laser_link;

	nh.param("laser_link", laser_link, std::string("laser"));
	nh.param("laser_scan_topic", laser_scan_topic, std::string("spinning_lidar/scan"));

	FeatureExtractorLOAM(nh, laser_scan_topic, laser_link);

	ros::spin();

	return EXIT_SUCCESS;
}
