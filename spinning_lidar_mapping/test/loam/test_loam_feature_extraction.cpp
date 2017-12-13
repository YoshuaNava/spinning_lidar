#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>

#include <pcl_ros/impl/transforms.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>


const double min_range_scan = 0.5;
const double max_range_scan = 60.0;
const double smoothness_thresh = 0.05;
const double pts_dist_thresh = 0.1;
const int num_sectors = 4;


std::string ir_interrupt_topic;

bool system_inited = false;
double init_time;
double time_start;
double time_elapsed;
double time_curr_scan = 0;
double time_last_scan = 0;
bool new_sweep = false;

int laser_rot_dir = 1;

pcl::PointCloud<pcl::PointXYZHSV>::Ptr cloud_features(new pcl::PointCloud<pcl::PointXYZHSV>());
pcl::PointCloud<pcl::PointXYZHSV>::Ptr cloud_assembled_ds(new pcl::PointCloud<pcl::PointXYZHSV>());

ros::Publisher cloud_features_pub;
ros::Publisher cloud_assembled_ds_pub;
tf::TransformListener *tf_listener_ptr;

int cloud_sort_index[1200];
int cloud_neighbors_picked[1200];

int skipFrameNum = 2;
int skipFrameCount = 0;


void irInterruptCallback(const std_msgs::Empty::ConstPtr& msg)
{
	time_start = time_last_scan - init_time;
	new_sweep = true;
}


void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr &cloud_in_msg)
{
	// Timing variables initialization/update
	if (!system_inited)
	{
		init_time = cloud_in_msg->header.stamp.toSec();
		time_start = init_time;
		system_inited = true;
	}
	time_last_scan = time_curr_scan;
	time_curr_scan = cloud_in_msg->header.stamp.toSec();
	time_elapsed = time_curr_scan - init_time;

	// Creation of PCL container from ROS Point Cloud msg
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_pcl(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::fromROSMsg(*cloud_in_msg, *cloud_in_pcl);
	int cloud_in_size = cloud_in_pcl->points.size();

	// We create an empty point cloud object in which we will put points for feature extraction
	int cloud_size = 0;
	pcl::PointXYZHSV point_in;
	pcl::PointCloud<pcl::PointXYZHSV>::Ptr laser_cloud_tmp(new pcl::PointCloud<pcl::PointXYZHSV>()), laser_cloud(new pcl::PointCloud<pcl::PointXYZHSV>());
	for (int i = 0; i < cloud_in_size; i++)
	{
		point_in.x = cloud_in_pcl->points[i].x;
		point_in.y = cloud_in_pcl->points[i].y;
		point_in.z = cloud_in_pcl->points[i].z;
		point_in.h = time_elapsed;
		point_in.v = 0;

		// If the point is not too close, we add it to the point cloud
		if (!((fabs(point_in.x) < min_range_scan) && (fabs(point_in.y) < min_range_scan) && (fabs(point_in.z) < min_range_scan)))
		{
			laser_cloud_tmp->push_back(point_in);
			cloud_sort_index[cloud_size] = cloud_size;
			cloud_neighbors_picked[cloud_size] = 0;
			cloud_size++;
		}
	}

	if(new_sweep)
	{
		cloud_features->clear();
    	cloud_assembled_ds->clear();
		new_sweep = false;
	}
	
	try
	{
		tf::StampedTransform transform;
		tf_listener_ptr->lookupTransform("/laser_mount", "/laser", ros::Time(0), transform);
		pcl_ros::transformPointCloud(*laser_cloud_tmp, *laser_cloud, transform);
    }
    catch (tf::TransformException ex)
	{
		ROS_ERROR("%s", ex.what());
    }

	// Calculation of smoothness criterion for each point of the cloud
	//// For the sum(XLki - XLkj, j!=i), in a neighborhood of 5 points to each side of the central point P, 
	//// you have to add the coords of P 10 times and substract the coordinates from the other points once
	//// The magnitude of the sum over x, y and z is stored in the point cloud S component
	//// This is done for each point in the cloud
	for (int i = 5; i < cloud_size - 5; i++)
	{
		float diff_X = laser_cloud->points[i - 5].x + laser_cloud->points[i - 4].x + laser_cloud->points[i - 3].x + laser_cloud->points[i - 2].x + laser_cloud->points[i - 1].x - 10 * laser_cloud->points[i].x + laser_cloud->points[i + 1].x + laser_cloud->points[i + 2].x + laser_cloud->points[i + 3].x + laser_cloud->points[i + 4].x + laser_cloud->points[i + 5].x;
		float diff_Y = laser_cloud->points[i - 5].y + laser_cloud->points[i - 4].y + laser_cloud->points[i - 3].y + laser_cloud->points[i - 2].y + laser_cloud->points[i - 1].y - 10 * laser_cloud->points[i].y + laser_cloud->points[i + 1].y + laser_cloud->points[i + 2].y + laser_cloud->points[i + 3].y + laser_cloud->points[i + 4].y + laser_cloud->points[i + 5].y;
		float diff_Z = laser_cloud->points[i - 5].z + laser_cloud->points[i - 4].z + laser_cloud->points[i - 3].z + laser_cloud->points[i - 2].z + laser_cloud->points[i - 1].z - 10 * laser_cloud->points[i].z + laser_cloud->points[i + 1].z + laser_cloud->points[i + 2].z + laser_cloud->points[i + 3].z + laser_cloud->points[i + 4].z + laser_cloud->points[i + 5].z;

		laser_cloud->points[i].s = diff_X * diff_X + diff_Y * diff_Y + diff_Z * diff_Z;
	}

	for (int i = 5; i < cloud_size - 6; i++)
	{
		// Derivative at a point with respect to its right-side neighbor
		float diff_X = laser_cloud->points[i + 1].x - laser_cloud->points[i].x;
		float diff_Y = laser_cloud->points[i + 1].y - laser_cloud->points[i].y;
		float diff_Z = laser_cloud->points[i + 1].z - laser_cloud->points[i].z;
		float diff_right = diff_X * diff_X + diff_Y * diff_Y + diff_Z * diff_Z;

		if (diff_right > smoothness_thresh)
		{
			// Estimation of overall depth/range of the points with respect to the laser
			float depth_pt = sqrt(laser_cloud->points[i].x * laser_cloud->points[i].x +
								laser_cloud->points[i].y * laser_cloud->points[i].y +
								laser_cloud->points[i].z * laser_cloud->points[i].z);

			float depth_next_pt = sqrt(laser_cloud->points[i + 1].x * laser_cloud->points[i + 1].x +
								laser_cloud->points[i + 1].y * laser_cloud->points[i + 1].y +
								laser_cloud->points[i + 1].z * laser_cloud->points[i + 1].z);
			
			// If the depth of point 1 is higher than that of point 2, normalize to point 2 depth, and viceversa
			if(depth_pt > depth_next_pt)
			{
				diff_X = laser_cloud->points[i + 1].x - laser_cloud->points[i].x * (depth_next_pt/depth_pt);
				diff_Y = laser_cloud->points[i + 1].y - laser_cloud->points[i].y * (depth_next_pt/depth_pt);
				diff_Z = laser_cloud->points[i + 1].z - laser_cloud->points[i].z * (depth_next_pt/depth_pt);
				float mag_diff = sqrt(diff_X * diff_X + diff_Y * diff_Y + diff_Z * diff_Z) / depth_pt;

				// If the projection of both points is about the same, choose point and neighbors
				if(mag_diff < pts_dist_thresh)
				{
					cloud_neighbors_picked[i-5] = 1;
					cloud_neighbors_picked[i-4] = 1;
					cloud_neighbors_picked[i-3] = 1;
					cloud_neighbors_picked[i-2] = 1;
					cloud_neighbors_picked[i-1] = 1;
					cloud_neighbors_picked[i] = 1;
				}
			}
			else
			{
				diff_X = laser_cloud->points[i + 1].x * (depth_pt/depth_next_pt) - laser_cloud->points[i].x;
				diff_Y = laser_cloud->points[i + 1].y * (depth_pt/depth_next_pt) - laser_cloud->points[i].y;
				diff_Z = laser_cloud->points[i + 1].z * (depth_pt/depth_next_pt) - laser_cloud->points[i].z;
				float mag_diff = sqrt(diff_X * diff_X + diff_Y * diff_Y + diff_Z * diff_Z) / depth_next_pt;

				// If the projection of both points is about the same, neighbors
				if(mag_diff < pts_dist_thresh)
				{
					cloud_neighbors_picked[i+1] = 1;
					cloud_neighbors_picked[i+2] = 1;
					cloud_neighbors_picked[i+3] = 1;
					cloud_neighbors_picked[i+4] = 1;
					cloud_neighbors_picked[i+5] = 1;
					cloud_neighbors_picked[i+6] = 1;
				}
			}
		}

		// Derivative at a point with respect to its left-side neighbor
		diff_X = laser_cloud->points[i].x - laser_cloud->points[i-1].x;
		diff_Y = laser_cloud->points[i].y - laser_cloud->points[i-1].y;
		diff_Z = laser_cloud->points[i].z - laser_cloud->points[i-1].z;
		float diff_left = diff_X * diff_X + diff_Y * diff_Y + diff_Z * diff_Z;

		float sqrd_dist_pt = laser_cloud->points[i].x * laser_cloud->points[i].x + laser_cloud->points[i].y * laser_cloud->points[i].y + laser_cloud->points[i].z * laser_cloud->points[i].z;

		// what does 0.25 and 20 represent?
		if ((diff_right > (0.25 * 0.25) / (20 * 20) * sqrd_dist_pt) && (diff_left > (0.25 * 0.25) / (20 * 20) * sqrd_dist_pt))
		{
			cloud_neighbors_picked[i] = 1;
		}
	}

	// Feature containers
	pcl::PointCloud<pcl::PointXYZHSV>::Ptr corner_points_sharp(new pcl::PointCloud<pcl::PointXYZHSV>());
	pcl::PointCloud<pcl::PointXYZHSV>::Ptr corner_points_less_sharp(new pcl::PointCloud<pcl::PointXYZHSV>());
	pcl::PointCloud<pcl::PointXYZHSV>::Ptr surf_points_flat(new pcl::PointCloud<pcl::PointXYZHSV>());
	pcl::PointCloud<pcl::PointXYZHSV>::Ptr surf_points_less_flat(new pcl::PointCloud<pcl::PointXYZHSV>());

	// Division of the point cloud into 4 sectors
	int start_points[num_sectors] = {5, 6 + int((cloud_size - 10) / 4.0), 6 + int((cloud_size - 10) / 2.0), 6 + int(3 * (cloud_size - 10) / 4.0)};
	int end_points[num_sectors] = {5 + int((cloud_size - 10) / 4.0), 5 + int((cloud_size - 10) / 2.0), 5 + int(3 * (cloud_size - 10) / 4.0), cloud_size - 6};

	// Feature extraction per sector
	for (int i = 0; i < num_sectors; i++)
	{
		int start_pt_index = start_points[i];
		int end_pt_index = end_points[i];

		// Ordering of cloud points based on their smoothness value
		for (int j = start_pt_index + 1; j <= end_pt_index; j++)
		{
			for (int k = j; k >= start_pt_index + 1; k--)
			{
				if (laser_cloud->points[cloud_sort_index[k]].s < laser_cloud->points[cloud_sort_index[k - 1]].s)
				{
					int temp = cloud_sort_index[k - 1];
					cloud_sort_index[k - 1] = cloud_sort_index[k];
					cloud_sort_index[k] = temp;
				}
			}
		}

		int num_largest_picked = 0;
		for (int j = end_pt_index; j >= start_pt_index; j--)
		{
			// If a point neighbors have not been picked, and the point is not too close nor too far, we choose it
			if (cloud_neighbors_picked[cloud_sort_index[j]] == 0 && laser_cloud->points[cloud_sort_index[j]].s > pts_dist_thresh &&
				(fabs(laser_cloud->points[cloud_sort_index[j]].x) > min_range_scan || fabs(laser_cloud->points[cloud_sort_index[j]].y) > min_range_scan || fabs(laser_cloud->points[cloud_sort_index[j]].z) > min_range_scan) &&
				fabs(laser_cloud->points[cloud_sort_index[j]].x) < max_range_scan && fabs(laser_cloud->points[cloud_sort_index[j]].y) < max_range_scan && fabs(laser_cloud->points[cloud_sort_index[j]].z) < max_range_scan)
			{

				num_largest_picked++;

				//Edge points selection based on smoothness values
				if (num_largest_picked <= 2)
				{
					laser_cloud->points[cloud_sort_index[j]].v = 2;
					corner_points_sharp->push_back(laser_cloud->points[cloud_sort_index[j]]);
				}
				else if (num_largest_picked <= 20)
				{
					laser_cloud->points[cloud_sort_index[j]].v = 1;
					corner_points_less_sharp->push_back(laser_cloud->points[cloud_sort_index[j]]);
				}
				else
				{
					break;
				}

				cloud_neighbors_picked[cloud_sort_index[j]] = 1;
				for (int k = 1; k <= 5; k++)
				{
					float diff_X = laser_cloud->points[cloud_sort_index[j] + k].x - laser_cloud->points[cloud_sort_index[j] + k - 1].x;
					float diff_Y = laser_cloud->points[cloud_sort_index[j] + k].y - laser_cloud->points[cloud_sort_index[j] + k - 1].y;
					float diff_Z = laser_cloud->points[cloud_sort_index[j] + k].z - laser_cloud->points[cloud_sort_index[j] + k - 1].z;
					float diff = diff_X * diff_X + diff_Y * diff_Y + diff_Z * diff_Z;
					if (diff > smoothness_thresh)
					{
						break;
					}

					cloud_neighbors_picked[cloud_sort_index[j] + k] = 1;
				}
				for (int k = -1; k >= -5; k--)
				{
					float diff_X = laser_cloud->points[cloud_sort_index[j] + k].x - laser_cloud->points[cloud_sort_index[j] + k + 1].x;
					float diff_Y = laser_cloud->points[cloud_sort_index[j] + k].y - laser_cloud->points[cloud_sort_index[j] + k + 1].y;
					float diff_Z = laser_cloud->points[cloud_sort_index[j] + k].z - laser_cloud->points[cloud_sort_index[j] + k + 1].z;
					float diff = diff_X * diff_X + diff_Y * diff_Y + diff_Z * diff_Z;
					if (diff > smoothness_thresh)
					{
						break;
					}

					cloud_neighbors_picked[cloud_sort_index[j] + k] = 1;
				}
			}
		}

		int num_smallest_picked = 0;
		for (int j = start_pt_index; j <= end_pt_index; j++)
		{
			// If the point neighbors have not been picked, and it is not too close nor too far, we choose it
			if (cloud_neighbors_picked[cloud_sort_index[j]] == 0 &&
				laser_cloud->points[cloud_sort_index[j]].s < 0.1 &&
				(fabs(laser_cloud->points[cloud_sort_index[j]].x) > min_range_scan || fabs(laser_cloud->points[cloud_sort_index[j]].y) > min_range_scan || fabs(laser_cloud->points[cloud_sort_index[j]].z) > min_range_scan) &&
				fabs(laser_cloud->points[cloud_sort_index[j]].x) < max_range_scan && fabs(laser_cloud->points[cloud_sort_index[j]].y) < max_range_scan && fabs(laser_cloud->points[cloud_sort_index[j]].z) < max_range_scan)
			{
				// Point is chosen as part of a surface
				laser_cloud->points[cloud_sort_index[j]].v = -1;
				surf_points_flat->push_back(laser_cloud->points[cloud_sort_index[j]]);

				num_smallest_picked++;
				// If we have already picked too many planes, we break the loop
				if (num_smallest_picked >= 4)
				{
					break;
				}

				cloud_neighbors_picked[cloud_sort_index[j]] = 1;
				for (int k = 1; k <= 5; k++)
				{
					float diff_X = laser_cloud->points[cloud_sort_index[j] + k].x - laser_cloud->points[cloud_sort_index[j] + k - 1].x;
					float diff_Y = laser_cloud->points[cloud_sort_index[j] + k].y - laser_cloud->points[cloud_sort_index[j] + k - 1].y;
					float diff_Z = laser_cloud->points[cloud_sort_index[j] + k].z - laser_cloud->points[cloud_sort_index[j] + k - 1].z;
					float diff = diff_X * diff_X + diff_Y * diff_Y + diff_Z * diff_Z;
					if (diff > smoothness_thresh)
					{
						break;
					}

					cloud_neighbors_picked[cloud_sort_index[j] + k] = 1;
				}
				for (int k = -1; k >= -5; k--)
				{
					float diff_X = laser_cloud->points[cloud_sort_index[j] + k].x - laser_cloud->points[cloud_sort_index[j] + k + 1].x;
					float diff_Y = laser_cloud->points[cloud_sort_index[j] + k].y - laser_cloud->points[cloud_sort_index[j] + k + 1].y;
					float diff_Z = laser_cloud->points[cloud_sort_index[j] + k].z - laser_cloud->points[cloud_sort_index[j] + k + 1].z;
					float diff = diff_X * diff_X + diff_Y * diff_Y + diff_Z * diff_Z;
					if (diff > smoothness_thresh)
					{
						break;
					}

					cloud_neighbors_picked[cloud_sort_index[j] + k] = 1;
				}
			}
		}
	}

	for (int i = 0; i < cloud_size; i++)
	{
		if (laser_cloud->points[i].v == 0)
		{
			surf_points_less_flat->push_back(laser_cloud->points[i]);
		}
	}

	// Voxel grid filtering on the points that are not so flat
	pcl::PointCloud<pcl::PointXYZHSV>::Ptr surf_points_less_flat_dsz(new pcl::PointCloud<pcl::PointXYZHSV>());
	pcl::VoxelGrid<pcl::PointXYZHSV> down_size_filter;
	down_size_filter.setInputCloud(surf_points_less_flat);
	down_size_filter.setLeafSize(0.1, 0.1, 0.1);
	down_size_filter.filter(*surf_points_less_flat_dsz);

	*cloud_features += *corner_points_sharp;
	*cloud_features += *surf_points_flat;
	*cloud_assembled_ds += *corner_points_less_sharp;
	*cloud_assembled_ds += *surf_points_less_flat_dsz;

	cloud_in_pcl->clear();
	laser_cloud_tmp->clear();
	laser_cloud->clear();
	corner_points_sharp->clear();
	corner_points_less_sharp->clear();
	surf_points_flat->clear();
	surf_points_less_flat->clear();
	surf_points_less_flat_dsz->clear();

	if (skipFrameCount >= skipFrameNum)
	{
		skipFrameCount = 0;

		sensor_msgs::PointCloud2 cloud_features_msg, cloud_assembled_ds_msg;;
		pcl::toROSMsg(*cloud_features, cloud_features_msg);
		cloud_features_msg.header.stamp = ros::Time().fromSec(time_curr_scan);
		cloud_features_msg.header.frame_id = "laser_mount";
		cloud_features_pub.publish(cloud_features_msg);

		pcl::toROSMsg(*cloud_assembled_ds, cloud_assembled_ds_msg);
		cloud_assembled_ds_msg.header.stamp = ros::Time().fromSec(time_last_scan);
		cloud_assembled_ds_msg.header.frame_id = "laser_mount";
		cloud_assembled_ds_pub.publish(cloud_assembled_ds_msg);
		ROS_INFO("Features extracted");
	}
	skipFrameCount++;
}



int main(int argc, char** argv)
{
	ros::init(argc, argv, "test_loam_feature_extraction");
	ros::NodeHandle nh;

	nh.param("ir_interrupt_topic", ir_interrupt_topic, std::string("spinning_lidar/ir_interrupt"));
	ros::Subscriber ir_interrupt_sub = nh.subscribe(ir_interrupt_topic, 1, irInterruptCallback);

	ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/spinning_lidar/filtered_cloud", 2, laserCloudHandler);
	cloud_features_pub = nh.advertise<sensor_msgs::PointCloud2>("/spinning_lidar/features_cloud", 1);
	cloud_assembled_ds_pub = nh.advertise<sensor_msgs::PointCloud2>("/spinning_lidar/assembled_ds_cloud", 1);

	tf::TransformListener tf_listener;
	tf_listener_ptr = &tf_listener;

	ros::Duration(1.0).sleep();
	ros::spin();

	return EXIT_SUCCESS;
}
