#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <laser_geometry/laser_geometry.h>

#include <pcl_ros/impl/transforms.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>




class FeatureExtractorLOAM
{
public:
	ros::NodeHandle nh_;
	ros::Subscriber ir_interrupt_sub;
	message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub;

	FeatureExtractorLOAM(ros::NodeHandle nh, std::string laser_scan_topic, std::string laser_link, std::string base_link) :
		nh_(nh),
		laser_sub(nh_, laser_scan_topic, 10),
		tf_laser_filter(laser_sub, tf_listener, base_link, 10),
		cloud_in_pcl(new pcl::PointCloud<pcl::PointXYZ>()),
		valid_cloud(new pcl::PointCloud<pcl::PointXYZHSV>()),
		cloud_features(new pcl::PointCloud<pcl::PointXYZHSV>()),
		cloud_assembled_ds(new pcl::PointCloud<pcl::PointXYZHSV>()),
		surf_points_less_flat_dsz(new pcl::PointCloud<pcl::PointXYZHSV>()),
		edge_points_sharp(new pcl::PointCloud<pcl::PointXYZHSV>()),
		edge_points_less_sharp(new pcl::PointCloud<pcl::PointXYZHSV>()),
		surf_points_flat(new pcl::PointCloud<pcl::PointXYZHSV>()),
		surf_points_less_flat(new pcl::PointCloud<pcl::PointXYZHSV>())
	{
		nh_.param("verbose_level", verbose_level, 1);
		nh_.param("min_dist_to_sensor", min_dist_to_sensor, 0.5);
		nh_.param("tf_filter_tol", tf_filter_tol, 0.01);
		nh_.param("base_link", base_link_, std::string("laser_mount"));
		nh_.param("laser_link_", laser_link_, std::string("laser"));
		nh_.param("laser_scan_topic_", laser_scan_topic_, std::string("spinning_lidar/scan"));
		nh_.param("filtered_scan_topic", filtered_scan_topic, std::string("spinning_lidar/filtered_scan"));
		nh_.param("filtered_cloud_topic", filtered_cloud_topic, std::string("spinning_lidar/filtered_cloud"));
		nh_.param("features_cloud_topic", features_cloud_topic, std::string("spinning_lidar/features_cloud"));
		nh_.param("assembled_ds_cloud_topic", assembled_ds_cloud_topic, std::string("spinning_lidar/assembled_ds_cloud"));
		nh_.param("ir_interrupt_topic", ir_interrupt_topic, std::string("spinning_lidar/ir_interrupt"));

		filtered_cloud_pub = nh_.advertise<sensor_msgs::PointCloud2>(filtered_cloud_topic, 1);
		cloud_features_pub = nh.advertise<sensor_msgs::PointCloud2>(features_cloud_topic, 1);
		cloud_assembled_ds_pub = nh.advertise<sensor_msgs::PointCloud2>(assembled_ds_cloud_topic, 1);

		verbose_level = 1;
		edge_features_pub = nh.advertise<sensor_msgs::PointCloud2>("spinning_lidar/edge_features", 1);
		plane_features_pub = nh.advertise<sensor_msgs::PointCloud2>("spinning_lidar/plane_features", 1);
		smoothness_vis_pub = nh.advertise<sensor_msgs::PointCloud2>("spinning_lidar/smoothness_cloud", 1);

		tf_laser_filter.setTolerance(ros::Duration(tf_filter_tol));
		tf_laser_filter.registerCallback( boost::bind(&FeatureExtractorLOAM::laserScanCallback, this, _1) );
		ir_interrupt_sub = nh_.subscribe(ir_interrupt_topic, 2, &FeatureExtractorLOAM::irInterruptCallback, this);
	}


private:
	const float inf = std::numeric_limits<float>::infinity();
	const int MAX_NUM_PLANE_FEATURES = 4;
	const int MAX_NUM_EDGE_FEATURES = 2;
	const int MAX_NUM_EDGES_MAP = 20;
	const double MIN_RANGE_SCAN = 0.5;
	const double MAX_RANGE_SCAN = 60.0;
	const double SMOOTHNESS_THRESH = 0.05;
	const double EDGE_PLANE_THRESH = 0.1;
	const double PTS_DIST_THRESH = 0.1;
	const int NUM_SECTORS = 4;

	int verbose_level;
	double tf_filter_tol;
	double min_dist_to_sensor;
	ros::Publisher filtered_cloud_pub;
	tf::TransformListener tf_listener;
	tf::MessageFilter<sensor_msgs::LaserScan> tf_laser_filter;
	laser_geometry::LaserProjection laser_projector;

	std::string laser_scan_topic_, filtered_scan_topic, filtered_cloud_topic, features_cloud_topic, assembled_ds_cloud_topic, ir_interrupt_topic;
	std::string laser_link_, base_link_;
	ros::Publisher filtered_scan_pub;

	bool system_inited = false;
	double init_time;
	double time_start;
	double time_elapsed;
	double time_curr_scan = 0;
	double time_last_scan = 0;
	bool new_sweep = false;

	int cloud_size = 0;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_pcl;
	pcl::PointCloud<pcl::PointXYZHSV>::Ptr valid_cloud;
	pcl::PointCloud<pcl::PointXYZHSV>::Ptr cloud_features;
	pcl::PointCloud<pcl::PointXYZHSV>::Ptr cloud_assembled_ds;
	pcl::PointCloud<pcl::PointXYZHSV>::Ptr surf_points_less_flat_dsz;

	/* Feature containers */
	pcl::PointCloud<pcl::PointXYZHSV>::Ptr edge_points_sharp;
	pcl::PointCloud<pcl::PointXYZHSV>::Ptr edge_points_less_sharp;
	pcl::PointCloud<pcl::PointXYZHSV>::Ptr surf_points_flat;
	pcl::PointCloud<pcl::PointXYZHSV>::Ptr surf_points_less_flat;

	ros::Publisher cloud_features_pub;
	ros::Publisher cloud_assembled_ds_pub;

	// Publishers for debugging
	ros::Publisher edge_features_pub;
	ros::Publisher plane_features_pub;
	ros::Publisher smoothness_vis_pub;

	int cloud_sort_index[1200];
	int cloud_neighbors_picked[1200];

	void updateTimeVariables(const sensor_msgs::PointCloud2 cloud_in_msg)
	{
		if (!system_inited)
		{
			init_time = cloud_in_msg.header.stamp.toSec();
			time_start = init_time;
			system_inited = true;
		}
		time_last_scan = time_curr_scan;
		time_curr_scan = cloud_in_msg.header.stamp.toSec();
		time_elapsed = time_curr_scan - init_time;
	}


	void extractValidPointsNewCloud()
	{
		int cloud_in_size = cloud_in_pcl->points.size();
		cloud_size = 0;
		pcl::PointXYZHSV point_in;
		for (int i = 0; i < cloud_in_size; i++)
		{
			point_in.x = cloud_in_pcl->points[i].x;
			point_in.y = cloud_in_pcl->points[i].y;
			point_in.z = cloud_in_pcl->points[i].z;
			point_in.h = time_elapsed;
			point_in.v = 0;

			/* If the point is not too close, we add it to the point cloud */
			if ((fabs(point_in.x) >= MIN_RANGE_SCAN) || (fabs(point_in.y) >= MIN_RANGE_SCAN) || (fabs(point_in.z) >= MIN_RANGE_SCAN))
			{
				valid_cloud->push_back(point_in);
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
	}


	void estimateSmoothnessCloud()
	{
		/* Estimation of smoothness criterion for each point of the cloud
		** For the sum(XLki - XLkj, j!=i), in a neighborhood of 5 points to each side of the central point P, 
		** you have to add the coords of P 10 times and substract the coordinates from the other points once
		** The magnitude of the sum over x, y and z is stored in the point cloud S component
		** This is done for each point in the cloud */
		for (int i = 5; i < cloud_size - 5; i++)
		{
			float diff_X = valid_cloud->points[i - 5].x + valid_cloud->points[i - 4].x
						 + valid_cloud->points[i - 3].x + valid_cloud->points[i - 2].x 
						 + valid_cloud->points[i - 1].x - 10 * valid_cloud->points[i].x 
						 + valid_cloud->points[i + 1].x + valid_cloud->points[i + 2].x 
						 + valid_cloud->points[i + 3].x + valid_cloud->points[i + 4].x 
						 + valid_cloud->points[i + 5].x;
			float diff_Y = valid_cloud->points[i - 5].y + valid_cloud->points[i - 4].y 
						 + valid_cloud->points[i - 3].y + valid_cloud->points[i - 2].y 
						 + valid_cloud->points[i - 1].y - 10 * valid_cloud->points[i].y 
						 + valid_cloud->points[i + 1].y + valid_cloud->points[i + 2].y 
						 + valid_cloud->points[i + 3].y + valid_cloud->points[i + 4].y 
						 + valid_cloud->points[i + 5].y;
			float diff_Z = valid_cloud->points[i - 5].z + valid_cloud->points[i - 4].z 
						 + valid_cloud->points[i - 3].z + valid_cloud->points[i - 2].z 
						 + valid_cloud->points[i - 1].z - 10 * valid_cloud->points[i].z 
						 + valid_cloud->points[i + 1].z + valid_cloud->points[i + 2].z 
						 + valid_cloud->points[i + 3].z + valid_cloud->points[i + 4].z 
						 + valid_cloud->points[i + 5].z;

			valid_cloud->points[i].s = diff_X * diff_X + diff_Y * diff_Y + diff_Z * diff_Z;
		}

		if(verbose_level == 1)
		{
			sensor_msgs::PointCloud2 smoothness_cloud_msg;
			pcl::toROSMsg(*valid_cloud, smoothness_cloud_msg);
			smoothness_cloud_msg.header.stamp = ros::Time().fromSec(time_curr_scan);
			smoothness_cloud_msg.header.frame_id = "laser_mount";	
			smoothness_vis_pub.publish(smoothness_cloud_msg);
		}
	}


	void rejectInvalidPoints()
	{	
		for (int i = 5; i < cloud_size - 6; i++)
		{
			/* Derivative at a point with respect to its right-side neighbor */
			float diff_X = valid_cloud->points[i + 1].x - valid_cloud->points[i].x;
			float diff_Y = valid_cloud->points[i + 1].y - valid_cloud->points[i].y;
			float diff_Z = valid_cloud->points[i + 1].z - valid_cloud->points[i].z;
			float diff_right = diff_X * diff_X + diff_Y * diff_Y + diff_Z * diff_Z;

			if (diff_right > SMOOTHNESS_THRESH)
			{
				/* Estimation of overall depth/range of the points with respect to the laser */
				float depth_pt = sqrt(valid_cloud->points[i].x * valid_cloud->points[i].x +
									valid_cloud->points[i].y * valid_cloud->points[i].y +
									valid_cloud->points[i].z * valid_cloud->points[i].z);

				float depth_next_pt = sqrt(valid_cloud->points[i + 1].x * valid_cloud->points[i + 1].x +
									valid_cloud->points[i + 1].y * valid_cloud->points[i + 1].y +
									valid_cloud->points[i + 1].z * valid_cloud->points[i + 1].z);
				
				/* If the depth of point 1 is higher than that of point 2, normalize to point 2 depth, and viceversa */
				if(depth_pt > depth_next_pt)
				{
					diff_X = valid_cloud->points[i + 1].x - valid_cloud->points[i].x * (depth_next_pt/depth_pt);
					diff_Y = valid_cloud->points[i + 1].y - valid_cloud->points[i].y * (depth_next_pt/depth_pt);
					diff_Z = valid_cloud->points[i + 1].z - valid_cloud->points[i].z * (depth_next_pt/depth_pt);
					float mag_diff = sqrt(diff_X * diff_X + diff_Y * diff_Y + diff_Z * diff_Z) / depth_pt;

					/* If the projection of both points is about the same, choose point and neighbors */
					if(mag_diff < PTS_DIST_THRESH)
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
					diff_X = valid_cloud->points[i + 1].x * (depth_pt/depth_next_pt) - valid_cloud->points[i].x;
					diff_Y = valid_cloud->points[i + 1].y * (depth_pt/depth_next_pt) - valid_cloud->points[i].y;
					diff_Z = valid_cloud->points[i + 1].z * (depth_pt/depth_next_pt) - valid_cloud->points[i].z;
					float mag_diff = sqrt(diff_X * diff_X + diff_Y * diff_Y + diff_Z * diff_Z) / depth_next_pt;

					/* If the projection of both points is about the same, neighbors */
					if(mag_diff < PTS_DIST_THRESH)
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

			/* Derivative at a point with respect to its left-side neighbor */
			diff_X = valid_cloud->points[i].x - valid_cloud->points[i-1].x;
			diff_Y = valid_cloud->points[i].y - valid_cloud->points[i-1].y;
			diff_Z = valid_cloud->points[i].z - valid_cloud->points[i-1].z;
			float diff_left = diff_X * diff_X + diff_Y * diff_Y + diff_Z * diff_Z;

			float sqrd_dist_pt = valid_cloud->points[i].x * valid_cloud->points[i].x + valid_cloud->points[i].y * valid_cloud->points[i].y + valid_cloud->points[i].z * valid_cloud->points[i].z;

			// what does 0.25 and 20 represent?
			if ((diff_right > (0.25 * 0.25) / (20 * 20) * sqrd_dist_pt) && (diff_left > (0.25 * 0.25) / (20 * 20) * sqrd_dist_pt))
			{
				cloud_neighbors_picked[i] = 1;
			}
		}
	}


	void extractFeatures()
	{
		/* Division of the point cloud into 4 sectors */
		int start_points[NUM_SECTORS] = {5, 6 + int((cloud_size - 10) / 4.0), 6 + int((cloud_size - 10) / 2.0), 6 + int(3 * (cloud_size - 10) / 4.0)};
		int end_points[NUM_SECTORS] = {5 + int((cloud_size - 10) / 4.0), 5 + int((cloud_size - 10) / 2.0), 5 + int(3 * (cloud_size - 10) / 4.0), cloud_size - 6};

		/* Feature extraction per sector */
		for (int i = 0; i < NUM_SECTORS; i++)
		{
			int start_pt_index = start_points[i];
			int end_pt_index = end_points[i];

			/* Ordering of cloud points based on their smoothness value, from very smooth to less smooth (decreasing) */
			for (int j = start_pt_index + 1; j <= end_pt_index; j++)
			{
				for (int k = j; k >= start_pt_index + 1; k--)
				{
					if (valid_cloud->points[cloud_sort_index[k]].s < valid_cloud->points[cloud_sort_index[k - 1]].s)
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
				/* If a point neighbors have not been picked, and the point is not too close nor too far, we choose it */
				if (cloud_neighbors_picked[cloud_sort_index[j]] == 0 && 
					valid_cloud->points[cloud_sort_index[j]].s > EDGE_PLANE_THRESH &&
				   (fabs(valid_cloud->points[cloud_sort_index[j]].x) > MIN_RANGE_SCAN || fabs(valid_cloud->points[cloud_sort_index[j]].y) > MIN_RANGE_SCAN || fabs(valid_cloud->points[cloud_sort_index[j]].z) > MIN_RANGE_SCAN) &&
				   fabs(valid_cloud->points[cloud_sort_index[j]].x) < MAX_RANGE_SCAN && fabs(valid_cloud->points[cloud_sort_index[j]].y) < MAX_RANGE_SCAN && fabs(valid_cloud->points[cloud_sort_index[j]].z) < MAX_RANGE_SCAN)
				{
					num_largest_picked++;

					/* Edge points selection based on smoothness values
					** The best <MAX_NUM_EDGE_FEATURES> are used as features.
					** The next <MAX_NUM_EDGES_MAP> are used for building the map.
					** The rest are discarded */
					if (num_largest_picked <= MAX_NUM_EDGE_FEATURES)
					{
						valid_cloud->points[cloud_sort_index[j]].v = 2;
						edge_points_sharp->push_back(valid_cloud->points[cloud_sort_index[j]]);
					}
					else if (num_largest_picked <= MAX_NUM_EDGES_MAP)
					{
						valid_cloud->points[cloud_sort_index[j]].v = 1;
						edge_points_less_sharp->push_back(valid_cloud->points[cloud_sort_index[j]]);
					}
					else
					{
						break;
					}

					/* We flag the point and its neighbors as already chosen */
					cloud_neighbors_picked[cloud_sort_index[j]] = 1;
					for (int k = 1; k <= 5; k++)
					{
						float diff_X = valid_cloud->points[cloud_sort_index[j] + k].x 
									 - valid_cloud->points[cloud_sort_index[j] + k - 1].x;
						float diff_Y = valid_cloud->points[cloud_sort_index[j] + k].y 
									 - valid_cloud->points[cloud_sort_index[j] + k - 1].y;
						float diff_Z = valid_cloud->points[cloud_sort_index[j] + k].z 
									 - valid_cloud->points[cloud_sort_index[j] + k - 1].z;
						float diff = diff_X * diff_X + diff_Y * diff_Y + diff_Z * diff_Z;
						if (diff > SMOOTHNESS_THRESH)
						{
							break;
						}

						cloud_neighbors_picked[cloud_sort_index[j] + k] = 1;
					}
					for (int k = -1; k >= -5; k--)
					{
						float diff_X = valid_cloud->points[cloud_sort_index[j] + k].x - valid_cloud->points[cloud_sort_index[j] + k + 1].x;
						float diff_Y = valid_cloud->points[cloud_sort_index[j] + k].y - valid_cloud->points[cloud_sort_index[j] + k + 1].y;
						float diff_Z = valid_cloud->points[cloud_sort_index[j] + k].z - valid_cloud->points[cloud_sort_index[j] + k + 1].z;
						float diff = diff_X * diff_X + diff_Y * diff_Y + diff_Z * diff_Z;
						if (diff > SMOOTHNESS_THRESH)
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
				/* If the point neighbors have not been picked, and it is not too close nor too far, we choose it */
				if (cloud_neighbors_picked[cloud_sort_index[j]] == 0 &&
					valid_cloud->points[cloud_sort_index[j]].s < EDGE_PLANE_THRESH &&
					(fabs(valid_cloud->points[cloud_sort_index[j]].x) > MIN_RANGE_SCAN || fabs(valid_cloud->points[cloud_sort_index[j]].y) > MIN_RANGE_SCAN || fabs(valid_cloud->points[cloud_sort_index[j]].z) > MIN_RANGE_SCAN) &&
					fabs(valid_cloud->points[cloud_sort_index[j]].x) < MAX_RANGE_SCAN && fabs(valid_cloud->points[cloud_sort_index[j]].y) < MAX_RANGE_SCAN && fabs(valid_cloud->points[cloud_sort_index[j]].z) < MAX_RANGE_SCAN)
				{
					/* Point is chosen as part of a surface */
					valid_cloud->points[cloud_sort_index[j]].v = -1;
					surf_points_flat->push_back(valid_cloud->points[cloud_sort_index[j]]);

					num_smallest_picked++;

					/* If we have already picked too many planes, we break the loop */
					if (num_smallest_picked >= MAX_NUM_PLANE_FEATURES)
					{
						break;
					}

					/* We flag the point and its neighbors as already chosen */
					cloud_neighbors_picked[cloud_sort_index[j]] = 1;
					for (int k = 1; k <= 5; k++)
					{
						float diff_X = valid_cloud->points[cloud_sort_index[j] + k].x 
									 - valid_cloud->points[cloud_sort_index[j] + k - 1].x;
						float diff_Y = valid_cloud->points[cloud_sort_index[j] + k].y 
									 - valid_cloud->points[cloud_sort_index[j] + k - 1].y;
						float diff_Z = valid_cloud->points[cloud_sort_index[j] + k].z 
									 - valid_cloud->points[cloud_sort_index[j] + k - 1].z;
						float diff = diff_X * diff_X + diff_Y * diff_Y + diff_Z * diff_Z;
						if (diff > SMOOTHNESS_THRESH)
						{
							break;
						}

						cloud_neighbors_picked[cloud_sort_index[j] + k] = 1;
					}
					for (int k = -1; k >= -5; k--)
					{
						float diff_X = valid_cloud->points[cloud_sort_index[j] + k].x 
									 - valid_cloud->points[cloud_sort_index[j] + k + 1].x;
						float diff_Y = valid_cloud->points[cloud_sort_index[j] + k].y 
									 - valid_cloud->points[cloud_sort_index[j] + k + 1].y;
						float diff_Z = valid_cloud->points[cloud_sort_index[j] + k].z 
									 - valid_cloud->points[cloud_sort_index[j] + k + 1].z;
						float diff = diff_X * diff_X + diff_Y * diff_Y + diff_Z * diff_Z;
						if (diff > SMOOTHNESS_THRESH)
						{
							break;
						}

						cloud_neighbors_picked[cloud_sort_index[j] + k] = 1;
					}
				}
			}
		}

		/* Save the rest of the point cloud for mapping */
		for (int i = 0; i < cloud_size; i++)
		{
			if (valid_cloud->points[i].v == 0)
			{
				surf_points_less_flat->push_back(valid_cloud->points[i]);
			}
		}

		/* Voxel grid filtering on the rest of the point cloud, to regularize it */
		pcl::VoxelGrid<pcl::PointXYZHSV> down_size_filter;
		down_size_filter.setInputCloud(surf_points_less_flat);
		down_size_filter.setLeafSize(0.1, 0.1, 0.1);
		down_size_filter.filter(*surf_points_less_flat_dsz);


		if(verbose_level == 1)
		{
			sensor_msgs::PointCloud2 edge_features_msg, plane_features_msg;
			pcl::toROSMsg(*edge_points_sharp, edge_features_msg);
			pcl::toROSMsg(*surf_points_flat, plane_features_msg);
			edge_features_msg.header.stamp = ros::Time().fromSec(time_curr_scan);
			edge_features_msg.header.frame_id = "laser_mount";
			plane_features_msg.header.stamp = ros::Time().fromSec(time_curr_scan);
			plane_features_msg.header.frame_id = "laser_mount";	
			edge_features_pub.publish(edge_features_msg);
			plane_features_pub.publish(plane_features_msg);
		}
	}


	void concatenateClouds()
	{
		/* Concatenation of point clouds */
		*cloud_features += *edge_points_sharp;
		*cloud_features += *surf_points_flat;
		*cloud_assembled_ds += *edge_points_less_sharp;
		*cloud_assembled_ds += *surf_points_less_flat_dsz;

		/* Variables clearing */
		cloud_in_pcl->clear();
		valid_cloud->clear();
		edge_points_sharp->clear();
		edge_points_less_sharp->clear();
		surf_points_flat->clear();
		surf_points_less_flat->clear();
		surf_points_less_flat_dsz->clear();
	}


	void publishClouds()
	{
		sensor_msgs::PointCloud2 cloud_features_msg, cloud_assembled_ds_msg;
		pcl::toROSMsg(*cloud_features, cloud_features_msg);
		cloud_features_msg.header.stamp = ros::Time().fromSec(time_curr_scan);
		cloud_features_msg.header.frame_id = "laser_mount";
		cloud_features_pub.publish(cloud_features_msg);

		pcl::toROSMsg(*cloud_assembled_ds, cloud_assembled_ds_msg);
		cloud_assembled_ds_msg.header.stamp = ros::Time().fromSec(time_last_scan);
		cloud_assembled_ds_msg.header.frame_id = "laser_mount";
		cloud_assembled_ds_pub.publish(cloud_assembled_ds_msg);
	}




	void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
	{
		/* Filtering of laser scan measurements that are too close to the sensor */
		// Reference: https://github.com/RobustFieldAutonomyLab/spin_hokuyo/blob/master/src/hokuyo_robot_filter.cpp
		sensor_msgs::LaserScan filtered_scan;
		int num_range_meas = scan->ranges.size();
		int num_int_meas = scan->intensities.size();
		filtered_scan.header = scan->header;
		filtered_scan.angle_min = scan->angle_min;
		filtered_scan.angle_max = scan->angle_max;
		filtered_scan.angle_increment = scan->angle_increment;
		filtered_scan.time_increment = scan->time_increment;
		filtered_scan.scan_time = scan->scan_time;
		filtered_scan.range_min = 0;
		filtered_scan.range_max = inf;
		filtered_scan.intensities[num_int_meas];
		filtered_scan.ranges.resize(num_range_meas);
		for(int n=0; n<num_range_meas ;n++)
		{
			if (scan->ranges[n] > min_dist_to_sensor)
			{ 
				filtered_scan.ranges[n] = scan->ranges[n];
			}
			else
			{
				// Set range to inf to "remove" point if it is so close that it could be the laser platform
				filtered_scan.ranges[n] = inf;
			}
		}
		filtered_scan_pub.publish(filtered_scan);

		/* Projection of laser scans into point clouds using TF */
    	sensor_msgs::PointCloud2 filtered_cloud, cloud_in_msg;
		try
		{
			laser_projector.transformLaserScanToPointCloud(laser_link_, filtered_scan, filtered_cloud, tf_listener);
			laser_projector.transformLaserScanToPointCloud(base_link_, filtered_scan, cloud_in_msg, tf_listener);
		}
		catch (tf::TransformException& e)
		{
			// std::cout << e.what();
			return;
		}
		filtered_cloud.header = scan->header;
		filtered_cloud_pub.publish(filtered_cloud);

		/* Timing variables initialization/update */
		updateTimeVariables(cloud_in_msg);

		/* Conversion of ROS PointCloud2 msg to PCL container */
		pcl::fromROSMsg(cloud_in_msg, *cloud_in_pcl);

		/* We create an empty point cloud object in which we will put points for feature extraction */
		extractValidPointsNewCloud();

		/* Estimation of the point cloud smoothness using the criteria shown in the paper */
		estimateSmoothnessCloud();

		/* Rejection of points that are almost parallel to the laser rays, or on the borders of objects */
	    rejectInvalidPoints();

		/* Extraction of edge and plane features */
	    extractFeatures();

		/* Concatenation of current clouds */
	    concatenateClouds();

	    publishClouds();

	}




	void irInterruptCallback(const std_msgs::Empty::ConstPtr& msg)
	{
		time_start = time_last_scan - init_time;
		new_sweep = true;
	}



};



