#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

ros::Publisher filtered_pub, planes_pub;

double voxel_leaf_size = 0.1;
double plane_dist_thresh = 0.01;

void 
cloud_cb (const pcl::PCLPointCloud2ConstPtr& input)
{
    ROS_INFO("New cloud received! It will be voxel-filtered and planes will be extracted from it");
    pcl::PCLPointCloud2 filtered_cloud, planes_cloud;
    
    // Voxel grid filtering
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(input);
    sor.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
    sor.filter(filtered_cloud);
    filtered_pub.publish(filtered_cloud);

    // Planes segmentation
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>), cloud_p (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(filtered_cloud, cloud);
    cloud_ptr = cloud.makeShared();
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices ());
    // Segmentation object declaration
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(plane_dist_thresh);
  
    // Indices extractor
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    int i = 0, num_points = (int) cloud_ptr->points.size ();
    // While 30% of the original cloud is still there
    int num_planes = 5;
    while ((cloud_ptr->points.size() > 0.3 * num_points) && (i < num_planes))
    {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud(cloud_ptr);
        seg.segment(*inliers, *coefficients);
        if (inliers->indices.size () == 0)
        {
        std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
        break;
        }

        // Extract the inliers
        extract.setInputCloud(cloud_ptr);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*cloud_p);
        std::cerr << "PointCloud representing the planar component: " << (cloud_p->width * cloud_p->height) << " data points." << std::endl;

        // Create the filtering object
        extract.setNegative(true);
        extract.filter (*cloud_f);
        cloud_ptr.swap(cloud_f);
        i++;

        ros::Duration(1.0).sleep();
        // Publish the planes point cloud
        pcl::toPCLPointCloud2 (*cloud_p, planes_cloud);
        planes_pub.publish(planes_cloud);
    }
}

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "test_pcl_ros");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe("/spinning_lidar/assembled_cloud", 1, cloud_cb);

    // Create ROS publishers for the output point clouds
    filtered_pub = nh.advertise<sensor_msgs::PointCloud2>("/spinning_lidar/filtered_cloud", 1);
    planes_pub = nh.advertise<sensor_msgs::PointCloud2> ("/spinning_lidar/planes_pub", 1);

    // Spin
    ros::spin();
}