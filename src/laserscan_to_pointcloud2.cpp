/*
 * laserscan_to_pointcloud2.cpp
 *
 *  Created on: 2017年6月8日
 *      Author: tbm
 */

#include "../include/laserscan_to_pointcloud2/laserscan_to_pointcloud2.h"
#include <iostream>
#include <string>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <functional>

#include <ros/ros.h>
#include <tf/transform_listener.h>

using namespace boost;



void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in, const ros::Publisher& pub )
{
	static laser_geometry::LaserProjection projector_;
	sensor_msgs::PointCloud2 cloud;
	projector_.projectLaser(*scan_in, cloud);
	pub.publish(cloud);
}

void concatenateCloud(sensor_msgs::PointCloud2Ptr cloud_ptr, sensor_msgs::PointCloud2Ptr cloud_ptr_10 )
{
	cloud_ptr_10->width += cloud_ptr->width;
	cloud_ptr_10->row_step += cloud_ptr->row_step;
	cloud_ptr_10->data.insert(cloud_ptr_10->data.end(), cloud_ptr->data.begin(), cloud_ptr->data.end());
	cloud_ptr_10->header.stamp = cloud_ptr->header.stamp;
}


void scanCallback2(const sensor_msgs::LaserScan::ConstPtr& scan_in, const ros::Publisher& pub)
{
	static int count = 0;
	static tf::TransformListener listener_;
	static sensor_msgs::PointCloud2Ptr cloud_ptr_10  = boost::make_shared<sensor_msgs::PointCloud2>();
	static laser_geometry::LaserProjection projector_;
	sensor_msgs::PointCloud2Ptr cloud_ptr = boost::make_shared<sensor_msgs::PointCloud2>();
	if(!listener_.waitForTransform(scan_in->header.frame_id, "party_1/instance_1/erlecopter_2/base_link_inertia", scan_in->header.stamp , ros::Duration(1.0)))
	{
		ROS_INFO("scan_in->header.frame_id: %s", (scan_in->header.frame_id).c_str());
		ROS_INFO("no transform to base_link_inertia");
		return ;
	}
	count++;
	projector_.transformLaserScanToPointCloud("party_1/instance_1/erlecopter_2/base_link_inertia",*scan_in, *cloud_ptr,listener_);
	if(count % 40 == 1)
	{
		count = 1;
		//cloud_ptr_10.reset();
		//cloud_ptr_10 = boost::make_shared<sensor_msgs::PointCloud2>();
		cloud_ptr_10->height = 1;
		cloud_ptr_10->width = 0;
		cloud_ptr_10->point_step = 20;
		cloud_ptr_10->row_step = 0;
		cloud_ptr_10->fields = cloud_ptr->fields;
		cloud_ptr_10->is_bigendian = false;
		cloud_ptr_10->is_dense = false;
		cloud_ptr_10->data.clear();
		cloud_ptr_10->data.reserve(144000);
		cloud_ptr_10->header.frame_id = cloud_ptr->header.frame_id;
		cloud_ptr_10->header.stamp = cloud_ptr->header.stamp;
	}
	concatenateCloud(cloud_ptr, cloud_ptr_10);
	if(count % 40 == 0 )
	{
		pub.publish(cloud_ptr_10);
	}
}

void scanCallback3(const sensor_msgs::LaserScan::ConstPtr& scan_in, const ros::Publisher& pub)
{
		static int count = 0;

}

int main(int argc, char** argv)
{

	std::cout << "hello world !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
	ros::init(argc, argv, "dddddd");
	ros::NodeHandle nh;
	ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("assembled_cloud_sub_topic", 500);
	ros::Publisher pub2 = nh.advertise<sensor_msgs::PointCloud2>("assembled_cloud_sub_topic_2", 500);
	ros::Publisher pub3 = nh.advertise<sensor_msgs::LaserScan>("assembled_laser_scan", 500);
	ros::Subscriber sub = nh.subscribe<sensor_msgs::LaserScan>("laser/scan/1", 100, boost::bind(scanCallback, _1, pub));
	ros::Subscriber sub2 = nh.subscribe<sensor_msgs::LaserScan>("laser/scan/1", 100, boost::bind(scanCallback2, _1, pub2));
	ros::Subscriber sub3 = nh.subscribe<sensor_msgs::LaserScan>("laser/scan/1", 100, boost::bind(scanCallback3, _1, pub3));
	ros::spin();
	return 0;
}


