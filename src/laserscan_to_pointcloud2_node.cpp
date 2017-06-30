/*
 * laserscan_to_pointcloud2_node.cpp
 *
 *  Created on: 2017年6月26日
 *      Author: tbm
 */


#include "../include/laserscan_to_pointcloud2/laserscan_to_pointcloud2_node.h"
#include <iostream>
#include <string>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <functional>

#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

using namespace boost;

std::string name_space;

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
	static tf::TransformListener listener_2_;
	static sensor_msgs::PointCloud2Ptr cloud_ptr_10  = boost::make_shared<sensor_msgs::PointCloud2>();
	static sensor_msgs::PointCloud2Ptr cloud_ptr_10_p = boost::make_shared<sensor_msgs::PointCloud2>();
	static laser_geometry::LaserProjection projector_;
	static bool is_dense = true;
	sensor_msgs::PointCloud2Ptr cloud_ptr = boost::make_shared<sensor_msgs::PointCloud2>();
	if(!listener_.waitForTransform(scan_in->header.frame_id, "/world", scan_in->header.stamp , ros::Duration(1.0)))
	{
		ROS_INFO("scan_in->header.frame_id: %s", (scan_in->header.frame_id).c_str());
		ROS_INFO("no transform to /world");
		return ;
	}
	count++;
	projector_.transformLaserScanToPointCloud("/world",*scan_in, *cloud_ptr,listener_);
	if(cloud_ptr->is_dense == false)
	{
		is_dense = false;
	}
	if(count % 200 == 1)
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
		//cloud_ptr_10->is_dense = false;
		cloud_ptr_10->data.clear();
		cloud_ptr_10->data.reserve(720000);
		cloud_ptr_10->header.frame_id = cloud_ptr->header.frame_id;
		cloud_ptr_10->header.stamp = cloud_ptr->header.stamp;
	}
	concatenateCloud(cloud_ptr, cloud_ptr_10);
	if(count % 200 == 0 )
	{
		cloud_ptr_10->is_dense = is_dense;
		//at line 130 in /tmp/binarydeb/ros-indigo-tf2-0.5.15/src/buffer_core.cpp
		//Warning: Invalid argument "/party_1/instance_1/erlecopter_2/base_link_inertia" passed to canTransform argument source_frame in tf2 frame_ids
		//cannot start with a '/' like:
		if(!listener_2_.waitForTransform("/world", name_space+"/base_link_inertia" , cloud_ptr_10->header.stamp, ros::Duration(1.0)))
		{
			ROS_INFO("cloud_ptr_10->header.frame_id: %s", (cloud_ptr_10->header.frame_id).c_str());
			ROS_INFO("no transform to base_link_inertia");
			return ;
		}
		pcl_ros::transformPointCloud(name_space+"/base_link_inertia", (*cloud_ptr_10), (*cloud_ptr_10_p), listener_2_);
		//pub.publish(cloud_ptr_10);
		pub.publish(cloud_ptr_10_p);
		//ROS_INFO("cloud_ptr_10 header.frame_id = %s", (cloud_ptr_10->header.frame_id).c_str());
		is_dense = true;
	}
}

void scanCallback3(const sensor_msgs::LaserScan::ConstPtr& scan_in, const ros::Publisher& pub)
{
		static int count = 0;

}

int main(int argc, char** argv)
{

	std::cout << "hello world !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
	ros::init(argc, argv, "laserscan_to_pointcloud2_node");
	//use the node in roslaunch file
	ros::NodeHandle nh;
	name_space = nh.getNamespace();
	name_space = name_space.erase(0,1);
	ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("assembled_cloud_sub_topic", 500);
	ros::Publisher pub2 = nh.advertise<sensor_msgs::PointCloud2>("assembled_cloud_sub_topic_2", 500);
	//ros::Publisher pub3 = nh.advertise<sensor_msgs::LaserScan>("assembled_laser_scan", 500);
	ros::Subscriber sub = nh.subscribe<sensor_msgs::LaserScan>("laser/scan/1", 100, boost::bind(scanCallback, _1, pub));
	ros::Subscriber sub2 = nh.subscribe<sensor_msgs::LaserScan>("laser/scan/1", 100, boost::bind(scanCallback2, _1, pub2));
	//ros::Subscriber sub3 = nh.subscribe<sensor_msgs::LaserScan>("laser/scan/1", 100, boost::bind(scanCallback3, _1, pub3));
	ros::spin();
	return 0;
}





