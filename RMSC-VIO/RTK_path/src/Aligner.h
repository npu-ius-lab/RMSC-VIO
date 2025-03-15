/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

#pragma once
#include <vector>
#include <map>
#include <ros/ros.h>
#include <iostream>
#include <mutex>
#include <thread>
#include <queue>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <ceres/ceres.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include "LocalCartesian.hpp"
#include <sensor_msgs/NavSatFix.h>
// #include "globalOpt.h"
#include "tic_toc.h"
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>

using namespace std;


// static bool aligned_ = false;
class Aligner
{
public:
	Aligner()
	{

	}
	Aligner(ros::NodeHandle& n);
	~Aligner()
	{

	}

	std::mutex m_buf;
	//初始帧对应的gps
	sensor_msgs::NavSatFix first_gps_;

	bool get_first_gps_ = false;
	bool can_optimize = false;
	bool aligned_ = false;
	int k = 0;

	//void gps_odom_callback(const sensor_msgs::NavSatFixConstPtr &GPS_msg, const nav_msgs::Odometry::ConstPtr &pose_msg,const nav_msgs::Odometry::ConstPtr &imu_msg);

	void gps_odom_callback(const sensor_msgs::NavSatFixConstPtr &GPS_msg, const nav_msgs::Odometry::ConstPtr &pose_msg);


	std::queue<sensor_msgs::NavSatFix> gps_queue_;
	std::queue<nav_msgs::Odometry> vio_queue_;


	std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> point_vector_;

	bool optimize();
	

	Eigen::Matrix3d R_ecef_enu_;
	Eigen::Vector3d ecef_first_;
	bool get_enu_local(Eigen::Matrix4d& matrix);
	bool get_xyz_enu(Eigen::Matrix4d& matrix);

	// GlobalOptimization globalEstimator;

	Eigen::Matrix3d estimated_Ri;
    Eigen::Vector3d estimated_ti;

	// friend void imupro_callback(const nav_msgs::Odometry:: ConstPtr &imupro_msg );
    //  Eigen::Matrix3d estimated_R;
    //  Eigen::Vector3d estimated_t;



	nav_msgs::Path estimated_enupath;

	nav_msgs::Path rtk_viopath, rtk_enupath;

private:
	ros::NodeHandle nh_;
	void LLA2ECEF(double longitude, double latitude, double altitude, double* xyz);
	ros::Publisher local_odom_pub_, enu_odom_pub_, estimated_odom_pub_,estimated_high_odom_pub_,estimated_enupath_pub_,rtk_enupath_pub_,rtk_viopath_pub_;

};