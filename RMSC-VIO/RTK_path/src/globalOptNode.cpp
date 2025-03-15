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

#include "ros/ros.h"
// #include "globalOpt.h"

#include "Aligner.h"

#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <cmath>
#include <stdio.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <fstream>
#include <queue>
#include <mutex>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>



// typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::NavSatFix, nav_msgs::Odometry,nav_msgs::Odometry> SyncPolicyGNSSOdom;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::NavSatFix, nav_msgs::Odometry> SyncPolicyGNSSOdom;
typedef shared_ptr<message_filters::Synchronizer<SyncPolicyGNSSOdom>> SynchronizerGNSSOdom;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "globalEstimator");
    ros::NodeHandle n("~");
    Aligner  aligner(n);
    shared_ptr<message_filters::Subscriber<sensor_msgs::NavSatFix>> gps_sub_;
    shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> odom_sub_;
    SynchronizerGNSSOdom sync_gps_pose_;        //时间同步

    gps_sub_.reset(new message_filters::Subscriber<sensor_msgs::NavSatFix>(n, "/mavros/global_position/raw/fix", 100));
    // odom_sub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(n, "/loop_fusion/odometry_rect", 100));
    odom_sub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(n, "/vins_estimator/odometry", 100));

    sync_gps_pose_.reset(new message_filters::Synchronizer<SyncPolicyGNSSOdom>(SyncPolicyGNSSOdom(100), *gps_sub_, *odom_sub_));
    sync_gps_pose_->registerCallback(boost::bind(&Aligner::gps_odom_callback, &aligner, _1, _2));
    
    ros::spin();
    return 0;
}
