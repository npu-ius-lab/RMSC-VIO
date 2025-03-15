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

#include "Aligner.h"
// #include "Factors.h"
#include <Eigen/Dense>
// #include"globalOpt.h"
#include<fstream>



using std::cout;
using std::endl;
using namespace Eigen;
using namespace std;


Aligner::Aligner(ros::NodeHandle& n)
{
    nh_ = n;

    local_odom_pub_ = nh_.advertise<nav_msgs::Odometry>("local_odom", 100);
    enu_odom_pub_ = nh_.advertise<nav_msgs::Odometry>("enu_odom", 100);
    estimated_odom_pub_ = nh_.advertise<nav_msgs::Odometry>("estimated_odom", 100);
    estimated_enupath_pub_=nh_.advertise<nav_msgs::Path>("enu_path", 100);
    rtk_viopath_pub_=nh_.advertise<nav_msgs::Path>("rtk_vio_path", 100);
    rtk_enupath_pub_=nh_.advertise<nav_msgs::Path>("rtk_enu_path", 100);
}

//地理坐标系2地球系
void Aligner::LLA2ECEF(double longitude, double latitude, double altitude, double* xyz)
{
    const double d2r = M_PI / 180;
    const double a = 6378137;//地球长半径
    // const double b = 6356752.3142;//地球短半径
    const double f = 1/298.257223565;//基准椭球体的极扁率
    const double f_inverse = 298.257223565;
    const double b = a - a / f_inverse; //f=a-ab
    const double e = sqrt(a * a - b * b) / a;//e^2 = (a^2-b^2)/a^2
    const double e_2 = e * e;
    double L = longitude * d2r;
    double B = latitude * d2r;
    double H = altitude;

    const double N = a/sqrt(1-e_2*sin(B)*sin(B)); //基准椭球体的曲率半径
    //LLA坐标转换为ECEF坐标
    xyz[0] = (N+H)*cos(B)*cos(L);
    xyz[1] = (N+H)*cos(B)*sin(L);
    xyz[2] = (N*(1-e_2)+H)*sin(B);
}

 
void Aligner::gps_odom_callback(const sensor_msgs::NavSatFixConstPtr &GPS_msg, const nav_msgs::Odometry::ConstPtr &pose_msg)
{

    cout << "gps_odom_callback: " << sin(M_PI/2) << endl;
    //角度转弧度
    const double d2r = M_PI / 180;
    //如果为第一帧gps信息，计算出ecef2enu的坐标转换矩阵
    if(!get_first_gps_)
    {
        first_gps_ = *GPS_msg;

        const double d2r = M_PI / 180;

        //求取初始帧gps信息的slat、clat、slon、clon
        double slat = sin(GPS_msg->latitude*d2r);
        double clat = cos(GPS_msg->latitude*d2r);
        double slon = sin(GPS_msg->longitude*d2r);
        double clon = cos(GPS_msg->longitude*d2r);
        //ecef坐标系到neu坐标系的坐标转换矩阵
       R_ecef_enu_ << -slon, clon, 0,
            -clon*slat , -slon*slat , clat,
            clon*clat , slon*clat , slat;

        
        double ecef_origin[3];
        //将第一帧的LLA坐标转化为ECEF坐标
        LLA2ECEF(GPS_msg->longitude,GPS_msg->latitude,GPS_msg->altitude,ecef_origin);
        ecef_first_ << ecef_origin[0], ecef_origin[1], ecef_origin[2];

        get_first_gps_ = true;
    }

    Eigen::Vector3d ecef_vec, enu_point, vio_point;

    double ecef[3];//三维  XYZ
    //LLA坐标转化到ECEF坐标系下
    if(GPS_msg->position_covariance[0] < 2.0)
    {
        LLA2ECEF(GPS_msg->longitude,GPS_msg->latitude,GPS_msg->altitude,ecef);
        ecef_vec << ecef[0], ecef[1], ecef[2];
        //第一帧gps信息作为原点，求取相对于原点的enu坐标系    enu_point = R_ecef_enu_ *(ecef_vec - ecef_first)
        enu_point = R_ecef_enu_ * (ecef_vec - ecef_first_);//相对于第一帧gps信息为enu坐标系原点
        //拿到VIO的坐标
        vio_point << pose_msg->pose.pose.position.x, pose_msg->pose.pose.position.y, pose_msg->pose.pose.position.z;


        // cout << "R_ecef_enu_:   " << R_ecef_enu_ << endl;
        // cout << "Identity:   " << R_ecef_enu_.transpose() * R_ecef_enu_ << endl;
        // cout << fixed;
        // cout.precision(10); 
        // cout << "LLH: " << GPS_msg->longitude << "|" << GPS_msg->latitude << "|" << GPS_msg->altitude << endl;
        // cout << "ecef_vec:   " << ecef_vec.transpose() << endl;
        // cout << "ecef_first_:   " << ecef_first_.transpose() << endl;
        // cout << "enu_point:   " << enu_point.transpose() << endl;
        // cout << "vio_point:   " << vio_point.transpose() << endl;

        // gps_queue_.push(*GPS_msg);
        //将enu_point 与vio_point 组成pair  push_back 到point_vector_
        point_vector_.push_back(std::make_pair(enu_point, vio_point));
        // if(!aligned_)
        // {
        //如果配对到的点数>50,可以进行对齐优化
            if(point_vector_.size() > 50)
                can_optimize = true;
            //开始优化
            optimize();
    // }
    // else{
    // nav_msgs::Odometry odom_msg;
    // odom_msg.header.frame_id = "world";
    // odom_msg.header.stamp = ros::Time::now();
    // odom_msg.pose.pose.position.x = enu_point.x();
    // odom_msg.pose.pose.position.y = enu_point.y();
    // odom_msg.pose.pose.position.z = enu_point.z();
    // odom_msg.pose.pose.orientation.x = 0;
    // odom_msg.pose.pose.orientation.y = 0;
    // odom_msg.pose.pose.orientation.z = 0;
    // odom_msg.pose.pose.orientation.w = 1;
    // enu_odom_pub_.publish(odom_msg);

    // odom_msg.pose.pose.position.x = vio_point.x();
    // odom_msg.pose.pose.position.y = vio_point.y();
    // odom_msg.pose.pose.position.z = vio_point.z();
    // odom_msg.pose.pose.orientation.x = pose_msg->pose.pose.orientation.x;
    // odom_msg.pose.pose.orientation.y = pose_msg->pose.pose.orientation.y;
    // odom_msg.pose.pose.orientation.z = pose_msg->pose.pose.orientation.z;
    // odom_msg.pose.pose.orientation.w = pose_msg->pose.pose.orientation.w;
    // local_odom_pub_.publish(odom_msg);

    if(aligned_)
    {
        // Eigen::Vector3d tt = estimated_Ri * vio_point + estimated_ti;
        // Eigen::Quaterniond RR(estimated_Ri * Quaterniond(pose_msg->pose.pose.orientation.w,
        //     pose_msg->pose.pose.orientation.x, pose_msg->pose.pose.orientation.y,
        //     pose_msg->pose.pose.orientation.z).toRotationMatrix());

        // odom_msg.pose.pose.position.x = tt.x();
        // odom_msg.pose.pose.position.y = tt.y();
        // odom_msg.pose.pose.position.z = tt.z();
        // odom_msg.pose.pose.orientation.x = RR.x();
        // odom_msg.pose.pose.orientation.y = RR.y();
        // odom_msg.pose.pose.orientation.z = RR.z();
        // odom_msg.pose.pose.orientation.w = RR.w();
        // estimated_odom_pub_.publish(odom_msg);

        // //发布enu_path
        // geometry_msgs::PoseStamped pose_stamped_enu;
        // estimated_enupath.header.stamp = ros::Time::now();
        // estimated_enupath.header.frame_id = "world";
        // pose_stamped_enu.pose.position.x = tt.x();
        // pose_stamped_enu.pose.position.y = tt.y();
        // pose_stamped_enu.pose.position.z = tt.z();
        // pose_stamped_enu.pose.orientation.w = RR.x();
        // pose_stamped_enu.pose.orientation.x = RR.y();
        // pose_stamped_enu.pose.orientation.y = RR.z();
        // pose_stamped_enu.pose.orientation.z = RR.w();
        // estimated_enupath.poses.push_back(pose_stamped_enu);
        // estimated_enupath_pub_.publish(estimated_enupath);

        //将RTK转化到相对坐标系
    Eigen::Vector3d t = estimated_Ri.transpose() * enu_point - estimated_Ri.transpose()*estimated_ti;
    // Eigen::Quaterniond R(Quaterniond(pose_msg->pose.pose.orientation.w,
    //     pose_msg->pose.pose.orientation.x, pose_msg->pose.pose.orientation.y,
    //     pose_msg->pose.pose.orientation.z).toRotationMatrix());
    //vio坐标系下的rtk轨迹
    geometry_msgs::PoseStamped pose_stamped_rtk;
    rtk_viopath.header.stamp = GPS_msg->header.stamp;
    rtk_viopath.header.frame_id = "world";
    pose_stamped_rtk.pose.position.x = t.x();
    pose_stamped_rtk.pose.position.y = t.y();
    pose_stamped_rtk.pose.position.z = t.z();
    pose_stamped_rtk.pose.orientation.w = 1;
    pose_stamped_rtk.pose.orientation.x = 0;
    pose_stamped_rtk.pose.orientation.y = 0;
    pose_stamped_rtk.pose.orientation.z = 0;
    rtk_viopath.poses.push_back(pose_stamped_rtk);
    rtk_viopath_pub_.publish(rtk_viopath);
    //enu坐标系下的rtk轨迹
    geometry_msgs::PoseStamped pose_stamped_rtk_enu;
    rtk_enupath.header.stamp = GPS_msg->header.stamp;
    rtk_enupath.header.frame_id = "world";
    pose_stamped_rtk_enu.pose.position.x = enu_point.x();
    pose_stamped_rtk_enu.pose.position.y = enu_point.y();
    pose_stamped_rtk_enu.pose.position.z = enu_point.z();
    pose_stamped_rtk_enu.pose.orientation.w = 1;
    pose_stamped_rtk_enu.pose.orientation.x = 0;
    pose_stamped_rtk_enu.pose.orientation.y = 0;
    pose_stamped_rtk_enu.pose.orientation.z = 0;
    rtk_enupath.poses.push_back(pose_stamped_rtk_enu);
    rtk_enupath_pub_.publish(rtk_enupath);

    //Vio坐标系下的rtk
    ofstream foutC("/home/xu/RMSC-VIO/RTK_vio.csv",ios::app);
    foutC.setf(ios::fixed, ios::floatfield);
    foutC.precision(9);
    foutC << GPS_msg->header.stamp.toSec()<< " ";
    foutC.precision(5);
    foutC <<t.x() << " "
            <<t.y()<< " "
            <<t.z()<< " "
            <<0<< " "
            <<0<< " "
            <<0<< " "
            <<1<< endl;
    foutC.close();
    //ENU坐标系下的rtk
    ofstream foutC1("/home/xu/RMSC-VIO/RTK_enu.csv",ios::app);
    foutC1.setf(ios::fixed, ios::floatfield);
    foutC1.precision(9);
    foutC1 << GPS_msg->header.stamp.toSec()<< " ";
    foutC1.precision(5);
    foutC1 <<enu_point.x() << " "
            <<enu_point.y()<< " "
            <<enu_point.z()<< " "
            <<0<< " "
            <<0<< " "
            <<0<< " "
            <<1<< endl;
    foutC1.close();
    }
    }
    
}

//估计enu坐标系和vio坐标系的旋转矩阵
bool Aligner::optimize()
{
    if(!can_optimize)
    {
            // estimated_Ri=estimated_Ri;
            // estimated_ti=estimated_ti;
                // cout<<"11111111111111111111111111"<<endl;
        return false;
    }
    // else{    
    Eigen::Vector3d p_hat, q_hat;
    p_hat << 0,0,0;
    q_hat << 0,0,0;
    // SUM( R * VIO + t - ENU)
    for(auto & pair: point_vector_)
    {
        p_hat += pair.second;
        q_hat += pair.first;
    }
    //vio坐标和enu坐标的平均值
    p_hat /= point_vector_.size();
    q_hat /= point_vector_.size();
    //Eigen::MatrixXd   动态大小的矩阵
    Eigen::MatrixXd X(3, point_vector_.size());
    Eigen::MatrixXd Y(3, point_vector_.size());

    int i = 0;
    for(auto & pair: point_vector_)
    {
        X.col(i) = pair.second - p_hat;
        Y.col(i) = pair.first - q_hat;
        i++;
    }
    Eigen::MatrixXd S = X*Y.transpose();
    JacobiSVD<MatrixXd> svd(S, ComputeFullU | ComputeFullV);
    Eigen::MatrixXd U = svd.matrixU();
    Eigen::MatrixXd V = svd.matrixV();
    double det = (V*U.transpose()).determinant();
    int rows = V.cols(); 
    Eigen::MatrixXd Lambda = Eigen::MatrixXd::Identity(rows,rows);
    Lambda(rows-1, rows-1) = det;

    estimated_Ri = V*Lambda*U.transpose();
    estimated_ti = q_hat - estimated_Ri * p_hat;
    //求解得到vio坐标系到enu坐标系的坐标转换矩阵
    // estimated_R=estimated_Ri;
    // estimated_t=estimated_ti;

    cout << "estimated_R" << endl;
    cout << estimated_Ri << endl;
    cout << "=====" << estimated_Ri.transpose() * estimated_Ri << endl;
    cout << "estimated_t" << estimated_ti.transpose() << endl;

    aligned_ = true;
    // }
    return true;

}


// bool Aligner::get_enu_local(Eigen::Matrix4d& matrix)
// {
//     if(aligned_)
//     {

//     }


//     return aligned_;
// }

// bool Aligner::get_xyz_enu(Eigen::Matrix4d& matrix)
// {
//     if(get_first_gps_)
//     {

//     }

//     return aligned_;
// }