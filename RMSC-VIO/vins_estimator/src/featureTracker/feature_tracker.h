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

#include <cstdio>
#include <iostream>
#include <queue>
#include <execinfo.h>
#include <csignal>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>

#include <opencv2/highgui.hpp>
#include <opencv2/cvconfig.h>
#include <opencv2/imgproc/types_c.h>

#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"
#include "../estimator/parameters.h"
//#include "../estimator/estimator.h"
#include "../utility/tic_toc.h"

using namespace std;
using namespace camodocal;
using namespace Eigen;

bool inBorder(const cv::Point2f &pt);
void reduceVector(vector<cv::Point2f> &v, vector<uchar> status);
void reduceVector(vector<int> &v, vector<uchar> status);

//std::vector<camodocal::CameraPtr> m_camera;

class FeatureTracker
{
public:
    FeatureTracker();
    map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> trackImage(int c, double _cur_time, const cv::Mat &_img, const cv::Mat &_img1 = cv::Mat());
    void setMask(int c);
    void readIntrinsicParameter(const vector<string> &calib_file);
    void showUndistortion(const string &name, int c);
    void rejectWithF(int c);
    void undistortedPoints();
    vector<cv::Point2f> undistortedPts(vector<cv::Point2f> &pts, camodocal::CameraPtr cam);
    vector<cv::Point2f> ptsVelocity(vector<int> &ids, vector<cv::Point2f> &pts, 
                                    map<int, cv::Point2f> &cur_id_pts, map<int, cv::Point2f> &prev_id_pts, int c);
    void showTwoImage(const cv::Mat &img1, const cv::Mat &img2, 
                      vector<cv::Point2f> pts1, vector<cv::Point2f> pts2);
    void drawTrack(const cv::Mat &imLeft, const cv::Mat &imRight, 
                                   vector<int> &curLeftIds,
                                   vector<cv::Point2f> &curLeftPts, 
                                   vector<cv::Point2f> &curRightPts,
                                   map<int, cv::Point2f> &prevLeftPtsMap,
                                   int c);
    void setPrediction(map<int, Eigen::Vector3d> &predictPts, int c);
    double distance(cv::Point2f &pt1, cv::Point2f &pt2);
    void removeOutliers(set<int> &removePtsIds);
    cv::Mat getTrackImage(int c);
    bool inBorder(const cv::Point2f &pt);

    int row, col;
    cv::Mat imTrack[2];
    cv::Mat mask; //图像掩码
    cv::Mat fisheye_mask; //鱼盐相机mask，用来去除边缘噪声
    cv::Mat prev_img[2], cur_img[2];//先前的图像和现在的图像
    vector<cv::Point2f> n_pts[2];//每一帧中提取的特征点数
    vector<cv::Point2f> predict_pts[2];//预测的特征坐标
    vector<cv::Point2f> predict_pts_debug[2];
    vector<cv::Point2f> prev_pts[2], cur_pts[2], cur_right_pts[2];//当前帧图像特征点像素坐标
    vector<cv::Point2f> prev_un_pts[2], cur_un_pts[2], cur_un_right_pts[2];//归一化坐标
    vector<cv::Point2f> pts_velocity[2], right_pts_velocity[2];//当前帧相对前一帧像素移动速度
    vector<int> ids[2], ids_right[2];//能够被跟踪到的特征点的id
    vector<int> track_cnt[2];//当前帧每个特征的被追踪的次数
    map<int, cv::Point2f> cur_un_pts_map[2], prev_un_pts_map[2];//构建一个id与归一化坐标的id
    map<int, cv::Point2f> cur_un_right_pts_map[2], prev_un_right_pts_map[2];
    map<int, cv::Point2f> prevLeftPtsMap[2];//上一帧左目中的点
    vector<camodocal::CameraPtr> m_camera;//相机类，双目有两个
    double cur_time[2];
    double prev_time[2];
    bool stereo_cam;
    int n_id;//用来作为特征点id
    bool hasPrediction;

    vector<pair<int, pair<cv::Point2f, int>>> cnt_pts_id[2];
};
