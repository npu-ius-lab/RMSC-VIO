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

#include "feature_tracker.h"
#include "../estimator/estimator.h"

bool FeatureTracker::inBorder(const cv::Point2f &pt)
{
    const int BORDER_SIZE = 1;
    int img_x = cvRound(pt.x);
    int img_y = cvRound(pt.y);
    return BORDER_SIZE <= img_x && img_x < col - BORDER_SIZE && BORDER_SIZE <= img_y && img_y < row - BORDER_SIZE;
}

double distance(cv::Point2f pt1, cv::Point2f pt2)
{
    //printf("pt1: %f %f pt2: %f %f\n", pt1.x, pt1.y, pt2.x, pt2.y);
    double dx = pt1.x - pt2.x;
    double dy = pt1.y - pt2.y;
    return sqrt(dx * dx + dy * dy);
}
//根据光流跟踪状态status，对未跟踪上的点进行剔除
void reduceVector(vector<cv::Point2f> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

void reduceVector(vector<int> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

FeatureTracker::FeatureTracker()
{
    stereo_cam = 0;
    n_id = 0;
    hasPrediction = false;
}

void FeatureTracker::setMask(int c)
{
    //std::cout<<"*****************setMask*******************************"<<std::endl;
    mask = cv::Mat(row, col, CV_8UC1, cv::Scalar(255));

    // prefer to keep features that are tracked for long time
    //vector<pair<int, pair<cv::Point2f, int>>> cnt_pts_id[c];

    for (unsigned int i = 0; i < cur_pts[c].size(); i++)
        cnt_pts_id[c].push_back(make_pair(track_cnt[c][i], make_pair(cur_pts[c][i], ids[c][i])));

    //std::cout<<"*****************setMask11111111111111111111111*******************************"<<std::endl;
    sort(cnt_pts_id[c].begin(), cnt_pts_id[c].end(), [](const pair<int, pair<cv::Point2f, int>> &a, const pair<int, pair<cv::Point2f, int>> &b)
         {
            return a.first > b.first;
         });

    //std::cout<<"*****************setMask2222222222222222222222222222222222*******************************"<<std::endl;
    cur_pts[c].clear();
    ids[c].clear();
    track_cnt[c].clear();

    for (auto &it : cnt_pts_id[c])
    {
    //std::cout<<"*****************setMask33333333333333333333333333333*******************************"<<std::endl;
        if (mask.at<uchar>(it.second.first) == 255)
        {
            cur_pts[c].push_back(it.second.first);
            ids[c].push_back(it.second.second);
            track_cnt[c].push_back(it.first);
            cv::circle(mask, it.second.first, MIN_DIST, 0, -1);
        }
    }
    cnt_pts_id[c].clear();
}

double FeatureTracker::distance(cv::Point2f &pt1, cv::Point2f &pt2)
{
    //printf("pt1: %f %f pt2: %f %f\n", pt1.x, pt1.y, pt2.x, pt2.y);
    double dx = pt1.x - pt2.x;
    double dy = pt1.y - pt2.y;
    return sqrt(dx * dx + dy * dy);
}
//对图像进行一系列操作，返回特征点featureFrame
map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> FeatureTracker::trackImage(int c, double _cur_time, const cv::Mat &_img, const cv::Mat &_img1)
{
    // std::cout<<"*****************trackImage*******************************"<<std::endl;
/*跟踪一帧图像，提取当前帧特征点
1、用前一帧运动估计特征点在当前帧的位置，如果特征点没有速度，就直接用前一帧该点位置
2、LK光流跟踪前一帧的特征点，正反向，删除跟丢的点；如果是双目，进行左右匹配，只删除右目跟丢的特征点
3、对于前后帧用LK光流跟踪到的匹配特征点，计算基础矩阵，用极限约束进一步剔除outlier点
4、如果特征点不够，剩余的用角点来凑；更新特征点跟踪次数
5、计算特征点归一化相机平面坐标，并计算相对于前一帧移动速度。
6、保存当前帧特征点数据（归一化相机平面坐标，像素坐标，归一化相机平面移动速度）
7、展示，左图特征点用颜色区分跟踪次数（红色少、蓝色多），画个箭头指向前一帧特征点位置，如果是双目，右图花个绿色点。
*/

    TicToc t_r;
    cur_time[c] = _cur_time;
    cur_img[c] = _img;
    row = cur_img[c].rows;
    col = cur_img[c].cols;
    cv::Mat rightImg = _img1;
    if(c == 2)
    {
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(2.0, cv::Size(10, 10));
        clahe->apply(cur_img[c], cur_img[c]);
        if(!rightImg.empty())
            clahe->apply(rightImg, rightImg);
    }
    
    cur_pts[c].clear();

    if (prev_pts[c].size() > 0)//如果上一帧有特征点，直接进行LK跟踪
    {
        TicToc t_o;
        vector<uchar> status;
        vector<float> err;
        if(hasPrediction)
        {   //如果有预测，当前特征等于预测特征坐标
            cur_pts[c] = predict_pts[c];
            //进行光流跟踪
            cv::calcOpticalFlowPyrLK(prev_img[c], cur_img[c], prev_pts[c], cur_pts[c], status, err, cv::Size(21, 21), 1, 
            cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01), cv::OPTFLOW_USE_INITIAL_FLOW);
            
            int succ_num = 0;
            for (size_t i = 0; i < status.size(); i++)
            {
                if (status[i])
                    succ_num++;//成功和上一帧匹配的特征点数
            }
            if (succ_num < 10)//成功匹配特征点数小于10，扩大搜索
               cv::calcOpticalFlowPyrLK(prev_img[c], cur_img[c], prev_pts[c], cur_pts[c], status, err, cv::Size(21, 21), 3);
        }
        else//如果没有预测，直接进行3层金字塔光流跟踪
            cv::calcOpticalFlowPyrLK(prev_img[c], cur_img[c], prev_pts[c], cur_pts[c], status, err, cv::Size(21, 21), 3);
        // reverse check 反向光流跟踪
        if(FLOW_BACK)
        {
            vector<uchar> reverse_status;
            vector<cv::Point2f> reverse_pts = prev_pts[c];
            cv::calcOpticalFlowPyrLK(cur_img[c], prev_img[c], cur_pts[c], reverse_pts, reverse_status, err, cv::Size(21, 21), 1, 
            cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01), cv::OPTFLOW_USE_INITIAL_FLOW);
            //cv::calcOpticalFlowPyrLK(cur_img, prev_img, cur_pts, reverse_pts, reverse_status, err, cv::Size(21, 21), 3); 
            for(size_t i = 0; i < status.size(); i++)
            {
                //如果前后都可以找到，且找到的距离小于0.5，则算匹配成功
                if(status[i] && reverse_status[i] && distance(prev_pts[c][i], reverse_pts[i]) <= 0.5)
                {
                    status[i] = 1;
                }
                else
                    status[i] = 0;
            }
        }
        
        for (int i = 0; i < int(cur_pts[c].size()); i++)
            if (status[i] && !inBorder(cur_pts[c][i]))//如果这个点不在图像内，则剔除
                status[i] = 0;
        reduceVector(prev_pts[c], status);//筛选后跟踪到的左目特征点
        reduceVector(cur_pts[c], status);//筛选后跟踪到的右目特征点
        reduceVector(ids[c], status);//筛选后跟踪到的特征点个数
        reduceVector(track_cnt[c], status);//
        ROS_DEBUG("temporal optical flow costs: %fms", t_o.toc());
        //printf("track cnt %d\n", (int)ids.size());
    }
    // std::cout<<"*****************11111111111111111111*******************************"<<std::endl;
    for (auto &n : track_cnt[c])
        n++;

    if (1)
    {
        //rejectWithF();
        ROS_DEBUG("set mask begins");
        TicToc t_m;
        setMask(c);
        ROS_DEBUG("set mask costs %fms", t_m.toc());

        ROS_DEBUG("detect feature begins");
        TicToc t_t;
        int n_max_cnt = MAX_CNT - static_cast<int>(cur_pts[c].size());
        if (n_max_cnt > 0)
        {
            if(mask.empty())
                cout << "mask is empty " << endl;
            if (mask.type() != CV_8UC1)
                cout << "mask type wrong " << endl;
            cv::goodFeaturesToTrack(cur_img[c], n_pts[c], MAX_CNT - cur_pts[c].size(), 0.01, MIN_DIST, mask);
        }
        else
            n_pts[c].clear();
        ROS_DEBUG("detect feature costs: %f ms", t_t.toc());

        for (auto &p : n_pts[c])
        {
            cur_pts[c].push_back(p);
            ids[c].push_back(n_id++);
            track_cnt[c].push_back(1);
        }
        //printf("feature cnt after add %d\n", (int)ids.size());
    }
    //像素坐标求归一化坐标
    cur_un_pts[c] = undistortedPts(cur_pts[c], m_camera[0]);
    //当前帧相对于前一帧，特征点沿x  y方向的像素移动速度
    pts_velocity[c] = ptsVelocity(ids[c], cur_un_pts[c], cur_un_pts_map[c], prev_un_pts_map[c], c);

    if(!_img1.empty() && stereo_cam)//如果右目不空，且为双相机
    {
        ids_right[c].clear();
        cur_right_pts[c].clear();
        cur_un_right_pts[c].clear();
        right_pts_velocity[c].clear();
        cur_un_right_pts_map[c].clear();
        if(!cur_pts[c].empty())
        {
            //printf("stereo image; track feature on right image\n");
            vector<cv::Point2f> reverseLeftPts;
            vector<uchar> status, statusRightLeft;
            vector<float> err;
            // cur left ---- cur right
            cv::calcOpticalFlowPyrLK(cur_img[c], rightImg, cur_pts[c], cur_right_pts[c], status, err, cv::Size(21, 21), 3);
            // reverse check cur right ---- cur left
            if(FLOW_BACK)
            {
                cv::calcOpticalFlowPyrLK(rightImg, cur_img[c], cur_right_pts[c], reverseLeftPts, statusRightLeft, err, cv::Size(21, 21), 3);
                for(size_t i = 0; i < status.size(); i++)
                {
                    if(status[i] && statusRightLeft[i] && inBorder(cur_right_pts[c][i]) && distance(cur_pts[c][i], reverseLeftPts[i]) <= 0.5)
                        status[i] = 1;
                    else
                        status[i] = 0;
                }
            }

            ids_right[c] = ids[c];
            reduceVector(cur_right_pts[c], status);
            reduceVector(ids_right[c], status);
            // only keep left-right pts
            /*
            reduceVector(cur_pts, status);
            reduceVector(ids, status);
            reduceVector(track_cnt, status);
            reduceVector(cur_un_pts, status);
            reduceVector(pts_velocity, status);
            */
            cur_un_right_pts[c] = undistortedPts(cur_right_pts[c], m_camera[1]);
            right_pts_velocity[c] = ptsVelocity(ids_right[c], cur_un_right_pts[c], cur_un_right_pts_map[c], prev_un_right_pts_map[c] , c);
        }
        prev_un_right_pts_map[c] = cur_un_right_pts_map[c];
    }
    if(SHOW_TRACK)
        drawTrack(cur_img[c], rightImg, ids[c], cur_pts[c], cur_right_pts[c], prevLeftPtsMap[c], c);

    prev_img[c] = cur_img[c];
    prev_pts[c] = cur_pts[c];
    prev_un_pts[c] = cur_un_pts[c];
    prev_un_pts_map[c] = cur_un_pts_map[c];
    prev_time[c] = cur_time[c];
    hasPrediction = false;

    prevLeftPtsMap[c].clear();
    for(size_t i = 0; i < cur_pts[c].size(); i++)
        prevLeftPtsMap[c][ids[c][i]] = cur_pts[c][i];
    map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> featureFrame;
    Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
    int feature_id, camera_id;
    double x, y, z, p_u, p_v, velocity_x, velocity_y;
    for (size_t i = 0; i < ids[c].size(); i++)
    {
        feature_id = ids[c][i];
        //double x, y ,z;
        x = cur_un_pts[c][i].x;
        y = cur_un_pts[c][i].y;
        z = 1;
        //double p_u, p_v;
        p_u = cur_pts[c][i].x;
        p_v = cur_pts[c][i].y;
        camera_id = 0;
        //double velocity_x, velocity_y;
        velocity_x = pts_velocity[c][i].x;
        velocity_y = pts_velocity[c][i].y;

        xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
        featureFrame[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
    }

    if (!_img1.empty() && stereo_cam)
    {
        for (size_t i = 0; i < ids_right[c].size(); i++)
        {
            feature_id = ids_right[c][i];
            //double x, y ,z;
            x = cur_un_right_pts[c][i].x;
            y = cur_un_right_pts[c][i].y;
            z = 1;
            //double p_u, p_v;
            p_u = cur_right_pts[c][i].x;
            p_v = cur_right_pts[c][i].y;
            camera_id = 1;
            //double velocity_x, velocity_y;
            velocity_x = right_pts_velocity[c][i].x;
            velocity_y = right_pts_velocity[c][i].y;

            //Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
            xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
            featureFrame[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
        }
    }
    //printf("feature track whole time %f\n", t_r.toc());
    return featureFrame;
}

void FeatureTracker::rejectWithF(int c)
{
    if (cur_pts[c].size() >= 8)
    {
        ROS_DEBUG("FM ransac begins");
        TicToc t_f;
        vector<cv::Point2f> un_cur_pts(cur_pts[c].size()), un_prev_pts(prev_pts[c].size());
        for (unsigned int i = 0; i < cur_pts[c].size(); i++)
        {
            Eigen::Vector3d tmp_p;
            m_camera[0]->liftProjective(Eigen::Vector2d(cur_pts[c][i].x, cur_pts[c][i].y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + col / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + row / 2.0;
            un_cur_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());

            m_camera[0]->liftProjective(Eigen::Vector2d(prev_pts[c][i].x, prev_pts[c][i].y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + col / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + row / 2.0;
            un_prev_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());
        }

        vector<uchar> status;
        cv::findFundamentalMat(un_cur_pts, un_prev_pts, cv::FM_RANSAC, F_THRESHOLD, 0.99, status);
        int size_a = cur_pts[c].size();
        reduceVector(prev_pts[c], status);
        reduceVector(cur_pts[c], status);
        reduceVector(cur_un_pts[c], status);
        reduceVector(ids[c], status);
        reduceVector(track_cnt[c], status);
        ROS_DEBUG("FM ransac: %d -> %lu: %f", size_a, cur_pts[c].size(), 1.0 * cur_pts[c].size() / size_a);
        ROS_DEBUG("FM ransac costs: %fms", t_f.toc());
    }
}

void FeatureTracker::readIntrinsicParameter(const vector<string> &calib_file)
{
    for (size_t i = 0; i < calib_file.size(); i++)
    {
        ROS_INFO("reading paramerter of camera %s", calib_file[i].c_str());
        camodocal::CameraPtr camera = CameraFactory::instance()->generateCameraFromYamlFile(calib_file[i]);
        m_camera.push_back(camera);
    }
    if (calib_file.size() == 2)
        stereo_cam = 1;
}

void FeatureTracker::showUndistortion(const string &name, int c)
{
    cv::Mat undistortedImg(row + 600, col + 600, CV_8UC1, cv::Scalar(0));
    vector<Eigen::Vector2d> distortedp, undistortedp;
    for (int i = 0; i < col; i++)
        for (int j = 0; j < row; j++)
        {
            Eigen::Vector2d a(i, j);
            Eigen::Vector3d b;
            m_camera[0]->liftProjective(a, b);
            distortedp.push_back(a);
            undistortedp.push_back(Eigen::Vector2d(b.x() / b.z(), b.y() / b.z()));
            //printf("%f,%f->%f,%f,%f\n)\n", a.x(), a.y(), b.x(), b.y(), b.z());
        }
    for (int i = 0; i < int(undistortedp.size()); i++)
    {
        cv::Mat pp(3, 1, CV_32FC1);
        pp.at<float>(0, 0) = undistortedp[i].x() * FOCAL_LENGTH + col / 2;
        pp.at<float>(1, 0) = undistortedp[i].y() * FOCAL_LENGTH + row / 2;
        pp.at<float>(2, 0) = 1.0;
        //cout << trackerData[0].K << endl;
        //printf("%lf %lf\n", p.at<float>(1, 0), p.at<float>(0, 0));
        //printf("%lf %lf\n", pp.at<float>(1, 0), pp.at<float>(0, 0));
        if (pp.at<float>(1, 0) + 300 >= 0 && pp.at<float>(1, 0) + 300 < row + 600 && pp.at<float>(0, 0) + 300 >= 0 && pp.at<float>(0, 0) + 300 < col + 600)
        {
            undistortedImg.at<uchar>(pp.at<float>(1, 0) + 300, pp.at<float>(0, 0) + 300) = cur_img[c].at<uchar>(distortedp[i].y(), distortedp[i].x());
        }
        else
        {
            //ROS_ERROR("(%f %f) -> (%f %f)", distortedp[i].y, distortedp[i].x, pp.at<float>(1, 0), pp.at<float>(0, 0));
        }
    }
    // turn the following code on if you need
    // cv::imshow(name, undistortedImg);
    // cv::waitKey(0);
}
//将像素坐标系下坐标转化为归一化相机坐标系下的坐标
vector<cv::Point2f> FeatureTracker::undistortedPts(vector<cv::Point2f> &pts, camodocal::CameraPtr cam)
{
    vector<cv::Point2f> un_pts;
    for (unsigned int i = 0; i < pts.size(); i++)
    {
        Eigen::Vector2d a(pts[i].x, pts[i].y);
        Eigen::Vector3d b;
        cam->liftProjective(a, b);
        un_pts.push_back(cv::Point2f(b.x() / b.z(), b.y() / b.z()));
    }
    return un_pts;
}

vector<cv::Point2f> FeatureTracker::ptsVelocity(vector<int> &ids, vector<cv::Point2f> &pts, 
                                            map<int, cv::Point2f> &cur_id_pts, map<int, cv::Point2f> &prev_id_pts, int c)
{
    vector<cv::Point2f> pts_velocity;
    cur_id_pts.clear();
    for (unsigned int i = 0; i < ids.size(); i++)
    {
        cur_id_pts.insert(make_pair(ids[i], pts[i]));
    }

    // caculate points velocity
    if (!prev_id_pts.empty())
    {
        double dt = cur_time[c] - prev_time[c];
        
        for (unsigned int i = 0; i < pts.size(); i++)
        {
            std::map<int, cv::Point2f>::iterator it;
            it = prev_id_pts.find(ids[i]);
            if (it != prev_id_pts.end())
            {
                double v_x = (pts[i].x - it->second.x) / dt;
                double v_y = (pts[i].y - it->second.y) / dt;
                pts_velocity.push_back(cv::Point2f(v_x, v_y));
            }
            else
                pts_velocity.push_back(cv::Point2f(0, 0));

        }
    }
    else
    {
        for (unsigned int i = 0; i < cur_pts[c].size(); i++)
        {
            pts_velocity.push_back(cv::Point2f(0, 0));
        }
    }
    return pts_velocity;
}

void FeatureTracker::drawTrack(const cv::Mat &imLeft, const cv::Mat &imRight, 
                               vector<int> &curLeftIds,
                               vector<cv::Point2f> &curLeftPts, 
                               vector<cv::Point2f> &curRightPts,
                               map<int, cv::Point2f> &prevLeftPtsMap,
                               int c)
{
    //int rows = imLeft.rows;
    int cols = imLeft.cols;
    if (!imRight.empty() && stereo_cam)
        cv::hconcat(imLeft, imRight, imTrack[c]);//将图片水平拼接
    else
        imTrack[c] = imLeft.clone();
    cv::cvtColor(imTrack[c], imTrack[c], CV_GRAY2RGB);
    //越红表示特征点看到的越久，如果都是蓝色，表示前端很差
    for (size_t j = 0; j < curLeftPts.size(); j++)
    {
        double len = std::min(1.0, 1.0 * track_cnt[c][j] / 20);
        cv::circle(imTrack[c], curLeftPts[j], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
    }
    if (!imRight.empty() && stereo_cam)
    {
        for (size_t i = 0; i < curRightPts.size(); i++)
        {
            cv::Point2f rightPt = curRightPts[i];
            rightPt.x += cols;
            cv::circle(imTrack[c], rightPt, 2, cv::Scalar(0, 255, 0), 2);
            //cv::Point2f leftPt = curLeftPtsTrackRight[i];
            //cv::line(imTrack, leftPt, rightPt, cv::Scalar(0, 255, 0), 1, 8, 0);
        }
    }
    
    map<int, cv::Point2f>::iterator mapIt;
    for (size_t i = 0; i < curLeftIds.size(); i++)
    {
        int id = curLeftIds[i];
        mapIt = prevLeftPtsMap.find(id);
        if(mapIt != prevLeftPtsMap.end())
        {
            cv::arrowedLine(imTrack[c], curLeftPts[i], mapIt->second, cv::Scalar(0, 255, 0), 1, 8, 0, 0.2);
        }
    }

    //draw prediction
    /*
    for(size_t i = 0; i < predict_pts_debug.size(); i++)
    {
        cv::circle(imTrack, predict_pts_debug[i], 2, cv::Scalar(0, 170, 255), 2);
    }
    */
    //printf("predict pts size %d \n", (int)predict_pts_debug.size());

    //cv::Mat imCur2Compress;
    //cv::resize(imCur2, imCur2Compress, cv::Size(cols, rows / 2));
}


void FeatureTracker::setPrediction(map<int, Eigen::Vector3d> &predictPts, int c)
{
    hasPrediction = true;
    predict_pts[c].clear();
    predict_pts_debug[c].clear();
    map<int, Eigen::Vector3d>::iterator itPredict;
    for (size_t i = 0; i < ids[c].size(); i++)
    {
        //printf("prevLeftId size %d prevLeftPts size %d\n",(int)prevLeftIds.size(), (int)prevLeftPts.size());
        int id = ids[c][i];
        itPredict = predictPts.find(id);
        if (itPredict != predictPts.end())
        {
            Eigen::Vector2d tmp_uv;
            m_camera[0]->spaceToPlane(itPredict->second, tmp_uv);
            predict_pts[c].push_back(cv::Point2f(tmp_uv.x(), tmp_uv.y()));
            predict_pts_debug[c].push_back(cv::Point2f(tmp_uv.x(), tmp_uv.y()));
        }
        else
        {
            predict_pts[c].push_back(prev_pts[0][i]);
            predict_pts[c].push_back(prev_pts[1][i]);
        }
    }
}


void FeatureTracker::removeOutliers(set<int> &removePtsIds)
{
    std::set<int>::iterator itSet;
    vector<uchar> status;
    for (size_t i = 0; i < ids[0].size(); i++)
    {
        itSet = removePtsIds.find(ids[0][i]);
        if(itSet != removePtsIds.end())
            status.push_back(0);
        else
            status.push_back(1);
    }

    reduceVector(prev_pts[0], status);
    reduceVector(ids[0], status);
    reduceVector(track_cnt[0], status);

    for (size_t i = 0; i < ids[1].size(); i++)
    {
        itSet = removePtsIds.find(ids[1][i]);
        if(itSet != removePtsIds.end())
            status.push_back(0);
        else
            status.push_back(1);
    }

    reduceVector(prev_pts[1], status);
    reduceVector(ids[1], status);
    reduceVector(track_cnt[1], status);
}


cv::Mat FeatureTracker::getTrackImage(int c)
{
    return imTrack[c];
}