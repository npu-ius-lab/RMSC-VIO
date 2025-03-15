/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "feature_manager.h"

int FeaturePerId::endFrame()
{
    return start_frame + feature_per_frame.size() - 1;
}

FeatureManager::FeatureManager(Matrix3d _Rs[])
    : Rs(_Rs)
{
    for (int i = 0; i < NUM_OF_CAM; i++)
        ric[i].setIdentity();
    for (int i = 0; i < NUM_OF_CAM; i++)
        ric1[i].setIdentity();
}

void FeatureManager::setRic(Matrix3d _ric[])
{
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        ric[i] = _ric[i];
    }
}

void FeatureManager::setRic1(Matrix3d _ric[])
{
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        ric1[i] = _ric[i];
    }
}

void FeatureManager::clearState()
{
    for(int c=0; c<NUM_OF_CAM; c++)
    {
    feature[c].clear();
    }
    // f_feature.clear();
}

int FeatureManager::getFeatureCount(int Cam_num)
{
    int cnt = 0;
    // for(int c=0; c<NUM_OF_CAM; c++)
    // {
        for (auto &it : feature[Cam_num])
        {
            it.used_num = it.feature_per_frame.size();
            if (it.used_num >= 4)
            {
                cnt++;
            }
        }
    // }
    return cnt;
}


//将当前帧的特征点信息加入到滑窗中，并且判断当前帧是否为关键帧
bool FeatureManager::addFeatureCheckParallax(int frame_count, const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> image[2], double td)
{
    //ROS_DEBUG("input feature: %d", (int)image.size());
    //ROS_DEBUG("num of feature: %d", getFeatureCount());
    // double parallax_sum[NUM_OF_CAM];//总平行度数组

    // int parallax_num[NUM_OF_CAM];//平行特征点数
    double parallax_sum=0;
    int parallax_num=0;

    // int old_feature[NUM_OF_CAM];
    // int new_feature[NUM_OF_CAM];

    last_track_num = 0;
    last_average_parallax= 0;
    new_feature_num = 0;
    long_track_num[0]= 0;
    long_track_num[1]= 0;

    bool flag = false;
    for(int c=0; c<NUM_OF_CAM; c++)
    {
        // last_track_num[c] = 0;
        // last_average_parallax[c] = 0;
        // new_feature_num[c] = 0;
        // long_track_num[c] = 0;
        //遍历当前帧的所有特征点
        for (auto &id_pts : image[c])
        {
            //把当前帧的特征点封装成一个FeaturePerFrame对象
            FeaturePerFrame f_per_fra(id_pts.second[0].second, td);//f_per_fra 中存放每个路标点在每一幅图像中的信息
            assert(id_pts.second[0].first == 0);//判断是否为左目
            if(id_pts.second.size() == 2)
            {
                f_per_fra.rightObservation(id_pts.second[1].second);
                assert(id_pts.second[1].first == 1);//判断是否为右目
            }

            int feature_id = id_pts.first;
            //it是一个类对象FeaturePerId   里面存放特征点id  feature_id、 开始观测到的帧start_frame、和特征点在每一幅图像中的信息f_per_fra
            //遍历滑窗内所有特征点，看能不能找到这个特征点
            auto it = find_if(feature[c].begin(), feature[c].end(), [feature_id](const FeaturePerId &it)
                            {
                return it.feature_id == feature_id;
                            });

            if (it == feature[c].end())//如果没找到该特征点，该特征点作为新的特征点，加入到滑窗特征点库中
            {
                feature[c].push_back(FeaturePerId(feature_id, frame_count));
                feature[c].back().feature_per_frame.push_back(f_per_fra);
                new_feature_num++;
            }
            else if (it->feature_id == feature_id)//如果该特征点已经在滑窗内被观察到，则补充上该特征点在当前帧中的数据，并且共试点统计数+1
            {
                it->feature_per_frame.push_back(f_per_fra);
                last_track_num++;
                if( it-> feature_per_frame.size() >= 4) //长时间追踪点
                    long_track_num[c]++;
            }
        }
    }

    if (frame_count < 2 || last_track_num < 20 || (long_track_num[0]+long_track_num[1]) < 40 || new_feature_num > 0.5 * last_track_num)
            return true;

    for(int c =0; c<NUM_OF_CAM;c++)
    {
        for (auto &it_per_id : feature[c])//遍历滑窗内所有特征点
        {
            //判断是否为平行特征点    如果在当前帧-2之前出现过并且至少i在当前帧-1还在，则为平行特征点
            if (it_per_id.start_frame <= frame_count - 2 &&
                it_per_id.start_frame + int(it_per_id.feature_per_frame.size()) - 1 >= frame_count - 1)
            {
                parallax_sum += compensatedParallax2(it_per_id, frame_count);
                parallax_num++; //++平行点数
            }
        }
    }

    if (parallax_num == 0)//判断标准1：平行特征点数为0，当前帧为关键帧，marg掉old
    {
        return true;
    }
    else//判断标准 2：平均平行度小于threshold，则不为关键帧，marg次新帧
    {
        // ROS_DEBUG("parallax_sum: %lf, parallax_num: %d", parallax_sum[c], parallax_num[c]);
        // ROS_DEBUG("current parallax: %lf", parallax_sum[c] / parallax_num[c] * FOCAL_LENGTH);
        last_average_parallax = parallax_sum / parallax_num * FOCAL_LENGTH;
        flag = parallax_sum / parallax_num >= MIN_PARALLAX;
    }
    return flag;//有一个相机判断为关键帧即为关键帧

    //默认主特征点为相机0的特征信息
    // m_feature = feature[0];
    // //如果相机0的特征信息小于60
    // for(int i=0; i<NUM_OF_CAM; i++)
    // {
    //     if(feature[0].size()<60 && feature[0].size()<feature[i].size())
    //     {
    //         m_feature = feature[i];
    //     }
    // }
    // bool flag[2]={false,false};
    // for(int c=0; c<NUM_OF_CAM; c++)
    // {
    //     //if (frame_count < 2 || last_track_num < 20)
    //     //if (frame_count < 2 || last_track_num < 20 || new_feature_num > 0.5 * last_track_num)
    //     //如果只有2帧或者  共试点不超过20个   或者长时间追踪点不超过40   或者新的特征点>共试点数量的一半，就次新帧为关键帧，marg掉old
    //     if (frame_count < 2 || last_track_num[c] < 20 || long_track_num[c] < 40 || new_feature_num[c] > 0.5 * last_track_num[c])
    //         return true;

    //     for (auto &it_per_id : feature[c])//遍历滑窗内所有特征点
    //     {
    //         //判断是否为平行特征点    如果在当前帧-2之前出现过并且至少i在当前帧-1还在，则为平行特征点
    //         if (it_per_id.start_frame <= frame_count - 2 &&
    //             it_per_id.start_frame + int(it_per_id.feature_per_frame.size()) - 1 >= frame_count - 1)
    //         {
    //             parallax_sum[c] += compensatedParallax2(it_per_id, frame_count);
    //             parallax_num[c]++; //++平行点数
    //         }
    //     }

    //     if (parallax_num[c] == 0)//判断标准1：平行特征点数为0，当前帧为关键帧，marg掉old
    //     {
    //         return true;
    //     }
    //     else//判断标准 2：平均平行度小于threshold，则不为关键帧，marg次新帧
    //     {
    //         ROS_DEBUG("parallax_sum: %lf, parallax_num: %d", parallax_sum[c], parallax_num[c]);
    //         ROS_DEBUG("current parallax: %lf", parallax_sum[c] / parallax_num[c] * FOCAL_LENGTH);
    //         last_average_parallax[c] = parallax_sum[c] / parallax_num[c] * FOCAL_LENGTH;
    //         flag[c] = parallax_sum[c] / parallax_num[c] >= MIN_PARALLAX;
    //     }
    // }
    // return flag[0]||flag[1];//有一个相机判断为关键帧即为关键帧
}
vector<pair<Vector3d, Vector3d>> FeatureManager::getCorresponding(int frame_count_l, int frame_count_r)
{
    vector<pair<Vector3d, Vector3d>> corres;
    for (auto &it : feature[0])
    {
        // 如果第一次出现该特征点的图像帧号<左目帧,并且
        if (it.start_frame <= frame_count_l && it.endFrame() >= frame_count_r)
        {
            Vector3d a = Vector3d::Zero(), b = Vector3d::Zero();
            int idx_l = frame_count_l - it.start_frame;
            int idx_r = frame_count_r - it.start_frame;

            a = it.feature_per_frame[idx_l].point;

            b = it.feature_per_frame[idx_r].point;
                
            corres.push_back(make_pair(a, b));
        }
    }
    return corres;
}

vector<pair<Vector3d, Vector3d>> FeatureManager::getCorresponding1(int frame_count_l, int frame_count_r)
{
    vector<pair<Vector3d, Vector3d>> corres;
    for (auto &it : feature[1])
    {
        // 如果第一次出现该特征点的图像帧号<左目帧,并且
        if (it.start_frame <= frame_count_l && it.endFrame() >= frame_count_r)
        {
            Vector3d a = Vector3d::Zero(), b = Vector3d::Zero();
            int idx_l = frame_count_l - it.start_frame;
            int idx_r = frame_count_r - it.start_frame;

            a = it.feature_per_frame[idx_l].point;

            b = it.feature_per_frame[idx_r].point;
                
            corres.push_back(make_pair(a, b));
        }
    }
    return corres;
}


void FeatureManager::setDepth(const VectorXd &x, int Cam_num)
{
    int feature_index = -1;
    // for(int c=0; c<NUM_OF_CAM; c++)
    // {
        for (auto &it_per_id : feature[Cam_num])
        {
            it_per_id.used_num = it_per_id.feature_per_frame.size();
            if (it_per_id.used_num < 4)
                continue;

            it_per_id.estimated_depth = 1.0 / x(++feature_index);
            //ROS_INFO("feature id %d , start_frame %d, depth %f ", it_per_id->feature_id, it_per_id-> start_frame, it_per_id->estimated_depth);
            if (it_per_id.estimated_depth < 0.15 )
            {
                it_per_id.solve_flag = 2;
            }
            else
                it_per_id.solve_flag = 1;
        }
    // }
}

void FeatureManager::removeFailures()
{
    for(int c=0; c<NUM_OF_CAM; c++)
    {
        for (auto it = feature[c].begin(), it_next = feature[c].begin();
            it != feature[c].end(); it = it_next)
        {
            it_next++;
            if (it->solve_flag == 2)
                feature[c].erase(it);
        }
    }
}

void FeatureManager::clearDepth()
{
    for(int c=0; c<NUM_OF_CAM; c++)
    {
        for (auto &it_per_id : feature[c])
            it_per_id.estimated_depth = -1;
    }
}

VectorXd FeatureManager::getDepthVector(int Cam_num)
{
    VectorXd dep_vec(getFeatureCount(Cam_num));
    int feature_index = -1;
    // for(int c=0; c<NUM_OF_CAM; c++)
    // {
        for (auto &it_per_id : feature[Cam_num])
        {
            it_per_id.used_num = it_per_id.feature_per_frame.size();
            if (it_per_id.used_num < 4)
                continue;
    #if 1
            dep_vec(++feature_index) = 1. / it_per_id.estimated_depth;
    #else
            dep_vec(++feature_index) = it_per_id->estimated_depth;
    #endif
        }
    // }
    return dep_vec;
}


void FeatureManager::triangulatePoint(Eigen::Matrix<double, 3, 4> &Pose0, Eigen::Matrix<double, 3, 4> &Pose1,
                        Eigen::Vector2d &point0, Eigen::Vector2d &point1, Eigen::Vector3d &point_3d)
{
    Eigen::Matrix4d design_matrix = Eigen::Matrix4d::Zero();
    design_matrix.row(0) = point0[0] * Pose0.row(2) - Pose0.row(0);
    design_matrix.row(1) = point0[1] * Pose0.row(2) - Pose0.row(1);
    design_matrix.row(2) = point1[0] * Pose1.row(2) - Pose1.row(0);
    design_matrix.row(3) = point1[1] * Pose1.row(2) - Pose1.row(1);
    Eigen::Vector4d triangulated_point;
    triangulated_point =
              design_matrix.jacobiSvd(Eigen::ComputeFullV).matrixV().rightCols<1>();
    point_3d(0) = triangulated_point(0) / triangulated_point(3);
    point_3d(1) = triangulated_point(1) / triangulated_point(3);
    point_3d(2) = triangulated_point(2) / triangulated_point(3);
}


bool FeatureManager::solvePoseByPnP(Eigen::Matrix3d &R, Eigen::Vector3d &P, 
                                      vector<cv::Point2f> &pts2D, vector<cv::Point3f> &pts3D)
{
    Eigen::Matrix3d R_initial;
    Eigen::Vector3d P_initial;

    // w_T_cam ---> cam_T_w 
    R_initial = R.inverse();
    P_initial = -(R_initial * P);

    //printf("pnp size %d \n",(int)pts2D.size() );
    if (int(pts2D.size()) < 4)
    {
        printf("feature tracking not enough, please slowly move you device! \n");
        return false;
    }
    cv::Mat r, rvec, t, D, tmp_r;
    cv::eigen2cv(R_initial, tmp_r);
    cv::Rodrigues(tmp_r, rvec);
    cv::eigen2cv(P_initial, t);
    cv::Mat K = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);  
    bool pnp_succ;
    pnp_succ = cv::solvePnP(pts3D, pts2D, K, D, rvec, t, 1);
    //pnp_succ = solvePnPRansac(pts3D, pts2D, K, D, rvec, t, true, 100, 8.0 / focalLength, 0.99, inliers);

    if(!pnp_succ)
    {
        printf("pnp failed ! \n");
        return false;
    }
    cv::Rodrigues(rvec, r);
    //cout << "r " << endl << r << endl;
    Eigen::MatrixXd R_pnp;
    cv::cv2eigen(r, R_pnp);
    Eigen::MatrixXd T_pnp;
    cv::cv2eigen(t, T_pnp);

    // cam_T_w ---> w_T_cam
    R = R_pnp.transpose();
    P = R * (-T_pnp);

    return true;
}
//Ps,Rs为得到的结果，根据上一帧的位姿pnp求解当前帧的位姿
//frameCnt为当前窗口帧数           Ps，Rs为世界坐标系下，当前bk的位置和姿态        tic、ric为相机坐标系和bk之间的转换
void FeatureManager::initFramePoseByPnP(int frameCnt, Vector3d Ps[], Matrix3d Rs[], Vector3d tic[], Matrix3d ric[], Vector3d tic1[], Matrix3d ric1[])
{
    // std::cout<<"******************initFramePoseByPnP******************************"<<std::endl;
    TicToc featureselect_PnP;
    if(frameCnt > 0)
    {
        vector<cv::Point2f> pts2D;  //存储2d点对应相机归一化坐标
        vector<cv::Point3f> pts3D;  //存储3d点对应世界坐标
            // for (auto &it_per_id : feature[0])//遍历滑窗内所有特征点
            // {
            //     // if(it_per_id.solve_flag != 1)
            //     // {
            //     //     continue;
            //     // }
            //     int index = frameCnt - it_per_id.start_frame; //当前帧-刚开始出现的帧
            //     if((int)it_per_id.feature_per_frame.size() >= index + 1)
            //     {
            //         //feature_per_frame【0】某个路标点被第一张图像观测到的信息
            //         //ric[0]表示第一个相机的左目
            //         Vector3d ptsInCam = ric[0] * (it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth) + tic[0]; 
            //         Vector3d ptsInWorld = Rs[it_per_id.start_frame] * ptsInCam + Ps[it_per_id.start_frame];//世界坐标系下特征点的坐标  根据开始观测到的帧位姿计算出

            //         cv::Point3f point3d(ptsInWorld.x(), ptsInWorld.y(), ptsInWorld.z());//世界坐标系下特征点的坐标  根据开始观测到的帧位姿计算出
            //         cv::Point2f point2d(it_per_id.feature_per_frame[index].point.x(), it_per_id.feature_per_frame[index].point.y());//当前帧观测到的2d像素坐标
            //         pts3D.push_back(point3d);
            //         pts2D.push_back(point2d); //当前帧的归一化平面坐标
            //     }
            // }
            // Eigen::Matrix3d RCam;
            // Eigen::Vector3d PCam;
            // // trans to w_T_cam         上一帧的相机位姿
            // RCam = Rs[frameCnt - 1] * ric[0];
            // PCam = Rs[frameCnt - 1] * tic[0] + Ps[frameCnt - 1];

            // if(solvePoseByPnP(RCam, PCam, pts2D, pts3D))
            // {
            //     // trans to w_T_imu
            //     Rs[frameCnt] = RCam * ric[0].transpose(); 
            //     Ps[frameCnt] = -RCam * ric[0].transpose() * tic[0] + PCam;
            //     Eigen::Quaterniond Q(Rs[frameCnt]);
            //     //cout << "frameCnt: " << frameCnt <<  " pnp Q " << Q.w() << " " << Q.vec().transpose() << endl;
            //     //cout << "frameCnt: " << frameCnt << " pnp P " << Ps[frameCnt].transpose() << endl;
            // }
        // std::cout<<"1111111111Cin feature1"<<std::endl;
        //1、 任取一个相机特征点进行pnp求解   相机0
        if(ave_select_residual <  ave_select_residual1)
        {
            for (auto &it_per_id : feature[0])//遍历滑窗内所有特征点
            {
                // if(it_per_id.solve_flag != 1)
                // {
                //     continue;
                // }
                int index = frameCnt - it_per_id.start_frame; //当前帧-刚开始出现的帧
                if((int)it_per_id.feature_per_frame.size() >= index + 1)
                {
                    //feature_per_frame【0】某个路标点被第一张图像观测到的信息
                    //ric[0]表示第一个相机的左目
                    Vector3d ptsInCam = ric[0] * (it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth) + tic[0]; 
                    Vector3d ptsInWorld = Rs[it_per_id.start_frame] * ptsInCam + Ps[it_per_id.start_frame];//世界坐标系下特征点的坐标  根据开始观测到的帧位姿计算出

                    cv::Point3f point3d(ptsInWorld.x(), ptsInWorld.y(), ptsInWorld.z());//世界坐标系下特征点的坐标  根据开始观测到的帧位姿计算出
                    cv::Point2f point2d(it_per_id.feature_per_frame[index].point.x(), it_per_id.feature_per_frame[index].point.y());//当前帧观测到的2d像素坐标
                    pts3D.push_back(point3d);
                    pts2D.push_back(point2d); //当前帧的归一化平面坐标
                }
            }
            Eigen::Matrix3d RCam;
            Eigen::Vector3d PCam;
            // trans to w_T_cam         上一帧的相机位姿
            RCam = Rs[frameCnt - 1] * ric[0];
            PCam = Rs[frameCnt - 1] * tic[0] + Ps[frameCnt - 1];

            if(solvePoseByPnP(RCam, PCam, pts2D, pts3D))
            {
                // trans to w_T_imu
                Rs[frameCnt] = RCam * ric[0].transpose(); 
                Ps[frameCnt] = -RCam * ric[0].transpose() * tic[0] + PCam;
                Eigen::Quaterniond Q(Rs[frameCnt]);
                //cout << "frameCnt: " << frameCnt <<  " pnp Q " << Q.w() << " " << Q.vec().transpose() << endl;
                //cout << "frameCnt: " << frameCnt << " pnp P " << Ps[frameCnt].transpose() << endl;
            }
        }
        else
        {
            for (auto &it_per_id : feature[1])//遍历滑窗内所有特征点
            {
                // if(it_per_id.solve_flag != 1 )
                // {
                //     continue;
                // }
                int index = frameCnt - it_per_id.start_frame; //当前帧-刚开始出现的帧
                if((int)it_per_id.feature_per_frame.size() >= index + 1)
                {
                    //feature_per_frame【0】某个路标点被第一张图像观测到的信息
                    //ric[0]表示第一个相机的左目
                    Vector3d ptsInCam = ric1[0] * (it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth) + tic1[0]; 
                    Vector3d ptsInWorld = Rs[it_per_id.start_frame] * ptsInCam + Ps[it_per_id.start_frame];//世界坐标系下特征点的坐标  根据开始观测到的帧位姿计算出

                    cv::Point3f point3d(ptsInWorld.x(), ptsInWorld.y(), ptsInWorld.z());//世界坐标系下特征点的坐标  根据开始观测到的帧位姿计算出
                    cv::Point2f point2d(it_per_id.feature_per_frame[index].point.x(), it_per_id.feature_per_frame[index].point.y());//当前帧观测到的2d像素坐标
                    pts3D.push_back(point3d);
                    pts2D.push_back(point2d); //当前帧的归一化平面坐标
                }
            }
            Eigen::Matrix3d RCam;
            Eigen::Vector3d PCam;
            // trans to w_T_cam         上一帧的相机位姿
            RCam = Rs[frameCnt - 1] * ric1[0];
            PCam = Rs[frameCnt - 1] * tic1[0] + Ps[frameCnt - 1];

            if(solvePoseByPnP(RCam, PCam, pts2D, pts3D))
            {
                // trans to w_T_imu
                Rs[frameCnt] = RCam * ric1[0].transpose(); 
                Ps[frameCnt] = -RCam * ric1[0].transpose() * tic1[0] + PCam;
                Eigen::Quaterniond Q(Rs[frameCnt]);
                //cout << "frameCnt: " << frameCnt <<  " pnp Q " << Q.w() << " " << Q.vec().transpose() << endl;
                //cout << "frameCnt: " << frameCnt << " pnp P " << Ps[frameCnt].transpose() << endl;
            }
        }
        // //2、根据求解出的当前帧位姿进行特征点筛选   计算筛选相机1特征点以及平均重投影误差
        featureselect(1, frameCnt, Ps, Rs, tic, ric, tic1, ric1);
    //     //3、根据相机1特征点pnp
    //     vector<cv::Point2f> pts2D_1;  //存储2d点对应相机归一化坐标
    //     vector<cv::Point3f> pts3D_1;  //存储3d点对应世界坐标
    //     for (auto &it_per_id : feature[1])//遍历滑窗内所有特征点
    //     {
    //         // if(it_per_id.solve_flag != 1 )
    //         // {
    //         //     continue;
    //         // }
    //         int index = frameCnt - it_per_id.start_frame; //当前帧-刚开始出现的帧
    //         if((int)it_per_id.feature_per_frame.size() >= index + 1)
    //         {
    //             //feature_per_frame【0】某个路标点被第一张图像观测到的信息
    //             //ric[0]表示第一个相机的左目
    //             Vector3d ptsInCam = ric1[0] * (it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth) + tic1[0]; 
    //             Vector3d ptsInWorld = Rs[it_per_id.start_frame] * ptsInCam + Ps[it_per_id.start_frame];//世界坐标系下特征点的坐标  根据开始观测到的帧位姿计算出

    //             cv::Point3f point3d(ptsInWorld.x(), ptsInWorld.y(), ptsInWorld.z());//世界坐标系下特征点的坐标  根据开始观测到的帧位姿计算出
    //             cv::Point2f point2d(it_per_id.feature_per_frame[index].point.x(), it_per_id.feature_per_frame[index].point.y());//当前帧观测到的2d像素坐标
    //             pts3D_1.push_back(point3d);
    //             pts2D_1.push_back(point2d); //当前帧的归一化平面坐标
    //         }
    //     }
    //     // trans to w_T_cam         上一帧的相机位姿
    //     RCam = Rs[frameCnt - 1] * ric1[0];
    //     PCam = Rs[frameCnt - 1] * tic1[0] + Ps[frameCnt - 1];

    //     if(solvePoseByPnP(RCam, PCam, pts2D_1, pts3D_1))
    //     {
    //         // trans to w_T_imu
    //         Rs[frameCnt] = RCam * ric1[0].transpose(); 
    //         Ps[frameCnt] = -RCam * ric1[0].transpose() * tic1[0] + PCam;
    //         Eigen::Quaterniond Q(Rs[frameCnt]);
    //         //cout << "frameCnt: " << frameCnt <<  " pnp Q " << Q.w() << " " << Q.vec().transpose() << endl;
    //         //cout << "frameCnt: " << frameCnt << " pnp P " << Ps[frameCnt].transpose() << endl;
    //     }
    //     // //4、根据相机1特征点解算pnp进行相机0特征点筛选
    //     featureselect(0, frameCnt, Ps, Rs, tic, ric, tic1, ric1);
    //     // //5、判断相机0特征平均重投影误差和相机1的大小，选择小的进行pnp作为当前帧位姿
    //     if(ave_select_residual < ave_select_residual1)
    //     {
    //         cout<<"00000000000000000ave_select_residual < 1111111111111ave_select_residual1 "<<endl;
    //         vector<cv::Point2f> pts2D;  //存储2d点对应相机归一化坐标
    //         vector<cv::Point3f> pts3D;  //存储3d点对应世界坐标
    //         for (auto &it_per_id : feature[0])//遍历滑窗内所有特征点
    //         {
    //             // if(it_per_id.solve_flag != 1 )
    //             // {
    //             //     continue;
    //             // }
    //             int index = frameCnt - it_per_id.start_frame; //当前帧-刚开始出现的帧
    //             if((int)it_per_id.feature_per_frame.size() >= index + 1)
    //             {
    //                 //feature_per_frame【0】某个路标点被第一张图像观测到的信息
    //                 //ric[0]表示第一个相机的左目
    //                 Vector3d ptsInCam = ric[0] * (it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth) + tic[0]; 
    //                 Vector3d ptsInWorld = Rs[it_per_id.start_frame] * ptsInCam + Ps[it_per_id.start_frame];//世界坐标系下特征点的坐标  根据开始观测到的帧位姿计算出

    //                 cv::Point3f point3d(ptsInWorld.x(), ptsInWorld.y(), ptsInWorld.z());//世界坐标系下特征点的坐标  根据开始观测到的帧位姿计算出
    //                 cv::Point2f point2d(it_per_id.feature_per_frame[index].point.x(), it_per_id.feature_per_frame[index].point.y());//当前帧观测到的2d像素坐标
    //                 pts3D.push_back(point3d);
    //                 pts2D.push_back(point2d); //当前帧的归一化平面坐标
    //             }
    //         }
    //         // trans to w_T_cam         上一帧的相机位姿
    //         RCam = Rs[frameCnt - 1] * ric[0];
    //         PCam = Rs[frameCnt - 1] * tic[0] + Ps[frameCnt - 1];

    //         if(solvePoseByPnP(RCam, PCam, pts2D, pts3D))
    //         {
    //             // trans to w_T_imu
    //             Rs[frameCnt] = RCam * ric[0].transpose(); 
    //             Ps[frameCnt] = -RCam * ric[0].transpose() * tic[0] + PCam;
    //             Eigen::Quaterniond Q(Rs[frameCnt]);
    //             //cout << "frameCnt: " << frameCnt <<  " pnp Q " << Q.w() << " " << Q.vec().transpose() << endl;
    //             //cout << "frameCnt: " << frameCnt << " pnp P " << Ps[frameCnt].transpose() << endl;
    //         }
    //     }
    }   
    printf("featureselect_PnP time: %f\n", featureselect_PnP.toc());
}
//双目三角化
void FeatureManager::triangulate(int frameCnt, Vector3d Ps[], Matrix3d Rs[], Vector3d tic[], Matrix3d ric[], Vector3d tic1[], Matrix3d ric1[])
{
    Eigen::Vector3d * Tic[2];
    Tic[0] = tic;
    Tic[1] = tic1;
    Eigen::Matrix3d * Ric[2];
    Ric[0] = ric;
    Ric[1] = ric1;

    for(int c=0; c<NUM_OF_CAM; c++) //遍历相机
    {
        for (auto &it_per_id : feature[c])//遍历所有特征点
        {
            if (it_per_id.estimated_depth > 0)
                continue;
            //通过左右目三角化
            if(STEREO && it_per_id.feature_per_frame[0].is_stereo)
            {
                int imu_i = it_per_id.start_frame;//该特征点开始的关键帧
                //利用imu的位姿计算左相机位姿
                Eigen::Matrix<double, 3, 4> leftPose;
                Eigen::Vector3d t0 = Ps[imu_i] + Rs[imu_i] * Tic[c][0];
                Eigen::Matrix3d R0 = Rs[imu_i] * Ric[c][0];
                leftPose.leftCols<3>() = R0.transpose();
                leftPose.rightCols<1>() = -R0.transpose() * t0;
                //cout << "left pose " << leftPose << endl;

                Eigen::Matrix<double, 3, 4> rightPose;
                Eigen::Vector3d t1 = Ps[imu_i] + Rs[imu_i] * Tic[c][1];
                Eigen::Matrix3d R1 = Rs[imu_i] * Ric[c][1];
                rightPose.leftCols<3>() = R1.transpose();
                rightPose.rightCols<1>() = -R1.transpose() * t1;
                //cout << "right pose " << rightPose << endl;

                Eigen::Vector2d point0, point1;
                Eigen::Vector3d point3d;
                point0 = it_per_id.feature_per_frame[0].point.head(2);
                point1 = it_per_id.feature_per_frame[0].pointRight.head(2);
                //cout << "point0 " << point0.transpose() << endl;
                //cout << "point1 " << point1.transpose() << endl;

                triangulatePoint(leftPose, rightPose, point0, point1, point3d);//利用svd方法三角化，得到imu坐标系下的三维点坐标
                Eigen::Vector3d localPoint;
                localPoint = leftPose.leftCols<3>() * point3d + leftPose.rightCols<1>();//求得左相机坐标系下的三维点坐标
                double depth = localPoint.z();
                if (depth > 0)
                    it_per_id.estimated_depth = depth;//得到左相机坐标系下的深度
                else
                    it_per_id.estimated_depth = INIT_DEPTH;
                /*
                Vector3d ptsGt = pts_gt[it_per_id.feature_id];
                printf("stereo %d pts: %f %f %f gt: %f %f %f \n",it_per_id.feature_id, point3d.x(), point3d.y(), point3d.z(),
                                                                ptsGt.x(), ptsGt.y(), ptsGt.z());
                */
                continue;
            }
            //通过前后帧图像来三角化
            else if(it_per_id.feature_per_frame.size() > 1)
            {
                int imu_i = it_per_id.start_frame;
                Eigen::Matrix<double, 3, 4> leftPose;
                Eigen::Vector3d t0 = Ps[imu_i] + Rs[imu_i] * Tic[c][0];
                Eigen::Matrix3d R0 = Rs[imu_i] * Ric[c][0];
                leftPose.leftCols<3>() = R0.transpose();
                leftPose.rightCols<1>() = -R0.transpose() * t0;

                imu_i++;
                Eigen::Matrix<double, 3, 4> rightPose;
                Eigen::Vector3d t1 = Ps[imu_i] + Rs[imu_i] * Tic[c][0];
                Eigen::Matrix3d R1 = Rs[imu_i] * Ric[c][0];
                rightPose.leftCols<3>() = R1.transpose();
                rightPose.rightCols<1>() = -R1.transpose() * t1;

                Eigen::Vector2d point0, point1;
                Eigen::Vector3d point3d;
                point0 = it_per_id.feature_per_frame[0].point.head(2);
                point1 = it_per_id.feature_per_frame[1].point.head(2);
                triangulatePoint(leftPose, rightPose, point0, point1, point3d);
                Eigen::Vector3d localPoint;
                localPoint = leftPose.leftCols<3>() * point3d + leftPose.rightCols<1>();
                double depth = localPoint.z();
                if (depth > 0)
                    it_per_id.estimated_depth = depth;
                else
                    it_per_id.estimated_depth = INIT_DEPTH;
                /*
                Vector3d ptsGt = pts_gt[it_per_id.feature_id];
                printf("motion  %d pts: %f %f %f gt: %f %f %f \n",it_per_id.feature_id, point3d.x(), point3d.y(), point3d.z(),
                                                                ptsGt.x(), ptsGt.y(), ptsGt.z());
                */
                continue;
            }
            it_per_id.used_num = it_per_id.feature_per_frame.size();//当前特征点被使用的次数
            if (it_per_id.used_num < 4)
                continue;

            int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;

            Eigen::MatrixXd svd_A(2 * it_per_id.feature_per_frame.size(), 4);
            int svd_idx = 0;

            Eigen::Matrix<double, 3, 4> P0;
            Eigen::Vector3d t0 = Ps[imu_i] + Rs[imu_i] * Tic[c][0];
            Eigen::Matrix3d R0 = Rs[imu_i] * Ric[c][0];
            P0.leftCols<3>() = Eigen::Matrix3d::Identity();
            P0.rightCols<1>() = Eigen::Vector3d::Zero();

            for (auto &it_per_frame : it_per_id.feature_per_frame)//遍历当前特征点在每一幅图像中的信息
            {
                imu_j++;

                Eigen::Vector3d t1 = Ps[imu_j] + Rs[imu_j] * Tic[c][0];
                Eigen::Matrix3d R1 = Rs[imu_j] * Ric[c][0];
                Eigen::Vector3d t = R0.transpose() * (t1 - t0);
                Eigen::Matrix3d R = R0.transpose() * R1;
                Eigen::Matrix<double, 3, 4> P;
                P.leftCols<3>() = R.transpose();
                P.rightCols<1>() = -R.transpose() * t;
                Eigen::Vector3d f = it_per_frame.point.normalized();
                svd_A.row(svd_idx++) = f[0] * P.row(2) - f[2] * P.row(0);
                svd_A.row(svd_idx++) = f[1] * P.row(2) - f[2] * P.row(1);

                if (imu_i == imu_j)
                    continue;
            }
            ROS_ASSERT(svd_idx == svd_A.rows());
            Eigen::Vector4d svd_V = Eigen::JacobiSVD<Eigen::MatrixXd>(svd_A, Eigen::ComputeThinV).matrixV().rightCols<1>();
            double svd_method = svd_V[2] / svd_V[3];
            //it_per_id->estimated_depth = -b / A;
            //it_per_id->estimated_depth = svd_V[2] / svd_V[3];

            it_per_id.estimated_depth = svd_method;
            //it_per_id->estimated_depth = INIT_DEPTH;

            if (it_per_id.estimated_depth < 0.1)
            {
                it_per_id.estimated_depth = INIT_DEPTH;
            }

        }
    }
}
//计算当前帧每个相机中每个特征点在初始帧和当前帧之间的重投影误差，将重投影误差增加的特征点滤掉
void FeatureManager::featureselect(int Cam_num, int frameCnt, Vector3d Ps[], Matrix3d Rs[], Vector3d tic[], Matrix3d ric[], Vector3d tic1[], Matrix3d ric1[])
{
    //1、计算所有相机中特征点在当前帧相对于最开始帧的投影误差
    TicToc t_select;
    select_num_0 = 1;//记录相机0多少个特征点在当前帧出现
    select_num_1 = 1;//记录相机1多少个特征点在当前帧出现
    // int feature0_size = (int)feature[0].size();
    // int feature1_size = (int)feature[1].size();
    // std::cout<<"1111111111111111111111111feature[0].size: "<<feature0_size<<std::endl;
    // std::cout<<"---------------------------------------------------------"<<std::endl;
    // std::cout<<"2222222222222222222222222feature[1].size: "<<feature1_size<<std::endl;
    // int sum_feature_size = feature0_size + feature1_size;
    // double coefficient_of_select_residual_ = feature0_size / 
    double select_residual_ = 0;
    double select_residual1_ = 0;
    double sum_select_residual_ = 1;
    double sum_select_residual1_ = 1;

    // ave_select_residual = 1;
    // ave_select_residual1 = 1 ;
    // if(Cam_num == 1 )
    // {
        for(auto &it_per_id : feature[1])
        {
            int index = frameCnt - it_per_id.start_frame; //当前帧-刚开始出现的帧
            //保证在当前帧出现
            if((int)it_per_id.feature_per_frame.size() < index + 1 )
            { 
                // it_per_id.solve_flag = 2;
                continue;
            }
                //feature_per_frame【1】某个路标点被第一张图像观测到的信息
            Vector3d pts_InCam_i = ric1[0] * (it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth) + tic1[0]; 
            Vector3d pts_InWorld_i = Rs[it_per_id.start_frame] * pts_InCam_i + Ps[it_per_id.start_frame];//世界坐标系下特征点的坐标  根据开始观测到的帧位姿计算出

            //特征点在当前帧的位姿
            Vector3d pts_InWorld_j = Rs[frameCnt].transpose() * (pts_InWorld_i - Ps[frameCnt]);            
            Vector3d pts_InCam_j = ric1[0].transpose() * (pts_InWorld_j - tic1[0]); 
            //
            cv::Point2f point2d_pro(pts_InCam_j.x()/pts_InCam_j.z(), pts_InCam_j.y()/pts_InCam_j.z());
            cv::Point2f point2d_ob(it_per_id.feature_per_frame[index].point.x(), it_per_id.feature_per_frame[index].point.y());
            select_residual1_ = sqrt((point2d_ob.x-point2d_pro.x)*(point2d_ob.x-point2d_pro.x) + (point2d_ob.y-point2d_pro.y)*(point2d_ob.y-point2d_pro.y));
            it_per_id.select_residual_ = select_residual1_;
            if(select_residual1_ <  ave_select_residual1)
            {
                select_num_1++;
                sum_select_residual1_ += select_residual1_;
            }
            // else 
            // {
            //     it_per_id.select_residual_ = 1;
            // }
        }
        ave_select_residual1 = sum_select_residual1_ /double(select_num_1);
        // cout<<"ave_select_residual1 = "<<ave_select_residual1<<endl;
    // }
    // else
    // {
        for(auto &it_per_id : feature[0])
        {
            int index = frameCnt - it_per_id.start_frame; //当前帧-刚开始出现的帧
            //保证在当前帧出现
            if((int)it_per_id.feature_per_frame.size() < index + 1 )
            { 
                // it_per_id.solve_flag = 2;
                continue;
            }
                //feature_per_frame【1】某个路标点被第一张图像观测到的信息
            Vector3d pts_InCam_i = ric[0] * (it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth) + tic[0]; 
            Vector3d pts_InWorld_i = Rs[it_per_id.start_frame] * pts_InCam_i + Ps[it_per_id.start_frame];//世界坐标系下特征点的坐标  根据开始观测到的帧位姿计算出

            //特征点在当前帧的位姿
            Vector3d pts_InWorld_j = Rs[frameCnt].transpose() * (pts_InWorld_i - Ps[frameCnt]);            
            Vector3d pts_InCam_j = ric[0].transpose() * (pts_InWorld_j - tic[0]); 
            //
            cv::Point2f point2d_pro(pts_InCam_j.x()/pts_InCam_j.z(), pts_InCam_j.y()/pts_InCam_j.z());
            cv::Point2f point2d_ob(it_per_id.feature_per_frame[index].point.x(), it_per_id.feature_per_frame[index].point.y());
            select_residual_ = sqrt((point2d_ob.x-point2d_pro.x)*(point2d_ob.x-point2d_pro.x) + (point2d_ob.y-point2d_pro.y)*(point2d_ob.y-point2d_pro.y));
            it_per_id.select_residual_ = select_residual_;
            if(select_residual_ < ave_select_residual)
            {
                select_num_0++;
                sum_select_residual_ += select_residual_;
            }
            // else 
            // {
            //     it_per_id.select_residual_ = 1;
            // }
        }
        ave_select_residual = sum_select_residual_ /double(select_num_0);
        // cout<<"ave_select_residual00000000000000000 = "<<ave_select_residual<<endl;
    ofstream foutC("/home/xu/output/RMSC_t_select.txt", ios::app);
        foutC.setf(ios::fixed, ios::floatfield);
        foutC.precision(9);
        foutC << ros::Time::now().toSec() << " ";
        foutC.precision(5);
        foutC << t_select.toc() <<endl;    
    foutC.close();
    // }
}


void FeatureManager::removeOutlier(set<int> &outlierIndex, set<int> &outlierIndex1)
{
    std::set<int>::iterator itSet;

    // if(!Opt_Select)
    // {
        for (auto it = feature[0].begin(), it_next = feature[0].begin();
            it != feature[0].end(); it = it_next)
        {
            it_next++;
            int index = it->feature_id;
            itSet = outlierIndex.find(index);
            if(itSet != outlierIndex.end())
            {
                feature[0].erase(it);
                //printf("remove outlier %d \n", index);
            }
        }
    // }
    // else
    // {
    std::set<int>::iterator itSet1;
        for (auto it = feature[1].begin(), it_next = feature[1].begin();
            it != feature[1].end(); it = it_next)
        {
            it_next++;
            int index = it->feature_id;
            itSet1 = outlierIndex1.find(index);
            if(itSet1 != outlierIndex1.end())
            {
                feature[1].erase(it);
                //printf("remove outlier %d \n", index);
            }
        }
    // }
    
}

void FeatureManager::removeBackShiftDepth(Eigen::Matrix3d marg_R, Eigen::Vector3d marg_P, Eigen::Matrix3d new_R, Eigen::Vector3d new_P)
{
    for(int c=0; c<NUM_OF_CAM; c++)
    {
        for (auto it = feature[c].begin(), it_next = feature[c].begin();
            it != feature[c].end(); it = it_next)
        {
            it_next++;

            if (it->start_frame != 0)
                it->start_frame--;
            else
            {
                Eigen::Vector3d uv_i = it->feature_per_frame[0].point;  
                it->feature_per_frame.erase(it->feature_per_frame.begin());
                if (it->feature_per_frame.size() < 2)
                {
                    feature[c].erase(it);
                    continue;
                }
                else
                {
                    Eigen::Vector3d pts_i = uv_i * it->estimated_depth;
                    Eigen::Vector3d w_pts_i = marg_R * pts_i + marg_P;
                    Eigen::Vector3d pts_j = new_R.transpose() * (w_pts_i - new_P);
                    double dep_j = pts_j(2);
                    if (dep_j > 0)
                        it->estimated_depth = dep_j;
                    else
                        it->estimated_depth = INIT_DEPTH;
                }
            }
            // remove tracking-lost feature after marginalize
            /*
            if (it->endFrame() < WINDOW_SIZE - 1)
            {
                feature.erase(it);
            }
            */
        }
    }
}

void FeatureManager::removeBack()
{
    for(int c=0; c<NUM_OF_CAM; c++)
    {
        for (auto it = feature[c].begin(), it_next = feature[c].begin();
            it != feature[c].end(); it = it_next)
        {
            it_next++;

            if (it->start_frame != 0)
                it->start_frame--;
            else
            {
                it->feature_per_frame.erase(it->feature_per_frame.begin());
                if (it->feature_per_frame.size() == 0)
                    feature[c].erase(it);
            }
        }
    }
}

void FeatureManager::removeFront(int frame_count)
{
    for(int c=0; c<NUM_OF_CAM; c++)
    {
        for (auto it = feature[c].begin(), it_next = feature[c].begin(); it != feature[c].end(); it = it_next)
        {
            it_next++;

            if (it->start_frame == frame_count)
            {
                it->start_frame--;
            }
            else
            {
                int j = WINDOW_SIZE - 1 - it->start_frame;
                if (it->endFrame() < frame_count - 1)
                    continue;
                it->feature_per_frame.erase(it->feature_per_frame.begin() + j);
                if (it->feature_per_frame.size() == 0)
                    feature[c].erase(it);
            }
        }
    }
}

double FeatureManager::compensatedParallax2(const FeaturePerId &it_per_id, int frame_count)
{
    //check the second last frame is keyframe or not
    //parallax betwwen seconde last frame and third last frame
    const FeaturePerFrame &frame_i = it_per_id.feature_per_frame[frame_count - 2 - it_per_id.start_frame];
    const FeaturePerFrame &frame_j = it_per_id.feature_per_frame[frame_count - 1 - it_per_id.start_frame];

    double ans = 0;
    Vector3d p_j = frame_j.point;

    double u_j = p_j(0);
    double v_j = p_j(1);

    Vector3d p_i = frame_i.point;
    Vector3d p_i_comp;

    //int r_i = frame_count - 2;
    //int r_j = frame_count - 1;
    //p_i_comp = ric[camera_id_j].transpose() * Rs[r_j].transpose() * Rs[r_i] * ric[camera_id_i] * p_i;
    p_i_comp = p_i;
    double dep_i = p_i(2);
    double u_i = p_i(0) / dep_i;
    double v_i = p_i(1) / dep_i;
    double du = u_i - u_j, dv = v_i - v_j;

    double dep_i_comp = p_i_comp(2);
    double u_i_comp = p_i_comp(0) / dep_i_comp;
    double v_i_comp = p_i_comp(1) / dep_i_comp;
    double du_comp = u_i_comp - u_j, dv_comp = v_i_comp - v_j;

    ans = max(ans, sqrt(min(du * du + dv * dv, du_comp * du_comp + dv_comp * dv_comp)));

    return ans;
}