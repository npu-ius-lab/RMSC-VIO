/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 *
 * This file is part of VINS.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "estimator.h"
#include "../utility/visualization.h"

Estimator::Estimator() : f_manager{Rs}
{
    ROS_INFO("init begins");
    initThreadFlag = false;
    clearState();
}

Estimator::~Estimator()
{
    if (MULTIPLE_THREAD)
    {
        processThread.join();
        printf("join thread \n");
    }
}

void Estimator::clearState()
{
    mProcess.lock();
    while (!accBuf.empty())
        accBuf.pop();
    while (!gyrBuf.empty())
        gyrBuf.pop();
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        while (!featureBuf[i].empty())
            featureBuf[i].pop();
    }
    prevTime = -1;
    curTime = 0;
    openExEstimation = 0;
    initP = Eigen::Vector3d(0, 0, 0);
    initR = Eigen::Matrix3d::Identity();
    inputImageCnt = 0;
    initFirstPoseFlag = false;

    for (int i = 0; i < WINDOW_SIZE + 1; i++)
    {
        Rs[i].setIdentity();
        Ps[i].setZero();
        Vs[i].setZero();
        Bas[i].setZero();
        Bgs[i].setZero();
        dt_buf[i].clear();
        linear_acceleration_buf[i].clear();
        angular_velocity_buf[i].clear();

        if (pre_integrations[i] != nullptr)
        {
            delete pre_integrations[i];
        }
        pre_integrations[i] = nullptr;
    }

    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        tic[i] = Vector3d::Zero();
        ric[i] = Matrix3d::Identity();
    }

    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        tic1[i] = Vector3d::Zero();
        ric1[i] = Matrix3d::Identity();
    }

    first_imu = false,
    sum_of_back = 0;
    sum_of_front = 0;
    frame_count = 0;
    solver_flag = INITIAL;
    initial_timestamp = 0;
    all_image_frame.clear();

    if (tmp_pre_integration != nullptr)
        delete tmp_pre_integration;
    if (last_marginalization_info != nullptr)
        delete last_marginalization_info;

    tmp_pre_integration = nullptr;
    last_marginalization_info = nullptr;
    last_marginalization_parameter_blocks.clear();

    f_manager.clearState();

    failure_occur = 0;

    mProcess.unlock();
}
// 设置参数，并开始processMeasurements线程
void Estimator::setParameter()
{
    mProcess.lock();
    //-------------------设置相机0的旋转矩阵
    for (int i = 0; i < NUM_OF_CAM; i++) // NUM_OF_CAM==2
    {
        tic[i] = TIC[i];
        ric[i] = RIC[i];
        cout << " exitrinsic cam " << i << endl
             << ric[i] << endl
             << tic[i].transpose() << endl;
    }
    f_manager.setRic(ric); //----------------将旋转矩阵导入到f_manager中
    //----------------如果为双相机，设置相机1的旋转矩阵
    if (DOUBLE_CAM)
    {
        for (int i = 0; i < NUM_OF_CAM; i++) // NUM_OF_CAM==2
        {
            tic1[i] = TIC1[i];
            ric1[i] = RIC1[i];
            cout << " exitrinsic cam " << i << endl
                 << ric1[i] << endl
                 << tic1[i].transpose() << endl;
        }
        f_manager.setRic1(ric1);
    }

    ProjectionTwoFrameOneCamFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
    ProjectionTwoFrameTwoCamFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
    ProjectionOneFrameTwoCamFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
    td = TD;
    g = G;
    cout << "set g " << g.transpose() << endl;
    // 读取相机内参
    featureTracker.readIntrinsicParameter(CAM_NAMES);

    std::cout << "MULTIPLE_THREAD is " << MULTIPLE_THREAD << '\n'; // 值为1
    if (MULTIPLE_THREAD && !initThreadFlag)
    { // 如果是单线程，且线程没有处理，则开启一个Estimator类内的新线程processMeasurements
        initThreadFlag = true;
        // 声明并定义一个新线程
        processThread = std::thread(&Estimator::processMeasurements, this);
    }
    mProcess.unlock();
}

void Estimator::changeSensorType(int use_imu, int use_stereo)
{
    bool restart = false;
    mProcess.lock();
    if (!use_imu && !use_stereo)
        printf("at least use two sensors! \n");
    else
    {
        if (USE_IMU != use_imu)
        {
            USE_IMU = use_imu;
            if (USE_IMU)
            {
                // reuse imu; restart system
                restart = true;
            }
            else
            {
                if (last_marginalization_info != nullptr)
                    delete last_marginalization_info;

                tmp_pre_integration = nullptr;
                last_marginalization_info = nullptr;
                last_marginalization_parameter_blocks.clear();
            }
        }

        STEREO = use_stereo;
        printf("use imu %d use stereo %d\n", USE_IMU, STEREO);
    }
    mProcess.unlock();
    if (restart)
    {
        clearState();
        setParameter();
    }
}

// 输入图像
// 1、featuretracker，提取当前帧的特征点
// 2、添加一帧特征点，processmeasurements处理
void Estimator::inputImage(double t, const cv::Mat &_img, const cv::Mat &_img_1, const cv::Mat &_img1, const cv::Mat &_img1_1)
{
    // std::cout<<"*****************inputImage*******************************"<<std::endl;
    inputImageCnt++;
    // map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> featureFrame;
    TicToc featureTrackerTime;
    
    if (_img1.empty())
        featureFrame[0] = featureTracker.trackImage(0, t, _img);
    else
        featureFrame[0] = featureTracker.trackImage(0, t, _img, _img1); // 即两幅图像中的特征点的信息（位置，像素位置，相机id,特征点id,以及像素速度）放入featureFrame中
    // printf("featureTracker time: %f\n", featureTrackerTime.toc());
    if (_img1_1.empty())
        featureFrame[1] = featureTracker.trackImage(1, t, _img_1);
    else
        featureFrame[1] = featureTracker.trackImage(1, t, _img_1, _img1_1); // 即两幅图像中的特征点的信息（位置，像素位置，相机id,特征点id,以及像素速度）放入featureFrame中
    // printf("0000000000featureTracker time: %f\n", featureTrackerTime.toc());
    ofstream foutC("/home/xu/output/RMSC_t_Tracker.txt", ios::app);
        foutC.setf(ios::fixed, ios::floatfield);
        foutC.precision(9);
        foutC << ros::Time::now().toSec() << " ";
        foutC.precision(5);
        foutC << featureTrackerTime.toc() <<endl;    
    foutC.close();
    // if(featureFrame[0].size() > featureFrame[1].size())
    // {

    // }

    if (SHOW_TRACK) // SHOW_TRACK==1
    {
        cv::Mat imgTrack = featureTracker.getTrackImage(0);
        pubTrackImage(imgTrack, t); // 发布TrackImage
    }

    if (SHOW_TRACK) // SHOW_TRACK==1
    {
        cv::Mat imgTrack = featureTracker.getTrackImage(1);
        pubTrackImage1(imgTrack, t); // 发布TrackImage
    }

    getfeatureBuf(t, inputImageCnt);
}

void Estimator::getfeatureBuf(double t, int Cnt)
{
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        if (MULTIPLE_THREAD) // MULTIPLE_THREAD==1
        {
            if (Cnt % 2 == 0)
            {
                mBuf[i].lock();
                featureBuf[i].push(make_pair(t, featureFrame[i]));
                mBuf[i].unlock();
            }
        }
        else
        {
            mBuf[i].lock();
            featureBuf[i].push(make_pair(t, featureFrame[i]));
            mBuf[i].unlock();
            TicToc processTime;
            processMeasurements();
            printf("process time: %f\n", processTime.toc());
        }
    }
}

void Estimator::inputIMU(double t, const Vector3d &linearAcceleration, const Vector3d &angularVelocity)
{
    imuBuf.lock();
    accBuf.push(make_pair(t, linearAcceleration));
    gyrBuf.push(make_pair(t, angularVelocity));
    // printf("input imu with time %f \n", t);
    imuBuf.unlock();

    if (solver_flag == NON_LINEAR) // 成功
    {
        mPropagate.lock();                                      // lock 函数的操作在形式上与 using 相似。其目的是锁定一段代码，使在同一时间，只有一个线程可以运行它。
        fastPredictIMU(t, linearAcceleration, angularVelocity); // 计算位置速度，姿态
        pubLatestOdometry(latest_P, latest_Q, latest_V, t);     // 发布odometry
        mPropagate.unlock();
    }
}

void Estimator::inputFeature(double t, const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &featureFrame)
{
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        mBuf[i].lock();
        featureBuf[i].push(make_pair(t, featureFrame)); // 把特征点信息放入featureBuf中
        mBuf[i].unlock();
    }

    if (!MULTIPLE_THREAD)      // MULTIPLE_THREAD==1
        processMeasurements(); // 不进行处理
}

bool Estimator::getIMUInterval(double t0, double t1, vector<pair<double, Eigen::Vector3d>> &accVector,
                               vector<pair<double, Eigen::Vector3d>> &gyrVector)
{
    if (accBuf.empty())
    {
        printf("not receive imu\n");
        return false;
    }
    // printf("get imu from %f %f\n", t0, t1);
    // printf("imu fornt time %f   imu end time %f\n", accBuf.front().first, accBuf.back().first);
    if (t1 <= accBuf.back().first)
    {
        while (accBuf.front().first <= t0)
        {
            accBuf.pop();
            gyrBuf.pop();
        }
        while (accBuf.front().first < t1)
        {
            accVector.push_back(accBuf.front());
            accBuf.pop();
            gyrVector.push_back(gyrBuf.front());
            gyrBuf.pop();
        }
        accVector.push_back(accBuf.front());
        gyrVector.push_back(gyrBuf.front());
    }
    else
    {
        printf("wait for imu\n");
        return false;
    }
    return true;
}

bool Estimator::IMUAvailable(double t)
{
    if (!accBuf.empty() && t <= accBuf.back().first)
        return true;
    else
        return false;
}

void Estimator::processMeasurements()
{
    // std::cout << "*****************CIN processMeasurements!*******************************" << std::endl;
    while (1)
    {
        // printf("process measurments\n");
        // std::cout<<"*****************66666666666666666666666666666666666666*******************************"<<std::endl;
        pair<double, map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>>> feature[2];
        vector<pair<double, Eigen::Vector3d>> accVector, gyrVector;
        if (!featureBuf[0].empty() || !featureBuf[1].empty())
        // if(!featureBuf[0].empty())
        {
            // for (int i = 0; i < NUM_OF_CAM; i++)
            // {
            //     mBuf[i].lock();
            //     // feature[i] = featureBuf[i].front();
            // }
            for (int i = 0; i < NUM_OF_CAM; i++)
            {
                // mBuf[i].lock();
                feature[i] = featureBuf[i].front();
            }
            // feature[0] = featureBuf[0].front();
            //  if(feature[0].second.size()>feature[1].second.size())
            //  {
            //      curTime = feature[0].first + td;
            //  }
            //  else
            //  {
            //      curTime = feature[1].first + td;
            //  }
            curTime = feature[0].first + td;
            // std::cout << "curTime: " << curTime << std::endl;
            while (1)
            {
                // 判读输入的时间t时候的imu是否可用
                if (!USE_IMU || IMUAvailable(curTime))
                    break;
                else
                {
                    printf("**************wait for imu ... \n");
                    if (!MULTIPLE_THREAD)
                        return;
                    std::chrono::milliseconds dura(5);
                    std::this_thread::sleep_for(dura);
                }
            }
            for (int i = 0; i < NUM_OF_CAM; i++)
            {
                // featureBuf[i].pop();
                mBuf[i].lock();
            }

            // mBuf[0].lock();
            // 对imu的时间进行判断，将队列中的imu数据存放到avvVector和gyrVector中，完成之后返回true
            if (USE_IMU)
                getIMUInterval(prevTime, curTime, accVector, gyrVector); // 将imu数值传输到accVector线加速度，gyrVector角速度

            for (int i = 0; i < NUM_OF_CAM; i++)
            {
                featureBuf[i].pop();
                // mBuf[i].lock();
            }
            for (int i = 0; i < NUM_OF_CAM; i++)
            {
                // featureBuf[i].pop();
                mBuf[i].unlock();
            }
            // featureBuf[0].pop();
            // mBuf[0].unlock();

            if (USE_IMU)
            {
                if (!initFirstPoseFlag)          // 第一次处理时为0
                    initFirstIMUPose(accVector); // 计算初始R0,并将....flag==1，初始化IMU的姿态，求解一个姿态角i，然后把航向角设为0
                for (size_t i = 0; i < accVector.size(); i++)
                {
                    double dt;
                    if (i == 0)
                        dt = accVector[i].first - prevTime;
                    else if (i == accVector.size() - 1)
                        dt = curTime - accVector[i - 1].first;
                    else
                        dt = accVector[i].first - accVector[i - 1].first;
                    processIMU(accVector[i].first, dt, accVector[i].second, gyrVector[i].second); // 处理IMU数据
                }
            }
            // featureFusion(feature[0], feature[1]);//------进行特征点融合
            mProcess.lock();

            // getric(feature,ESTIMATE_EXTRINSIC,ric,RIC);//得到相机0与imu之间的旋转矩阵ric
            // getric1(feature1,ESTIMATE_EXTRINSIC1,ric1,RIC1);//得到相机1与imu之间的旋转矩阵ric1

            // featureFusion(feature, feature1);//------进行特征点融合
            processImage(feature[0].second, feature[1].second, feature[0].first);
            prevTime = curTime;

            printStatistics(*this, 0);

            std_msgs::Header header;
            header.frame_id = "world";
            header.stamp = ros::Time(feature[0].first);

            pubTF(*this, header);       // 发布world,body,camera的转换关系
            pubOdometry(*this, header); // 发布较为准确的odometry
            pubKeyPoses(*this, header);
            pubCameraPose(*this, header); // 发布相机的位置
            pubCameraPose1(*this, header); // 发布相机1的位置
            pubPointCloud(*this, header); // 发布点云信息
            pubPointCloud1(*this, header); // 发布点云1信息
            pubKeyframe(*this);
            // pubTF(*this, header);//发布world,body,camera的转换关系
            mProcess.unlock();
        }
        if (!MULTIPLE_THREAD)
            break;
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    } // while结束
}

// 第一帧IMU姿态初始化
// 用初始时刻加速度方向对齐重力加速度方向，得到一个旋转，使得初始IMU的z轴指向重力加速度方向
void Estimator::initFirstIMUPose(vector<pair<double, Eigen::Vector3d>> &accVector)
{
    printf("init first imu pose\n");
    initFirstPoseFlag = true;
    // return;
    Eigen::Vector3d averAcc(0, 0, 0);
    int n = (int)accVector.size();
    for (size_t i = 0; i < accVector.size(); i++)
    {
        averAcc = averAcc + accVector[i].second;
    }
    averAcc = averAcc / n;
    printf("averge acc %f %f %f\n", averAcc.x(), averAcc.y(), averAcc.z());
    Matrix3d R0 = Utility::g2R(averAcc);
    double yaw = Utility::R2ypr(R0).x();
    R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
    Rs[0] = R0;
    cout << "init R0 " << endl
         << Rs[0] << endl;
    // Vs[0] = Vector3d(5, 0, 0);
}

void Estimator::initFirstPose(Eigen::Vector3d p, Eigen::Matrix3d r)
{
    Ps[0] = p;
    Rs[0] = r;
    initP = p;
    initR = r;
}

/*处理一帧IMU数据，积分
    用前一图像帧位姿，前一图像帧与当前图像帧之间的IMU数据，积分计算得到当前图像帧位姿
    t当前时刻、dt与前一帧时间间隔、linear_acceleration当前时刻加速度
*/
void Estimator::processIMU(double t, double dt, const Vector3d &linear_acceleration, const Vector3d &angular_velocity)
{

    if (!first_imu) // 初始为0
    {
        first_imu = true;
        acc_0 = linear_acceleration;
        gyr_0 = angular_velocity;
    }

    if (!pre_integrations[frame_count])
    {
        // 给当前帧建立一个预计分
        pre_integrations[frame_count] = new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]};
    }
    if (frame_count != 0)
    {
        // 进行预计分
        pre_integrations[frame_count]->push_back(dt, linear_acceleration, angular_velocity);
        // if(solver_flag != NON_LINEAR)
        tmp_pre_integration->push_back(dt, linear_acceleration, angular_velocity);

        dt_buf[frame_count].push_back(dt);
        linear_acceleration_buf[frame_count].push_back(linear_acceleration);
        angular_velocity_buf[frame_count].push_back(angular_velocity);

        int j = frame_count; // imu预计分求取当前帧的Rs、Ps、Vs
        Vector3d un_acc_0 = Rs[j] * (acc_0 - Bas[j]) - g;
        Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - Bgs[j];
        Rs[j] *= Utility::deltaQ(un_gyr * dt).toRotationMatrix();
        Vector3d un_acc_1 = Rs[j] * (linear_acceleration - Bas[j]) - g;
        Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
        Ps[j] += dt * Vs[j] + 0.5 * dt * dt * un_acc;
        Vs[j] += dt * un_acc;
    }
    // 更新下一次的初始值
    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;
}

// void Estimator::getric(const pair<double, map<int, vector<pair<int, Eigen::Matrix<double, 7, 1> > > > > &feature, int &estimate_EXTRINSIC, Matrix3d *r, std::vector<Eigen::Matrix3d> &R)
// {
//     //std::cout<<"******************getric******************************"<<std::endl;

//     // if (f_manager.addFeatureCheckParallax(frame_count, feature.second, td))
//     // {
//     //     marginalization_flag = MARGIN_OLD;
//     //     //printf("keyframe\n");
//     // }
//     // else
//     // {
//     //     marginalization_flag = MARGIN_SECOND_NEW;
//     //     //printf("non-keyframe\n");
//     // }

//     //ROS_DEBUG("%s", marginalization_flag ? "Non-keyframe" : "Keyframe");
//     // ROS_DEBUG("Solving %d", frame_count);
//     // ROS_DEBUG("number of feature: %d", f_manager.getFeatureCount());
//     // const double header =feature.first;
//     // Headers[frame_count] = header;

//     // ImageFrame imageframe(feature.second, header);
//     // imageframe.pre_integration = tmp_pre_integration;
//     // all_image_frame.insert(make_pair(header, imageframe));
//     // tmp_pre_integration = new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]};

//     if(estimate_EXTRINSIC == 2)
//     {
//         ROS_INFO("calibrating extrinsic param, rotation movement is needed");
//         if (frame_count != 0)
//         {
//             vector<pair<Vector3d, Vector3d>> corres = f_manager.getCorresponding(frame_count - 1, frame_count);
//             Matrix3d calib_ric;
//             if (initial_ex_rotation.CalibrationExRotation(corres, pre_integrations[frame_count]->delta_q, calib_ric))
//             {
//                 ROS_WARN("initial extrinsic rotation calib success");
//                 ROS_WARN_STREAM("initial extrinsic rotation: " << endl << calib_ric);
//                 r[0] = calib_ric;
//                 R[0] = calib_ric;
//                 estimate_EXTRINSIC = 1;
//             }
//         }
//     }
// }

// void Estimator::getric1(const pair<double, map<int, vector<pair<int, Eigen::Matrix<double, 7, 1> > > > > &feature, int &estimate_EXTRINSIC, Matrix3d *r, std::vector<Eigen::Matrix3d> &R)
// {
//         //std::cout<<"******************getric1******************************"<<std::endl;

//     if(estimate_EXTRINSIC == 2)
//     {
//         ROS_INFO("calibrating extrinsic param, rotation movement is needed");
//         if (frame_count != 0)
//         {
//             vector<pair<Vector3d, Vector3d>> corres = f_manager.getCorresponding1(frame_count - 1, frame_count);
//             Matrix3d calib_ric;
//             if (initial_ex_rotation.CalibrationExRotation(corres, pre_integrations[frame_count]->delta_q, calib_ric))
//             {
//                 ROS_WARN("initial extrinsic rotation calib success");
//                 ROS_WARN_STREAM("initial extrinsic rotation: " << endl << calib_ric);
//                 r[0] = calib_ric;
//                 R[0] = calib_ric;
//                 estimate_EXTRINSIC = 1;
//             }
//         }
//     }
// }

//----进行不同相机的特征点融合
// void Estimator::featureFusion(pair<double, map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>>> &feature, pair<double, map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>>> &feature1)
// {

//     std::cout << "******************featureFusion******************************" << std::endl;

//     // ROS_DEBUG("Adding 0camera feature points %lu", feature.second.size());
//     // ROS_DEBUG("Adding 1camera feature points %lu", feature1.second.size());
//     // std::cout<<"222222222222222222222222222222222feature.size = "<<feature.second.size()<<std::endl;
//     // std::cout<<"2222222222222222222222222222222222feature1.size = "<<feature1.second.size()<<std::endl;

//     // rcc[0]=ric[0]*ric1[0];
//     // map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> temp_image;

//     map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> temp_featureFrame;
//     // vector<pair<int, Eigen::Matrix<double, 7, 1>>> temp;

//     double x, y, z;
//     double p_u, p_v;
//     double velocity_x, velocity_y;
//     Eigen::Matrix<double, 7, 1> xyz_uv_velocity1;
//     Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
//     // Eigen::Matrix<double, 3, 1> temp_c1;//相机1中的特征的归一化坐标
//     // Eigen::Matrix<double, 3, 1> temp_c;//相机1转到0中的特征的归一化坐标

//     Eigen::Matrix<double, 3, 1> temp_c1; // 相机1中的特征的像素坐标
//     Eigen::Matrix<double, 3, 1> temp_c;  // 相机1转到0中的特征的像素坐标
//     vector<cv::Point2f> un_pts;

//     Matrix3d rcc;
//     rcc << 1, 0, 0,
//         0, 0, 1,
//         0, -1, 0;
//     Vector3d tcc;
//     tcc << 0, 0.01, 0;

//     // 遍历每一个特征点it
//     for (map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>>::iterator it = feature1.second.begin(); it != feature1.second.end(); it++)
//     {
//         int feature_id = it->first;
//         for (auto i : it->second)
//         {
//             xyz_uv_velocity1 << i.second;
//             temp_c1[0] = xyz_uv_velocity1[3];
//             temp_c1[1] = xyz_uv_velocity1[4];
//             temp_c1[2] = 1;
//             // temp_c1[2]=xyz_uv_velocity1[2];
//             //  std::cout<<"*************************************************************** "<<std::endl;
//             //  std::cout<<"ric1[0]= "<<ric1[0]<<std::endl;
//             //  std::cout<<"ric[0]= "<<ric[0]<<std::endl;
//             //  std::cout<<"tic[0]= "<<tic1[0]<<std::endl;
//             //  std::cout<<"tic1[0]= "<<tic1[0]<<std::endl;

//             // temp_c << ric[0].transpose()*(ric1[0]*temp_c1 + tic1[0])-tic[0];
//             temp_c << rcc * temp_c1 + tcc;
//             // x = temp_c[0];
//             // y = temp_c[1];
//             // z = temp_c[2];
//             p_u = temp_c[0];
//             p_v = temp_c[1];

//             Eigen::Vector2d a(temp_c[0], temp_c[1]);
//             Eigen::Vector3d b;
//             featureTracker.m_camera[0]->liftProjective(a, b);
//             // un_pts.push_back(cv::Point2f(b.x() / b.z(), b.y() / b.z()));
//             //  p_u = xyz_uv_velocity1[3];
//             //  p_v = xyz_uv_velocity1[4];
//             x = b.x() / b.z();
//             y = b.y() / b.z();
//             z = 1;

//             velocity_x = xyz_uv_velocity1[5];
//             velocity_y = xyz_uv_velocity1[6];

//             int camera_id = i.first;

//             Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
//             xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
//             temp_featureFrame[feature_id].emplace_back(camera_id, xyz_uv_velocity);
//         }
//         //     for(auto i : it->second)
//         // {
//         //     xyz_uv_velocity1 << i.second;
//         //     temp_c1[0]=xyz_uv_velocity1[3];
//         //     temp_c1[1]=xyz_uv_velocity1[4];
//         //     temp_c1[2]=1;
//         //     temp_c << ric[0].transpose()*(ric1[0]*temp_c1 + tic1[0])-tic[0];

//         //     p_u = temp_c[0];
//         //     p_v = temp_c[1];

//         //     Eigen::Vector2d a(temp_c[0], temp_c[1]);
//         //     Eigen::Vector3d b;
//         //     m_camera->liftProjective(a, b);
//         //     //un_pts.push_back(cv::Point2f(b.x() / b.z(), b.y() / b.z()));

//         //     x = un_pts[it->first].x;
//         //     y = un_pts[it->first].y;
//         //     z = 1;

//         //     velocity_x = xyz_uv_velocity1[5];
//         //     velocity_y = xyz_uv_velocity1[6];

//         //     int camera_id = 0;

//         //     Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
//         //     xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
//         //     temp_featureFrame[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
//         // }
//     }
//     // std::cout<<"88888888888888888888888888888888888888888888888888888888temp_featureFrame.size = "<<temp_featureFrame.size()<<std::endl;
//     // queue<pair<double, map<int, vector<pair<int, Eigen::Matrix<double, 7, 1> > > > > > tempfeatureBuf;
//     // mBuf.lock();
//     // tempfeatureBuf.push(make_pair(feature1.first,temp_featureFrame));
//     // std::cout<<"tempfeatureBuf.size = "<<tempfeatureBuf.size()<<std::endl;
//     // mBuf.unlock();

//     for (map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>>::iterator it = feature.second.begin(); it != feature.second.end(); it++)
//     {
//         int feature_id = it->first;
//         // Eigen::Matrix<double, 3, 1> temp_c1;//相机1中的特征的归一化坐标
//         // Eigen::Matrix<double, 3, 1> temp_c;//相机1转到0中的特征的归一化坐标
//         for (auto i : it->second)
//         {
//             xyz_uv_velocity << i.second;
//             // temp_c1[0]=xyz_uv_velocity1[0];
//             // temp_c1[1]=xyz_uv_velocity1[1];
//             // temp_c1[2]=xyz_uv_velocity1[2];
//             // temp_c << ric[0].transpose()*(ric1[0]*temp_c1 + tic1[0])+tic->transpose();

//             x = xyz_uv_velocity[0];
//             y = xyz_uv_velocity[1];
//             z = xyz_uv_velocity[2];

//             p_u = xyz_uv_velocity[3];
//             p_v = xyz_uv_velocity[4];

//             velocity_x = xyz_uv_velocity[5];
//             velocity_y = xyz_uv_velocity[6];

//             int camera_id = i.first;

//             Eigen::Matrix<double, 7, 1> xyz_uv_velocity1;
//             xyz_uv_velocity1 << x, y, z, p_u, p_v, velocity_x, velocity_y;
//             temp_featureFrame[feature_id].emplace_back(camera_id, xyz_uv_velocity1);
//         }

//         // for(auto i : it->second)
//         // {
//         //     xyz_uv_velocity << i.second;
//         //     //temp_c1[0]=xyz_uv_velocity1[0];
//         //     //temp_c1[1]=xyz_uv_velocity1[1];
//         //     //temp_c1[2]=xyz_uv_velocity1[2];
//         //     //temp_c << ric[0].transpose()*(ric1[0]*temp_c1 + tic1[0])+tic->transpose();

//         //     x = xyz_uv_velocity[0];
//         //     y = xyz_uv_velocity[1];
//         //     z = xyz_uv_velocity[2];

//         //     p_u = xyz_uv_velocity[3];
//         //     p_v = xyz_uv_velocity[4];

//         //     velocity_x = xyz_uv_velocity[5];
//         //     velocity_y = xyz_uv_velocity[6];

//         //     int camera_id = 1;

//         //     Eigen::Matrix<double, 7, 1> xyz_uv_velocity1;
//         //     xyz_uv_velocity1 << x, y, z, p_u, p_v, velocity_x, velocity_y;
//         //     temp_featureFrame[feature_id].emplace_back(camera_id,  xyz_uv_velocity1);
//         // }
//     }
//     // std::cout<<"999999999999999999999999999999999999999after_temp_featureFrame.size = "<<temp_featureFrame.size()<<std::endl;
//     mBuf[0].lock();
//     // tempfeatureBuf.push(make_pair(feature.first,temp_featureFrame));
//     // std::cout<<"After_fusion_tempfeatureBuf.size = "<<tempfeatureBuf.size()<<std::endl;
//     feature = make_pair(feature.first, temp_featureFrame);
//     // std::cout<<"3333333333333333333333333333333333333333333333333fusion_feature.size = "<<feature.second.size()<<std::endl;
//     mBuf[0].unlock();
//     // feature = tempfeatureBuf.front();

//     // mBuf.lock();
//     // tempfeatureBuf.pop();
//     // mBuf.unlock();
//     //     for(map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>>::iterator it = image1.begin();it != image1.end();it++)
//     //     {
//     //         //image[(image.size())]=rcc[0]*it->second.;
//     //         vector<pair<int, Eigen::Matrix<double, 7, 1>>> temp;
//     //         for(auto &i : it->second)
//     //         {
//     //             temp.push_back(pair<int, Eigen::Matrix<double, 7, 1>>(i.first,rcc[0]*i.second));
//     //         }
//     //         image.insert(pair<int, vector>(image.size(), temp));
//     //     }

//     // //------------------------------------------------------------------------------------**************************************************************
//     //     for(map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>>::iterator it = image1.begin();it != image1.end();it++)
//     //     {
//     //         //image[(image.size())]=rcc[0]*it->second.;
//     //         vector<pair<int, Eigen::Matrix<double, 7, 1>>> temp;
//     //         for(auto &i : it->second)
//     //         {
//     //             temp.push_back(pair<int, Eigen::Matrix<double, 7, 1>>(i.first,rcc[0]*i.second));
//     //         }
//     //         image.insert(pair<int, vector>(image.size(), temp));
//     //     }
//     //------------------------------------------------------------------------------------------------------******************************************//
// }

void Estimator::processImage(const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image1, const double header)
{
    // std::cout << "******************processImage******************************" << std::endl;
    // std::cout<<"55555555555555555555555555555555555fusion_feature.size = "<<image.size()<<std::endl;
    map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> _image[2];
    _image[0] = image;  // 相机0的特征
    _image[1] = image1; // 相机1的特征
    ROS_DEBUG("new image coming ------------------------------------------");
    ROS_DEBUG("Adding feature points %lu", image.size() + image1.size());
    // std::cout<<"0000000000000000000000000000fusion_feature0.size = "<<_image[0].size()<<std::endl;
    // std::cout<<"11111111111111111111111111111fusion_feature1.size = "<<_image[1].size()<<std::endl;
    // 1、检测关键帧
    if (f_manager.addFeatureCheckParallax(frame_count, _image, td))
    {
        marginalization_flag = MARGIN_OLD;
        // printf("keyframe\n");
    }
    else
    {
        marginalization_flag = MARGIN_SECOND_NEW;
        // printf("non-keyframe\n");
    }

    ROS_DEBUG("%s", marginalization_flag ? "Non-keyframe" : "Keyframe");
    ROS_DEBUG("Solving %d", frame_count);
    // ROS_DEBUG("number of feature: %d", f_manager.getFeatureCount());
    Headers[frame_count] = header;
    // 2、估计相机和IMU的外参
    // 将图像信息、时间以及预积分值存放在图像帧类中
    ImageFrame imageframe(image, image1, header);
    // 当前预计分（前一帧与当前帧之间的imu预计分）
    imageframe.pre_integration = tmp_pre_integration;
    // ImageFrame imageframe1(image1, header);
    // imageframe1.pre_integration = tmp_pre_integration;
    all_image_frame.insert(make_pair(header, imageframe));
    // all_image_frame1.insert(make_pair(header, imageframe1));
    // all_image_frame.insert(make_pair(header, imageframe1)); //???????map<> 不能有相同的key值 ，  需要使用multimap，双目除了陀螺仪零偏，用不到all_image_frame
    // 重置预计分器
    tmp_pre_integration = new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]};

    if(ESTIMATE_EXTRINSIC == 2)
    {
        ROS_INFO("calibrating extrinsic param, rotation movement is needed");
        if (frame_count != 0)
        {
            vector<pair<Vector3d, Vector3d>> corres = f_manager.getCorresponding(frame_count - 1, frame_count);
            Matrix3d calib_ric;
            if (initial_ex_rotation.CalibrationExRotation(corres, pre_integrations[frame_count]->delta_q, calib_ric))
            {
                ROS_WARN("initial extrinsic rotation calib success");
                ROS_WARN_STREAM("initial extrinsic rotation: " << endl << calib_ric);
                ric[0] = calib_ric;
                RIC[0] = calib_ric;
                ESTIMATE_EXTRINSIC = 1;
            }
        }
    }

    if(ESTIMATE_EXTRINSIC1 == 2)
    {
        ROS_INFO("calibrating extrinsic1 param, rotation movement is needed");
        if (frame_count != 0)
        {
            vector<pair<Vector3d, Vector3d>> corres1 = f_manager.getCorresponding1(frame_count - 1, frame_count);
            Matrix3d calib_ric1;
            if (initial_ex_rotation.CalibrationExRotation(corres1, pre_integrations[frame_count]->delta_q, calib_ric1))
            {
                ROS_WARN("initial extrinsic rotation calib success");
                ROS_WARN_STREAM("initial extrinsic rotation: " << endl << calib_ric1);
                ric1[0] = calib_ric1;
                RIC1[0] = calib_ric1;
                ESTIMATE_EXTRINSIC1 = 1;
            }
        }
    }


    if (solver_flag == INITIAL)
    {
        //----------------------------------------------------------monocular + IMU initilization
        if (!STEREO && USE_IMU)
        {
            if (frame_count == WINDOW_SIZE)
            {
                bool result = false;
                if (ESTIMATE_EXTRINSIC != 2 && (header - initial_timestamp) > 0.1 && ESTIMATE_EXTRINSIC1 != 2)
                {                                // 确保有足够的frame参与初始化，有外参，且当前帧时间戳大于初始化时间戳+0.1s
                    result = initialStructure(); // 执行视觉惯性联合初始化
                    initial_timestamp = header;  // 更新初始化时间戳
                }
                if (result) // 初始化成功则进行一次非线性优化
                {
                    optimization();
                    updateLatestStates();
                    solver_flag = NON_LINEAR; // 进行非线性优化
                    slideWindow();
                    ROS_INFO("Initialization finish!");
                }
                else
                    slideWindow();
            }
        }

        // stereo + IMU initilization
        if (STEREO && USE_IMU)
        {
            f_manager.initFramePoseByPnP(frame_count, Ps, Rs, tic, ric, tic1, ric1); // 利用pnp计算位姿，第一次之后得到深度，通过PNP计算得到当前帧的位姿
            f_manager.triangulate(frame_count, Ps, Rs, tic, ric, tic1, ric1);        // 三角化三维坐标，得到当前帧每一个路标点的3D点逆深度
            // f_manager.featureselect(frame_count, Ps, Rs, tic, ric, tic1, ric1);
            // f_manager.featureselect();
            // f_manager.featurefusion(frame_count, ric, ric1, tic, tic1);
            // f_manager.initFramePoseByPnP(frame_count, Ps, Rs, tic, ric);//利用pnp计算位姿，第一次之后得到深度，通过PNP计算得到当前帧的位姿
            if (frame_count == WINDOW_SIZE) // 当前的帧是否满足初始化
            {
                map<double, ImageFrame>::iterator frame_it;
                int i = 0;
                //给每一帧图像位姿赋值，根据帧间PNP和三角化求解得到
                // if(!f_manager.Opt_Select)
                // {
                for (frame_it = all_image_frame.begin(); frame_it != all_image_frame.end(); frame_it++)
                {
                    frame_it->second.R = Rs[i];
                    frame_it->second.T = Ps[i];
                    i++;
                }
                solveGyroscopeBias(all_image_frame, Bgs); // 对IMU陀螺零篇进行估计
                // }
                // else
                // {
                // for (frame_it = all_image_frame1.begin(); frame_it != all_image_frame1.end(); frame_it++)
                // {
                //     frame_it->second.R = Rs[i];
                //     frame_it->second.T = Ps[i];
                //     i++;
                // }
                // solveGyroscopeBias(all_image_frame1, Bgs); // 对IMU陀螺零篇进行估计  
                // }
                for (int i = 0; i <= WINDOW_SIZE; i++)
                {
                    pre_integrations[i]->repropagate(Vector3d::Zero(), Bgs[i]);
                }
                optimization(); // 非线性最小二乘优化重投影误差
                updateLatestStates();
                solver_flag = NON_LINEAR;
                slideWindow();
                ROS_INFO("Initialization finish!");
            }
        }

        // ----------------------------------------------------------------------stereo only initilization
        // if(STEREO && !USE_IMU)
        // {
        //     f_manager.initFramePoseByPnP(frame_count, Ps, Rs, tic, ric);
        //     f_manager.triangulate(frame_count, Ps, Rs, tic, ric);
        //     optimization();

        //     if(frame_count == WINDOW_SIZE)
        //     {
        //         optimization();
        //         updateLatestStates();
        //         solver_flag = NON_LINEAR;
        //         slideWindow();
        //         ROS_INFO("Initialization finish!");
        //     }
        // }

        if (frame_count < WINDOW_SIZE) // 如果当前帧数不满足初始化，frame_count++
        {
            frame_count++;
            int prev_frame = frame_count - 1;
            Ps[frame_count] = Ps[prev_frame];
            Vs[frame_count] = Vs[prev_frame];
            Rs[frame_count] = Rs[prev_frame];
            Bas[frame_count] = Bas[prev_frame];
            Bgs[frame_count] = Bgs[prev_frame];
        }
    }
    else
    {
        TicToc t_solve;
        if (!USE_IMU)
            f_manager.initFramePoseByPnP(frame_count, Ps, Rs, tic, ric, tic1, ric1); // 利用pnp计算位姿，第一次之后得到深度，通过PNP计算得到当前帧的位姿

        f_manager.triangulate(frame_count, Ps, Rs, tic, ric, tic1, ric1); // 三角化三维坐标，得到当前帧每一个路标点的3D点逆深度
        // f_manager.featurefusion(frame_count, ric, ric1, tic, tic1);
        // f_manager.featureselect(frame_count, Ps, Rs, tic, ric, tic1, ric1);
        f_manager.initFramePoseByPnP(frame_count, Ps, Rs, tic, ric, tic1, ric1); // 利用pnp计算位姿，第一次之后得到深度，通过PNP计算得到当前帧的位姿
        // f_manager.feature[1].clear();
        optimization();
        set<int> removeIndex;
        set<int> removeIndex1;
        outliersRejection(removeIndex, removeIndex1);
        f_manager.removeOutlier(removeIndex, removeIndex1);
        if (!MULTIPLE_THREAD)
        {
            featureTracker.removeOutliers(removeIndex);
            predictPtsInNextFrame();
        }

        ROS_DEBUG("solver costs: %fms", t_solve.toc());

        if (failureDetection())
        {
            ROS_WARN("failure detection!");
            failure_occur = 1;
            clearState();
            setParameter();
            ROS_WARN("system reboot!");
            return;
        }

        slideWindow();
        f_manager.removeFailures();
        // prepare output of VINS
        key_poses.clear();
        for (int i = 0; i <= WINDOW_SIZE; i++)
            key_poses.push_back(Ps[i]);

        last_R = Rs[WINDOW_SIZE];
        last_P = Ps[WINDOW_SIZE];
        last_R0 = Rs[0];
        last_P0 = Ps[0];
        updateLatestStates();
    }
}

bool Estimator::initialStructure()
{
    TicToc t_sfm;
    // check imu observibility
    // 一、确保IMU有足够的excitation
    // 通过计算滑窗内所有帧的线加速度的标准差，判断IMU是否有充分运动激励，以判断是否进行初始化。
    {
        map<double, ImageFrame>::iterator frame_it;
        Vector3d sum_g;
        // all_image_frame 包含滑窗内所有帧的视觉和IMU信息

        // 第一次循环，求出滑窗内的平均线加速度
        for (frame_it = all_image_frame.begin(), frame_it++; frame_it != all_image_frame.end(); frame_it++)
        {
            double dt = frame_it->second.pre_integration->sum_dt;
            Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt;
            sum_g += tmp_g;
        }
        Vector3d aver_g;
        aver_g = sum_g * 1.0 / ((int)all_image_frame.size() - 1);
        double var = 0;
        // 第二次循环，求出滑窗内的线加速度的标准差
        for (frame_it = all_image_frame.begin(), frame_it++; frame_it != all_image_frame.end(); frame_it++)
        {
            double dt = frame_it->second.pre_integration->sum_dt;
            Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt;
            var += (tmp_g - aver_g).transpose() * (tmp_g - aver_g); // 计算加速度方差
            // cout << "frame g " << tmp_g.transpose() << endl;
        }
        var = sqrt(var / ((int)all_image_frame.size() - 1)); // 加速度标准差
        // ROS_WARN("IMU variation %f!", var);
        if (var < 0.25)
        {
            ROS_INFO("IMU excitation not enouth!");
            // return false;
        }
    }
    // global sfm
    // 二、将f_manager中的所有feature在所有帧的归一化坐标保存到Vector sfm_f中
    Quaterniond Q[frame_count + 1];
    Vector3d T[frame_count + 1];
    map<int, Vector3d> sfm_tracked_points;
    vector<SFMFeature> sfm_f; // 一个装着特征点信息的结构体
    // 向容器中放数据
    // for (int c = 0; c < NUM_OF_CAM; c++)
    // {
        // 遍历主相机的所有特征点
        for (auto &it_per_id : f_manager.feature[0])
        {
            int imu_j = it_per_id.start_frame - 1; // 从start_frame开始帧编号
            SFMFeature tmp_feature;
            tmp_feature.state = false;                             // 状态，未三角化
            tmp_feature.id = it_per_id.feature_id;                 // 特征点id
            for (auto &it_per_frame : it_per_id.feature_per_frame) // 对于当前特征点  在每一帧的坐标
            {
                imu_j++;                             // 帧标号+1
                Vector3d pts_j = it_per_frame.point; // 当前特征在编号为imu_j++的帧的皈依化坐标
                tmp_feature.observation.push_back(make_pair(imu_j, Eigen::Vector2d{pts_j.x(), pts_j.y()}));
                // 把当前特征在当前帧的坐标和当前帧的编号配对
            }
            sfm_f.push_back(tmp_feature); // sfm_f中存放着不同的特征
        }
    // }
    // 三、在滑窗（0-9）中找到第一个满足要求的帧（第l帧），它与最新一帧（frame_count=10）有足够的共试点的平行度，并求出这两帧之间的相对位置变化关系。
    // 3.1、定义容器
    Matrix3d relative_R;
    Vector3d relative_T;
    int l; // 滑窗中满足与最新帧视差关系的那一帧的帧编号

    // 3.2、两帧之间的视察判断，并得到两帧之间的相对位姿变化关系
    if (!relativePose(relative_R, relative_T, l))
    { // 这里的第l帧是从第一帧开始到滑窗中第一个满足与当前帧的平均视察足够大的帧l，会作为参考帧到下面的全局sfm使用，得到的RT为当前帧到第l帧的坐标变换
        ROS_INFO("Not enough features or parallax; Move device around");
        return false;
    }

    // 四、对窗口内每个图像帧求解sfm问题
    // 得到所有图像帧相对于参考帧的旋转四元数Q、平移向量T和特征点坐标
    GlobalSFM sfm;
    // 传入frame_count、l、relative_R、relative_T、sfm_f
    // 通过construct得到了各帧在帧l系下的位姿，以及特征点在l系下的3D坐标
    if (!sfm.construct(frame_count + 1, Q, T, l,
                       relative_R, relative_T,
                       sfm_f, sfm_tracked_points))
    {
        ROS_DEBUG("global SFM failed!");
        marginalization_flag = MARGIN_OLD;
        return false;
    }

    // solve pnp for all frame
    // 五、给滑窗外的图像帧提供初始的RT估计，然后solvepnp进行优化
    map<double, ImageFrame>::iterator frame_it;
    map<int, Vector3d>::iterator it;
    frame_it = all_image_frame.begin();
    // 遍历所有的图像帧
    for (int i = 0; frame_it != all_image_frame.end(); frame_it++)
    {
        // provide initial guess
        cv::Mat r, rvec, t, D, tmp_r;
        // 1、边界判断：对于滑窗内的帧，把他们设为关键帧，并获得他们对应的IMU坐标系到I系的旋转平移
        if ((frame_it->first) == Headers[i])
        {
            frame_it->second.is_key_frame = true;
            frame_it->second.R = Q[i].toRotationMatrix() * RIC[0].transpose();
            frame_it->second.T = T[i];
            i++;
            continue;
        }
        // 2、边界判断：如果当前帧的时间戳大于滑窗内第i帧的时间戳，那么i++
        if ((frame_it->first) > Headers[i])
        {
            i++;
        }
        // 3、对滑窗外的所有帧，求出他们对应的IMU坐标系到i帧的旋转平移
        Matrix3d R_inital = (Q[i].inverse()).toRotationMatrix();
        Vector3d P_inital = -R_inital * T[i];
        cv::eigen2cv(R_inital, tmp_r);
        cv::Rodrigues(tmp_r, rvec);
        cv::eigen2cv(P_inital, t);

        frame_it->second.is_key_frame = false;
        vector<cv::Point3f> pts_3_vector;
        vector<cv::Point2f> pts_2_vector;
        for (auto &id_pts : frame_it->second.points)
        {
            int feature_id = id_pts.first;
            for (auto &i_p : id_pts.second)
            {
                it = sfm_tracked_points.find(feature_id);
                if (it != sfm_tracked_points.end())
                {
                    Vector3d world_pts = it->second;
                    cv::Point3f pts_3(world_pts(0), world_pts(1), world_pts(2));
                    pts_3_vector.push_back(pts_3);
                    Vector2d img_pts = i_p.second.head<2>();
                    cv::Point2f pts_2(img_pts(0), img_pts(1));
                    pts_2_vector.push_back(pts_2);
                }
            }
        }
        cv::Mat K = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
        if (pts_3_vector.size() < 6)
        {
            cout << "pts_3_vector size " << pts_3_vector.size() << endl;
            ROS_DEBUG("Not enough points for solve pnp !");
            return false;
        }
        if (!cv::solvePnP(pts_3_vector, pts_2_vector, K, D, rvec, t, 1))
        {
            ROS_DEBUG("solve pnp fail!");
            return false;
        }
        cv::Rodrigues(rvec, r);
        MatrixXd R_pnp, tmp_R_pnp;
        cv::cv2eigen(r, tmp_R_pnp);
        R_pnp = tmp_R_pnp.transpose();
        MatrixXd T_pnp;
        cv::cv2eigen(t, T_pnp);
        T_pnp = R_pnp * (-T_pnp);
        frame_it->second.R = R_pnp * RIC[0].transpose(); // 传入的为bk到l系的旋转平移
        frame_it->second.T = T_pnp;
    }
    // 六、核心VisualInitialAlign
    if (visualInitialAlign())
        return true;
    else
    {
        ROS_INFO("misalign visual structure with IMU");
        return false;
    }
}

bool Estimator::visualInitialAlign()
{
    // 1、计算陀螺仪偏执，尺度，重力加速度和速度
    TicToc t_g;
    VectorXd x;
    // solve scale
    bool result = VisualIMUAlignment(all_image_frame, Bgs, g, x);
    if (!result)
    {
        ROS_DEBUG("solve g failed!");
        return false;
    }

    // change state
    // 传递所有图像帧的位姿Ps、Rs，并将其置为关键帧
    for (int i = 0; i <= frame_count; i++)
    {
        Matrix3d Ri = all_image_frame[Headers[i]].R;
        Vector3d Pi = all_image_frame[Headers[i]].T;
        Ps[i] = Pi;
        Rs[i] = Ri;
        all_image_frame[Headers[i]].is_key_frame = true;
    }

    // 将Ps、Vs、depth尺度s缩放后从l帧转变为相对于c0帧图像坐标系下
    double s = (x.tail<1>())(0);
    // IMU的bias改变，重新计算滑窗内的预计分
    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        pre_integrations[i]->repropagate(Vector3d::Zero(), Bgs[i]);
    }
    for (int i = frame_count; i >= 0; i--)
        Ps[i] = s * Ps[i] - Rs[i] * TIC[0] - (s * Ps[0] - Rs[0] * TIC[0]);
    int kv = -1;
    map<double, ImageFrame>::iterator frame_i;
    for (frame_i = all_image_frame.begin(); frame_i != all_image_frame.end(); frame_i++)
    {
        if (frame_i->second.is_key_frame)
        {
            kv++;
            Vs[kv] = frame_i->second.R * x.segment<3>(kv * 3);
        }
    }

    Matrix3d R0 = Utility::g2R(g);
    double yaw = Utility::R2ypr(R0 * Rs[0]).x();
    R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
    g = R0 * g;
    // Matrix3d rot_diff = R0 * Rs[0].transpose();
    // 通过将重力旋转到z轴上，得到世界坐标系和相机坐标系C0之间的旋转矩阵rot_diff
    Matrix3d rot_diff = R0;
    // 将所有坐标系从参考坐标系C0转到i世界坐标系w
    for (int i = 0; i <= frame_count; i++)
    {
        Ps[i] = rot_diff * Ps[i];
        Rs[i] = rot_diff * Rs[i];
        Vs[i] = rot_diff * Vs[i];
    }
    ROS_DEBUG_STREAM("g0     " << g.transpose());
    ROS_DEBUG_STREAM("my R0  " << Utility::R2ypr(Rs[0]).transpose());

    f_manager.clearDepth();
    f_manager.triangulate(frame_count, Ps, Rs, tic, ric, tic1, ric1);

    return true;
}

bool Estimator::relativePose(Matrix3d &relative_R, Vector3d &relative_T, int &l)
{
    // find previous frame which contians enough correspondance and parallex with newest frame
    // a、计算滑窗内的每一帧与最新帧之间的视差，直到找到第一个满足要求的帧，作为我们的第l帧
    for (int i = 0; i < WINDOW_SIZE; i++)
    {
        // 滑窗内的所有帧和最新一帧进行视差比较
        vector<pair<Vector3d, Vector3d>> corres;
        corres = f_manager.getCorresponding(i, WINDOW_SIZE); // 寻找第i帧到窗口最后一帧的对应特征点的归一化坐标
        if (corres.size() > 20)
        {
            double sum_parallax = 0;
            double average_parallax; // 计算平均视差
            for (int j = 0; j < int(corres.size()); j++)
            { // 第j个对应点在第i帧和最后一帧的（x，y）
                Vector2d pts_0(corres[j].first(0), corres[j].first(1));
                Vector2d pts_1(corres[j].second(0), corres[j].second(1));
                double parallax = (pts_0 - pts_1).norm();
                sum_parallax = sum_parallax + parallax;

            } // 计算平均视差
            average_parallax = 1.0 * sum_parallax / int(corres.size());
            // 判断是否满足初始化条件：视差》30
            // b、计算i帧与最新一帧的相对位姿关系
            if (average_parallax * 460 > 30 && m_estimator.solveRelativeRT(corres, relative_R, relative_T))
            {          // solveRelativeRT通过基础矩阵计算当前帧与第l帧之间的R和T，并判断内点数目是否足够     这个relative_R, relative_T是把最新一帧旋转到第l帧的旋转平移
                l = i; // 同时返回窗口最后一帧和第l帧的R，T
                ROS_DEBUG("average_parallax %f choose l %d and newest frame to triangulate the whole structure", average_parallax * 460, l);
                return true; // 一旦这一帧与当前帧视差足够大，那就不再继续找
            }
        }
    }
    return false; // 如钩没有满足关系的帧，整个初始化initialstructure失败
}

//ceres使用double数据类型，将ector转化为double
void Estimator::vector2double()
{
    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        para_Pose[i][0] = Ps[i].x();
        para_Pose[i][1] = Ps[i].y();
        para_Pose[i][2] = Ps[i].z();
        Quaterniond q{Rs[i]};
        para_Pose[i][3] = q.x();
        para_Pose[i][4] = q.y();
        para_Pose[i][5] = q.z();
        para_Pose[i][6] = q.w();

        if (USE_IMU)
        {
            para_SpeedBias[i][0] = Vs[i].x();
            para_SpeedBias[i][1] = Vs[i].y();
            para_SpeedBias[i][2] = Vs[i].z();

            para_SpeedBias[i][3] = Bas[i].x();
            para_SpeedBias[i][4] = Bas[i].y();
            para_SpeedBias[i][5] = Bas[i].z();

            para_SpeedBias[i][6] = Bgs[i].x();
            para_SpeedBias[i][7] = Bgs[i].y();
            para_SpeedBias[i][8] = Bgs[i].z();
        }
    }

    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        para_Ex_Pose[i][0] = tic[i].x();
        para_Ex_Pose[i][1] = tic[i].y();
        para_Ex_Pose[i][2] = tic[i].z();
        Quaterniond q{ric[i]};
        para_Ex_Pose[i][3] = q.x();
        para_Ex_Pose[i][4] = q.y();
        para_Ex_Pose[i][5] = q.z();
        para_Ex_Pose[i][6] = q.w();
    }

    for (int c = 0; c < NUM_OF_CAM; c++)
    {
        para_Ex_Pose1[c][0] = tic1[c].x();
        para_Ex_Pose1[c][1] = tic1[c].y();
        para_Ex_Pose1[c][2] = tic1[c].z();
        Quaterniond q{ric1[c]};
        para_Ex_Pose1[c][3] = q.x();
        para_Ex_Pose1[c][4] = q.y();
        para_Ex_Pose1[c][5] = q.z();
        para_Ex_Pose1[c][6] = q.w();
    }
    // if(!f_manager.Opt_Select)
    // {
    //特征点0的逆深度
    VectorXd dep = f_manager.getDepthVector(0);
    for (int i = 0; i < f_manager.getFeatureCount(0); i++)
        para_Feature[i][0] = dep(i);
    // }
    // else
    // {
    //特征点1的逆深度
    VectorXd dep1 = f_manager.getDepthVector(1);
    for (int i = 0; i < f_manager.getFeatureCount(1); i++)
        para_Feature1[i][0] = dep1(i);
    // }
    para_Td[0][0] = td;
}

//将ceres优化以后的double数据类型重新复制给优化变量（Vector类型）
void Estimator::double2vector()
{
    //固定了先验信息，第0帧的位姿
    Vector3d origin_R0 = Utility::R2ypr(Rs[0]);
    Vector3d origin_P0 = Ps[0];

    if (failure_occur)
    {
        origin_R0 = Utility::R2ypr(last_R0);
        origin_P0 = last_P0;
        failure_occur = 0;
    }

    if (USE_IMU)
    {
        Vector3d origin_R00 = Utility::R2ypr(Quaterniond(para_Pose[0][6],
                                                         para_Pose[0][3],
                                                         para_Pose[0][4],
                                                         para_Pose[0][5])
                                                 .toRotationMatrix());
        double y_diff = origin_R0.x() - origin_R00.x();
        // TODO
        Matrix3d rot_diff = Utility::ypr2R(Vector3d(y_diff, 0, 0));
        if (abs(abs(origin_R0.y()) - 90) < 1.0 || abs(abs(origin_R00.y()) - 90) < 1.0)
        {
            ROS_DEBUG("euler singular point!");
            rot_diff = Rs[0] * Quaterniond(para_Pose[0][6],
                                           para_Pose[0][3],
                                           para_Pose[0][4],
                                           para_Pose[0][5])
                                   .toRotationMatrix()
                                   .transpose();
        }

        for (int i = 0; i <= WINDOW_SIZE; i++)
        {

            Rs[i] = rot_diff * Quaterniond(para_Pose[i][6], para_Pose[i][3], para_Pose[i][4], para_Pose[i][5]).normalized().toRotationMatrix();

            Ps[i] = rot_diff * Vector3d(para_Pose[i][0] - para_Pose[0][0],
                                        para_Pose[i][1] - para_Pose[0][1],
                                        para_Pose[i][2] - para_Pose[0][2]) +
                    origin_P0;

            Vs[i] = rot_diff * Vector3d(para_SpeedBias[i][0],
                                        para_SpeedBias[i][1],
                                        para_SpeedBias[i][2]);

            Bas[i] = Vector3d(para_SpeedBias[i][3],
                              para_SpeedBias[i][4],
                              para_SpeedBias[i][5]);

            Bgs[i] = Vector3d(para_SpeedBias[i][6],
                              para_SpeedBias[i][7],
                              para_SpeedBias[i][8]);
        }
    }
    else
    {
        for (int i = 0; i <= WINDOW_SIZE; i++)
        {
            Rs[i] = Quaterniond(para_Pose[i][6], para_Pose[i][3], para_Pose[i][4], para_Pose[i][5]).normalized().toRotationMatrix();

            Ps[i] = Vector3d(para_Pose[i][0], para_Pose[i][1], para_Pose[i][2]);
        }
    }

    if (USE_IMU)
    {

        for (int i = 0; i < NUM_OF_CAM; i++)
        {
            tic[i] = Vector3d(para_Ex_Pose[i][0],
                              para_Ex_Pose[i][1],
                              para_Ex_Pose[i][2]);
            ric[i] = Quaterniond(para_Ex_Pose[i][6],
                                 para_Ex_Pose[i][3],
                                 para_Ex_Pose[i][4],
                                 para_Ex_Pose[i][5])
                         .normalized()
                         .toRotationMatrix();
        }

        for (int c = 0; c < NUM_OF_CAM; c++)
        {
            tic1[c] = Vector3d(para_Ex_Pose1[c][0],
                              para_Ex_Pose1[c][1],
                              para_Ex_Pose1[c][2]);
            ric1[c] = Quaterniond(para_Ex_Pose1[c][6],
                                 para_Ex_Pose1[c][3],
                                 para_Ex_Pose1[c][4],
                                 para_Ex_Pose1[c][5])
                         .normalized()
                         .toRotationMatrix();
        }
    }

    // if(!f_manager.Opt_Select)
    // {
    VectorXd dep = f_manager.getDepthVector(0);
    for (int i = 0; i < f_manager.getFeatureCount(0); i++)
        dep(i) = para_Feature[i][0];
    f_manager.setDepth(dep, 0);
    // }
    // else
    // {
    VectorXd dep1 = f_manager.getDepthVector(1);
    for (int i = 0; i < f_manager.getFeatureCount(1); i++)
        dep1(i) = para_Feature1[i][0];
    f_manager.setDepth(dep1, 1);
    // }
    if (USE_IMU)
        td = para_Td[0][0];
}

bool Estimator::failureDetection()
{
    return false;
    // if (f_manager.last_track_num < 2)
    // {
    //     ROS_INFO(" little feature %d", f_manager.last_track_num);
    //     //return true;
    // }
    if (Bas[WINDOW_SIZE].norm() > 2.5)
    {
        ROS_INFO(" big IMU acc bias estimation %f", Bas[WINDOW_SIZE].norm());
        return true;
    }
    if (Bgs[WINDOW_SIZE].norm() > 1.0)
    {
        ROS_INFO(" big IMU gyr bias estimation %f", Bgs[WINDOW_SIZE].norm());
        return true;
    }
    /*
    if (tic(0) > 1)
    {
        ROS_INFO(" big extri param estimation %d", tic(0) > 1);
        return true;
    }
    */
    Vector3d tmp_P = Ps[WINDOW_SIZE];
    if ((tmp_P - last_P).norm() > 5)
    {
        // ROS_INFO(" big translation");
        // return true;
    }
    if (abs(tmp_P.z() - last_P.z()) > 1)
    {
        // ROS_INFO(" big z translation");
        // return true;
    }
    Matrix3d tmp_R = Rs[WINDOW_SIZE];
    Matrix3d delta_R = tmp_R.transpose() * last_R;
    Quaterniond delta_Q(delta_R);
    double delta_angle;
    delta_angle = acos(delta_Q.w()) * 2.0 / 3.14 * 180.0;
    if (delta_angle > 50)
    {
        ROS_INFO(" big delta_angle ");
        // return true;
    }
    return false;
}

void Estimator::optimization()
{
    // std::cout<<"Cin optimization!!!!!!!!!!!!!!!!!!!!!!!!!!11"<<std::endl;
    TicToc t_whole, t_prepare;
    //------------------------------e、给ParameterBlock赋值
    vector2double();

    //------------------------------a、声明和引入鲁棒核函数
    ceres::Problem problem;
    ceres::LossFunction *loss_function;
    // loss_function = NULL;
    loss_function = new ceres::HuberLoss(1.0);
    // loss_function = new ceres::CauchyLoss(1.0 / FOCAL_LENGTH);
    // ceres::LossFunction* loss_function = new ceres::HuberLoss(1.0);

    //--------------------------------b、添加各种待优化量X——位姿优化量
    //这块出现了新数据结构para_Pose[i]和para_SpeedBias[i]，这是因为ceres传入的都是double类型的，在vector2double()里初始化的。
    for (int i = 0; i < frame_count + 1; i++)//包括最新的第11帧
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Pose[i], SIZE_POSE, local_parameterization);
        if (USE_IMU)
            problem.AddParameterBlock(para_SpeedBias[i], SIZE_SPEEDBIAS);
    }
    if (!USE_IMU)
        problem.SetParameterBlockConstant(para_Pose[0]);

    //-------------------------------c、添加各种待优化量X——相机外参
    // if(!f_manager.Opt_Select)
    // {
        for (int i = 0; i < NUM_OF_CAM; i++)
        {
            ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
            problem.AddParameterBlock(para_Ex_Pose[i], SIZE_POSE, local_parameterization);
            if ((ESTIMATE_EXTRINSIC && frame_count == WINDOW_SIZE && Vs[0].norm() > 0.2) || openExEstimation)
            {
                // ROS_INFO("estimate extinsic param");
                openExEstimation = 1;
            }
            else
            {
                // ROS_INFO("fix extinsic param");
                problem.SetParameterBlockConstant(para_Ex_Pose[i]);
            }
        }
    // }
    // else{
        for (int c = 0; c < NUM_OF_CAM; c++)
        {
            ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
            problem.AddParameterBlock(para_Ex_Pose1[c], SIZE_POSE, local_parameterization);
            if ((ESTIMATE_EXTRINSIC1 && frame_count == WINDOW_SIZE && Vs[0].norm() > 0.2) || openExEstimation)
            {
                // ROS_INFO("estimate extinsic param");
                openExEstimation = 1;
            }
            else
            {
                // ROS_INFO("fix extinsic param");
                problem.SetParameterBlockConstant(para_Ex_Pose1[c]);
            }
        }
    // }
    //--------------------------------------d、添加各种待优化量X——IMU-image时间同步误差
    problem.AddParameterBlock(para_Td[0], 1);

    if (!ESTIMATE_TD || Vs[0].norm() < 0.2)
        problem.SetParameterBlockConstant(para_Td[0]);

    //--------------------------------------f、添加各种残差——先验信息残差
    //在这里出现了2个新的数据结构。在第一次执行代码时，没有先验信息，所以跳过。第二次执行的时候就有了。last_marginalization_info的赋值出现在后面的代码里。
    if (last_marginalization_info && last_marginalization_info->valid)
    {
        // construct new marginlization_factor
        MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
        problem.AddResidualBlock(marginalization_factor, NULL,
                                 last_marginalization_parameter_blocks);
    }
    //---------------------------------------g、添加各种残差——IMU残差
    if (USE_IMU)
    {
        for (int i = 0; i < frame_count; i++)
        {
            int j = i + 1;
            if (pre_integrations[j]->sum_dt > 10.0)
                continue;
            IMUFactor *imu_factor = new IMUFactor(pre_integrations[j]);
            problem.AddResidualBlock(imu_factor, NULL, para_Pose[i], para_SpeedBias[i], para_Pose[j], para_SpeedBias[j]);
        }
    }
    //----------------------------------------h、添加各种残差——重投影残差
    //注意：IMU的残差是相邻两帧，但是视觉不是
    //视觉加入的两帧是观测到同一特征的最近两帧
    int f_m_cnt = 0; //统计有多少特征用于非线性优化
    int f_m_cnt_0 = 0;
    int f_m_cnt_1 = 0;
    // if(!f_manager.Opt_Select)
    // {
        int feature_index = -1;
        for (auto &it_per_id : f_manager.feature[0])
        {//遍历每一个特征
            if(f_m_cnt_0 > 85)
            {
                continue;
            }
            // if(it_per_id.select_residual_ > f_manager.ave_select_residual)
            // {
            //     continue;
            // }
            it_per_id.used_num = it_per_id.feature_per_frame.size();
            if (it_per_id.used_num < 4)
                continue;//必须满足特征出现的次数>4帧

            // if (it_per_id.estimated_depth > 105)
            //     continue;
            ++feature_index;//统计有效特征点数
            //得到观测到该特征的首帧
            int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
            //该特征在首帧中的归一化坐标
            Vector3d pts_i = it_per_id.feature_per_frame[0].point;

            for (auto &it_per_frame : it_per_id.feature_per_frame)
            {//遍历当前特征的每一帧信息
                imu_j++;
                //先把左目重投影误差加入
                if (imu_i != imu_j)
                {
                    Vector3d pts_j = it_per_frame.point;//得到第二个特征点
                    ProjectionTwoFrameOneCamFactor *f_td = new ProjectionTwoFrameOneCamFactor(pts_i, pts_j, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocity,
                                                                                            it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                    problem.AddResidualBlock(f_td, loss_function, para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index], para_Td[0]);
                }

                if (STEREO && it_per_frame.is_stereo)
                {//如果为双目，加入左右目的冲投影误差
                    Vector3d pts_j_right = it_per_frame.pointRight;
                    if (imu_i != imu_j)
                    {
                        ProjectionTwoFrameTwoCamFactor *f = new ProjectionTwoFrameTwoCamFactor(pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                                                                                            it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                        problem.AddResidualBlock(f, loss_function, para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Ex_Pose[1], para_Feature[feature_index], para_Td[0]);
                    }
                    else
                    {
                        ProjectionOneFrameTwoCamFactor *f = new ProjectionOneFrameTwoCamFactor(pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                                                                                            it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                        problem.AddResidualBlock(f, loss_function, para_Ex_Pose[0], para_Ex_Pose[1], para_Feature[feature_index], para_Td[0]);
                    }
                }
            }
            f_m_cnt++;
            f_m_cnt_0++;
        }
    // }
    // else
    // {
        int feature_index1 = -1;
        for (auto &it_per_id : f_manager.feature[1])
        {//遍历每一个特征
            if(f_m_cnt_1 > 85)
            {
                continue;
            }
            // if(it_per_id.select_residual_ > f_manager.ave_select_residual1)
            // {
            //     continue;
            // }
            it_per_id.used_num = it_per_id.feature_per_frame.size();
            if (it_per_id.used_num < 4)
                continue;//必须满足特征出现的次数>4帧

            // if (it_per_id.estimated_depth > 105)
            //     continue;
            ++feature_index1;//统计有效特征点数
            //得到观测到该特征的首帧
            int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
            //该特征在首帧中的归一化坐标
            Vector3d pts_i = it_per_id.feature_per_frame[0].point;

            for (auto &it_per_frame : it_per_id.feature_per_frame)
            {//遍历当前特征的每一帧信息
                imu_j++;
                //先把左目重投影误差加入
                if (imu_i != imu_j)
                {
                    Vector3d pts_j = it_per_frame.point;//得到第二个特征点
                    ProjectionTwoFrameOneCamFactor *f_td = new ProjectionTwoFrameOneCamFactor(pts_i, pts_j, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocity,
                                                                                            it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                    problem.AddResidualBlock(f_td, loss_function, para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose1[0], para_Feature1[feature_index1], para_Td[0]);
                }

                if (STEREO && it_per_frame.is_stereo)
                {//如果为双目，加入左右目的冲投影误差
                    Vector3d pts_j_right = it_per_frame.pointRight;
                    if (imu_i != imu_j)
                    {
                        ProjectionTwoFrameTwoCamFactor *f = new ProjectionTwoFrameTwoCamFactor(pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                                                                                            it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                        problem.AddResidualBlock(f, loss_function, para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose1[0], para_Ex_Pose1[1], para_Feature1[feature_index1], para_Td[0]);
                    }
                    else
                    {
                        ProjectionOneFrameTwoCamFactor *f = new ProjectionOneFrameTwoCamFactor(pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                                                                                            it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                        problem.AddResidualBlock(f, loss_function, para_Ex_Pose1[0], para_Ex_Pose1[1], para_Feature1[feature_index1], para_Td[0]);
                    }
                }
                
            }
            f_m_cnt++;
            f_m_cnt_1++;
        }
    // }

    std::cout<<"11111111111111111111f_m_cnt = "<<f_m_cnt<<std::endl;
    // std::cout<<"11111111111111111111111111111111111 = "<<std::endl;
    std::cout<<"2222222222222222222222222f_m_cnt_0 = "<<f_m_cnt_0<<std::endl;
    // std::cout<<"22222222222222222222222222222222222 = "<<std::endl;
    std::cout<<"333333333333333333333333333333f_m_cnt_1 = "<<f_m_cnt_1<<std::endl;
    std::cout<<"33333333333333333333333333333333333 = "<<std::endl;
    ROS_DEBUG("visual measurement count: %d", f_m_cnt);
    // printf("prepare for ceres: %f \n", t_prepare.toc());
    //----------------------------------------------i、求解
    ceres::Solver::Options options;

    options.linear_solver_type = ceres::DENSE_SCHUR;
    // options.num_threads = 2;
    options.trust_region_strategy_type = ceres::DOGLEG;
    options.max_num_iterations = NUM_ITERATIONS;
    // options.use_explicit_schur_complement = true;
    // options.minimizer_progress_to_stdout = true;
    // options.use_nonmonotonic_steps = true;
    if (marginalization_flag == MARGIN_OLD)
        options.max_solver_time_in_seconds = SOLVER_TIME * 4.0 / 5.0;
    else
        options.max_solver_time_in_seconds = SOLVER_TIME;
    TicToc t_solver;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    // cout << summary.BriefReport() << endl;
    ROS_DEBUG("Iterations : %d", static_cast<int>(summary.iterations.size()));
    // printf("solver costs: %f \n", t_solver.toc());
    ofstream foutC("/home/xu/output/RMSC_feature_cnt.txt", ios::app);
        foutC.setf(ios::fixed, ios::floatfield);
        foutC.precision(9);
        foutC << ros::Time::now().toSec() << " ";
        foutC.precision(5);
        foutC << f_m_cnt_0 << " "
              << f_m_cnt_1 << " "
              << f_m_cnt << " "
              << t_solver.toc() <<endl;    
    foutC.close();
    //-----------------------------------------------j、将优化后的量转回Vector
    double2vector();
    // printf("frame_count: %d \n", frame_count);

    //二、边缘化
    //这一部分值边缘化，不求解，求解留给下一轮优化的第一部分进行
    if (frame_count < WINDOW_SIZE)
        return;

    TicToc t_whole_marginalization;
    //--------------------------------------------------K、marg_old
    if (marginalization_flag == MARGIN_OLD)
    {
        //1）首先把上一轮残差的信息加进来
        MarginalizationInfo *marginalization_info = new MarginalizationInfo();
        vector2double();

        if (last_marginalization_info && last_marginalization_info->valid)
        {
            //先验误差会一直保存，而不是只使用一次
            //如果上一次边缘化的信息存在
            //要边缘化的参数块是para_Pose[0] para_SpeedBias[0] 以及 para_Feature[feature_index](滑窗内的第feature_index个点的逆深度)
            vector<int> drop_set; //将最老帧的先验信息干掉
            for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++)
            {   //查询last_marginalization_parameter_blocks中是首帧状态量的序号
                if (last_marginalization_parameter_blocks[i] == para_Pose[0] ||
                    last_marginalization_parameter_blocks[i] == para_SpeedBias[0])
                    drop_set.push_back(i);
            }
            // construct new marginlization_factor构造边缘化的factor
            MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(marginalization_factor, NULL,
                                                                           last_marginalization_parameter_blocks,
                                                                           drop_set);//添加上一次边缘化的参数块
            marginalization_info->addResidualBlockInfo(residual_block_info);
        }

        if (USE_IMU)
        {   //2）加入marg的imu信息，marg的第0帧imu信息
            if (pre_integrations[1]->sum_dt < 10.0)
            {
                IMUFactor *imu_factor = new IMUFactor(pre_integrations[1]);
                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(imu_factor, NULL,
                                                                               vector<double *>{para_Pose[0], para_SpeedBias[0], para_Pose[1], para_SpeedBias[1]},
                                                                               vector<int>{0, 1});
                marginalization_info->addResidualBlockInfo(residual_block_info);
            }
        }
        //3）加入marg的视觉信息
        // if(!f_manager.Opt_Select)
        // {   
            //添加视觉的先验，只添加起始帧为旧帧且观察次数大于4的feature
            int feature_index = -1;
            for (auto &it_per_id : f_manager.feature[0])//遍历0的所有feature
            {
                it_per_id.used_num = it_per_id.feature_per_frame.size();
                if (it_per_id.used_num < 4)//观测次数不小于4
                    continue;

                ++feature_index;

                int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
                if (imu_i != 0) //只选择被边缘化的帧0的feature
                    continue;

                Vector3d pts_i = it_per_id.feature_per_frame[0].point;//该特征点在起始帧下的归一化坐标

                for (auto &it_per_frame : it_per_id.feature_per_frame)
                {
                    imu_j++;
                    if (imu_i != imu_j)
                    {
                        Vector3d pts_j = it_per_frame.point;
                        ProjectionTwoFrameOneCamFactor *f_td = new ProjectionTwoFrameOneCamFactor(pts_i, pts_j, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocity,
                                                                                                  it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                        ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f_td, loss_function,
                                                                                       vector<double *>{para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index], para_Td[0]},
                                                                                       vector<int>{0, 3});
                        marginalization_info->addResidualBlockInfo(residual_block_info);
                    }
                    if (STEREO && it_per_frame.is_stereo)
                    {
                        Vector3d pts_j_right = it_per_frame.pointRight;
                        if (imu_i != imu_j)
                        {
                            ProjectionTwoFrameTwoCamFactor *f = new ProjectionTwoFrameTwoCamFactor(pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                                                                                                   it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f, loss_function,
                                                                                           vector<double *>{para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Ex_Pose[1], para_Feature[feature_index], para_Td[0]},
                                                                                           vector<int>{0, 4});
                            marginalization_info->addResidualBlockInfo(residual_block_info);
                        }
                        else
                        {
                            ProjectionOneFrameTwoCamFactor *f = new ProjectionOneFrameTwoCamFactor(pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                                                                                                   it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f, loss_function,
                                                                                           vector<double *>{para_Ex_Pose[0], para_Ex_Pose[1], para_Feature[feature_index], para_Td[0]},
                                                                                           vector<int>{2});
                            marginalization_info->addResidualBlockInfo(residual_block_info);
                        }
                    }
                }
            }
        // }
        // else
        // {
            //添加视觉的先验，只添加起始帧为旧帧且观察次数大于4的feature
            int feature_index1 = -1;
            for (auto &it_per_id : f_manager.feature[1])//遍历0的所有feature
            {
                it_per_id.used_num = it_per_id.feature_per_frame.size();
                if (it_per_id.used_num < 4)//观测次数不小于4
                    continue;

                ++feature_index1;

                int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
                if (imu_i != 0) //只选择被边缘化的帧0的feature
                    continue;

                Vector3d pts_i = it_per_id.feature_per_frame[0].point;//该特征点在起始帧下的归一化坐标

                for (auto &it_per_frame : it_per_id.feature_per_frame)
                {
                    imu_j++;
                    if (imu_i != imu_j)
                    {
                        Vector3d pts_j = it_per_frame.point;
                        ProjectionTwoFrameOneCamFactor *f_td = new ProjectionTwoFrameOneCamFactor(pts_i, pts_j, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocity,
                                                                                                  it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                        ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f_td, loss_function,
                                                                                       vector<double *>{para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose1[0], para_Feature1[feature_index1], para_Td[0]},
                                                                                       vector<int>{0, 3});
                        marginalization_info->addResidualBlockInfo(residual_block_info);
                    }
                    if (STEREO && it_per_frame.is_stereo)
                    {
                        Vector3d pts_j_right = it_per_frame.pointRight;
                        if (imu_i != imu_j)
                        {
                            ProjectionTwoFrameTwoCamFactor *f = new ProjectionTwoFrameTwoCamFactor(pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                                                                                                   it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f, loss_function,
                                                                                           vector<double *>{para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose1[0], para_Ex_Pose1[1], para_Feature1[feature_index1], para_Td[0]},
                                                                                           vector<int>{0, 4});
                            marginalization_info->addResidualBlockInfo(residual_block_info);
                        }
                        else
                        {
                            ProjectionOneFrameTwoCamFactor *f = new ProjectionOneFrameTwoCamFactor(pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                                                                                                   it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f, loss_function,
                                                                                           vector<double *>{para_Ex_Pose1[0], para_Ex_Pose1[1], para_Feature1[feature_index1], para_Td[0]},
                                                                                           vector<int>{2});
                            marginalization_info->addResidualBlockInfo(residual_block_info);
                        }
                    }
                }
            // }
        }
        //4）将三个ResidualBlockInfo中的参数块综合到marginalization_info中
        TicToc t_pre_margin;
        marginalization_info->preMarginalize();
        ROS_DEBUG("pre marginalization %f ms", t_pre_margin.toc());

        TicToc t_margin;
        marginalization_info->marginalize();
        ROS_DEBUG("marginalization %f ms", t_margin.toc());

        std::unordered_map<long, double *> addr_shift;
        for (int i = 1; i <= WINDOW_SIZE; i++)
        {
            addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
            if (USE_IMU)
                addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1];
        }
        // if(!f_manager.Opt_Select)
        // {
            for (int i = 0; i < NUM_OF_CAM; i++)
                addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];
        // }
        // else
        // {
            for (int i = 0; i < NUM_OF_CAM; i++)
                addr_shift[reinterpret_cast<long>(para_Ex_Pose1[i])] = para_Ex_Pose1[i];
        // }
        addr_shift[reinterpret_cast<long>(para_Td[0])] = para_Td[0];

        vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);

        if (last_marginalization_info)
            delete last_marginalization_info;
        last_marginalization_info = marginalization_info;
        last_marginalization_parameter_blocks = parameter_blocks;
    }
    else
    {   //-----------------------------------------I、marg_new
        //如果第二最新帧不是关键帧的话，则把这帧的视觉测量舍弃掉而保留IMU测量值在滑动窗口中
        if (last_marginalization_info &&
            std::count(std::begin(last_marginalization_parameter_blocks), std::end(last_marginalization_parameter_blocks), para_Pose[WINDOW_SIZE - 1]))
        {

            MarginalizationInfo *marginalization_info = new MarginalizationInfo();
            vector2double();
            if (last_marginalization_info && last_marginalization_info->valid)
            {
                vector<int> drop_set;
                for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++)
                {
                    ROS_ASSERT(last_marginalization_parameter_blocks[i] != para_SpeedBias[WINDOW_SIZE - 1]);
                    if (last_marginalization_parameter_blocks[i] == para_Pose[WINDOW_SIZE - 1])
                        drop_set.push_back(i);
                }
                // construct new marginlization_factor
                MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(marginalization_factor, NULL,
                                                                               last_marginalization_parameter_blocks,
                                                                               drop_set);

                marginalization_info->addResidualBlockInfo(residual_block_info);
            }

            TicToc t_pre_margin;
            ROS_DEBUG("begin marginalization");
            marginalization_info->preMarginalize();
            ROS_DEBUG("end pre marginalization, %f ms", t_pre_margin.toc());

            TicToc t_margin;
            ROS_DEBUG("begin marginalization");
            marginalization_info->marginalize();
            ROS_DEBUG("end marginalization, %f ms", t_margin.toc());

            std::unordered_map<long, double *> addr_shift;
            for (int i = 0; i <= WINDOW_SIZE; i++)
            {
                if (i == WINDOW_SIZE - 1)
                    continue;
                else if (i == WINDOW_SIZE)
                {
                    addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
                    if (USE_IMU)
                        addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1];
                }
                else
                {
                    addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i];
                    if (USE_IMU)
                        addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i];
                }
            }
            // if(!f_manager.Opt_Select)
            // {
                for (int i = 0; i < NUM_OF_CAM; i++)
                    addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];
            // }
            // else
            // {
                for (int i = 0; i < NUM_OF_CAM; i++)
                    addr_shift[reinterpret_cast<long>(para_Ex_Pose1[i])] = para_Ex_Pose1[i];
            // }
            addr_shift[reinterpret_cast<long>(para_Td[0])] = para_Td[0];

            vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);
            if (last_marginalization_info)
                delete last_marginalization_info;
            last_marginalization_info = marginalization_info;
            last_marginalization_parameter_blocks = parameter_blocks;
        }
    }


    // printf("whole marginalization costs: %f \n", t_whole_marginalization.toc());
    // printf("whole time for ceres: %f \n", t_whole.toc());
    ofstream foutC1("/home/xu/output/RMSC_t_whole.txt", ios::app);
        foutC1.setf(ios::fixed, ios::floatfield);
        foutC1.precision(9);
        foutC1 << ros::Time::now().toSec() << " ";
        foutC1.precision(5);
        foutC1 << t_whole.toc() <<endl;    
    foutC1.close();
}

void Estimator::slideWindow()
{
    TicToc t_margin;
    if (marginalization_flag == MARGIN_OLD)
    {
        double t_0 = Headers[0];

        back_R0 = Rs[0];
        back_P0 = Ps[0];
        if (frame_count == WINDOW_SIZE)
        {
            for (int i = 0; i < WINDOW_SIZE; i++)
            {
                Headers[i] = Headers[i + 1];
                Rs[i].swap(Rs[i + 1]);
                Ps[i].swap(Ps[i + 1]);
                if (USE_IMU)
                {
                    std::swap(pre_integrations[i], pre_integrations[i + 1]);

                    dt_buf[i].swap(dt_buf[i + 1]);
                    linear_acceleration_buf[i].swap(linear_acceleration_buf[i + 1]);
                    angular_velocity_buf[i].swap(angular_velocity_buf[i + 1]);

                    Vs[i].swap(Vs[i + 1]);
                    Bas[i].swap(Bas[i + 1]);
                    Bgs[i].swap(Bgs[i + 1]);
                }
            }
            Headers[WINDOW_SIZE] = Headers[WINDOW_SIZE - 1];
            Ps[WINDOW_SIZE] = Ps[WINDOW_SIZE - 1];
            Rs[WINDOW_SIZE] = Rs[WINDOW_SIZE - 1];

            if (USE_IMU)
            {
                Vs[WINDOW_SIZE] = Vs[WINDOW_SIZE - 1];
                Bas[WINDOW_SIZE] = Bas[WINDOW_SIZE - 1];
                Bgs[WINDOW_SIZE] = Bgs[WINDOW_SIZE - 1];

                delete pre_integrations[WINDOW_SIZE];
                pre_integrations[WINDOW_SIZE] = new IntegrationBase{acc_0, gyr_0, Bas[WINDOW_SIZE], Bgs[WINDOW_SIZE]};

                dt_buf[WINDOW_SIZE].clear();
                linear_acceleration_buf[WINDOW_SIZE].clear();
                angular_velocity_buf[WINDOW_SIZE].clear();
            }

            if (true || solver_flag == INITIAL)
            {
                // if(!f_manager.Opt_Select)
                // {
                    map<double, ImageFrame>::iterator it_0;
                    it_0 = all_image_frame.find(t_0);
                    if(it_0->second.pre_integration != nullptr)
                        delete it_0->second.pre_integration;
                    it_0->second.pre_integration = nullptr;
                    all_image_frame.erase(all_image_frame.begin(), it_0);
                // }
                // else
                // {
                //     map<double, ImageFrame>::iterator it_1;
                //     it_1 = all_image_frame1.find(t_0);
                //     delete it_1->second.pre_integration;
                //     all_image_frame1.erase(all_image_frame1.begin(), it_1);
                // }
            }
            slideWindowOld();
        }
    }
    else
    {
        if (frame_count == WINDOW_SIZE)
        {
            Headers[frame_count - 1] = Headers[frame_count];
            Ps[frame_count - 1] = Ps[frame_count];
            Rs[frame_count - 1] = Rs[frame_count];

            if (USE_IMU)
            {
                for (unsigned int i = 0; i < dt_buf[frame_count].size(); i++)
                {
                    double tmp_dt = dt_buf[frame_count][i];
                    Vector3d tmp_linear_acceleration = linear_acceleration_buf[frame_count][i];
                    Vector3d tmp_angular_velocity = angular_velocity_buf[frame_count][i];

                    pre_integrations[frame_count - 1]->push_back(tmp_dt, tmp_linear_acceleration, tmp_angular_velocity);

                    dt_buf[frame_count - 1].push_back(tmp_dt);
                    linear_acceleration_buf[frame_count - 1].push_back(tmp_linear_acceleration);
                    angular_velocity_buf[frame_count - 1].push_back(tmp_angular_velocity);
                }

                Vs[frame_count - 1] = Vs[frame_count];
                Bas[frame_count - 1] = Bas[frame_count];
                Bgs[frame_count - 1] = Bgs[frame_count];

                delete pre_integrations[WINDOW_SIZE];
                pre_integrations[WINDOW_SIZE] = new IntegrationBase{acc_0, gyr_0, Bas[WINDOW_SIZE], Bgs[WINDOW_SIZE]};

                dt_buf[WINDOW_SIZE].clear();
                linear_acceleration_buf[WINDOW_SIZE].clear();
                angular_velocity_buf[WINDOW_SIZE].clear();
            }
            slideWindowNew();
        }
    }
}

void Estimator::slideWindowNew()
{
    sum_of_front++;
    f_manager.removeFront(frame_count);
}

void Estimator::slideWindowOld()
{
    sum_of_back++;

    bool shift_depth = solver_flag == NON_LINEAR ? true : false;
    if (shift_depth)
    {
        Matrix3d R0, R1;
        Vector3d P0, P1;
        R0 = back_R0 * ric[0];
        R1 = Rs[0] * ric[0];
        P0 = back_P0 + back_R0 * tic[0];
        P1 = Ps[0] + Rs[0] * tic[0];
        f_manager.removeBackShiftDepth(R0, P0, R1, P1);
    }
    else
        f_manager.removeBack();
}

void Estimator::getPoseInWorldFrame(Eigen::Matrix4d &T)
{
    T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = Rs[frame_count];
    T.block<3, 1>(0, 3) = Ps[frame_count];
}

void Estimator::getPoseInWorldFrame(int index, Eigen::Matrix4d &T)
{
    T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = Rs[index];
    T.block<3, 1>(0, 3) = Ps[index];
}

void Estimator::predictPtsInNextFrame()
{
    // printf("predict pts in next frame\n");
    if (frame_count < 2)
        return;
    // predict next pose. Assume constant velocity motion
    Eigen::Matrix4d curT, prevT, nextT;
    getPoseInWorldFrame(curT);
    getPoseInWorldFrame(frame_count - 1, prevT);
    nextT = curT * (prevT.inverse() * curT);
    map<int, Eigen::Vector3d> predictPts[NUM_OF_CAM];
    // for (int c = 0; c < NUM_OF_CAM; c++)
    // {
    if(!f_manager.Opt_Select)
    {
        for (auto &it_per_id : f_manager.feature[0])
        {
            if (it_per_id.estimated_depth > 0)
            {
                int firstIndex = it_per_id.start_frame;
                int lastIndex = it_per_id.start_frame + it_per_id.feature_per_frame.size() - 1;
                // printf("cur frame index  %d last frame index %d\n", frame_count, lastIndex);
                if ((int)it_per_id.feature_per_frame.size() >= 2 && lastIndex == frame_count)
                {
                    double depth = it_per_id.estimated_depth;
                    Vector3d pts_j = ric[0] * (depth * it_per_id.feature_per_frame[0].point) + tic[0];
                    Vector3d pts_w = Rs[firstIndex] * pts_j + Ps[firstIndex];
                    Vector3d pts_local = nextT.block<3, 3>(0, 0).transpose() * (pts_w - nextT.block<3, 1>(0, 3));
                    Vector3d pts_cam = ric[0].transpose() * (pts_local - tic[0]);
                    int ptsIndex = it_per_id.feature_id;
                    predictPts[0][ptsIndex] = pts_cam;
                }
            }
        }
        featureTracker.setPrediction(predictPts[0], 0);
    }
    else
    {
        for (auto &it_per_id : f_manager.feature[1])
        {
            if (it_per_id.estimated_depth > 0)
            {
                int firstIndex = it_per_id.start_frame;
                int lastIndex = it_per_id.start_frame + it_per_id.feature_per_frame.size() - 1;
                // printf("cur frame index  %d last frame index %d\n", frame_count, lastIndex);
                if ((int)it_per_id.feature_per_frame.size() >= 2 && lastIndex == frame_count)
                {
                    double depth = it_per_id.estimated_depth;
                    Vector3d pts_j = ric1[0] * (depth * it_per_id.feature_per_frame[0].point) + tic1[0];
                    Vector3d pts_w = Rs[firstIndex] * pts_j + Ps[firstIndex];
                    Vector3d pts_local = nextT.block<3, 3>(0, 0).transpose() * (pts_w - nextT.block<3, 1>(0, 3));
                    Vector3d pts_cam = ric1[0].transpose() * (pts_local - tic1[0]);
                    int ptsIndex = it_per_id.feature_id;
                    predictPts[1][ptsIndex] = pts_cam;
                }
            }
        }
        featureTracker.setPrediction(predictPts[1], 1);
    }// printf("estimator output %d predict pts\n",(int)predictPts.size());
}

double Estimator::reprojectionError(Matrix3d &Ri, Vector3d &Pi, Matrix3d &rici, Vector3d &tici,
                                    Matrix3d &Rj, Vector3d &Pj, Matrix3d &ricj, Vector3d &ticj,
                                    double depth, Vector3d &uvi, Vector3d &uvj)
{
    Vector3d pts_w = Ri * (rici * (depth * uvi) + tici) + Pi;
    Vector3d pts_cj = ricj.transpose() * (Rj.transpose() * (pts_w - Pj) - ticj);
    Vector2d residual = (pts_cj / pts_cj.z()).head<2>() - uvj.head<2>();
    double rx = residual.x();
    double ry = residual.y();
    return sqrt(rx * rx + ry * ry);
}

void Estimator::outliersRejection(set<int> &removeIndex, set<int> &removeIndex1)
{
    // return;
        int feature_index = -1;
    // if(!f_manager.Opt_Select)
    // {
        for (auto &it_per_id : f_manager.feature[0])
        {
            double err = 0;
            int errCnt = 0;
            it_per_id.used_num = it_per_id.feature_per_frame.size();
            if (it_per_id.used_num < 4)
                continue;
            feature_index++;
            int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
            Vector3d pts_i = it_per_id.feature_per_frame[0].point;
            double depth = it_per_id.estimated_depth;
            for (auto &it_per_frame : it_per_id.feature_per_frame)
            {
                imu_j++;
                if (imu_i != imu_j)
                {
                    Vector3d pts_j = it_per_frame.point;
                    double tmp_error = reprojectionError(Rs[imu_i], Ps[imu_i], ric[0], tic[0],
                                                        Rs[imu_j], Ps[imu_j], ric[0], tic[0],
                                                        depth, pts_i, pts_j);
                    err += tmp_error;
                    errCnt++;
                    // printf("tmp_error %f\n", FOCAL_LENGTH / 1.5 * tmp_error);
                }
                // need to rewrite projecton factor.........
                if (STEREO && it_per_frame.is_stereo)
                {

                    Vector3d pts_j_right = it_per_frame.pointRight;
                    if (imu_i != imu_j)
                    {
                        double tmp_error = reprojectionError(Rs[imu_i], Ps[imu_i], ric[0], tic[0],
                                                            Rs[imu_j], Ps[imu_j], ric[1], tic[1],
                                                            depth, pts_i, pts_j_right);
                        err += tmp_error;
                        errCnt++;
                        // printf("tmp_error %f\n", FOCAL_LENGTH / 1.5 * tmp_error);
                    }
                    else
                    {
                        double tmp_error = reprojectionError(Rs[imu_i], Ps[imu_i], ric[0], tic[0],
                                                            Rs[imu_j], Ps[imu_j], ric[1], tic[1],
                                                            depth, pts_i, pts_j_right);
                        err += tmp_error;
                        errCnt++;
                        // printf("tmp_error %f\n", FOCAL_LENGTH / 1.5 * tmp_error);
                    }
                }
            }
            double ave_err = err / errCnt;
            if (ave_err * FOCAL_LENGTH > 3)
                removeIndex.insert(it_per_id.feature_id);
        }
    // }
    // else
    // {
        int feature_index1 = -1;
        for (auto &it_per_id : f_manager.feature[1])
        {
            double err = 0;
            int errCnt = 0;
            it_per_id.used_num = it_per_id.feature_per_frame.size();
            if (it_per_id.used_num < 4)
                continue;
            feature_index1++;
            int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
            Vector3d pts_i = it_per_id.feature_per_frame[0].point;
            double depth = it_per_id.estimated_depth;
            for (auto &it_per_frame : it_per_id.feature_per_frame)
            {
                imu_j++;
                if (imu_i != imu_j)
                {
                    Vector3d pts_j = it_per_frame.point;
                    double tmp_error = reprojectionError(Rs[imu_i], Ps[imu_i], ric1[0], tic1[0],
                                                        Rs[imu_j], Ps[imu_j], ric1[0], tic1[0],
                                                        depth, pts_i, pts_j);
                    err += tmp_error;
                    errCnt++;
                    // printf("tmp_error %f\n", FOCAL_LENGTH / 1.5 * tmp_error);
                }
                // need to rewrite projecton factor.........
                if (STEREO && it_per_frame.is_stereo)
                {

                    Vector3d pts_j_right = it_per_frame.pointRight;
                    if (imu_i != imu_j)
                    {
                        double tmp_error = reprojectionError(Rs[imu_i], Ps[imu_i], ric1[0], tic1[0],
                                                            Rs[imu_j], Ps[imu_j], ric1[1], tic1[1],
                                                            depth, pts_i, pts_j_right);
                        err += tmp_error;
                        errCnt++;
                        // printf("tmp_error %f\n", FOCAL_LENGTH / 1.5 * tmp_error);
                    }
                    else
                    {
                        double tmp_error = reprojectionError(Rs[imu_i], Ps[imu_i], ric1[0], tic1[0],
                                                            Rs[imu_j], Ps[imu_j], ric1[1], tic1[1],
                                                            depth, pts_i, pts_j_right);
                        err += tmp_error;
                        errCnt++;
                        // printf("tmp_error %f\n", FOCAL_LENGTH / 1.5 * tmp_error);
                    }
                }
            }
            double ave_err = err / errCnt;
            if (ave_err * FOCAL_LENGTH > 3)
                removeIndex1.insert(it_per_id.feature_id);
        }   
    // }
}
// 使用上一时刻的姿态进行快速的IMU积分，根据processIMU的最新数据Ps[frame_count]、Rs[frame_count]、Vs[frame_count]、Bas[frame_count]、Bgs[frame_count]来进行预计分。
// 预测最新的P、V、Q姿态，latest_P、latest_V、latest_Q、latest_acc_0、latest_gyr_0最新时刻的姿态。
// 作用是为了刷新姿态的输出，但是这个值的误差相对较大，是未经过非线性优化获取的初值。
void Estimator::fastPredictIMU(double t, Eigen::Vector3d linear_acceleration, Eigen::Vector3d angular_velocity)
{
    double dt = t - latest_time;
    latest_time = t;
    Eigen::Vector3d un_acc_0 = latest_Q * (latest_acc_0 - latest_Ba) - g;
    Eigen::Vector3d un_gyr = 0.5 * (latest_gyr_0 + angular_velocity) - latest_Bg;
    latest_Q = latest_Q * Utility::deltaQ(un_gyr * dt);
    Eigen::Vector3d un_acc_1 = latest_Q * (linear_acceleration - latest_Ba) - g;
    Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
    latest_P = latest_P + dt * latest_V + 0.5 * dt * dt * un_acc;
    latest_V = latest_V + dt * un_acc;
    latest_acc_0 = linear_acceleration;
    latest_gyr_0 = angular_velocity;
}

void Estimator::updateLatestStates()
{
    mPropagate.lock();
    latest_time = Headers[frame_count] + td;
    latest_P = Ps[frame_count];
    latest_Q = Rs[frame_count];
    latest_V = Vs[frame_count];
    latest_Ba = Bas[frame_count];
    latest_Bg = Bgs[frame_count];
    latest_acc_0 = acc_0;
    latest_gyr_0 = gyr_0;
    mBuf[0].lock();
    queue<pair<double, Eigen::Vector3d>> tmp_accBuf = accBuf;
    queue<pair<double, Eigen::Vector3d>> tmp_gyrBuf = gyrBuf;
    mBuf[0].unlock();
    while (!tmp_accBuf.empty())
    {
        double t = tmp_accBuf.front().first;
        Eigen::Vector3d acc = tmp_accBuf.front().second;
        Eigen::Vector3d gyr = tmp_gyrBuf.front().second;
        fastPredictIMU(t, acc, gyr);
        tmp_accBuf.pop();
        tmp_gyrBuf.pop();
    }
    mPropagate.unlock();
}
