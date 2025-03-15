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

#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "estimator/estimator.h"
#include "estimator/parameters.h"
#include "utility/visualization.h"

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>

Estimator estimator;

queue<sensor_msgs::ImuConstPtr> imu_buf;
queue<sensor_msgs::PointCloudConstPtr> feature_buf;
//无时间同步
queue<sensor_msgs::ImageConstPtr> img0_buf;
queue<sensor_msgs::ImageConstPtr> img1_buf;
//img0_buf1，img1_buf1新加入相机流
queue<sensor_msgs::ImageConstPtr> img0_buf1;
queue<sensor_msgs::ImageConstPtr> img1_buf1;
std::mutex m_buf[2];
//使用时间同步
// queue<sensor_msgs::ImageConstPtr> img0_bufl;
// queue<sensor_msgs::ImageConstPtr> img0_bufr;
// queue<sensor_msgs::ImageConstPtr> img1_bufl;
// queue<sensor_msgs::ImageConstPtr> img1_bufr;
// std::mutex m_buf;


// void image_callback(const sensor_msgs::ImageConstPtr &img0_msgl, const sensor_msgs::ImageConstPtr &img0_msgr, const sensor_msgs::ImageConstPtr &img1_msgl, const sensor_msgs::ImageConstPtr &img1_msgr)
// {
//     m_buf.lock();
//     img0_bufl.push(img0_msgl);
//     img0_bufr.push(img0_msgr);
//     img1_bufl.push(img1_msgl);
//     img1_bufr.push(img1_msgr);
//     m_buf.unlock();
// }


//----------------------------------相机0
void img0_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    m_buf[0].lock();
                //std::cout<<"************************************************"<<std::endl;
    img0_buf.push(img_msg);
    m_buf[0].unlock();
}

void img1_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    m_buf[0].lock();
    img1_buf.push(img_msg);
    m_buf[0].unlock();
}
// // //----------------------------------相机1
void img0_callback1(const sensor_msgs::ImageConstPtr &img_msg)
{
    m_buf[1].lock();
    img0_buf1.push(img_msg);
    m_buf[1].unlock();
}

void img1_callback1(const sensor_msgs::ImageConstPtr &img_msg)
{
    m_buf[1].lock();
    img1_buf1.push(img_msg);
    m_buf[1].unlock();
}

//从msg中获取图片，返回值为cv：：Mat，输入为当前图像msg的指针
cv::Mat getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg)
{
    cv_bridge::CvImageConstPtr ptr;
    if (img_msg->encoding == "8UC1")
    {
        sensor_msgs::Image img;
        img.header = img_msg->header;
        img.height = img_msg->height;
        img.width = img_msg->width;
        img.is_bigendian = img_msg->is_bigendian;
        img.step = img_msg->step;
        img.data = img_msg->data;
        img.encoding = "mono8";
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    }
    else
        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);

    cv::Mat img = ptr->image.clone();
    return img;
}

// void process()
// {
//     std::cout<<"*****************************DOUBLE_CAMERA*******************"<<std::endl;
//     TicToc input;
//     cv::Mat imagel[2],  imager[2];  //image0 左目图像、image1 右目图像
//     std_msgs::Header header;
//     double time;
//     m_buf.lock();
//     if(!img0_bufl.empty())
//     {
//         std::cout<<"*****************************DOUBLE_CAMERA*******************"<<std::endl;
//         time = img0_bufl.front()->header.stamp.toSec();
//         imagel[0] = getImageFromMsg(img0_bufl.front());
//         img0_bufl.pop();
//     }
//     if(!img0_bufr.empty())
//     {
//         imager[0] = getImageFromMsg(img0_bufr.front());
//         img0_bufr.pop();
//     }
//     if(!img1_bufl.empty())
//     {
//         imagel[1] = getImageFromMsg(img1_bufl.front());
//         img1_bufl.pop();
//     }
//     if(!img1_bufl.empty())
//     {
//         imager[1] = getImageFromMsg(img1_bufr.front());
//         img1_bufr.pop();
//     }
//     if(!imagel[0].empty()|| !imagel[1].empty())
//     {
//         estimator.inputImage(time, imagel[0],imagel[1],imager[0],imager[1]);
//     }
//     m_buf.unlock();
// }

//使用message_filters
// void sync_process()
// {
//     while(1)
//     {
//         if(DOUBLE_CAM)//----------------------如果为双相机
//         {
//             //std::cout<<"*****************************DOUBLE_CAMERA*******************"<<std::endl;
//             TicToc input;
//             cv::Mat imagel[2],  imager[2];  //image0 左目图像、image1 右目图像
//             std_msgs::Header header[2];
//             std_msgs::Header header1[2];
//             double time[2];
//             m_buf.lock();

//             if(img0_bufl.empty()||img1_bufl.empty())
//             {

//                 m_buf.unlock();

//             }
//             else//对准两个相机的左目图像时间
//             {
//                 // 0.003s sync tolerance
//                 double time0_1 = img0_bufl.front()->header.stamp.toSec();//相机0的左目时间
//                 double time0_2 = img1_bufl.front()->header.stamp.toSec();//相机1的左目时间
//                 double time1_1 = time0_1;
//                 double time1_2 = time0_2;
//                 if(time0_1 < time0_2 - 0.03)
//                 {
//                     img0_bufl.pop();
//                     img1_bufl.pop();
//                     printf("throw 1img\n");
//                 }
//                 else if(time0_1 > time0_2 + 0.03)
//                 {
//                     img0_bufl.pop();
//                     img1_bufl.pop();
//                     printf("throw 2img\n");
//                 }
//                 else//左右两个相机得到图像的时间小于0.003s才对其进行处理
//                 {
//                     time[0] = img0_bufl.front()->header.stamp.toSec();
//                     header[0] = img0_bufl.front()->header;
//                     imagel[0] = getImageFromMsg(img0_bufl.front());//相机1的左目图像
//                     img0_bufl.pop();
//                     time[1] = img1_bufl.front()->header.stamp.toSec();
//                     header[1] = img1_bufl.front()->header;                    
//                     imagel[1] = getImageFromMsg(img1_bufl.front());//相机2的左目图像
//                     img1_bufl.pop();
//                     // double sync_time = (time[0]+time[1])/2.0;
//                     // double sync_time = time[0];
//                     double sync_time = time[1];
//                             //printf("------------------------1111111111111111111111time0       : %f\n", time[0]);
//                     //printf("0000000000000000000000000000000000time0       : %f\n", sync_time);
//                     //printf("find 1img0 and 2img0\n");
//                     if(STEREO)//----------------------------如果是双目
//                     {
//                         if(!img0_bufr.empty())
//                         {
//                             //std::cout<<"***************************STEREO*******************"<<std::endl;
//                             time1_1 = img0_bufr.front()->header.stamp.toSec();//相机1又目时间
//                             // header1[0] = img1_buf.front()->header;
//                             // 0.003s sync tolerance
//                             //---------------------------------相机1的右目
//                             //printf("1111111111111111111111time0       : %f\n", time[0]);
//                             //printf("222222222222222222222222222time1_1       : %f\n", time1_1);
//                             if(time1_1 < time[0] - 0.003)
//                             {       
//                                 img0_bufr.pop();
//                                 printf("throw 11img1\n");
//                             }
//                             if(time1_1 > time[0] + 0.003)
//                             {
//                                 img0_bufr.pop();
//                                 printf("throw 11img1\n");
//                             }
//                             else//相机1左右两目得到图像的时间小于0.003s才对其进行处理
//                             {
//                                 imager[0] = getImageFromMsg(img0_bufr.front());
//                                 time1_1 = img0_bufr.front()->header.stamp.toSec();
//                                 img0_bufr.pop();
//                             }
//                         }
//                         if(!img1_bufr.empty())
//                         {
//                             //std::cout<<"***************************STEREO*******************"<<std::endl;
//                             time1_2 = img1_bufr.front()->header.stamp.toSec();//相机2又目时间
//                             // header[0] = img0_buf.front()->header;
//                             // 0.003s sync tolerance
//                             //---------------------------------相机2的右目
//                             //printf("333333333333333333333333time1       : %f\n", time[1]);
//                             //printf("444444444444444444444444444time1_2       : %f\n", time1_2);
//                             if(time1_2 < time[1] - 0.003)
//                             {       
//                                 img1_bufr.pop();
//                                 printf("throw 21img1\n");
//                             }
//                             if(time1_2 > time[1] + 0.003)
//                             {
//                                 img1_bufr.pop();
//                                 printf("throw 21img1\n");
//                             }
//                             else//相机1左右两目得到图像的时间小于0.003s才对其进行处理
//                             {
//                                 imager[1] = getImageFromMsg(img1_bufr.front());
//                                 time1_2 = img1_bufr.front()->header.stamp.toSec();
//                                 img1_bufr.pop();
//                             }
//                         }
//                         // double temp_time = (time1_1 + time1_2)/2.0;  
//                         // sync_time = (temp_time + sync_time)/2.0;
//                         if(!imagel[0].empty()|| !imagel[1].empty())
//                         {
//                             // printf("1111111111111inputImage  time       : %f\n", input.toc());
//                         //std::cout<<"image1.size = "<<image1[0].size()<<std::endl;
//                             estimator.inputImage(sync_time, imagel[0],imagel[1],imager[0],imager[1]);
//                         }
//                     }
//                     else//--------------------------单目
//                     {
//                         //std::cout<<"*****************************ONE_CAMERA*******************"<<std::endl;
//                         //std::cout<<"image1.size = "<<image1[0].size()<<std::endl;
//                         // if(!image0[0].empty()|| !image0[1].empty())
//                             estimator.inputImage(sync_time,imagel[0],imagel[1], imager[0], imager[1]);
//                     }

//                 }
//                 m_buf.unlock();
//             }
//         }

//         std::chrono::milliseconds dura(2);
//         std::this_thread::sleep_for(dura);
//     }
// }

// extract images with same timestamp from four topics
//原来的不使用message_filters
void sync_process()
{
    while(1)
    {
        if(DOUBLE_CAM)//----------------------如果为双相机
        {
            //std::cout<<"*****************************DOUBLE_CAMERA*******************"<<std::endl;
            TicToc input;
            cv::Mat image0[2],  image1[2];  //image0 左目图像、image1 右目图像
            std_msgs::Header header[2];
            std_msgs::Header header1[2];
            double time[2];
            for(int i=0;i<NUM_OF_CAM;i++)
            {
                m_buf[i].lock();
            }
            if(img0_buf.empty()||img0_buf1.empty())
            {
                for(int i=0;i<NUM_OF_CAM;i++)
                {
                    m_buf[i].unlock();
                }
            }
            else//对准两个相机的左目图像时间
            {
                // 0.003s sync tolerance
                double time0_1 = img0_buf.front()->header.stamp.toSec();//相机0的左目时间
                double time0_2 = img0_buf1.front()->header.stamp.toSec();//相机1的左目时间
                double time1_1 = time0_1;
                double time1_2 = time0_2;
                if(time0_1 < time0_2 - 0.06)
                {
                    img0_buf.pop();
                    img1_buf.pop();
                    printf("throw 1img\n");
                }
                else if(time0_1 > time0_2 + 0.06)
                {
                    img0_buf1.pop();
                    img1_buf1.pop();
                    printf("throw 2img\n");
                }
                else//左右两个相机得到图像的时间小于0.003s才对其进行处理
                {
                    time[0] = img0_buf.front()->header.stamp.toSec();
                    header[0] = img0_buf.front()->header;
                    image0[0] = getImageFromMsg(img0_buf.front());//相机1的左目图像
                    img0_buf.pop();
                    time[1] = img0_buf1.front()->header.stamp.toSec();
                    header[1] = img0_buf1.front()->header;                    
                    image0[1] = getImageFromMsg(img0_buf1.front());//相机2的左目图像
                    img0_buf1.pop();
                    // double sync_time = (time[0]+time[1])/2.0;
                    // double sync_time = time[0];
                    double sync_time = time[1];
                            //printf("------------------------1111111111111111111111time0       : %f\n", time[0]);
                    //printf("0000000000000000000000000000000000time0       : %f\n", sync_time);
                    //printf("find 1img0 and 2img0\n");
                    if(STEREO)//----------------------------如果是双目
                    {
                        if(!img1_buf.empty())
                        {
                            //std::cout<<"***************************STEREO*******************"<<std::endl;
                            time1_1 = img1_buf.front()->header.stamp.toSec();//相机1又目时间
                            // header1[0] = img1_buf.front()->header;
                            // 0.003s sync tolerance
                            //---------------------------------相机1的右目
                            //printf("1111111111111111111111time0       : %f\n", time[0]);
                            //printf("222222222222222222222222222time1_1       : %f\n", time1_1);
                            if(time1_1 < time[0] - 0.006)
                            {       
                                img1_buf.pop();
                                printf("throw 11img1\n");
                            }
                            if(time1_1 > time[0] + 0.006)
                            {
                                img1_buf.pop();
                                printf("throw 11img1\n");
                            }
                            else//相机1左右两目得到图像的时间小于0.003s才对其进行处理
                            {
                                image1[0] = getImageFromMsg(img1_buf.front());
                                time1_1 = img1_buf.front()->header.stamp.toSec();
                                img1_buf.pop();
                            }
                        }
                        if(!img1_buf1.empty())
                        {
                            //std::cout<<"***************************STEREO*******************"<<std::endl;
                            time1_2 = img1_buf1.front()->header.stamp.toSec();//相机2又目时间
                            // header[0] = img0_buf.front()->header;
                            // 0.003s sync tolerance
                            //---------------------------------相机2的右目
                            //printf("333333333333333333333333time1       : %f\n", time[1]);
                            //printf("444444444444444444444444444time1_2       : %f\n", time1_2);
                            if(time1_2 < time[1] - 0.006)
                            {       
                                img1_buf1.pop();
                                printf("throw 21img1\n");
                            }
                            if(time1_2 > time[1] + 0.006)
                            {
                                img1_buf1.pop();
                                printf("throw 21img1\n");
                            }
                            else//相机1左右两目得到图像的时间小于0.003s才对其进行处理
                            {
                                image1[1] = getImageFromMsg(img1_buf1.front());
                                time1_2 = img1_buf1.front()->header.stamp.toSec();
                                img1_buf1.pop();
                            }
                        }
                        // double temp_time = (time1_1 + time1_2)/2.0;  
                        // sync_time = (temp_time + sync_time)/2.0;
                        if(!image1[0].empty()|| !image1[1].empty())
                        {
                            // printf("1111111111111inputImage  time       : %f\n", input.toc());
                        //std::cout<<"image1.size = "<<image1[0].size()<<std::endl;
                            estimator.inputImage(sync_time, image0[0],image0[1],image1[0],image1[1]);
                        }
                    }
                    else//--------------------------单目
                    {
                        //std::cout<<"*****************************ONE_CAMERA*******************"<<std::endl;
                        //std::cout<<"image1.size = "<<image1[0].size()<<std::endl;
                        // if(!image0[0].empty()|| !image0[1].empty())
                            estimator.inputImage(sync_time,image0[0],image0[1], image1[0], image1[1]);
                    }

                }
                for(int i=0;i<NUM_OF_CAM;i++)
                {
                    m_buf[i].unlock();
                }
            }
        }
        else//--------------------------单相机
        {
        if(STEREO)//STEREO==1
        {
            cv::Mat image0, image1;
            std_msgs::Header header;
            double time = 0;
            m_buf[0].lock();
            if (!img0_buf.empty() && !img1_buf.empty())//如果imgbuf不为空的话，进行处理
            {
                double time0 = img0_buf.front()->header.stamp.toSec();
                double time1 = img1_buf.front()->header.stamp.toSec();
                // 0.003s sync tolerance
                if(time0 < time1 - 0.003)
                {
                    img0_buf.pop();
                    printf("throw img0\n");
                }
                else if(time0 > time1 + 0.003)
                {
                    img1_buf.pop();
                    printf("throw img1\n");
                }
                else//左右两个相机得到图像的时间小于0.003s才对其进行处理
                {
                    time = img0_buf.front()->header.stamp.toSec();
                    header = img0_buf.front()->header;
                    image0 = getImageFromMsg(img0_buf.front());
                    img0_buf.pop();
                    image1 = getImageFromMsg(img1_buf.front());
                    img1_buf.pop();
                    //printf("find img0 and img1\n");
                }
            }
            m_buf[0].unlock();
        //    if(!image0.empty())
        //        estimator.inputImage(time, image0, image1);//重点语句**************************
        }//双目处理过程结束
        else
        {
            cv::Mat image;
            std_msgs::Header header;
            double time = 0;
            m_buf[0].lock();
            if(!img0_buf.empty())
            {
                time = img0_buf.front()->header.stamp.toSec();
                header = img0_buf.front()->header;
                image = getImageFromMsg(img0_buf.front());
                img0_buf.pop();
            }
            m_buf[0].unlock();
        //    if(!image.empty())
        //        estimator.inputImage(time, image);
        }
        }
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}

//输入imu的msg信息，进行结算并把imu数据输入到estimator
void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg)
{
    double t = imu_msg->header.stamp.toSec();
    double dx = imu_msg->linear_acceleration.x;
    double dy = imu_msg->linear_acceleration.y;
    double dz = imu_msg->linear_acceleration.z;
    double rx = imu_msg->angular_velocity.x;
    double ry = imu_msg->angular_velocity.y;
    double rz = imu_msg->angular_velocity.z;
    Vector3d acc(dx, dy, dz);//线加速度
    Vector3d gyr(rx, ry, rz);//角速度
    estimator.inputIMU(t, acc, gyr);
    return;
}
//把特征点的点云msg、输入到estimator
void feature_callback(const sensor_msgs::PointCloudConstPtr &feature_msg)
{
    map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> featureFrame;
    for (unsigned int i = 0; i < feature_msg->points.size(); i++)
    {
        int feature_id = feature_msg->channels[0].values[i];
        int camera_id = feature_msg->channels[1].values[i];
        double x = feature_msg->points[i].x;
        double y = feature_msg->points[i].y;
        double z = feature_msg->points[i].z;
        double p_u = feature_msg->channels[2].values[i];
        double p_v = feature_msg->channels[3].values[i];
        double velocity_x = feature_msg->channels[4].values[i];
        double velocity_y = feature_msg->channels[5].values[i];
        if(feature_msg->channels.size() > 5)
        {
            double gx = feature_msg->channels[6].values[i];
            double gy = feature_msg->channels[7].values[i];
            double gz = feature_msg->channels[8].values[i];
            pts_gt[feature_id] = Eigen::Vector3d(gx, gy, gz);
            //printf("receive pts gt %d %f %f %f\n", feature_id, gx, gy, gz);
        }
        ROS_ASSERT(z == 1);//如果z不等于1，则程序终止
        Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
        xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
        featureFrame[feature_id].emplace_back(camera_id,  xyz_uv_velocity);//emplace_back与push_back作用相同
    }
    double t = feature_msg->header.stamp.toSec();
    estimator.inputFeature(t, featureFrame);//将特征点的位置速度以及相机id(featureFrame)放入Featurebuf中
    return;
}

void restart_callback(const std_msgs::BoolConstPtr &restart_msg)
{
    if (restart_msg->data == true)
    {
        ROS_WARN("restart the estimator!");
        estimator.clearState();
        estimator.setParameter();
    }
    return;
}

void imu_switch_callback(const std_msgs::BoolConstPtr &switch_msg)
{
    if (switch_msg->data == true)
    {
        //ROS_WARN("use IMU!");
        estimator.changeSensorType(1, STEREO);
    }
    else
    {
        //ROS_WARN("disable IMU!");
        estimator.changeSensorType(0, STEREO);
    }
    return;
}

void cam_switch_callback(const std_msgs::BoolConstPtr &switch_msg)
{
    if (switch_msg->data == true)
    {
        //ROS_WARN("use stereo!");
        estimator.changeSensorType(USE_IMU, 1);
    }
    else
    {
        //ROS_WARN("use mono camera (left)!");
        estimator.changeSensorType(USE_IMU, 0);
    }
    return;
}

// typedef message_filters::sync_policies::ApproximateTime<const sensor_msgs::ImageConstPtr, const sensor_msgs::ImageConstPtr, const sensor_msgs::ImageConstPtr, const sensor_msgs::ImageConstPtr> SyncPolicyImage;
// typedef shared_ptr<message_filters::Synchronizer<SyncPolicyImage>> SynchronizerImage;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vins_estimator");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);//将ros输出的信息默认改为info

    if(argc != 2)
    {
        printf("please intput: rosrun vins vins_node [config file] \n"
               "for example: rosrun vins vins_node "
               "~/catkin_ws/src/VINS-Fusion/config/euroc/euroc_stereo_imu_config.yaml \n");
        return 1;
    }

    string config_file = argv[1];
    printf("config_file: %s\n", argv[1]);

    readParameters(config_file);//根据配置文件读取一些参数
    estimator.setParameter();//进一步设置参数

#ifdef EIGEN_DONT_PARALLELIZE
    ROS_DEBUG("EIGEN_DONT_PARALLELIZE");
#endif

    ROS_WARN("waiting for image and imu...");

    registerPub(n);

    ros::Subscriber sub_imu = n.subscribe(IMU_TOPIC, 2000, imu_callback, ros::TransportHints().tcpNoDelay());//将接受到的角速度以及加速度信息传到IMU中，
   //IMU_TOPIC为imu0，，如果solver_flag == NON_LINEAR，则将计算出来位置速度信息发布（即将求解出来的odometry）以话题：imu_propagate的形式发布
    ros::Subscriber sub_feature = n.subscribe("/feature_tracker/feature", 2000, feature_callback);//将特征点信息放入featurebuf//该话题没有内容

    //message_filters  报错
    // shared_ptr<message_filters::Subscriber<const sensor_msgs::ImageConstPtr>> img0_subl_;
    // shared_ptr<message_filters::Subscriber<const sensor_msgs::ImageConstPtr>> img0_subr_;
    // shared_ptr<message_filters::Subscriber<const sensor_msgs::ImageConstPtr>> img1_subl_;
    // shared_ptr<message_filters::Subscriber<const sensor_msgs::ImageConstPtr>> img1_subr_;
    // SynchronizerImage sync_image_;
    // img0_subl_.reset(new message_filters::Subscriber<const sensor_msgs::ImageConstPtr>(n,IMAGE0_TOPIC,100));
    // img0_subr_.reset(new message_filters::Subscriber<const sensor_msgs::ImageConstPtr>(n,IMAGE1_TOPIC,100));

    // img1_subl_.reset(new message_filters::Subscriber<const sensor_msgs::ImageConstPtr>(n,IMAGE0_TOPIC1,100));
    // img1_subr_.reset(new message_filters::Subscriber<const sensor_msgs::ImageConstPtr>(n,IMAGE1_TOPIC1,100));

    // sync_image_.reset(new message_filters::Synchronizer<SyncPolicyImage>(SyncPolicyImage(10), *img0_subl_,*img0_subr_, *img1_subl_,*img1_subr_));
    // sync_image_->registerCallback(boost::bind(image_callback, _1 ,_2, _3, _4));

    //正常不用message_filters
    ros::Subscriber sub_img0 = n.subscribe(IMAGE0_TOPIC, 100, img0_callback);//将话题中的信息放入
    ros::Subscriber sub_img1 = n.subscribe(IMAGE1_TOPIC, 100, img1_callback);//-------------相机0

    ros::Subscriber sub_img0_1 = n.subscribe(IMAGE0_TOPIC1, 100, img0_callback1);//将话题中的信息放入
    ros::Subscriber sub_img1_1 = n.subscribe(IMAGE1_TOPIC1, 100, img1_callback1);//-----------相机1
    //使用message_filters
    // message_filters::Subscriber<sensor_msgs::Image> img0_subl_(n,IMAGE0_TOPIC,10);
    // message_filters::Subscriber<sensor_msgs::Image> img0_subr_(n,IMAGE1_TOPIC,10);
    // message_filters::Subscriber<sensor_msgs::Image> img1_subl_(n,IMAGE0_TOPIC1,10);
    // message_filters::Subscriber<sensor_msgs::Image> img1_subr_(n,IMAGE0_TOPIC1,10);
    // typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image> SyncPolicyImage;
    // message_filters::Synchronizer<SyncPolicyImage> SynchronizerImage(SyncPolicyImage(100),img0_subl_,img0_subr_,img1_subl_,img1_subr_);

    // SynchronizerImage.registerCallback(boost::bind(&image_callback, _1, _2, _3, _4));


    ros::Subscriber sub_restart = n.subscribe("/vins_restart", 100, restart_callback);//重新开始
    ros::Subscriber sub_imu_switch = n.subscribe("/vins_imu_switch", 100, imu_switch_callback);//选择是否加入imu
    ros::Subscriber sub_cam_switch = n.subscribe("/vins_cam_switch", 100, cam_switch_callback);//选择双目还是单目相机

    //创建sync_thread线程，指向sync_process，这里边处理了processMeasurements的线程
    // std::thread sync_thread{sync_process};//对图片信息进行处理
    std::thread sync_thread{sync_process};//对图片信息进行处理
    ros::spin();//用于触发topic，service的响应队列
    //如果你的程序写了相关的消息订阅函数，那么程序在执行过程中，除了主程序以外，ROS还会自动在后台按照你规定的格式，接受订阅的消息，但是所接受到的消息不是
    //立刻就被处理，而是必须要等到ros::spin()或ros::spinOnce()执行的时候才被调用，这就是消息回到函数的原理。

    return 0;
}
