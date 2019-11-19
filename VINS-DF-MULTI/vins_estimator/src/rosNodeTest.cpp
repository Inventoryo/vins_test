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

Estimator estimator;

queue<sensor_msgs::ImuConstPtr> imu_buf;
queue<sensor_msgs::PointCloudConstPtr> feature_buf;
queue<sensor_msgs::ImageConstPtr> img0_buf;
queue<sensor_msgs::ImageConstPtr> img1_buf;
queue<sensor_msgs::ImageConstPtr> img2_buf;
queue<sensor_msgs::ImageConstPtr> img3_buf;
queue<sensor_msgs::ImageConstPtr> img4_buf;
queue<sensor_msgs::ImageConstPtr> img5_buf;
std::mutex m_buf;


void img0_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    m_buf.lock();
    img0_buf.push(img_msg);
    m_buf.unlock();
}

void img1_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    m_buf.lock();
    img1_buf.push(img_msg);
    m_buf.unlock();
}

void img2_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    m_buf.lock();
    img2_buf.push(img_msg);
    m_buf.unlock();
}

void img3_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    m_buf.lock();
    img3_buf.push(img_msg);
    m_buf.unlock();
}

void img4_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    m_buf.lock();
    img4_buf.push(img_msg);
    m_buf.unlock();
}

void img5_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    m_buf.lock();
    img5_buf.push(img_msg);
    m_buf.unlock();
}

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

static bool CMP (const pair<double,int>&A,const pair<double,int>&B){
    return A.first<=B.first;
}

// extract images with same timestamp from two topics
void sync_process()
{
    while(1)
    {

        cv::Mat image0, image1, image2, image3, image4, image5;
        std_msgs::Header header;
        list<pair<double,int>> time_sync;
        if(!img0_buf.empty())
            time_sync.push_back(make_pair(img0_buf.front()->header.stamp.toSec(),0));
        if(!img1_buf.empty())
            time_sync.push_back(make_pair(img1_buf.front()->header.stamp.toSec(),1));
        if(!img2_buf.empty())
            time_sync.push_back(make_pair(img2_buf.front()->header.stamp.toSec(),2));
        if(!img3_buf.empty())
            time_sync.push_back(make_pair(img3_buf.front()->header.stamp.toSec(),3));
        if(!img4_buf.empty())
            time_sync.push_back(make_pair(img4_buf.front()->header.stamp.toSec(),4));
        if(!img5_buf.empty())
            time_sync.push_back(make_pair(img5_buf.front()->header.stamp.toSec(),5));
        if(time_sync.size()==NUM_OF_CAM){
            time_sync.sort(CMP);
            if(time_sync.front().first-time_sync.back().first>0.003){//时间未同步
                int pop_buf_num = time_sync.front().second;
                if(pop_buf_num ==0)
                    img0_buf.pop();
                else if(pop_buf_num ==1)
                    img1_buf.pop();
                else if(pop_buf_num ==2)
                    img2_buf.pop();
                else if(pop_buf_num ==3)
                    img3_buf.pop();
                else if(pop_buf_num ==4)
                    img4_buf.pop();
                else if(pop_buf_num ==5)
                    img5_buf.pop();
            }else{//时间已同步
                double time = time_sync.front().first;
                vector<pair<cv::Mat, cv::Mat>> images;
                if(NUM_OF_CAM==1){
                    image0 = getImageFromMsg(img0_buf.front());
                    img0_buf.pop();

                    for(int i =390;i<480;i++){
                        for(int j =200;j<460;j++){
                            image0.at<uchar>(i,j)=image0.at<uchar>(389,319);
                        }
                    }
                    images.push_back(make_pair(image0,cv::Mat()));
                }else if(NUM_OF_CAM==2){
                    image0 = getImageFromMsg(img0_buf.front());
                    img0_buf.pop();
                    image1 = getImageFromMsg(img1_buf.front());
                    img1_buf.pop();
                    /*
                    for(int i =390;i<480;i++){
                        for(int j =200;j<460;j++){
                            image0.at<uchar>(i,j)=image0.at<uchar>(389,319);
                            image1.at<uchar>(i,j)=image1.at<uchar>(389,319);
                        }
                    }
                     */
                    images.push_back(make_pair(image0,image1));
                }else if(NUM_OF_CAM==4){
                    image0 = getImageFromMsg(img0_buf.front());
                    img0_buf.pop();
                    image1 = getImageFromMsg(img1_buf.front());
                    img1_buf.pop();
                    image2 = getImageFromMsg(img2_buf.front());
                    img2_buf.pop();
                    image3 = getImageFromMsg(img3_buf.front());
                    img3_buf.pop();
                    for(int i =390;i<480;i++){
                        for(int j =200;j<460;j++){
                            image0.at<uchar>(i,j)=image0.at<uchar>(389,319);
                            image1.at<uchar>(i,j)=image1.at<uchar>(389,319);
                            image2.at<uchar>(i,j)=image2.at<uchar>(389,319);
                            image3.at<uchar>(i,j)=image3.at<uchar>(389,319);
                        }
                    }
                    images.push_back(make_pair(image0,image1));
                    images.push_back(make_pair(image2,image3));
                }else if(NUM_OF_CAM==6){
                    image0 = getImageFromMsg(img0_buf.front());
                    img0_buf.pop();
                    image1 = getImageFromMsg(img1_buf.front());
                    img1_buf.pop();
                    image2 = getImageFromMsg(img2_buf.front());
                    img2_buf.pop();
                    image3 = getImageFromMsg(img3_buf.front());
                    img3_buf.pop();
                    image4 = getImageFromMsg(img4_buf.front());
                    img4_buf.pop();
                    image5 = getImageFromMsg(img5_buf.front());
                    img5_buf.pop();
                    for(int i =390;i<480;i++){
                        for(int j =200;j<460;j++){
                            image0.at<uchar>(i,j)=image0.at<uchar>(389,319);
                            image1.at<uchar>(i,j)=image1.at<uchar>(389,319);
                            image2.at<uchar>(i,j)=image2.at<uchar>(389,319);
                            image3.at<uchar>(i,j)=image3.at<uchar>(389,319);
                        }
                    }
                    images.push_back(make_pair(image0,image1));
                    images.push_back(make_pair(image2,image3));
                    images.push_back(make_pair(image4,cv::Mat()));
                    images.push_back(make_pair(image5,cv::Mat()));
                }

                //cout<<" images.size() = "<<images.size()<<endl;
                estimator.inputImage(time, images);
            }
        }

        /*
        cv::Mat image0, image1, image2, image3, image4, image5;
        std_msgs::Header header;


        if(STEREO)
        {
            cv::Mat image0, image1;
            std_msgs::Header header;
            double time = 0;
            m_buf.lock();
            if (!img0_buf.empty() && !img1_buf.empty())
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
                else
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
            m_buf.unlock();
            if(!image0.empty())
                estimator.inputImage(time, image0, image1);
        }
        else
        {
            cv::Mat image;
            std_msgs::Header header;
            double time = 0;
            m_buf.lock();
            if(!img0_buf.empty())
            {
                time = img0_buf.front()->header.stamp.toSec();
                header = img0_buf.front()->header;
                image = getImageFromMsg(img0_buf.front());
                img0_buf.pop();
            }
            m_buf.unlock();
            if(!image.empty())
                estimator.inputImage(time, image);
        }
        */
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}


void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg)
{
    double t = imu_msg->header.stamp.toSec();
    double dx = imu_msg->linear_acceleration.x;
    double dy = imu_msg->linear_acceleration.y;
    double dz = imu_msg->linear_acceleration.z;
    double rx = imu_msg->angular_velocity.x;
    double ry = imu_msg->angular_velocity.y;
    double rz = imu_msg->angular_velocity.z;
    Vector3d acc(dx, dy, dz);
    Vector3d gyr(rx, ry, rz);
    estimator.inputIMU(t, acc, gyr);
    return;
}


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
        ROS_ASSERT(z == 1);
        Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
        xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
        featureFrame[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
    }
    double t = feature_msg->header.stamp.toSec();
    estimator.inputFeature(t, featureFrame);
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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vins_estimator");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    if(argc != 2)
    {
        printf("please intput: rosrun vins vins_node [config file] \n"
               "for example: rosrun vins vins_node "
               "~/catkin_ws/src/VINS-Fusion/config/euroc/euroc_stereo_imu_config.yaml \n");
        return 1;
    }

    string config_file = argv[1];
    printf("config_file: %s\n", argv[1]);

    readParameters(config_file);
    estimator.setParameter();

#ifdef EIGEN_DONT_PARALLELIZE
    ROS_DEBUG("EIGEN_DONT_PARALLELIZE");
#endif

    ROS_WARN("waiting for image and imu...");

    registerPub(n);

    ros::Subscriber sub_imu = n.subscribe(IMU_TOPIC, 2000, imu_callback, ros::TransportHints().tcpNoDelay());
    ros::Subscriber sub_feature = n.subscribe("/feature_tracker/feature", 2000, feature_callback);
    ros::Subscriber sub_img0 = n.subscribe(IMAGE0_TOPIC, 100, img0_callback);
    ros::Subscriber sub_img1 = n.subscribe(IMAGE1_TOPIC, 100, img1_callback);
    ros::Subscriber sub_img2 = n.subscribe(IMAGE2_TOPIC, 100, img2_callback);
    ros::Subscriber sub_img3 = n.subscribe(IMAGE3_TOPIC, 100, img3_callback);
    ros::Subscriber sub_img4 = n.subscribe(IMAGE4_TOPIC, 100, img4_callback);
    ros::Subscriber sub_img5 = n.subscribe(IMAGE5_TOPIC, 100, img5_callback);

    ros::Subscriber sub_restart = n.subscribe("/vins_restart", 100, restart_callback);
    ros::Subscriber sub_imu_switch = n.subscribe("/vins_imu_switch", 100, imu_switch_callback);
    ros::Subscriber sub_cam_switch = n.subscribe("/vins_cam_switch", 100, cam_switch_callback);

    std::thread sync_thread{sync_process};
    ros::spin();

    return 0;
}
