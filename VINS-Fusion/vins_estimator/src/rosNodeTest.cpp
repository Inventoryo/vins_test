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
//todo 增加双目
queue<sensor_msgs::ImageConstPtr> img2_buf;
queue<sensor_msgs::ImageConstPtr> img3_buf;
//todo 增加两个单目
queue<sensor_msgs::ImageConstPtr> img4_buf;
queue<sensor_msgs::ImageConstPtr> img5_buf;


std::mutex m_buf;

void img0_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    m_buf.lock();
    img0_buf.push(img_msg);
    //std::cout<<"grab image0 success!"<<std::endl;
    m_buf.unlock();
}

void img1_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    m_buf.lock();
    img1_buf.push(img_msg);
    m_buf.unlock();
    //std::cout<<"grab image1 success!"<<std::endl;
}

//todo 增加双目回调函数
void img2_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    m_buf.lock();
    img2_buf.push(img_msg);
    m_buf.unlock();
    //std::cout<<"grab image2 success!"<<std::endl;
}

void img3_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    m_buf.lock();
    img3_buf.push(img_msg);
    m_buf.unlock();
    //std::cout<<"grab image3 success!"<<std::endl;
}

void img4_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    m_buf.lock();
    img4_buf.push(img_msg);
    m_buf.unlock();
    //std::cout<<"grab image3 success!"<<std::endl;
}

void img5_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    m_buf.lock();
    img5_buf.push(img_msg);
    m_buf.unlock();
    //std::cout<<"grab image3 success!"<<std::endl;
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

// extract images with same timestamp from two topics
void sync_process()
{
    while(1)
    {

        if(STEREO)
        {
            cv::Mat image0, image1;
            cv::Mat image2, image3;//todo 增加双目
            cv::Mat image4, image5;
            //std::cout<<"STEREO is true"<<std::endl;
            std_msgs::Header header;
            double time = 0;
            m_buf.lock();
            if (!img0_buf.empty() && !img1_buf.empty()&&NUM_OF_CAM == 2)
            {
                double time0 = img0_buf.front()->header.stamp.toSec();
                double time1 = img1_buf.front()->header.stamp.toSec();
                //std::cout<<"time0 - time1 = "<<time0-time1<<std::endl;
                if(time0 < time1)
                {
                    img0_buf.pop();
                    printf("throw img0\n");
                }
                else if(time0 > time1)
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


            if (!img2_buf.empty() && !img3_buf.empty()&&!img0_buf.empty() && !img1_buf.empty()&&NUM_OF_CAM == 4)//todo 另一个双目，假设两个双目的时间是同步的
            {

                double time2 = img2_buf.front()->header.stamp.toSec();
                double time3 = img3_buf.front()->header.stamp.toSec();
                double time0 = img0_buf.front()->header.stamp.toSec();
                double time1 = img1_buf.front()->header.stamp.toSec();
                //std::cout<<"time0 - time2 = "<<time0-time2<<std::endl;

                if(time0 < time1)
                {
                    img0_buf.pop();
                    printf("throw img0\n");
                }
                else if(time0 > time1)
                {
                    img1_buf.pop();
                    printf("throw img1\n");
                }


                if(time0 > time2)
                {
                    std::cout<<"time0 - time2 = "<<time0-time2<<std::endl;
                    img2_buf.pop();
                    img3_buf.pop();
                    printf("throw img2\n");
                }else if(time0 < time2-0.05)
                {
                    img0_buf.pop();
                    printf("throw img0\n");
                }


                if(time0 == time1&&abs(time0-time2) <= 0.05){

                    time = img0_buf.front()->header.stamp.toSec();
                    header = img0_buf.front()->header;
                    image0 = getImageFromMsg(img0_buf.front());

                    img0_buf.pop();
                    image1 = getImageFromMsg(img1_buf.front());
                    img1_buf.pop();
                    image2 = getImageFromMsg(img2_buf.front());
                    img2_buf.pop();
                    image3 = getImageFromMsg(img3_buf.front());
                    img3_buf.pop();
                    //std::cout<<"time0-time2 = "<<time0-time2<<"  time0-time3 = "<<time0-time3<<std::endl;
                }
            }

            if (!img2_buf.empty() && !img3_buf.empty() && !img0_buf.empty() && !img1_buf.empty() && !img4_buf.empty() && !img5_buf.empty() && NUM_OF_CAM == 6)//todo 另一个双目，假设两个双目的时间是同步的
            {

                double time2 = img2_buf.front()->header.stamp.toSec();
                double time3 = img3_buf.front()->header.stamp.toSec();
                double time0 = img0_buf.front()->header.stamp.toSec();
                double time1 = img1_buf.front()->header.stamp.toSec();
                double time4 = img4_buf.front()->header.stamp.toSec();
                double time5 = img5_buf.front()->header.stamp.toSec();
                //std::cout<<"time0 - time2 = "<<time0-time2<<std::endl;
                //std::cout<<"time0-time2 = "<<time0-time2<<"  time0-time4 = "<<time0-time4<<"  time0-time5 = "<<time0-time5<<std::endl;

                if(time0 < time1)
                {
                    img0_buf.pop();
                    printf("throw img0\n");
                }
                else if(time0 > time1)
                {
                    img1_buf.pop();
                    printf("throw img1\n");
                }


                if(time0 > time2)
                {
                    std::cout<<"time0 - time2 = "<<time0-time2<<std::endl;
                    img2_buf.pop();
                    printf("throw img2\n");
                }

                if(time0 > time3){
                    img3_buf.pop();
                    printf("throw img3\n");
                }

                if(time0 > time4){
                    img4_buf.pop();
                    printf("throw img3\n");
                }

                if(time0 > time5){
                    img5_buf.pop();
                    printf("throw img3\n");
                }


                if(time0 == time1&&time0 <= time2 && time0 <= time3 && time0 <= time4&& time0 <= time5 ){



                    time = img0_buf.front()->header.stamp.toSec();
                    header = img0_buf.front()->header;
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

                    //std::cout<<"time0-time2 = "<<time0-time2<<"  time0-time3 = "<<time0-time3<<std::endl;
                }
            }

            m_buf.unlock();
            if(!image0.empty() && image2.empty()){//todo 一个双目
                //std::cout<<"one stereo"<<std::endl;
                for(int i=390;i<480;i++){
                    for(int j = 200;j<460;j++){
                        image0.at<uchar>(i,j) = image0.at<uchar>(389,319);
                    }
                }
                estimator.inputImage(time, image0, image1);
            }
            else if(!image0.empty() && !image2.empty()&&image4.empty()){//todo 两个双目
                //std::cout<<"two stereo"<<std::endl;
                //cv::Mat mask1 = cv::Mat(mask.size(),CV_8UC1,CvScalar(1));
                //cv::Mat imageROI0 = image0(cv::Rect(250, 400, 140, 79));
                //std::cout<<"imageROI0.cols = "<<imageROI0.cols<<std::endl;
                for(int i=390;i<480;i++){
                    for(int j = 200;j<460;j++){
                        image0.at<uchar>(i,j) = image0.at<uchar>(389,319);
                        image2.at<uchar>(i,j) = image2.at<uchar>(389,319);
                    }
                }
                //imageROI0.zeros(79,140,CV_8UC1);

                //std::cout<<"imageROI0.cols = "<<std::endl;
                estimator.inputImage(time, image0, image1,image2,image3);
            }
            else if(!image0.empty() && !image2.empty() && !image4.empty()){
                for(int i=390;i<480;i++){
                    for(int j = 200;j<460;j++){
                        image0.at<uchar>(i,j) = image0.at<uchar>(389,319);
                        image2.at<uchar>(i,j) = image2.at<uchar>(389,319);
                    }
                }
                //std::cout<<"ready to input image "<<std::endl;
                estimator.inputImage(time, image0, image1,image2,image3, image4,image5);//todo 六目
            }

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

void imu_callback1(const sensor_msgs::ImuConstPtr &imu_msg)
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
    estimator.inputIMU1(t, acc, gyr);
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
        m_buf.lock();
        while(!feature_buf.empty())
            feature_buf.pop();
        while(!imu_buf.empty())
            imu_buf.pop();
        m_buf.unlock();
        estimator.clearState();
        estimator.setParameter();
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
    std::cout<<"config_file = "<<config_file<<std::endl;
    estimator.setParameter();//初始化估计器

#ifdef EIGEN_DONT_PARALLELIZE
    ROS_DEBUG("EIGEN_DONT_PARALLELIZE");
#endif

    ROS_WARN("waiting for image and imu...");

    registerPub(n);

    ros::Subscriber sub_imu0 = n.subscribe(IMU_TOPIC, 2000, imu_callback, ros::TransportHints().tcpNoDelay());//IMU
    ros::Subscriber sub_imu1 = n.subscribe(IMU_TOPIC1, 2000, imu_callback1, ros::TransportHints().tcpNoDelay());//IMU

    ros::Subscriber sub_feature = n.subscribe("/feature_tracker/feature", 2000, feature_callback);//
    ros::Subscriber sub_img0 = n.subscribe(IMAGE0_TOPIC, 100, img0_callback);
    ros::Subscriber sub_img1 = n.subscribe(IMAGE1_TOPIC, 100, img1_callback);
    //todo 两个双目
    ros::Subscriber sub_img2 = n.subscribe(IMAGE2_TOPIC, 100, img2_callback);
    ros::Subscriber sub_img3 = n.subscribe(IMAGE3_TOPIC, 100, img3_callback);

    //todo 两个单目
    ros::Subscriber sub_img4 = n.subscribe(IMAGE4_TOPIC, 100, img4_callback);
    ros::Subscriber sub_img5 = n.subscribe(IMAGE5_TOPIC, 100, img5_callback);

    std::thread sync_thread{sync_process};//融合
    ros::spin();

    return 0;
}
