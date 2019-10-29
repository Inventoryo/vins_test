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

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>

#include "utility/utility.h"

const std::string groundTruth_path = "/home/bb/output/groundTruth.csv";
Eigen::Vector3d latest_acc_0(0,0,9.8);
Eigen::Vector3d latest_gyr_0(0,0,0);
Eigen::Vector3d latest_P(0,0,0);
Eigen::Vector3d g(0,0,9.8);

Eigen::Vector3d latest_V(0,0,0);

Eigen::Matrix3d latest_Q = Eigen::Matrix3d::Identity();
double latest_time ;

nav_msgs::Path imu_path,odom_path;


ros::Publisher  pub_ground_truth_imu;
ros::Publisher  pub_ground_truth_odom;
ros::Subscriber sub_imu;
ros::Subscriber sub_odom;

Eigen::Vector3d constPimu(0,0,0);
Eigen::Vector3d Podom(0,0,0);

bool first_imu = true;
void fastPredictIMU(double t, Eigen::Vector3d linear_acceleration, Eigen::Vector3d angular_velocity, Eigen::Quaterniond q)
{

    if(first_imu){
        latest_time = t ;
        first_imu = false;
        latest_acc_0 = linear_acceleration;
    };

    double dt = t - latest_time;

    latest_time = t;
    Eigen::Vector3d un_acc_0 = latest_Q * latest_acc_0 - g;
    Eigen::Vector3d un_gyr = 0.5 * (latest_gyr_0 + angular_velocity) * dt;

    //Eigen::Matrix3d rotation_matrix3;
    //rotation_matrix3 = Eigen::AngleAxisd(un_gyr[0], Eigen::Vector3d::UnitX()) *
     //                  Eigen::AngleAxisd(un_gyr[1], Eigen::Vector3d::UnitY()) *
     //                  Eigen::AngleAxisd(un_gyr[2], Eigen::Vector3d::UnitZ());
    //latest_Q = latest_Q * rotation_matrix3;

    //latest_Q = latest_Q * Utility::deltaQ(un_gyr * dt);
    latest_Q = q.toRotationMatrix();


    Eigen::Vector3d un_acc_1 = latest_Q * linear_acceleration - g;


    Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);

    latest_P = latest_P + dt * latest_V + 0.5 * dt * dt * un_acc;
    latest_V = latest_V + dt * un_acc;

    latest_acc_0 = linear_acceleration;
    latest_gyr_0 = angular_velocity;


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

    Eigen::Quaterniond qd;
    qd.x() = imu_msg->orientation.x;
    qd.y() = imu_msg->orientation.y;
    qd.z() = imu_msg->orientation.z;
    qd.w() = imu_msg->orientation.w;
    Eigen::Vector3d acc(dx, dy, dz);

    Eigen::Vector3d gyr(rx, ry, rz);

    fastPredictIMU(t,acc,gyr,qd);

    geometry_msgs::PoseStamped pose_stamped;

    pose_stamped.header = imu_msg->header;
    pose_stamped.header.frame_id = "world";
    pose_stamped.pose.position.x = latest_P(0);
    pose_stamped.pose.position.y = latest_P(1);
    pose_stamped.pose.position.z = latest_P(2);

    imu_path.header = imu_msg->header;
    imu_path.header.frame_id = "world";
    imu_path.poses.push_back(pose_stamped);
    pub_ground_truth_imu.publish(imu_path);

}

void odom_callback(const nav_msgs::OdometryConstPtr &odom_msg){

    Podom(0) = odom_msg->pose.pose.position.x;
    Podom(1) = odom_msg->pose.pose.position.y;
    Podom(2) = odom_msg->pose.pose.position.z;
    Eigen::Quaterniond Q;
    Q.w() = odom_msg->pose.pose.orientation.w;
    Q.x() = odom_msg->pose.pose.orientation.x;
    Q.y() = odom_msg->pose.pose.orientation.y;
    Q.z() = odom_msg->pose.pose.orientation.z;


    Podom = Podom-constPimu;
    //std::cout<<"Podom = "<<Podom<<std::endl;
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header = odom_msg->header;
    pose_stamped.header.frame_id = "world";
    pose_stamped.pose.position.x = Podom(0);
    pose_stamped.pose.position.y = Podom(1);
    pose_stamped.pose.position.z = Podom(2);
    pose_stamped.pose.orientation.w = odom_msg->pose.pose.orientation.w;
    pose_stamped.pose.orientation.x = odom_msg->pose.pose.orientation.x;
    pose_stamped.pose.orientation.y = odom_msg->pose.pose.orientation.y;
    pose_stamped.pose.orientation.z = odom_msg->pose.pose.orientation.z;

    odom_path.header = odom_msg->header;
    odom_path.header.frame_id = "world";
    odom_path.poses.push_back(pose_stamped);
    pub_ground_truth_odom.publish(odom_path);

    //write to file
    std::ofstream output(groundTruth_path,std::ios::app);
    output.setf(std::ios::fixed, std::ios::floatfield);
    output<<odom_msg->header.stamp.toSec() << " ";
    output.precision(6);
    output << pose_stamped.pose.position.x << " "
           << pose_stamped.pose.position.y << " "
           << pose_stamped.pose.position.z << " "
           << pose_stamped.pose.orientation.x << " "
           << pose_stamped.pose.orientation.y << " "
           << pose_stamped.pose.orientation.z << " "
           << pose_stamped.pose.orientation.w  <<std::endl;
    output.close();
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "vins_estimator");
    ros::NodeHandle n("~");

    ROS_INFO("waiting for imu_perfect");
    sub_imu = n.subscribe("/prius/sensor_imu_in_6camera_perfectasd", 2000, imu_callback);//IMU
    sub_odom = n.subscribe("/base_pose_ground_truthasd",2000, odom_callback);//odom

    pub_ground_truth_imu = n.advertise<nav_msgs::Path>("/groundtruth_imu",100);
    pub_ground_truth_odom = n.advertise<nav_msgs::Path>("/groundtruth_odom",100);
    ros::spin();

    return 0;
}
