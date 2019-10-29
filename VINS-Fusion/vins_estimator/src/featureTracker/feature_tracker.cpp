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

void FeatureTracker::setMask()
{
    mask = cv::Mat(row, col, CV_8UC1, cv::Scalar(255));

    // prefer to keep features that are tracked for long time
    vector<pair<int, pair<cv::Point2f, int>>> cnt_pts_id;

    for (unsigned int i = 0; i < cur_pts.size(); i++)
        cnt_pts_id.push_back(make_pair(track_cnt[i], make_pair(cur_pts[i], ids[i])));

    sort(cnt_pts_id.begin(), cnt_pts_id.end(), [](const pair<int, pair<cv::Point2f, int>> &a, const pair<int, pair<cv::Point2f, int>> &b)
         {
            return a.first > b.first;
         });

    cur_pts.clear();
    ids.clear();
    track_cnt.clear();

    for (auto &it : cnt_pts_id)
    {
        if (mask.at<uchar>(it.second.first) == 255)
        {
            cur_pts.push_back(it.second.first);
            ids.push_back(it.second.second);
            track_cnt.push_back(it.first);
            cv::circle(mask, it.second.first, MIN_DIST, 0, -1);
        }
    }
}

void FeatureTracker::setMask2()
{
    mask = cv::Mat(row, col, CV_8UC1, cv::Scalar(255));

    // prefer to keep features that are tracked for long time
    vector<pair<int, pair<cv::Point2f, int>>> cnt_pts_id;

    for (unsigned int i = 0; i < cur_pts2.size(); i++)
        cnt_pts_id.push_back(make_pair(track_cnt2[i], make_pair(cur_pts2[i], ids2[i])));

    sort(cnt_pts_id.begin(), cnt_pts_id.end(), [](const pair<int, pair<cv::Point2f, int>> &a, const pair<int, pair<cv::Point2f, int>> &b)
    {
        return a.first > b.first;
    });

    cur_pts2.clear();
    ids2.clear();
    track_cnt2.clear();

    for (auto &it : cnt_pts_id)
    {
        if (mask.at<uchar>(it.second.first) == 255)
        {
            cur_pts2.push_back(it.second.first);
            ids2.push_back(it.second.second);
            track_cnt2.push_back(it.first);
            cv::circle(mask, it.second.first, MIN_DIST, 0, -1);
        }
    }
}

void FeatureTracker::setMask3()
{
    mask = cv::Mat(row, col, CV_8UC1, cv::Scalar(255));

    // prefer to keep features that are tracked for long time
    vector<pair<int, pair<cv::Point2f, int>>> cnt_pts_id;

    for (unsigned int i = 0; i < cur_pts3.size(); i++)
        cnt_pts_id.push_back(make_pair(track_cnt3[i], make_pair(cur_pts3[i], ids3[i])));

    sort(cnt_pts_id.begin(), cnt_pts_id.end(), [](const pair<int, pair<cv::Point2f, int>> &a, const pair<int, pair<cv::Point2f, int>> &b)
    {
        return a.first > b.first;
    });

    cur_pts3.clear();
    ids3.clear();
    track_cnt3.clear();

    for (auto &it : cnt_pts_id)
    {
        if (mask.at<uchar>(it.second.first) == 255)
        {
            cur_pts3.push_back(it.second.first);
            ids3.push_back(it.second.second);
            track_cnt3.push_back(it.first);
            cv::circle(mask, it.second.first, MIN_DIST, 0, -1);
        }
    }
}

void FeatureTracker::setMask4()
{
    mask = cv::Mat(row, col, CV_8UC1, cv::Scalar(255));

    // prefer to keep features that are tracked for long time
    vector<pair<int, pair<cv::Point2f, int>>> cnt_pts_id;

    for (unsigned int i = 0; i < cur_pts4.size(); i++)
        cnt_pts_id.push_back(make_pair(track_cnt4[i], make_pair(cur_pts4[i], ids4[i])));

    sort(cnt_pts_id.begin(), cnt_pts_id.end(), [](const pair<int, pair<cv::Point2f, int>> &a, const pair<int, pair<cv::Point2f, int>> &b)
    {
        return a.first > b.first;
    });

    cur_pts4.clear();
    ids4.clear();
    track_cnt4.clear();

    for (auto &it : cnt_pts_id)
    {
        if (mask.at<uchar>(it.second.first) == 255)
        {
            cur_pts4.push_back(it.second.first);
            ids4.push_back(it.second.second);
            track_cnt4.push_back(it.first);
            cv::circle(mask, it.second.first, MIN_DIST, 0, -1);
        }
    }
}

void FeatureTracker::addPoints()
{
    for (auto &p : n_pts)
    {
        cur_pts.push_back(p);
        ids.push_back(n_id++);
        track_cnt.push_back(1);
    }
}

double FeatureTracker::distance(cv::Point2f &pt1, cv::Point2f &pt2)
{
    //printf("pt1: %f %f pt2: %f %f\n", pt1.x, pt1.y, pt2.x, pt2.y);
    double dx = pt1.x - pt2.x;
    double dy = pt1.y - pt2.y;
    return sqrt(dx * dx + dy * dy);
}

map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> FeatureTracker::trackImage(double _cur_time, const cv::Mat &_img, const cv::Mat &_img1)
{
    TicToc t_r;
    cur_time = _cur_time;
    cur_img = _img;
    row = cur_img.rows;
    col = cur_img.cols;
    cv::Mat rightImg = _img1;
    /*
    {
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
        clahe->apply(cur_img, cur_img);
        if(!rightImg.empty())
            clahe->apply(rightImg, rightImg);
    }
    */
    cur_pts.clear();

    if (prev_pts.size() > 0)//ywc 之前的特征点
    {
        TicToc t_o;
        vector<uchar> status;
        vector<float> err;
        if(hasPrediction)//如果有prediction,則用cv::TermCriteria方式進行特徵跟蹤
        {
            cur_pts = predict_pts;//ywc 当前帧的特征点
            cv::calcOpticalFlowPyrLK(prev_img, cur_img, prev_pts, cur_pts, status, err, cv::Size(21, 21), 1, 
            cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01), cv::OPTFLOW_USE_INITIAL_FLOW);//光流跟踪,左图与前一时刻
            
            int succ_num = 0;
            for (size_t i = 0; i < status.size(); i++)
            {
                if (status[i])
                    succ_num++;
            }
            if (succ_num < 10)
               cv::calcOpticalFlowPyrLK(prev_img, cur_img, prev_pts, cur_pts, status, err, cv::Size(21, 21), 3);
        }
        else
            cv::calcOpticalFlowPyrLK(prev_img, cur_img, prev_pts, cur_pts, status, err, cv::Size(21, 21), 3);
        // reverse check
        if(FLOW_BACK)
        {
            vector<uchar> reverse_status;
            vector<cv::Point2f> reverse_pts = prev_pts;
            cv::calcOpticalFlowPyrLK(cur_img, prev_img, cur_pts, reverse_pts, reverse_status, err, cv::Size(21, 21), 1, 
            cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01), cv::OPTFLOW_USE_INITIAL_FLOW);
            //cv::calcOpticalFlowPyrLK(cur_img, prev_img, cur_pts, reverse_pts, reverse_status, err, cv::Size(21, 21), 3); 
            for(size_t i = 0; i < status.size(); i++)
            {
                if(status[i] && reverse_status[i] && distance(prev_pts[i], reverse_pts[i]) <= 0.5)
                {
                    status[i] = 1;
                }
                else
                    status[i] = 0;
            }
        }
        
        for (int i = 0; i < int(cur_pts.size()); i++)
            if (status[i] && !inBorder(cur_pts[i]))//特征点在当前帧视野内
                status[i] = 0;
        reduceVector(prev_pts, status);//根据status的情况，去掉prev_pts中跟踪失败的点
        reduceVector(cur_pts, status);
        reduceVector(ids, status);
        reduceVector(track_cnt, status);
        ROS_DEBUG("temporal optical flow costs: %fms", t_o.toc());
        //printf("track cnt %d\n", (int)ids.size());
    }

    for (auto &n : track_cnt)//每個有效跟踪的点，跟踪数++
        n++;

    if (1)
    {
        //rejectWithF();
        ROS_DEBUG("set mask begins");
        TicToc t_m;
        setMask();
        ROS_DEBUG("set mask costs %fms", t_m.toc());

        ROS_DEBUG("detect feature begins");
        TicToc t_t;
        int n_max_cnt = MAX_CNT - static_cast<int>(cur_pts.size());
        if (n_max_cnt > 0)
        {
            if(mask.empty())
                cout << "mask is empty " << endl;
            if (mask.type() != CV_8UC1)
                cout << "mask type wrong " << endl;
            cv::goodFeaturesToTrack(cur_img, n_pts, MAX_CNT - cur_pts.size(), 0.01, MIN_DIST, mask);
        }//极大值抑制
        else
            n_pts.clear();
        ROS_DEBUG("detect feature costs: %fms", t_t.toc());

        ROS_DEBUG("add feature begins");
        TicToc t_a;
        addPoints();//极大值抑制，并赋予特征点ID，存入ids
        ROS_DEBUG("selectFeature costs: %fms", t_a.toc());
    }

    cur_un_pts = undistortedPts(cur_pts, m_camera[0]);//投影到归一化平面
    pts_velocity = ptsVelocity(ids, cur_un_pts, cur_un_pts_map, prev_un_pts_map);//计算特征点在归一化平面上的移动速度，并更新cur_un_pts_map

    if(!_img1.empty() && stereo_cam)//左右光流跟踪
    {
        ids_right.clear();
        cur_right_pts.clear();
        cur_un_right_pts.clear();
        right_pts_velocity.clear();
        cur_un_right_pts_map.clear();
        if(!cur_pts.empty())
        {
            //printf("stereo image; track feature on right image\n");
            vector<cv::Point2f> reverseLeftPts;
            vector<uchar> status, statusRightLeft;
            vector<float> err;
            // cur left ---- cur right
            cv::calcOpticalFlowPyrLK(cur_img, rightImg, cur_pts, cur_right_pts, status, err, cv::Size(21, 21), 3);//光流跟踪,左图与右图
            // reverse check cur right ---- cur left
            if(FLOW_BACK)
            {
                cv::calcOpticalFlowPyrLK(rightImg, cur_img, cur_right_pts, reverseLeftPts, statusRightLeft, err, cv::Size(21, 21), 3);
                for(size_t i = 0; i < status.size(); i++)
                {
                    if(status[i] && statusRightLeft[i] && inBorder(cur_right_pts[i]) && distance(cur_pts[i], reverseLeftPts[i]) <= 0.5)
                        status[i] = 1;
                    else
                        status[i] = 0;
                }
            }

            ids_right = ids;//不在右相机提取新的特征点
            reduceVector(cur_right_pts, status);
            reduceVector(ids_right, status);
            // only keep left-right pts
            /*
            reduceVector(cur_pts, status);
            reduceVector(ids, status);
            reduceVector(track_cnt, status);
            reduceVector(cur_un_pts, status);
            reduceVector(pts_velocity, status);
            */
            cur_un_right_pts = undistortedPts(cur_right_pts, m_camera[1]);
            right_pts_velocity = ptsVelocity(ids_right, cur_un_right_pts, cur_un_right_pts_map, prev_un_right_pts_map);
        }
        prev_un_right_pts_map = cur_un_right_pts_map;
    }
    if(SHOW_TRACK)
        drawTrack(cur_img, rightImg, ids, cur_pts, cur_right_pts, prevLeftPtsMap);

    prev_img = cur_img;
    prev_pts = cur_pts;
    prev_un_pts = cur_un_pts;
    prev_un_pts_map = cur_un_pts_map;
    prev_time = cur_time;
    hasPrediction = false;

    prevLeftPtsMap.clear();
    for(size_t i = 0; i < cur_pts.size(); i++)
        prevLeftPtsMap[ids[i]] = cur_pts[i];

    map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> featureFrame;//map<int:特征点;vector中有两个元素，特征点在左相机，特征点在右相机
    for (size_t i = 0; i < ids.size(); i++)
    {
        int feature_id = ids[i];
        double x, y ,z;
        x = cur_un_pts[i].x;
        y = cur_un_pts[i].y;
        z = 1;
        double p_u, p_v;
        p_u = cur_pts[i].x;
        p_v = cur_pts[i].y;
        int camera_id = 0;
        double velocity_x, velocity_y;
        velocity_x = pts_velocity[i].x;
        velocity_y = pts_velocity[i].y;

        Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
        xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;//归一化平面坐标，像素坐标，像素运动速度
        featureFrame[feature_id].emplace_back(camera_id,  xyz_uv_velocity);//特征点在左相机信息
    }

    if (!_img1.empty() && stereo_cam)
    {
        for (size_t i = 0; i < ids_right.size(); i++)
        {
            int feature_id = ids_right[i];
            double x, y ,z;
            x = cur_un_right_pts[i].x;
            y = cur_un_right_pts[i].y;
            z = 1;
            double p_u, p_v;
            p_u = cur_right_pts[i].x;
            p_v = cur_right_pts[i].y;
            int camera_id = 1;
            double velocity_x, velocity_y;
            velocity_x = right_pts_velocity[i].x;
            velocity_y = right_pts_velocity[i].y;

            Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
            xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
            featureFrame[feature_id].emplace_back(camera_id,  xyz_uv_velocity);//同时被左右相机追踪到的点
        }
    }

    //printf("feature track whole time %f\n", t_r.toc());
    return featureFrame;
}

//todo 增加对多个双目的支持
map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> FeatureTracker::trackImage(double _cur_time, const cv::Mat &_img, const cv::Mat &_img1, const cv::Mat &_img2, const cv::Mat &_img3)
{
    TicToc t_r;
    cur_time = _cur_time;
    cur_img = _img;
    row = cur_img.rows;
    col = cur_img.cols;
    cv::Mat rightImg = _img1;
    /*
    {
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
        clahe->apply(cur_img, cur_img);
        if(!rightImg.empty())
            clahe->apply(rightImg, rightImg);
    }
    */
    cur_pts.clear();

    if (prev_pts.size() > 0)//ywc 之前的特征点
    {
        TicToc t_o;
        vector<uchar> status;
        vector<float> err;
        if(hasPrediction)
        {
            cur_pts = predict_pts;//ywc 当前帧的特征点
            cv::calcOpticalFlowPyrLK(prev_img, cur_img, prev_pts, cur_pts, status, err, cv::Size(21, 21), 1,
                                     cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01), cv::OPTFLOW_USE_INITIAL_FLOW);//光流跟踪,左图与前一时刻

            int succ_num = 0;
            for (size_t i = 0; i < status.size(); i++)
            {
                if (status[i])
                    succ_num++;
            }
            if (succ_num < 10)
                cv::calcOpticalFlowPyrLK(prev_img, cur_img, prev_pts, cur_pts, status, err, cv::Size(21, 21), 3);
        }
        else
            cv::calcOpticalFlowPyrLK(prev_img, cur_img, prev_pts, cur_pts, status, err, cv::Size(21, 21), 3);
        // reverse check
        if(FLOW_BACK)
        {
            vector<uchar> reverse_status;
            vector<cv::Point2f> reverse_pts = prev_pts;
            cv::calcOpticalFlowPyrLK(cur_img, prev_img, cur_pts, reverse_pts, reverse_status, err, cv::Size(21, 21), 1,
                                     cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01), cv::OPTFLOW_USE_INITIAL_FLOW);
            //cv::calcOpticalFlowPyrLK(cur_img, prev_img, cur_pts, reverse_pts, reverse_status, err, cv::Size(21, 21), 3);
            for(size_t i = 0; i < status.size(); i++)
            {
                if(status[i] && reverse_status[i] && distance(prev_pts[i], reverse_pts[i]) <= 0.5)
                {
                    status[i] = 1;
                }
                else
                    status[i] = 0;
            }
        }

        for (int i = 0; i < int(cur_pts.size()); i++)
            if (status[i] && !inBorder(cur_pts[i]))
                status[i] = 0;
        reduceVector(prev_pts, status);
        reduceVector(cur_pts, status);
        reduceVector(ids, status);
        reduceVector(track_cnt, status);
        ROS_DEBUG("temporal optical flow costs: %fms", t_o.toc());
        //printf("track cnt %d\n", (int)ids.size());
    }

    for (auto &n : track_cnt)//特征点被跟踪次数
        n++;

    if (1)
    {
        //rejectWithF();
        ROS_DEBUG("set mask begins");
        TicToc t_m;
        setMask();
        ROS_DEBUG("set mask costs %fms", t_m.toc());

        ROS_DEBUG("detect feature begins");
        TicToc t_t;
        int n_max_cnt = MAX_CNT - static_cast<int>(cur_pts.size());
        if (n_max_cnt > 0)
        {
            if(mask.empty())
                cout << "mask is empty " << endl;
            if (mask.type() != CV_8UC1)
                cout << "mask type wrong " << endl;
            cv::goodFeaturesToTrack(cur_img, n_pts, MAX_CNT - cur_pts.size(), 0.01, MIN_DIST, mask);
        }//极大值抑制
        else
            n_pts.clear();
        ROS_DEBUG("detect feature costs: %fms", t_t.toc());

        ROS_DEBUG("add feature begins");
        TicToc t_a;
        addPoints();
        ROS_DEBUG("selectFeature costs: %fms", t_a.toc());
    }

    cur_un_pts = undistortedPts(cur_pts, m_camera[0]);
    pts_velocity = ptsVelocity(ids, cur_un_pts, cur_un_pts_map, prev_un_pts_map);

    if(!_img1.empty() && stereo_cam)
    {
        ids_right.clear();
        cur_right_pts.clear();
        cur_un_right_pts.clear();
        right_pts_velocity.clear();
        cur_un_right_pts_map.clear();
        if(!cur_pts.empty())
        {
            //printf("stereo image; track feature on right image\n");
            vector<cv::Point2f> reverseLeftPts;
            vector<uchar> status, statusRightLeft;
            vector<float> err;
            // cur left ---- cur right
            cv::calcOpticalFlowPyrLK(cur_img, rightImg, cur_pts, cur_right_pts, status, err, cv::Size(21, 21), 3);//光流跟踪,左图与右图
            // reverse check cur right ---- cur left
            if(FLOW_BACK)
            {
                cv::calcOpticalFlowPyrLK(rightImg, cur_img, cur_right_pts, reverseLeftPts, statusRightLeft, err, cv::Size(21, 21), 3);
                for(size_t i = 0; i < status.size(); i++)
                {
                    if(status[i] && statusRightLeft[i] && inBorder(cur_right_pts[i]) && distance(cur_pts[i], reverseLeftPts[i]) <= 0.5)
                        status[i] = 1;
                    else
                        status[i] = 0;
                }
            }

            ids_right = ids;
            reduceVector(cur_right_pts, status);
            reduceVector(ids_right, status);
            // only keep left-right pts
            /*
            reduceVector(cur_pts, status);
            reduceVector(ids, status);
            reduceVector(track_cnt, status);
            reduceVector(cur_un_pts, status);
            reduceVector(pts_velocity, status);
            */
            cur_un_right_pts = undistortedPts(cur_right_pts, m_camera[1]);
            right_pts_velocity = ptsVelocity(ids_right, cur_un_right_pts, cur_un_right_pts_map, prev_un_right_pts_map);
        }
        prev_un_right_pts_map = cur_un_right_pts_map;
    }
    //if(SHOW_TRACK)
        //drawTrack(cur_img, rightImg, ids, cur_pts, cur_right_pts, prevLeftPtsMap);

    prev_img = cur_img;
    prev_pts = cur_pts;
    prev_un_pts = cur_un_pts;
    prev_un_pts_map = cur_un_pts_map;



    prevLeftPtsMap.clear();
    for(size_t i = 0; i < cur_pts.size(); i++)
        prevLeftPtsMap[ids[i]] = cur_pts[i];

    map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> featureFrame;
    for (size_t i = 0; i < ids.size(); i++)
    {
        int feature_id = ids[i];
        double x, y ,z;
        x = cur_un_pts[i].x;
        y = cur_un_pts[i].y;
        z = 1;
        double p_u, p_v;
        p_u = cur_pts[i].x;
        p_v = cur_pts[i].y;
        int camera_id = 0;
        double velocity_x, velocity_y;
        velocity_x = pts_velocity[i].x;
        velocity_y = pts_velocity[i].y;

        Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
        xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;//归一化平面坐标，像素坐标，像素运动速度
        featureFrame[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
    }

    if (!_img1.empty() && stereo_cam)
    {
        for (size_t i = 0; i < ids_right.size(); i++)
        {
            int feature_id = ids_right[i];
            double x, y ,z;
            x = cur_un_right_pts[i].x;
            y = cur_un_right_pts[i].y;
            z = 1;
            double p_u, p_v;
            p_u = cur_right_pts[i].x;
            p_v = cur_right_pts[i].y;
            int camera_id = 1;
            double velocity_x, velocity_y;
            velocity_x = right_pts_velocity[i].x;
            velocity_y = right_pts_velocity[i].y;

            Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
            xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
            featureFrame[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
        }
    }

    //printf("feature track whole time %f\n", t_r.toc());
    //return featureFrame;

    //todo 第二个双目处理
    //std::cout<<"about to proccess another stereo"<<std::endl;
    cur_img2 = _img2;
    row = cur_img2.rows;
    col = cur_img2.cols;

    cv::Mat rightImg2 = _img3;

    cur_pts2.clear();

    if (prev_pts2.size() > 0)//ywc 之前的特征点
    {
        TicToc t_o;
        vector<uchar> status;
        vector<float> err;
        if(hasPrediction)
        {
            cur_pts2 = predict_pts2;//ywc 当前帧的特征点
            cv::calcOpticalFlowPyrLK(prev_img2, cur_img2, prev_pts2, cur_pts2, status, err, cv::Size(21, 21), 1,
                                     cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01), cv::OPTFLOW_USE_INITIAL_FLOW);//光流跟踪,左图与前一时刻

            int succ_num = 0;
            for (size_t i = 0; i < status.size(); i++)
            {
                if (status[i])
                    succ_num++;
            }
            if (succ_num < 10)
                cv::calcOpticalFlowPyrLK(prev_img2, cur_img2, prev_pts2, cur_pts2, status, err, cv::Size(21, 21), 3);
        }
        else
            cv::calcOpticalFlowPyrLK(prev_img2, cur_img2, prev_pts2, cur_pts2, status, err, cv::Size(21, 21), 3);
        // reverse check
        if(FLOW_BACK)
        {
            vector<uchar> reverse_status;
            vector<cv::Point2f> reverse_pts = prev_pts2;
            cv::calcOpticalFlowPyrLK(cur_img2, prev_img2, cur_pts2, reverse_pts, reverse_status, err, cv::Size(21, 21), 1,
                                     cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01), cv::OPTFLOW_USE_INITIAL_FLOW);
            //cv::calcOpticalFlowPyrLK(cur_img2, prev_img, cur_pts, reverse_pts, reverse_status, err, cv::Size(21, 21), 3);
            for(size_t i = 0; i < status.size(); i++)
            {
                if(status[i] && reverse_status[i] && distance(prev_pts2[i], reverse_pts[i]) <= 0.5)
                {
                    status[i] = 1;
                }
                else
                    status[i] = 0;
            }
        }

        for (int i = 0; i < int(cur_pts2.size()); i++)
            if (status[i] && !inBorder(cur_pts2[i]))
                status[i] = 0;
        reduceVector(prev_pts2, status);
        reduceVector(cur_pts2, status);
        reduceVector(ids2, status);
        reduceVector(track_cnt2, status);
        ROS_DEBUG("temporal optical flow costs: %fms", t_o.toc());
        //printf("track cnt %d\n", (int)ids2.size());
    }

    for (auto &n : track_cnt2)//
        n++;

    if (1)
    {
        //rejectWithF();
        ROS_DEBUG("set mask begins");
        TicToc t_m;
        setMask2();
        ROS_DEBUG("set mask costs %fms", t_m.toc());

        ROS_DEBUG("detect feature begins");
        TicToc t_t;
        int n_max_cnt = MAX_CNT - static_cast<int>(cur_pts2.size());
        if (n_max_cnt > 0)
        {
            if(mask.empty())
                cout << "mask is empty " << endl;
            if (mask.type() != CV_8UC1)
                cout << "mask type wrong " << endl;
            cv::goodFeaturesToTrack(cur_img2, n_pts2, MAX_CNT - cur_pts2.size(), 0.01, MIN_DIST, mask);
        }//极大值抑制
        else
            n_pts2.clear();
        ROS_DEBUG("detect feature costs: %fms", t_t.toc());

        ROS_DEBUG("add feature begins");
        TicToc t_a;
        for (auto &p : n_pts2)
        {
            cur_pts2.push_back(p);
            ids2.push_back(n_id++);
            track_cnt2.push_back(1);
        }
        ROS_DEBUG("selectFeature costs: %fms", t_a.toc());
    }

    cur_un_pts2 = undistortedPts(cur_pts2, m_camera[2]);
    pts_velocity2 = ptsVelocity(ids2, cur_un_pts2, cur_un_pts_map2, prev_un_pts_map2);

    if(!_img3.empty() && stereo_cam)
    {
        ids_right2.clear();
        cur_right_pts2.clear();
        cur_un_right_pts2.clear();
        right_pts_velocity2.clear();
        cur_un_right_pts_map2.clear();
        if(!cur_pts2.empty())
        {
            //printf("stereo image; track feature on right image\n");
            vector<cv::Point2f> reverseLeftPts;
            vector<uchar> status, statusRightLeft;
            vector<float> err;
            // cur left ---- cur right
            cv::calcOpticalFlowPyrLK(cur_img2, rightImg2, cur_pts2, cur_right_pts2, status, err, cv::Size(21, 21), 3);//光流跟踪,左图与右图
            // reverse check cur right ---- cur left
            if(FLOW_BACK)
            {
                cv::calcOpticalFlowPyrLK(rightImg2, cur_img2, cur_right_pts2, reverseLeftPts, statusRightLeft, err, cv::Size(21, 21), 3);
                for(size_t i = 0; i < status.size(); i++)
                {
                    if(status[i] && statusRightLeft[i] && inBorder(cur_right_pts2[i]) && distance(cur_pts2[i], reverseLeftPts[i]) <= 0.5)
                        status[i] = 1;
                    else
                        status[i] = 0;
                }
            }

            ids_right2 = ids2;
            reduceVector(cur_right_pts2, status);
            reduceVector(ids_right2, status);

            cur_un_right_pts2 = undistortedPts(cur_right_pts2, m_camera[3]);
            right_pts_velocity2 = ptsVelocity(ids_right2, cur_un_right_pts2, cur_un_right_pts_map2, prev_un_right_pts_map2);
        }
        prev_un_right_pts_map2 = cur_un_right_pts_map2;
    }
    if(SHOW_TRACK)
        drawTrack(cur_img2, rightImg2, ids, cur_pts2, cur_right_pts2, prevLeftPtsMap2);

    prev_img2 = cur_img2;
    prev_pts2 = cur_pts2;
    prev_un_pts2 = cur_un_pts2;
    prev_un_pts_map2 = cur_un_pts_map2;

    hasPrediction = false;

    prevLeftPtsMap2.clear();
    for(size_t i = 0; i < cur_pts2.size(); i++)
        prevLeftPtsMap2[ids2[i]] = cur_pts2[i];

    //std::cout<<"about to transform to the front left camera"<<std::endl;
    for (size_t i = 0; i < ids2.size(); i++)
    {
        int feature_id = ids2[i];
        double x, y ,z;
        x = cur_un_pts2[i].x;
        y = cur_un_pts2[i].y;
        z = 1;
        double p_u, p_v;
        p_u = cur_pts2[i].x;
        p_v = cur_pts2[i].y;
        int camera_id = 2;//第二个双目左
        double velocity_x, velocity_y;
        velocity_x = pts_velocity2[i].x;
        velocity_y = pts_velocity2[i].y;

        //todo 将特征点信息统一到一个双目下，左相机
        /*
        Eigen::Vector3d xyz2(x,y,z);
        Eigen::Vector3d xyz1;
        Eigen::Vector3d Tic00;
        Tic00(0) = TIC[0](1);
        Tic00(1) = TIC[0](2);
        Tic00(2) = -TIC[0](0);
        //std::cout<<"xyz in cam2 "<<xyz2<<std::endl;
        xyz1 = RIC[2] * xyz2 + TIC[2];
        //std::cout<<"xyz in IMU "<<xyz1<<std::endl;

        xyz1 = RIC[0].inverse()*xyz1 +Tic00;
        //std::cout<<"xyz in cam0 "<<xyz1<<std::endl;
        //xyz1 = RIC[0].inverse() * (RIC[2] * xyz2 - TIC[2]) + TIC[0];
        x = xyz1(0);
        y = xyz1(1);
        z = xyz1(2);
        //std::cout<<"xyz transformation success "<<xyz1<<std::endl;

        Eigen::Vector2d puv1;
        m_camera[0]->spaceToPlane(xyz1,puv1);
        p_u = puv1(0);
        p_v = puv1(1);
        //std::cout<<"pu,pv transformation success "<<puv1<<std::endl;

        Eigen::Vector3d vxy2(velocity_x,velocity_y,1);
        Eigen::Vector3d vxy1;
        vector<double> param0,param2;
        m_camera[2]->writeParameters(param2);
        m_camera[0]->writeParameters(param0);
        vxy2(0) = vxy2(0)/param2[4];
        vxy2(1) = vxy2(1)/param2[5];
        vxy1 = RIC[0].inverse()*RIC[2]*vxy2;
        vxy1(0) = vxy1(0)*param0[4];
        vxy1(1) = vxy1(0)*param0[5];
        velocity_x = vxy1(0);
        velocity_y = vxy1(1);
        //std::cout<<"velocity transformation success "<<vxy1<<std::endl;
        */
        Eigen::Matrix<double, 7, 1> xyz_uv_velocity;

        xyz_uv_velocity << x, y, z, p_u, p_v, -velocity_x, velocity_y;//归一化平面坐标，像素坐标，像素运动速度,左相机
        //std::cout<<"xyz_uv_velocity" << xyz_uv_velocity<< std::endl;
        featureFrame[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
    }
    //std::cout<<"about to transform to the front right camera"<<std::endl;
    if (!_img3.empty() && stereo_cam)
    {
        for (size_t i = 0; i < ids_right2.size(); i++)
        {
            int feature_id = ids_right2[i];
            double x, y ,z;
            x = cur_un_right_pts2[i].x;
            y = cur_un_right_pts2[i].y;
            z = 1;
            double p_u, p_v;
            p_u = cur_right_pts2[i].x;
            p_v = cur_right_pts2[i].y;
            int camera_id = 3;//第二个双目右
            double velocity_x, velocity_y;
            velocity_x = right_pts_velocity2[i].x;
            velocity_y = right_pts_velocity2[i].y;


            //todo 将特征点信息统一到一个双目下,右相机
            /*
            Eigen::Vector3d xyz2(x,y,z);
            Eigen::Vector3d xyz1;
            Eigen::Vector3d Tic1;
            Tic1(0) = TIC[1](1);
            Tic1(1) = TIC[1](2);
            Tic1(2) = -TIC[1](0);
            //std::cout<<"xyz in cam2 "<<xyz2<<std::endl;
            xyz1 = RIC[3] * xyz2 + TIC[3];
            //std::cout<<"xyz in IMU "<<xyz1<<std::endl;
            xyz1 = RIC[1].inverse()*xyz1 +Tic1;
            //std::cout<<"xyz in cam0 "<<xyz1<<std::endl;
            x = xyz1(0);
            y = xyz1(1);
            z = xyz1(2);

            Eigen::Vector2d puv1;
            m_camera[1]->spaceToPlane(xyz1,puv1);
            p_u = puv1(0);
            p_v = puv1(1);

            Eigen::Vector3d vxy2(velocity_x,velocity_y,1);
            Eigen::Vector3d vxy1;
            vector<double> param1,param3;
            m_camera[3].get()->writeParameters(param3);
            m_camera[1].get()->writeParameters(param1);
            vxy2(0) = vxy2(0)/param3[4];
            vxy2(1) = vxy2(1)/param3[5];
            vxy1 = RIC[0].inverse()*RIC[2]*vxy2;
            vxy1(0) = vxy1(0)*param1[4];
            vxy1(1) = vxy1(0)*param1[5];
            velocity_x = vxy1(0);
            velocity_y = vxy1(1);
            */

            Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
            xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
            featureFrame[feature_id].emplace_back(camera_id,  xyz_uv_velocity);//右相机
        }
    }

    hasPrediction = false;
    prev_time = cur_time;
    //printf("feature track whole time %f\n", t_r.toc());
    return featureFrame;
}

//todo 六目
map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> FeatureTracker::trackImage(double _cur_time, const cv::Mat &_img, const cv::Mat &_img1, const cv::Mat &_img2, const cv::Mat &_img3, const cv::Mat &_img4, const cv::Mat &_img5)
{
    TicToc t_r;
    cur_time = _cur_time;

    //todo 第一，二相机
    cur_img = _img;
    row = cur_img.rows;
    col = cur_img.cols;
    cv::Mat rightImg = _img1;
    /*
    {
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
        clahe->apply(cur_img, cur_img);
        if(!rightImg.empty())
            clahe->apply(rightImg, rightImg);
    }
    */
    cur_pts.clear();

    if (prev_pts.size() > 0)//ywc 之前的特征点
    {
        TicToc t_o;
        vector<uchar> status;
        vector<float> err;
        if(hasPrediction)
        {
            cur_pts = predict_pts;//ywc 当前帧的特征点
            cv::calcOpticalFlowPyrLK(prev_img, cur_img, prev_pts, cur_pts, status, err, cv::Size(21, 21), 1,
                                     cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01), cv::OPTFLOW_USE_INITIAL_FLOW);//光流跟踪,左图与前一时刻

            int succ_num = 0;
            for (size_t i = 0; i < status.size(); i++)
            {
                if (status[i])
                    succ_num++;
            }
            if (succ_num < 10)
                cv::calcOpticalFlowPyrLK(prev_img, cur_img, prev_pts, cur_pts, status, err, cv::Size(21, 21), 3);
        }
        else
            cv::calcOpticalFlowPyrLK(prev_img, cur_img, prev_pts, cur_pts, status, err, cv::Size(21, 21), 3);
        // reverse check
        if(FLOW_BACK)
        {
            vector<uchar> reverse_status;
            vector<cv::Point2f> reverse_pts = prev_pts;
            cv::calcOpticalFlowPyrLK(cur_img, prev_img, cur_pts, reverse_pts, reverse_status, err, cv::Size(21, 21), 1,
                                     cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01), cv::OPTFLOW_USE_INITIAL_FLOW);
            //cv::calcOpticalFlowPyrLK(cur_img, prev_img, cur_pts, reverse_pts, reverse_status, err, cv::Size(21, 21), 3);
            for(size_t i = 0; i < status.size(); i++)
            {
                if(status[i] && reverse_status[i] && distance(prev_pts[i], reverse_pts[i]) <= 0.5)
                {
                    status[i] = 1;
                }
                else
                    status[i] = 0;
            }
        }

        for (int i = 0; i < int(cur_pts.size()); i++)
            if (status[i] && !inBorder(cur_pts[i]))
                status[i] = 0;
        reduceVector(prev_pts, status);
        reduceVector(cur_pts, status);
        reduceVector(ids, status);
        reduceVector(track_cnt, status);
        ROS_DEBUG("temporal optical flow costs: %fms", t_o.toc());
        //printf("track cnt %d\n", (int)ids.size());
    }

    for (auto &n : track_cnt)//特征点被跟踪次数
        n++;

    if (1)
    {
        //rejectWithF();
        ROS_DEBUG("set mask begins");
        TicToc t_m;
        setMask();
        ROS_DEBUG("set mask costs %fms", t_m.toc());

        ROS_DEBUG("detect feature begins");
        TicToc t_t;
        int n_max_cnt = MAX_CNT - static_cast<int>(cur_pts.size());
        if (n_max_cnt > 0)
        {
            if(mask.empty())
                cout << "mask is empty " << endl;
            if (mask.type() != CV_8UC1)
                cout << "mask type wrong " << endl;
            cv::goodFeaturesToTrack(cur_img, n_pts, MAX_CNT - cur_pts.size(), 0.01, MIN_DIST, mask);
        }//极大值抑制
        else
            n_pts.clear();
        ROS_DEBUG("detect feature costs: %fms", t_t.toc());

        ROS_DEBUG("add feature begins");
        TicToc t_a;
        addPoints();
        ROS_DEBUG("selectFeature costs: %fms", t_a.toc());
    }

    cur_un_pts = undistortedPts(cur_pts, m_camera[0]);
    pts_velocity = ptsVelocity(ids, cur_un_pts, cur_un_pts_map, prev_un_pts_map);

    if(!_img1.empty() && stereo_cam)
    {
        ids_right.clear();
        cur_right_pts.clear();
        cur_un_right_pts.clear();
        right_pts_velocity.clear();
        cur_un_right_pts_map.clear();
        if(!cur_pts.empty())
        {
            //printf("stereo image; track feature on right image\n");
            vector<cv::Point2f> reverseLeftPts;
            vector<uchar> status, statusRightLeft;
            vector<float> err;
            // cur left ---- cur right
            cv::calcOpticalFlowPyrLK(cur_img, rightImg, cur_pts, cur_right_pts, status, err, cv::Size(21, 21), 3);//光流跟踪,左图与右图
            // reverse check cur right ---- cur left
            if(FLOW_BACK)
            {
                cv::calcOpticalFlowPyrLK(rightImg, cur_img, cur_right_pts, reverseLeftPts, statusRightLeft, err, cv::Size(21, 21), 3);
                for(size_t i = 0; i < status.size(); i++)
                {
                    if(status[i] && statusRightLeft[i] && inBorder(cur_right_pts[i]) && distance(cur_pts[i], reverseLeftPts[i]) <= 0.5)
                        status[i] = 1;
                    else
                        status[i] = 0;
                }
            }

            ids_right = ids;
            reduceVector(cur_right_pts, status);
            reduceVector(ids_right, status);
            // only keep left-right pts
            /*
            reduceVector(cur_pts, status);
            reduceVector(ids, status);
            reduceVector(track_cnt, status);
            reduceVector(cur_un_pts, status);
            reduceVector(pts_velocity, status);
            */
            cur_un_right_pts = undistortedPts(cur_right_pts, m_camera[1]);
            right_pts_velocity = ptsVelocity(ids_right, cur_un_right_pts, cur_un_right_pts_map, prev_un_right_pts_map);
        }
        prev_un_right_pts_map = cur_un_right_pts_map;
    }
    //if(SHOW_TRACK)
        //drawTrack(cur_img, rightImg, ids, cur_pts, cur_right_pts, prevLeftPtsMap);

    prev_img = cur_img;
    prev_pts = cur_pts;
    prev_un_pts = cur_un_pts;
    prev_un_pts_map = cur_un_pts_map;
    prevLeftPtsMap.clear();
    for(size_t i = 0; i < cur_pts.size(); i++)
        prevLeftPtsMap[ids[i]] = cur_pts[i];

    map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> featureFrame;
    for (size_t i = 0; i < ids.size(); i++)
    {
        int feature_id = ids[i];
        double x, y ,z;
        x = cur_un_pts[i].x;
        y = cur_un_pts[i].y;
        z = 1;
        double p_u, p_v;
        p_u = cur_pts[i].x;
        p_v = cur_pts[i].y;
        int camera_id = 0;
        double velocity_x, velocity_y;
        velocity_x = pts_velocity[i].x;
        velocity_y = pts_velocity[i].y;

        Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
        xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;//归一化平面坐标，像素坐标，像素运动速度
        featureFrame[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
    }

    if (!_img1.empty() && stereo_cam)
    {
        for (size_t i = 0; i < ids_right.size(); i++)
        {
            int feature_id = ids_right[i];
            double x, y ,z;
            x = cur_un_right_pts[i].x;
            y = cur_un_right_pts[i].y;
            z = 1;
            double p_u, p_v;
            p_u = cur_right_pts[i].x;
            p_v = cur_right_pts[i].y;
            int camera_id = 1;
            double velocity_x, velocity_y;
            velocity_x = right_pts_velocity[i].x;
            velocity_y = right_pts_velocity[i].y;

            Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
            xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
            featureFrame[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
        }
    }

    //printf("feature track whole time %f\n", t_r.toc());
    //return featureFrame;

    //todo 第三，四相机
    cur_img2 = _img2;
    row = cur_img2.rows;
    col = cur_img2.cols;
    cv::Mat rightImg2 = _img3;
    cur_pts2.clear();

    if (prev_pts2.size() > 0)//ywc 之前的特征点
    {
        TicToc t_o;
        vector<uchar> status;
        vector<float> err;
        if(hasPrediction)
        {
            cur_pts2 = predict_pts2;//ywc 当前帧的特征点
            cv::calcOpticalFlowPyrLK(prev_img2, cur_img2, prev_pts2, cur_pts2, status, err, cv::Size(21, 21), 1,
                                     cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01), cv::OPTFLOW_USE_INITIAL_FLOW);//光流跟踪,左图与前一时刻

            int succ_num = 0;
            for (size_t i = 0; i < status.size(); i++)
            {
                if (status[i])
                    succ_num++;
            }
            if (succ_num < 10)
                cv::calcOpticalFlowPyrLK(prev_img2, cur_img2, prev_pts2, cur_pts2, status, err, cv::Size(21, 21), 3);
        }
        else
            cv::calcOpticalFlowPyrLK(prev_img2, cur_img2, prev_pts2, cur_pts2, status, err, cv::Size(21, 21), 3);
        // reverse check
        if(FLOW_BACK)
        {
            vector<uchar> reverse_status;
            vector<cv::Point2f> reverse_pts = prev_pts2;
            cv::calcOpticalFlowPyrLK(cur_img2, prev_img2, cur_pts2, reverse_pts, reverse_status, err, cv::Size(21, 21), 1,
                                     cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01), cv::OPTFLOW_USE_INITIAL_FLOW);
            //cv::calcOpticalFlowPyrLK(cur_img2, prev_img, cur_pts, reverse_pts, reverse_status, err, cv::Size(21, 21), 3);
            for(size_t i = 0; i < status.size(); i++)
            {
                if(status[i] && reverse_status[i] && distance(prev_pts2[i], reverse_pts[i]) <= 0.5)
                {
                    status[i] = 1;
                }
                else
                    status[i] = 0;
            }
        }

        for (int i = 0; i < int(cur_pts2.size()); i++)
            if (status[i] && !inBorder(cur_pts2[i]))
                status[i] = 0;
        reduceVector(prev_pts2, status);
        reduceVector(cur_pts2, status);
        reduceVector(ids2, status);
        reduceVector(track_cnt2, status);
        ROS_DEBUG("temporal optical flow costs: %fms", t_o.toc());
        //printf("track cnt %d\n", (int)ids2.size());
    }

    for (auto &n : track_cnt2)//
        n++;

    if (1)
    {
        //rejectWithF();
        ROS_DEBUG("set mask begins");
        TicToc t_m;
        setMask2();
        ROS_DEBUG("set mask costs %fms", t_m.toc());

        ROS_DEBUG("detect feature begins");
        TicToc t_t;
        int n_max_cnt = MAX_CNT - static_cast<int>(cur_pts2.size());
        if (n_max_cnt > 0)
        {
            if(mask.empty())
                cout << "mask is empty " << endl;
            if (mask.type() != CV_8UC1)
                cout << "mask type wrong " << endl;
            cv::goodFeaturesToTrack(cur_img2, n_pts2, MAX_CNT - cur_pts2.size(), 0.01, MIN_DIST, mask);
        }//极大值抑制
        else
            n_pts2.clear();
        ROS_DEBUG("detect feature costs: %fms", t_t.toc());

        ROS_DEBUG("add feature begins");
        TicToc t_a;
        for (auto &p : n_pts2)
        {
            cur_pts2.push_back(p);
            ids2.push_back(n_id++);
            track_cnt2.push_back(1);
        }
        ROS_DEBUG("selectFeature costs: %fms", t_a.toc());
    }

    cur_un_pts2 = undistortedPts(cur_pts2, m_camera[2]);
    pts_velocity2 = ptsVelocity(ids2, cur_un_pts2, cur_un_pts_map2, prev_un_pts_map2);

    if(!_img3.empty() && stereo_cam)
    {
        ids_right2.clear();
        cur_right_pts2.clear();
        cur_un_right_pts2.clear();
        right_pts_velocity2.clear();
        cur_un_right_pts_map2.clear();
        if(!cur_pts2.empty())
        {
            //printf("stereo image; track feature on right image\n");
            vector<cv::Point2f> reverseLeftPts;
            vector<uchar> status, statusRightLeft;
            vector<float> err;
            // cur left ---- cur right
            cv::calcOpticalFlowPyrLK(cur_img2, rightImg2, cur_pts2, cur_right_pts2, status, err, cv::Size(21, 21), 3);//光流跟踪,左图与右图
            // reverse check cur right ---- cur left
            if(FLOW_BACK)
            {
                cv::calcOpticalFlowPyrLK(rightImg2, cur_img2, cur_right_pts2, reverseLeftPts, statusRightLeft, err, cv::Size(21, 21), 3);
                for(size_t i = 0; i < status.size(); i++)
                {
                    if(status[i] && statusRightLeft[i] && inBorder(cur_right_pts2[i]) && distance(cur_pts2[i], reverseLeftPts[i]) <= 0.5)
                        status[i] = 1;
                    else
                        status[i] = 0;
                }
            }

            ids_right2 = ids2;
            reduceVector(cur_right_pts2, status);
            reduceVector(ids_right2, status);

            cur_un_right_pts2 = undistortedPts(cur_right_pts2, m_camera[3]);
            right_pts_velocity2 = ptsVelocity(ids_right2, cur_un_right_pts2, cur_un_right_pts_map2, prev_un_right_pts_map2);
        }
        prev_un_right_pts_map2 = cur_un_right_pts_map2;
    }
    //if(SHOW_TRACK)
        //drawTrack(cur_img2, rightImg2, ids, cur_pts2, cur_right_pts2, prevLeftPtsMap2);

    prev_img2 = cur_img2;
    prev_pts2 = cur_pts2;
    prev_un_pts2 = cur_un_pts2;
    prev_un_pts_map2 = cur_un_pts_map2;

    prevLeftPtsMap2.clear();
    for(size_t i = 0; i < cur_pts2.size(); i++)
        prevLeftPtsMap2[ids2[i]] = cur_pts2[i];

    //std::cout<<"about to transform to the front left camera"<<std::endl;
    for (size_t i = 0; i < ids2.size(); i++)
    {
        int feature_id = ids2[i];
        double x, y ,z;
        x = cur_un_pts2[i].x;
        y = cur_un_pts2[i].y;
        z = 1;
        double p_u, p_v;
        p_u = cur_pts2[i].x;
        p_v = cur_pts2[i].y;
        int camera_id = 2;//第二个双目左
        double velocity_x, velocity_y;
        velocity_x = pts_velocity2[i].x;
        velocity_y = pts_velocity2[i].y;

        Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
        xyz_uv_velocity << x, y, z, p_u, p_v, -velocity_x, velocity_y;//归一化平面坐标，像素坐标，像素运动速度,左相机
        //std::cout<<"xyz_uv_velocity" << xyz_uv_velocity<< std::endl;
        featureFrame[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
    }
    //std::cout<<"about to transform to the front right camera"<<std::endl;
    if (!_img3.empty() && stereo_cam)
    {
        for (size_t i = 0; i < ids_right2.size(); i++)
        {
            int feature_id = ids_right2[i];
            double x, y ,z;
            x = cur_un_right_pts2[i].x;
            y = cur_un_right_pts2[i].y;
            z = 1;
            double p_u, p_v;
            p_u = cur_right_pts2[i].x;
            p_v = cur_right_pts2[i].y;
            int camera_id = 3;//第二个双目右
            double velocity_x, velocity_y;
            velocity_x = right_pts_velocity2[i].x;
            velocity_y = right_pts_velocity2[i].y;

            Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
            xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
            featureFrame[feature_id].emplace_back(camera_id,  xyz_uv_velocity);//右相机
        }
    }

    //todo 第五相机
    cur_img3 = _img4;
    row = cur_img3.rows;
    col = cur_img3.cols;
    cur_pts3.clear();

    if (prev_pts3.size() > 0)//ywc 之前的特征点
    {
        TicToc t_o;
        vector<uchar> status;
        vector<float> err;
        if(hasPrediction)
        {
            cur_pts3 = predict_pts3;//ywc 当前帧的特征点
            cv::calcOpticalFlowPyrLK(prev_img3, cur_img3, prev_pts3, cur_pts3, status, err, cv::Size(21, 21), 1,
                                     cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01), cv::OPTFLOW_USE_INITIAL_FLOW);//光流跟踪,左图与前一时刻

            int succ_num = 0;
            for (size_t i = 0; i < status.size(); i++)
            {
                if (status[i])
                    succ_num++;
            }
            if (succ_num < 10)
                cv::calcOpticalFlowPyrLK(prev_img3, cur_img3, prev_pts3, cur_pts3, status, err, cv::Size(21, 21), 3);
        }
        else
            cv::calcOpticalFlowPyrLK(prev_img3, cur_img3, prev_pts3, cur_pts3, status, err, cv::Size(21, 21), 3);
        // reverse check
        if(FLOW_BACK)
        {
            vector<uchar> reverse_status;
            vector<cv::Point2f> reverse_pts = prev_pts3;
            cv::calcOpticalFlowPyrLK(cur_img3, prev_img3, cur_pts3, reverse_pts, reverse_status, err, cv::Size(21, 21), 1,
                                     cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01), cv::OPTFLOW_USE_INITIAL_FLOW);
            //cv::calcOpticalFlowPyrLK(cur_img2, prev_img, cur_pts, reverse_pts, reverse_status, err, cv::Size(21, 21), 3);
            for(size_t i = 0; i < status.size(); i++)
            {
                if(status[i] && reverse_status[i] && distance(prev_pts3[i], reverse_pts[i]) <= 0.5)
                {
                    status[i] = 1;
                }
                else
                    status[i] = 0;
            }
        }

        for (int i = 0; i < int(cur_pts3.size()); i++)
            if (status[i] && !inBorder(cur_pts3[i]))
                status[i] = 0;
        reduceVector(prev_pts3, status);
        reduceVector(cur_pts3, status);
        reduceVector(ids3, status);
        reduceVector(track_cnt3, status);
        ROS_DEBUG("temporal optical flow costs: %fms", t_o.toc());
        //printf("track cnt %d\n", (int)ids2.size());
    }

    for (auto &n : track_cnt3)//
        n++;

    if (1)
    {
        //rejectWithF();
        ROS_DEBUG("set mask begins");
        TicToc t_m;
        setMask3();
        ROS_DEBUG("set mask costs %fms", t_m.toc());

        ROS_DEBUG("detect feature begins");
        TicToc t_t;
        int n_max_cnt = MAX_CNT - static_cast<int>(cur_pts3.size());
        if (n_max_cnt > 0)
        {
            if(mask.empty())
                cout << "mask is empty " << endl;
            if (mask.type() != CV_8UC1)
                cout << "mask type wrong " << endl;
            cv::goodFeaturesToTrack(cur_img3, n_pts3, MAX_CNT - cur_pts3.size(), 0.01, MIN_DIST, mask);
        }//极大值抑制
        else
            n_pts3.clear();
        ROS_DEBUG("detect feature costs: %fms", t_t.toc());

        ROS_DEBUG("add feature begins");
        TicToc t_a;
        for (auto &p : n_pts3)
        {
            cur_pts3.push_back(p);
            ids3.push_back(n_id++);
            track_cnt3.push_back(1);
        }
        ROS_DEBUG("selectFeature costs: %fms", t_a.toc());
    }

    cur_un_pts3 = undistortedPts(cur_pts3, m_camera[4]);
    pts_velocity3 = ptsVelocity(ids3, cur_un_pts3, cur_un_pts_map3, prev_un_pts_map3);

    //if(SHOW_TRACK)
       // drawTrack(cur_img3, cur_img3, ids, cur_pts, cur_right_pts3, prevLeftPtsMap3);

    prev_img3 = cur_img3;
    prev_pts3 = cur_pts3;
    prev_un_pts3 = cur_un_pts3;
    prev_un_pts_map3 = cur_un_pts_map3;

    prevLeftPtsMap3.clear();
    for(size_t i = 0; i < cur_pts3.size(); i++)
        prevLeftPtsMap3[ids3[i]] = cur_pts3[i];

    //std::cout<<"about to transform to the front left camera"<<std::endl;
    for (size_t i = 0; i < ids3.size(); i++)
    {
        int feature_id = ids3[i];
        double x, y ,z;
        x = cur_un_pts3[i].x;
        y = cur_un_pts3[i].y;
        z = 1;
        double p_u, p_v;
        p_u = cur_pts3[i].x;
        p_v = cur_pts3[i].y;
        int camera_id = 4;//第二个双目左
        double velocity_x, velocity_y;
        velocity_x = pts_velocity3[i].x;
        velocity_y = pts_velocity3[i].y;

        Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
        xyz_uv_velocity << x, y, z, p_u, p_v, -velocity_x, velocity_y;//归一化平面坐标，像素坐标，像素运动速度,左相机
        //std::cout<<"xyz_uv_velocity" << xyz_uv_velocity<< std::endl;
        featureFrame[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
    }

    //todo 第六相机
    cur_img4 = _img5;
    row = cur_img4.rows;
    col = cur_img4.cols;
    cur_pts4.clear();

    if (prev_pts4.size() > 0)//ywc 之前的特征点
    {
        TicToc t_o;
        vector<uchar> status;
        vector<float> err;
        if(hasPrediction)
        {
            cur_pts4 = predict_pts4;//ywc 当前帧的特征点
            cv::calcOpticalFlowPyrLK(prev_img4, cur_img4, prev_pts4, cur_pts4, status, err, cv::Size(21, 21), 1,
                                     cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01), cv::OPTFLOW_USE_INITIAL_FLOW);//光流跟踪,左图与前一时刻

            int succ_num = 0;
            for (size_t i = 0; i < status.size(); i++)
            {
                if (status[i])
                    succ_num++;
            }
            if (succ_num < 10)
                cv::calcOpticalFlowPyrLK(prev_img4, cur_img4, prev_pts4, cur_pts4, status, err, cv::Size(21, 21), 3);
        }
        else
            cv::calcOpticalFlowPyrLK(prev_img4, cur_img4, prev_pts4, cur_pts4, status, err, cv::Size(21, 21), 3);
        // reverse check
        if(FLOW_BACK)
        {
            vector<uchar> reverse_status;
            vector<cv::Point2f> reverse_pts = prev_pts4;
            cv::calcOpticalFlowPyrLK(cur_img4, prev_img4, cur_pts4, reverse_pts, reverse_status, err, cv::Size(21, 21), 1,
                                     cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01), cv::OPTFLOW_USE_INITIAL_FLOW);
            //cv::calcOpticalFlowPyrLK(cur_img2, prev_img, cur_pts, reverse_pts, reverse_status, err, cv::Size(21, 21), 3);
            for(size_t i = 0; i < status.size(); i++)
            {
                if(status[i] && reverse_status[i] && distance(prev_pts4[i], reverse_pts[i]) <= 0.5)
                {
                    status[i] = 1;
                }
                else
                    status[i] = 0;
            }
        }

        for (int i = 0; i < int(cur_pts4.size()); i++)
            if (status[i] && !inBorder(cur_pts4[i]))
                status[i] = 0;
        reduceVector(prev_pts4, status);
        reduceVector(cur_pts4, status);
        reduceVector(ids4, status);
        reduceVector(track_cnt4, status);
        ROS_DEBUG("temporal optical flow costs: %fms", t_o.toc());
        //printf("track cnt %d\n", (int)ids2.size());
    }

    for (auto &n : track_cnt4)//
        n++;

    if (1)
    {
        //rejectWithF();
        ROS_DEBUG("set mask begins");
        TicToc t_m;
        setMask4();
        ROS_DEBUG("set mask costs %fms", t_m.toc());

        ROS_DEBUG("detect feature begins");
        TicToc t_t;
        int n_max_cnt = MAX_CNT - static_cast<int>(cur_pts4.size());
        if (n_max_cnt > 0)
        {
            if(mask.empty())
                cout << "mask is empty " << endl;
            if (mask.type() != CV_8UC1)
                cout << "mask type wrong " << endl;
            cv::goodFeaturesToTrack(cur_img4, n_pts4, MAX_CNT - cur_pts4.size(), 0.01, MIN_DIST, mask);
        }//极大值抑制
        else
            n_pts4.clear();
        ROS_DEBUG("detect feature costs: %fms", t_t.toc());

        ROS_DEBUG("add feature begins");
        TicToc t_a;
        for (auto &p : n_pts4)
        {
            cur_pts4.push_back(p);
            ids4.push_back(n_id++);
            track_cnt4.push_back(1);
        }
        ROS_DEBUG("selectFeature costs: %fms", t_a.toc());
    }

    cur_un_pts4 = undistortedPts(cur_pts4, m_camera[5]);
    pts_velocity4 = ptsVelocity(ids4, cur_un_pts4, cur_un_pts_map4, prev_un_pts_map4);

    if(SHOW_TRACK)
        drawTrack(cur_img4, cur_img4, ids4, cur_pts4, cur_right_pts4, prevLeftPtsMap4);

    prev_img4 = cur_img4;
    prev_pts4 = cur_pts4;
    prev_un_pts4 = cur_un_pts4;
    prev_un_pts_map4 = cur_un_pts_map4;

    prevLeftPtsMap4.clear();
    for(size_t i = 0; i < cur_pts4.size(); i++)
        prevLeftPtsMap4[ids4[i]] = cur_pts4[i];

    //std::cout<<"about to transform to the front left camera"<<std::endl;
    for (size_t i = 0; i < ids4.size(); i++)
    {
        int feature_id = ids4[i];
        double x, y ,z;
        x = cur_un_pts4[i].x;
        y = cur_un_pts4[i].y;
        z = 1;
        double p_u, p_v;
        p_u = cur_pts4[i].x;
        p_v = cur_pts4[i].y;
        int camera_id = 5;//
        double velocity_x, velocity_y;
        velocity_x = pts_velocity4[i].x;
        velocity_y = pts_velocity4[i].y;

        Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
        xyz_uv_velocity << x, y, z, p_u, p_v, -velocity_x, velocity_y;//归一化平面坐标，像素坐标，像素运动速度,you相机
        //std::cout<<"xyz_uv_velocity" << xyz_uv_velocity<< std::endl;
        featureFrame[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
    }


    hasPrediction = false;
    prev_time = cur_time;
    //printf("feature track whole time %f\n", t_r.toc());
    return featureFrame;
}

void FeatureTracker::rejectWithF()
{
    if (cur_pts.size() >= 8)
    {
        ROS_DEBUG("FM ransac begins");
        TicToc t_f;
        vector<cv::Point2f> un_cur_pts(cur_pts.size()), un_prev_pts(prev_pts.size());
        for (unsigned int i = 0; i < cur_pts.size(); i++)
        {
            Eigen::Vector3d tmp_p;
            m_camera[0]->liftProjective(Eigen::Vector2d(cur_pts[i].x, cur_pts[i].y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + col / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + row / 2.0;
            un_cur_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());

            m_camera[0]->liftProjective(Eigen::Vector2d(prev_pts[i].x, prev_pts[i].y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + col / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + row / 2.0;
            un_prev_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());
        }

        vector<uchar> status;
        cv::findFundamentalMat(un_cur_pts, un_prev_pts, cv::FM_RANSAC, F_THRESHOLD, 0.99, status);
        int size_a = cur_pts.size();
        reduceVector(prev_pts, status);
        reduceVector(cur_pts, status);
        reduceVector(cur_un_pts, status);
        reduceVector(ids, status);
        reduceVector(track_cnt, status);
        ROS_DEBUG("FM ransac: %d -> %lu: %f", size_a, cur_pts.size(), 1.0 * cur_pts.size() / size_a);
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
    if (calib_file.size() == 2||calib_file.size() == 4||calib_file.size() == 6)
        stereo_cam = 1;
}

void FeatureTracker::showUndistortion(const string &name)
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
            undistortedImg.at<uchar>(pp.at<float>(1, 0) + 300, pp.at<float>(0, 0) + 300) = cur_img.at<uchar>(distortedp[i].y(), distortedp[i].x());
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

vector<cv::Point2f> FeatureTracker::ptsVelocity(vector<int> &ids, vector<cv::Point2f> &pts, //ids, cur_un_pts, cur_un_pts_map, prev_un_pts_map
                                            map<int, cv::Point2f> &cur_id_pts, map<int, cv::Point2f> &prev_id_pts)
{
    vector<cv::Point2f> pts_velocity;
    cur_id_pts.clear();
    for (unsigned int i = 0; i < ids.size(); i++)
    {
        cur_id_pts.insert(make_pair(ids[i], pts[i]));//当前点插入地图
    }

    // caculate points velocity
    if (!prev_id_pts.empty())
    {
        double dt = cur_time - prev_time;
        
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
        for (unsigned int i = 0; i < pts.size(); i++)//todo cur_pts 改为pts
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
                               map<int, cv::Point2f> &prevLeftPtsMap)
{
    //int rows = imLeft.rows;
    int cols = imLeft.cols;
    if (!imRight.empty() && stereo_cam)
        cv::hconcat(imLeft, imRight, imTrack);
    else
        imTrack = imLeft.clone();
    cv::cvtColor(imTrack, imTrack, CV_GRAY2RGB);

    for (size_t j = 0; j < curLeftPts.size(); j++)
    {
        double len = std::min(1.0, 1.0 * track_cnt[j] / 20);
        cv::circle(imTrack, curLeftPts[j], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
    }
    if (!imRight.empty() && stereo_cam)
    {
        for (size_t i = 0; i < curRightPts.size(); i++)
        {
            cv::Point2f rightPt = curRightPts[i];
            rightPt.x += cols;
            cv::circle(imTrack, rightPt, 2, cv::Scalar(0, 255, 0), 2);
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
            cv::arrowedLine(imTrack, curLeftPts[i], mapIt->second, cv::Scalar(0, 255, 0), 1, 8, 0, 0.2);
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

    // turn the following code on if you need
     cv::imshow("tracking", imTrack);
     cv::waitKey(2);
}


void FeatureTracker::setPrediction(map<int, Eigen::Vector3d> &predictPts)
{
    hasPrediction = true;
    predict_pts.clear();
    predict_pts_debug.clear();
    map<int, Eigen::Vector3d>::iterator itPredict;
    for (size_t i = 0; i < ids.size(); i++)
    {
        //printf("prevLeftId size %d prevLeftPts size %d\n",(int)prevLeftIds.size(), (int)prevLeftPts.size());
        int id = ids[i];
        itPredict = predictPts.find(id);
        if (itPredict != predictPts.end())
        {
            Eigen::Vector2d tmp_uv;
            m_camera[0]->spaceToPlane(itPredict->second, tmp_uv);
            predict_pts.push_back(cv::Point2f(tmp_uv.x(), tmp_uv.y()));
            predict_pts_debug.push_back(cv::Point2f(tmp_uv.x(), tmp_uv.y()));
        }
        else
            predict_pts.push_back(prev_pts[i]);
    }
}


void FeatureTracker::removeOutliers(set<int> &removePtsIds)
{
    std::set<int>::iterator itSet;
    vector<uchar> status;
    for (size_t i = 0; i < ids.size(); i++)
    {
        itSet = removePtsIds.find(ids[i]);
        if(itSet != removePtsIds.end())
            status.push_back(0);
        else
            status.push_back(1);
    }

    reduceVector(prev_pts, status);
    reduceVector(ids, status);
    reduceVector(track_cnt, status);
}


cv::Mat FeatureTracker::getTrackImage()
{
    return imTrack;
}