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

bool FeatureTracker::inBorder(const cv::Point2f &pt,const int row,const int col)
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
    n_id = 0;

}

void FeatureTracker::setMask(int camera_sys)
{

    mask = cv::Mat(track_param_[camera_sys]->row, track_param_[camera_sys]->col, CV_8UC1, cv::Scalar(255));

    // prefer to keep features that are tracked for long time
    vector<pair<int, pair<cv::Point2f, int>>> cnt_pts_id;

    for (unsigned int i = 0; i < track_param_[camera_sys]->cur_pts.size(); i++)
        cnt_pts_id.push_back(make_pair(track_param_[camera_sys]->track_cnt[i], make_pair(track_param_[camera_sys]->cur_pts[i], track_param_[camera_sys]->ids[i])));

    sort(cnt_pts_id.begin(), cnt_pts_id.end(), [](const pair<int, pair<cv::Point2f, int>> &a, const pair<int, pair<cv::Point2f, int>> &b)
         {
            return a.first > b.first;
         });

    track_param_[camera_sys]->cur_pts.clear();
    track_param_[camera_sys]->ids.clear();
    track_param_[camera_sys]->track_cnt.clear();

    for (auto &it : cnt_pts_id)
    {
        if (mask.at<uchar>(it.second.first) == 255)
        {
            track_param_[camera_sys]->cur_pts.push_back(it.second.first);
            track_param_[camera_sys]->ids.push_back(it.second.second);
            track_param_[camera_sys]->track_cnt.push_back(it.first);
            cv::circle(mask, it.second.first, MIN_DIST, 0, -1);
        }
    }
}

double FeatureTracker::distance(cv::Point2f &pt1, cv::Point2f &pt2)
{
    //printf("pt1: %f %f pt2: %f %f\n", pt1.x, pt1.y, pt2.x, pt2.y);
    double dx = pt1.x - pt2.x;
    double dy = pt1.y - pt2.y;
    return sqrt(dx * dx + dy * dy);
}

map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> FeatureTracker::trackImage(double _cur_time, vector<pair<cv::Mat, cv::Mat>>& IMAGE)
{
    TicToc t_r;
    cur_time = _cur_time;
    int camera_id =0;
    map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> featureFrame;
    for(int img_index =0;img_index<IMAGE.size();img_index++){
        if(!IMAGE[img_index].first.empty()){
            track_param_[img_index]->cur_img= IMAGE[img_index].first;
            track_param_[img_index]->row = track_param_[img_index]->cur_img.rows;
            track_param_[img_index]->col = track_param_[img_index]->cur_img.cols;
            cv::Mat rightImg = IMAGE[img_index].second;
            /*
            {
                cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
                clahe->apply(cur_img, cur_img);
                if(!rightImg.empty())
                    clahe->apply(rightImg, rightImg);
            }
            */
            track_param_[img_index]->cur_pts.clear();
            if (track_param_[img_index]->prev_pts.size() > 0)
            {
                TicToc t_o;
                vector<uchar> status;
                vector<float> err;
                if(track_param_[img_index]->hasPrediction)
                {
                    track_param_[img_index]->cur_pts = track_param_[img_index]->predict_pts;
                    cv::calcOpticalFlowPyrLK(track_param_[img_index]->prev_img, track_param_[img_index]->cur_img,
                            track_param_[img_index]->prev_pts, track_param_[img_index]->cur_pts, status, err, cv::Size(21, 21), 1,
                                             cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01), cv::OPTFLOW_USE_INITIAL_FLOW);

                    int succ_num = 0;
                    for (size_t i = 0; i < status.size(); i++)
                    {
                        if (status[i])
                            succ_num++;
                    }
                    if (succ_num < 10)
                        cv::calcOpticalFlowPyrLK(track_param_[img_index]->prev_img, track_param_[img_index]->cur_img,
                                track_param_[img_index]->prev_pts, track_param_[img_index]->cur_pts, status, err, cv::Size(21, 21), 3);
                }
                else
                    cv::calcOpticalFlowPyrLK(track_param_[img_index]->prev_img, track_param_[img_index]->cur_img,
                            track_param_[img_index]->prev_pts, track_param_[img_index]->cur_pts, status, err, cv::Size(21, 21), 3);
                // reverse check
                if(FLOW_BACK)
                {
                    vector<uchar> reverse_status;
                    vector<cv::Point2f> reverse_pts = track_param_[img_index]->prev_pts;
                    cv::calcOpticalFlowPyrLK(track_param_[img_index]->cur_img, track_param_[img_index]->prev_img,
                            track_param_[img_index]->cur_pts, reverse_pts, reverse_status, err, cv::Size(21, 21), 1,
                                             cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01), cv::OPTFLOW_USE_INITIAL_FLOW);
                    //cv::calcOpticalFlowPyrLK(cur_img, prev_img, cur_pts, reverse_pts, reverse_status, err, cv::Size(21, 21), 3);
                    for(size_t i = 0; i < status.size(); i++)
                    {
                        if(status[i] && reverse_status[i] && distance(track_param_[img_index]->prev_pts[i], reverse_pts[i]) <= 0.5)
                        {
                            status[i] = 1;
                        }
                        else
                            status[i] = 0;
                    }
                }

                for (int i = 0; i < int(track_param_[img_index]->cur_pts.size()); i++)
                    if (status[i] && !inBorder(track_param_[img_index]->cur_pts[i],track_param_[img_index]->row, track_param_[img_index]->col))
                        status[i] = 0;
                reduceVector(track_param_[img_index]->prev_pts, status);
                reduceVector(track_param_[img_index]->cur_pts, status);
                reduceVector(track_param_[img_index]->ids, status);
                reduceVector(track_param_[img_index]->track_cnt, status);
                ROS_DEBUG("temporal optical flow costs: %fms", t_o.toc());
                //printf("track cnt %d\n", (int)ids.size());
            }

            for (auto &n : track_param_[img_index]->track_cnt)
                n++;
            if (1)
            {
                //rejectWithF(track_param_[img_index]);
                ROS_DEBUG("set mask begins");
                TicToc t_m;
                setMask(img_index);
                ROS_DEBUG("set mask costs %fms", t_m.toc());

                ROS_DEBUG("detect feature begins");
                TicToc t_t;
                int n_max_cnt = MAX_CNT - static_cast<int>(track_param_[img_index]->cur_pts.size());
                if (n_max_cnt > 0)
                {
                    if(mask.empty())
                        cout << "mask is empty " << endl;
                    if (mask.type() != CV_8UC1)
                        cout << "mask type wrong " << endl;
                    cv::goodFeaturesToTrack(track_param_[img_index]->cur_img, track_param_[img_index]->n_pts, MAX_CNT - track_param_[img_index]->cur_pts.size(), 0.01, MIN_DIST, mask);
                }
                else
                    track_param_[img_index]->n_pts.clear();
                ROS_DEBUG("detect feature costs: %f ms", t_t.toc());

                for (auto &p : track_param_[img_index]->n_pts)
                {
                    track_param_[img_index]->cur_pts.push_back(p);
                    track_param_[img_index]->ids.push_back(n_id++);
                    track_param_[img_index]->track_cnt.push_back(1);
                }
                //printf("feature cnt after add %d\n", (int)ids.size());
            }

            track_param_[img_index]->cur_un_pts = undistortedPts(track_param_[img_index]->cur_pts, m_camera[camera_id]);
            track_param_[img_index]->pts_velocity = ptsVelocity(track_param_[img_index]->ids, track_param_[img_index]->cur_un_pts,
                    track_param_[img_index]->cur_un_pts_map, track_param_[img_index]->prev_un_pts_map, track_param_[img_index]->cur_pts.size());
            if(!IMAGE[img_index].second.empty() && track_param_[img_index]->stereo_cam)
            {
                track_param_[img_index]->ids_right.clear();
                track_param_[img_index]->cur_right_pts.clear();
                track_param_[img_index]->cur_un_right_pts.clear();
                track_param_[img_index]->right_pts_velocity.clear();
                track_param_[img_index]->cur_un_right_pts_map.clear();
                if(!track_param_[img_index]->cur_pts.empty())
                {
                    //printf("stereo image; track feature on right image\n");
                    vector<cv::Point2f> reverseLeftPts;
                    vector<uchar> status, statusRightLeft;
                    vector<float> err;
                    // cur left ---- cur right
                    cv::calcOpticalFlowPyrLK(track_param_[img_index]->cur_img, rightImg, track_param_[img_index]->cur_pts,
                            track_param_[img_index]->cur_right_pts, status, err, cv::Size(21, 21), 3);
                    // reverse check cur right ---- cur left

                    if(FLOW_BACK)
                    {

                        cv::calcOpticalFlowPyrLK(rightImg, track_param_[img_index]->cur_img, track_param_[img_index]->cur_right_pts,
                                reverseLeftPts, statusRightLeft, err, cv::Size(21, 21), 3);
                        for(size_t i = 0; i < status.size(); i++)
                        {
                            distance(track_param_[img_index]->cur_pts[i], reverseLeftPts[i]);
                            if(status[i] && statusRightLeft[i] && inBorder(track_param_[img_index]->cur_right_pts[i],track_param_[img_index]->row,
                                    track_param_[img_index]->col) && distance(track_param_[img_index]->cur_pts[i], reverseLeftPts[i]) <= 0.5)
                                status[i] = 1;
                            else
                                status[i] = 0;
                        }

                    }

                    track_param_[img_index]->ids_right = track_param_[img_index]->ids;
                    reduceVector(track_param_[img_index]->cur_right_pts, status);
                    reduceVector(track_param_[img_index]->ids_right, status);
                    // only keep left-right pts
                    /*
                    reduceVector(cur_pts, status);
                    reduceVector(ids, status);
                    reduceVector(track_cnt, status);
                    reduceVector(cur_un_pts, status);
                    reduceVector(pts_velocity, status);
                    */
                    track_param_[img_index]->cur_un_right_pts = undistortedPts(track_param_[img_index]->cur_right_pts, m_camera[camera_id+1]);
                    track_param_[img_index]->right_pts_velocity = ptsVelocity(track_param_[img_index]->ids_right, track_param_[img_index]->cur_un_right_pts,
                            track_param_[img_index]->cur_un_right_pts_map, track_param_[img_index]->prev_un_right_pts_map, track_param_[img_index]->cur_pts.size());
                }
                track_param_[img_index]->prev_un_right_pts_map = track_param_[img_index]->cur_un_right_pts_map;

            }
            if(SHOW_TRACK&&img_index==0)
                drawTrack(img_index, rightImg);

            track_param_[img_index]->prev_img = track_param_[img_index]->cur_img;
            track_param_[img_index]->prev_pts = track_param_[img_index]->cur_pts;
            track_param_[img_index]->prev_un_pts = track_param_[img_index]->cur_un_pts;
            track_param_[img_index]->prev_un_pts_map = track_param_[img_index]->cur_un_pts_map;

            track_param_[img_index]->prevLeftPtsMap.clear();
            for(size_t i = 0; i < track_param_[img_index]->cur_pts.size(); i++)
                track_param_[img_index]->prevLeftPtsMap[track_param_[img_index]->ids[i]] = track_param_[img_index]->cur_pts[i];


            for (size_t i = 0; i < track_param_[img_index]->ids.size(); i++)
            {
                int feature_id = track_param_[img_index]->ids[i];
                double x, y ,z;
                x = track_param_[img_index]->cur_un_pts[i].x;
                y = track_param_[img_index]->cur_un_pts[i].y;
                z = 1;
                double p_u, p_v;
                p_u = track_param_[img_index]->cur_pts[i].x;
                p_v = track_param_[img_index]->cur_pts[i].y;
                double velocity_x, velocity_y;
                velocity_x = track_param_[img_index]->pts_velocity[i].x;
                velocity_y = track_param_[img_index]->pts_velocity[i].y;

                Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
                xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
                featureFrame[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
            }
            camera_id++;
            if (!IMAGE[img_index].second.empty() && track_param_[img_index]->stereo_cam)
            {

                for (size_t i = 0; i < track_param_[img_index]->ids_right.size(); i++)
                {
                    int feature_id = track_param_[img_index]->ids_right[i];
                    double x, y ,z;
                    x = track_param_[img_index]->cur_un_right_pts[i].x;
                    y = track_param_[img_index]->cur_un_right_pts[i].y;
                    z = 1;
                    double p_u, p_v;
                    p_u = track_param_[img_index]->cur_right_pts[i].x;
                    p_v = track_param_[img_index]->cur_right_pts[i].y;
                    double velocity_x, velocity_y;
                    velocity_x = track_param_[img_index]->right_pts_velocity[i].x;
                    velocity_y = track_param_[img_index]->right_pts_velocity[i].y;
                    Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
                    xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
                    featureFrame[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
                }
                camera_id++;
            }
        }
        track_param_[img_index]->hasPrediction = false;
    }
    //cout<<"camera_id"<<camera_id<<endl;
    prev_time = cur_time;
    //printf("feature track whole time %f\n", t_r.toc());
    return featureFrame;
}

void FeatureTracker::rejectWithF(int camera_sys)
{
    if (track_param_[camera_sys]->cur_pts.size() >= 8)
    {
        ROS_DEBUG("FM ransac begins");
        TicToc t_f;
        vector<cv::Point2f> un_cur_pts(track_param_[camera_sys]->cur_pts.size()), un_prev_pts(track_param_[camera_sys]->prev_pts.size());
        for (unsigned int i = 0; i < track_param_[camera_sys]->cur_pts.size(); i++)
        {
            Eigen::Vector3d tmp_p;
            m_camera[0]->liftProjective(Eigen::Vector2d(track_param_[camera_sys]->cur_pts[i].x, track_param_[camera_sys]->cur_pts[i].y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + track_param_[camera_sys]->col / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + track_param_[camera_sys]->row / 2.0;
            un_cur_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());

            m_camera[0]->liftProjective(Eigen::Vector2d(track_param_[camera_sys]->prev_pts[i].x, track_param_[camera_sys]->prev_pts[i].y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + track_param_[camera_sys]->col / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + track_param_[camera_sys]->row / 2.0;
            un_prev_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());
        }

        vector<uchar> status;
        cv::findFundamentalMat(un_cur_pts, un_prev_pts, cv::FM_RANSAC, F_THRESHOLD, 0.99, status);
        int size_a = track_param_[camera_sys]->cur_pts.size();
        reduceVector(track_param_[camera_sys]->prev_pts, status);
        reduceVector(track_param_[camera_sys]->cur_pts, status);
        reduceVector(track_param_[camera_sys]->cur_un_pts, status);
        reduceVector(track_param_[camera_sys]->ids, status);
        reduceVector(track_param_[camera_sys]->track_cnt, status);
        ROS_DEBUG("FM ransac: %d -> %lu: %f", size_a, track_param_[camera_sys]->cur_pts.size(), 1.0 * track_param_[camera_sys]->cur_pts.size() / size_a);
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

    for(int i =0;i<NUM_OF_CAM;i++){
        if(i==0){
            track_param *trackPa = new track_param();
            trackPa->stereo_cam = false;
            track_param_.push_back(trackPa);
        }else if(i==1){
            track_param_[0]->stereo_cam=true;
        }else if(i==2){
            track_param *trackPa = new track_param();
            trackPa->stereo_cam = false;
            track_param_.push_back(trackPa);
        }else if(i==3){
            track_param_[1]->stereo_cam=true;
        }else if(i==4){
            track_param *trackPa = new track_param();
            trackPa->stereo_cam = false;
            track_param_.push_back(trackPa);
        }else if(i==5){
            track_param *trackPa = new track_param();
            trackPa->stereo_cam = false;
            track_param_.push_back(trackPa);
        }
    }

}

void FeatureTracker::showUndistortion(const string &name, int camera_sys)
{
    cv::Mat undistortedImg(track_param_[camera_sys]->row + 600, track_param_[camera_sys]->col + 600, CV_8UC1, cv::Scalar(0));
    vector<Eigen::Vector2d> distortedp, undistortedp;
    for (int i = 0; i < track_param_[camera_sys]->col; i++)
        for (int j = 0; j < track_param_[camera_sys]->row; j++)
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
        pp.at<float>(0, 0) = undistortedp[i].x() * FOCAL_LENGTH + track_param_[camera_sys]->col / 2;
        pp.at<float>(1, 0) = undistortedp[i].y() * FOCAL_LENGTH + track_param_[camera_sys]->row / 2;
        pp.at<float>(2, 0) = 1.0;
        //cout << trackerData[0].K << endl;
        //printf("%lf %lf\n", p.at<float>(1, 0), p.at<float>(0, 0));
        //printf("%lf %lf\n", pp.at<float>(1, 0), pp.at<float>(0, 0));
        if (pp.at<float>(1, 0) + 300 >= 0 && pp.at<float>(1, 0) + 300 < track_param_[camera_sys]->row + 600 && pp.at<float>(0, 0) + 300 >= 0 && pp.at<float>(0, 0) + 300 < track_param_[camera_sys]->col + 600)
        {
            undistortedImg.at<uchar>(pp.at<float>(1, 0) + 300, pp.at<float>(0, 0) + 300) = track_param_[camera_sys]->cur_img.at<uchar>(distortedp[i].y(), distortedp[i].x());
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

vector<cv::Point2f> FeatureTracker::ptsVelocity(vector<int> &ids, vector<cv::Point2f> &pts,
                                            map<int, cv::Point2f> &cur_id_pts, map<int, cv::Point2f> &prev_id_pts,
                                            size_t cur_pt_size)
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
        for (unsigned int i = 0; i < cur_pt_size; i++)
        {
            pts_velocity.push_back(cv::Point2f(0, 0));
        }
    }
    return pts_velocity;
}

void FeatureTracker::drawTrack(int camera_sys,cv::Mat& imRight)
{
    //int rows = imLeft.rows;
    int cols = track_param_[camera_sys]->cur_img.cols;
    if (!track_param_[camera_sys]->cur_img.empty() && track_param_[camera_sys]->stereo_cam)
        cv::hconcat(track_param_[camera_sys]->cur_img, imRight, imTrack);
    else
        imTrack = track_param_[camera_sys]->cur_img.clone();
    cv::cvtColor(imTrack, imTrack, CV_GRAY2RGB);

    for (size_t j = 0; j < track_param_[camera_sys]->cur_pts.size(); j++)
    {
        double len = std::min(1.0, 1.0 * track_param_[camera_sys]->track_cnt[j] / 20);
        cv::circle(imTrack, track_param_[camera_sys]->cur_pts[j], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
    }
    if (!imRight.empty() && track_param_[camera_sys]->stereo_cam)
    {
        for (size_t i = 0; i < track_param_[camera_sys]->cur_right_pts.size(); i++)
        {
            cv::Point2f rightPt = track_param_[camera_sys]->cur_right_pts[i];
            rightPt.x += cols;
            cv::circle(imTrack, rightPt, 2, cv::Scalar(0, 255, 0), 2);
            //cv::Point2f leftPt = curLeftPtsTrackRight[i];
            //cv::line(imTrack, leftPt, rightPt, cv::Scalar(0, 255, 0), 1, 8, 0);
        }
    }

    map<int, cv::Point2f>::iterator mapIt;
    for (size_t i = 0; i < track_param_[camera_sys]->ids.size(); i++)
    {
        int id = track_param_[camera_sys]->ids[i];
        mapIt = track_param_[camera_sys]->prevLeftPtsMap.find(id);
        if(mapIt != track_param_[camera_sys]->prevLeftPtsMap.end())
        {
            cv::arrowedLine(imTrack, track_param_[camera_sys]->cur_pts[i], mapIt->second, cv::Scalar(0, 255, 0), 1, 8, 0, 0.2);
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
    cv::imshow("tracking", imTrack);
    cv::waitKey(2);
}


void FeatureTracker::setPrediction(map<int, Eigen::Vector3d> &predictPts,int camera_sys)
{
    track_param_[camera_sys]->hasPrediction = true;
    track_param_[camera_sys]->predict_pts.clear();
    track_param_[camera_sys]->predict_pts_debug.clear();
    map<int, Eigen::Vector3d>::iterator itPredict;
    for (size_t i = 0; i < track_param_[camera_sys]->ids.size(); i++)
    {
        //printf("prevLeftId size %d prevLeftPts size %d\n",(int)prevLeftIds.size(), (int)prevLeftPts.size());
        int id = track_param_[camera_sys]->ids[i];
        itPredict = predictPts.find(id);
        if (itPredict != predictPts.end())
        {
            Eigen::Vector2d tmp_uv;
            m_camera[camera_sys]->spaceToPlane(itPredict->second, tmp_uv);
            track_param_[camera_sys]->predict_pts.push_back(cv::Point2f(tmp_uv.x(), tmp_uv.y()));
            track_param_[camera_sys]->predict_pts_debug.push_back(cv::Point2f(tmp_uv.x(), tmp_uv.y()));
        }
        else
            track_param_[camera_sys]->predict_pts.push_back(track_param_[camera_sys]->prev_pts[i]);
    }
}


void FeatureTracker::removeOutliers(set<int> &removePtsIds, int camera_sys )
{
    std::set<int>::iterator itSet;
    vector<uchar> status;
    for (size_t i = 0; i < track_param_[camera_sys]->ids.size(); i++)
    {
        itSet = removePtsIds.find(track_param_[camera_sys]->ids[i]);
        if(itSet != removePtsIds.end())
            status.push_back(0);
        else
            status.push_back(1);
    }

    reduceVector(track_param_[camera_sys]->prev_pts, status);
    reduceVector(track_param_[camera_sys]->ids, status);
    reduceVector(track_param_[camera_sys]->track_cnt, status);
}


cv::Mat FeatureTracker::getTrackImage()
{
    return imTrack;
}