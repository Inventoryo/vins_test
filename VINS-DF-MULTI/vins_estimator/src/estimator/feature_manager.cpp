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
}

void FeatureManager::setRic(Matrix3d _ric[])
{
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        ric[i] = _ric[i];
    }
}

void FeatureManager::clearState()
{
    feature.clear();
}

int FeatureManager::getFeatureCount()
{
    int cnt = 0;
    for (auto &it : feature)
    {
        it.used_num = it.feature_per_frame.size();
        if (it.used_num >= 4)
        {
            cnt++;
        }
    }
    return cnt;
}


bool FeatureManager::addFeatureCheckParallax(int frame_count, const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, double td)
{
    ROS_DEBUG("input feature: %d", (int)image.size());
    ROS_DEBUG("num of feature: %d", getFeatureCount());
    double parallax_sum = 0;
    int parallax_num = 0;
    last_track_num = 0;
    last_average_parallax = 0;
    new_feature_num = 0;
    long_track_num = 0;
    for (auto &id_pts : image)
    {
        FeaturePerFrame f_per_fra(id_pts.second[0].second, td);//特征点在左相机下的表示

        assert(id_pts.second[0].first == 0||id_pts.second[0].first == 2||id_pts.second[0].first == 4||id_pts.second[0].first == 5);//判断特征点是否在左相机下，如果不是，程序终止
        if(id_pts.second.size() == 2)//是否为双目
        {
            f_per_fra.rightObservation(id_pts.second[1].second);//右相机观察信息
            assert(id_pts.second[1].first == 1||id_pts.second[1].first == 3);
        }

        int feature_id = id_pts.first;
        auto it = find_if(feature.begin(), feature.end(), [feature_id](const FeaturePerId &it)
                          {
            return it.feature_id == feature_id;
                          });

        if (it == feature.end())//如果特征点是新的
        {
            feature.push_back(FeaturePerId(feature_id, frame_count));//初始化特征点
            feature.back().feature_per_frame.push_back(f_per_fra);
            feature.back().camera_id = id_pts.second[0].first;//todo 所在相机，多目

            new_feature_num++;
        }
        else if (it->feature_id == feature_id)
        {
            it->feature_per_frame.push_back(f_per_fra);
            last_track_num++;
            if( it-> feature_per_frame.size() >= 4)
                long_track_num++;
        }
    }

    //if (frame_count < 2 || last_track_num < 20)
    //if (frame_count < 2 || last_track_num < 20 || new_feature_num > 0.5 * last_track_num)
    if (frame_count < 2 || last_track_num < 20 || long_track_num < 40 || new_feature_num > 0.5 * last_track_num)
        return true;

    for (auto &it_per_id : feature)
    {
        if (it_per_id.start_frame <= frame_count - 2 &&
            it_per_id.start_frame + int(it_per_id.feature_per_frame.size()) - 1 >= frame_count - 1)
        {
            parallax_sum += compensatedParallax2(it_per_id, frame_count);
            parallax_num++;
        }
    }

    if (parallax_num == 0)
    {
        return true;
    }
    else
    {
        ROS_DEBUG("parallax_sum: %lf, parallax_num: %d", parallax_sum, parallax_num);
        ROS_DEBUG("current parallax: %lf", parallax_sum / parallax_num * FOCAL_LENGTH);
        last_average_parallax = parallax_sum / parallax_num * FOCAL_LENGTH;
        return parallax_sum / parallax_num >= MIN_PARALLAX;
    }
}

vector<pair<Vector3d, Vector3d>> FeatureManager::getCorresponding(int frame_count_l, int frame_count_r)
{
    vector<pair<Vector3d, Vector3d>> corres;
    for (auto &it : feature)
    {
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

void FeatureManager::setDepth(const VectorXd &x)
{
    int feature_index = -1;
    for (auto &it_per_id : feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (it_per_id.used_num < 4)
            continue;

        it_per_id.estimated_depth = 1.0 / x(++feature_index);
        //ROS_INFO("feature id %d , start_frame %d, depth %f ", it_per_id->feature_id, it_per_id-> start_frame, it_per_id->estimated_depth);
        if (it_per_id.estimated_depth < 0)
        {
            it_per_id.solve_flag = 2;
        }
        else
            it_per_id.solve_flag = 1;
    }
}

void FeatureManager::removeFailures()
{
    for (auto it = feature.begin(), it_next = feature.begin();
         it != feature.end(); it = it_next)
    {
        it_next++;
        if (it->solve_flag == 2)
            feature.erase(it);
    }
}

void FeatureManager::clearDepth()
{
    for (auto &it_per_id : feature)
        it_per_id.estimated_depth = -1;
}

VectorXd FeatureManager::getDepthVector()
{
    VectorXd dep_vec(getFeatureCount());
    int feature_index = -1;
    for (auto &it_per_id : feature)
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
    return dep_vec;
}


void FeatureManager::triangulatePoint(Eigen::Matrix<double, 3, 4> &leftPose, Eigen::Matrix<double, 3, 4> &rightPose,
                        Eigen::Vector2d &point0, Eigen::Vector2d &point1, Eigen::Vector3d &point_3d,
                        double &depth, double &cov2)
{
    Eigen::Matrix4d design_matrix = Eigen::Matrix4d::Zero();
    design_matrix.row(0) = point0[0] * leftPose.row(2) - leftPose.row(0);
    design_matrix.row(1) = point0[1] * leftPose.row(2) - leftPose.row(1);
    design_matrix.row(2) = point1[0] * rightPose.row(2) - rightPose.row(0);
    design_matrix.row(3) = point1[1] * rightPose.row(2) - rightPose.row(1);
    Eigen::Vector4d triangulated_point;
    triangulated_point =
              design_matrix.jacobiSvd(Eigen::ComputeFullV).matrixV().rightCols<1>();
    point_3d(0) = triangulated_point(0) / triangulated_point(3);
    point_3d(1) = triangulated_point(1) / triangulated_point(3);
    point_3d(2) = triangulated_point(2) / triangulated_point(3);//世界系下的三维坐标

    //todo 深度值
    Eigen::Vector3d localPoint;
    localPoint = leftPose.leftCols<3>() * point_3d + leftPose.rightCols<1>();//局部系下的三维坐标
    if (localPoint.z() > 0){
        depth = localPoint.z();
        //cov2 = 10;
        //计算不确定度

        Vector3d t = (leftPose.leftCols<3>()) * ((leftPose.leftCols<3>()).transpose()*(leftPose.rightCols<1>())-(rightPose.leftCols<3>()).transpose()*(rightPose.rightCols<1>()));//R1T*(t2-t1)

        Vector3d p = localPoint;
        Vector3d a = p - t;

        double alpha = acos( p.dot(t) / (t.norm() * p.norm()) );
        double beta =acos( a.dot(-t)/(a.norm() * t.norm()) );
        double beta_prime = beta +atan(1.5/FOCAL_LENGTH);
        double gamma = M_PI - alpha - beta_prime;
        //cout<<"alpha"<<alpha<<" beta_prime"<<beta_prime<<" depth"<<depth<<endl;
        double p_prime = t.norm() * sin(beta_prime) / sin(gamma);
        double d_cov = p_prime - depth;
        cov2 =  d_cov * d_cov;//协方差

        /*
        double alpha = acos( p.dot(t) / (t.norm() * p.norm()) );
        double beta = M_PI - acos( a.dot(t)/(a.norm() * t.norm()) );
        double beta_prime = beta - atan(1.5/FOCAL_LENGTH);
        double gamma = M_PI - alpha - beta_prime;
        //std::cout<<"t.norm = "<<t.norm()<<" alpha = "<<alpha*180/M_PI<<" beta_prime = "<<beta_prime*180/M_PI<<" gamma = "<<gamma*180/M_PI<<std::endl;
        double p_prime = t.norm() * sin(beta_prime) / sin(gamma);
        double d_cov = p_prime - depth;
        cov2 =  d_cov * d_cov;//协方差\
         */
        //cout<<"cov2"<<cov2<<endl;

    }else{
        depth = INIT_DEPTH;
        cov2 = 100;
    }

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

void FeatureManager::initFramePoseByPnP(int frameCnt, Vector3d Ps[], Matrix3d Rs[], Vector3d tic[], Matrix3d ric[])
{

    if(frameCnt > 0)
    {
        vector<cv::Point2f> pts2D;
        vector<cv::Point3f> pts3D;
        for (auto &it_per_id : feature)
        {
            if(it_per_id.camera_id!=0)//todo 初始化阶段，只用前置双目
                continue;
            if (it_per_id.estimated_depth > 0)
            {
                int index = frameCnt - it_per_id.start_frame;
                if((int)it_per_id.feature_per_frame.size() >= index + 1)
                {
                    Vector3d ptsInCam = ric[0] * (it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth) + tic[0];
                    Vector3d ptsInWorld = Rs[it_per_id.start_frame] * ptsInCam + Ps[it_per_id.start_frame];

                    cv::Point3f point3d(ptsInWorld.x(), ptsInWorld.y(), ptsInWorld.z());
                    cv::Point2f point2d(it_per_id.feature_per_frame[index].point.x(), it_per_id.feature_per_frame[index].point.y());
                    pts3D.push_back(point3d);
                    pts2D.push_back(point2d); 
                }
            }
        }
        Eigen::Matrix3d RCam;
        Eigen::Vector3d PCam;
        // trans to w_T_cam
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
}

void FeatureManager::triangulate(int frameCnt, Vector3d Ps[], Matrix3d Rs[], Vector3d tic[], Matrix3d ric[], bool MARGIN_OLD)
{
    for (auto &it_per_id : feature)
    {
        if (it_per_id.estimated_depth == NAN||it_per_id.estimated_depth <= 0){//若深度值未初始化
            if(STEREO && it_per_id.feature_per_frame[0].is_stereo)
            {
                int imu_i = it_per_id.start_frame;
                Eigen::Matrix<double, 3, 4> leftPose;
                Eigen::Vector3d t0 = Ps[imu_i] + Rs[imu_i] * tic[it_per_id.camera_id];//世界坐标系下的位姿 todo 多目
                Eigen::Matrix3d R0 = Rs[imu_i] * ric[it_per_id.camera_id];
                leftPose.leftCols<3>() = R0.transpose();
                leftPose.rightCols<1>() = -R0.transpose() * t0;
                //cout << "left pose " << leftPose << endl;

                Eigen::Matrix<double, 3, 4> rightPose;
                Eigen::Vector3d t1 = Ps[imu_i] + Rs[imu_i] * tic[it_per_id.camera_id+1];//todo 多目
                Eigen::Matrix3d R1 = Rs[imu_i] * ric[it_per_id.camera_id+1];//todo 多目
                rightPose.leftCols<3>() = R1.transpose();
                rightPose.rightCols<1>() = -R1.transpose() * t1;
                //cout << "right pose " << rightPose << endl;

                Eigen::Vector2d point0, point1;
                Eigen::Vector3d point3d;
                point0 = it_per_id.feature_per_frame[0].point.head(2);
                point1 = it_per_id.feature_per_frame[0].pointRight.head(2);
                //cout << "point0 " << point0.transpose() << endl;
                //cout << "point1 " << point1.transpose() << endl;

                triangulatePoint(leftPose, rightPose, point0, point1, point3d, it_per_id.estimated_depth, it_per_id.estimated_cov);

                /*
                Vector3d ptsGt = pts_gt[it_per_id.feature_id];
                printf("stereo %d pts: %f %f %f gt: %f %f %f \n",it_per_id.feature_id, point3d.x(), point3d.y(), point3d.z(),
                                                                ptsGt.x(), ptsGt.y(), ptsGt.z());
                */
                //Vector3d t = (leftPose.leftCols<3>()) * ((leftPose.leftCols<3>()).transpose()*(leftPose.rightCols<1>())-(rightPose.leftCols<3>()).transpose()*(rightPose.rightCols<1>()));
                //cout<<"t="<<t<<endl;
                continue;
            }
            else if(it_per_id.feature_per_frame.size() > 1)
            {
                int imu_i = it_per_id.start_frame;
                Eigen::Matrix<double, 3, 4> leftPose;
                Eigen::Vector3d t0 = Ps[imu_i] + Rs[imu_i] * tic[it_per_id.camera_id];//todo 多目
                Eigen::Matrix3d R0 = Rs[imu_i] * ric[it_per_id.camera_id];//todo 多目
                leftPose.leftCols<3>() = R0.transpose();
                leftPose.rightCols<1>() = -R0.transpose() * t0;

                imu_i++;
                Eigen::Matrix<double, 3, 4> rightPose;
                Eigen::Vector3d t1 = Ps[imu_i] + Rs[imu_i] * tic[it_per_id.camera_id];//todo 多目
                Eigen::Matrix3d R1 = Rs[imu_i] * ric[it_per_id.camera_id];//todo 多目
                rightPose.leftCols<3>() = R1.transpose();
                rightPose.rightCols<1>() = -R1.transpose() * t1;

                Eigen::Vector2d point0, point1;
                Eigen::Vector3d point3d;
                point0 = it_per_id.feature_per_frame[0].point.head(2);
                point1 = it_per_id.feature_per_frame[1].point.head(2);
                triangulatePoint(leftPose, rightPose, point0, point1, point3d, it_per_id.estimated_depth, it_per_id.estimated_cov);
                //cout<<"it_per_id.estimated_cov"<<it_per_id.estimated_cov<<endl;
                /*
                Vector3d ptsGt = pts_gt[it_per_id.feature_id];
                printf("motion  %d pts: %f %f %f gt: %f %f %f \n",it_per_id.feature_id, point3d.x(), point3d.y(), point3d.z(),
                                                                ptsGt.x(), ptsGt.y(), ptsGt.z());
                */
                continue;
            }
        }

        it_per_id.used_num = it_per_id.feature_per_frame.size();
        //cout<<"it_per_id.used_num "<<it_per_id.used_num<<endl;
        if (it_per_id.used_num < 4)
            continue;
        if(segma<=0.0001){
            if(it_per_id.estimated_depth > 0)
                continue;
            int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
            Eigen::MatrixXd svd_A(2 * it_per_id.feature_per_frame.size(), 4);
            int svd_idx = 0;

            Eigen::Matrix<double, 3, 4> P0;
            Eigen::Vector3d t0 = Ps[imu_i] + Rs[imu_i] * tic[it_per_id.camera_id];//todo 多目
            Eigen::Matrix3d R0 = Rs[imu_i] * ric[it_per_id.camera_id];//todo 多目
            P0.leftCols<3>() = Eigen::Matrix3d::Identity();
            P0.rightCols<1>() = Eigen::Vector3d::Zero();

            for (auto &it_per_frame : it_per_id.feature_per_frame)
            {
                imu_j++;
                Eigen::Vector3d t1 = Ps[imu_j] + Rs[imu_j] * tic[it_per_id.camera_id];//todo 多目
                Eigen::Matrix3d R1 = Rs[imu_j] * ric[it_per_id.camera_id];//todo 多目
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
            it_per_id.estimated_cov = 100;
            if (it_per_id.estimated_depth < 0.1)
            {
                it_per_id.estimated_depth = INIT_DEPTH;
                it_per_id.estimated_cov = 100;
            }
        } else if(MARGIN_OLD){
            //cout<<"before DF estimated_depth= "<<it_per_id.estimated_depth<<" estimated_cov= "<<it_per_id.estimated_cov<<endl;
            double temp_depth, temp_cov;
            int imu_i = it_per_id.start_frame;
            int imu_j = imu_i +1;
            Eigen::Matrix<double, 3, 4> leftPose;
            Eigen::Vector3d t0 = Ps[imu_i] + Rs[imu_i] * tic[it_per_id.camera_id];//todo 多目
            Eigen::Matrix3d R0 = Rs[imu_i] * ric[it_per_id.camera_id];//todo 多目
            leftPose.leftCols<3>() = R0.transpose();
            leftPose.rightCols<1>() = -R0.transpose() * t0;
            Eigen::Vector2d point0 = it_per_id.feature_per_frame[imu_i].point.head(2);

            for(;imu_j<imu_i + it_per_id.used_num;imu_j++){//深度滤波
                Eigen::Matrix<double, 3, 4> rightPose;
                Eigen::Vector3d t1 = Ps[imu_j] + Rs[imu_j] * tic[it_per_id.camera_id];//todo 多目
                Eigen::Matrix3d R1 = Rs[imu_j] * ric[it_per_id.camera_id];//todo 多目
                rightPose.leftCols<3>() = R1.transpose();
                rightPose.rightCols<1>() = -R1.transpose() * t1;
                Eigen::Vector2d point1;
                Eigen::Vector3d point3d;
                point1 = it_per_id.feature_per_frame[imu_j].point.head(2);

                triangulatePoint(leftPose, rightPose, point0, point1, point3d, temp_depth, temp_cov);

                if(temp_depth!=INIT_DEPTH&&temp_depth>it_per_id.estimated_depth/2&&temp_cov!=NAN){//当结果有效时进行深度融合
                    //cout<<"merging"<<endl;
                    it_per_id.estimated_depth = (temp_cov * it_per_id.estimated_depth + it_per_id.estimated_cov * temp_depth)/(temp_cov + it_per_id.estimated_cov);
                    it_per_id.estimated_cov = it_per_id.estimated_cov * temp_cov/(it_per_id.estimated_cov + temp_cov);
                }
            }

            //cout<<"after DF estimated_depth= "<<temp_depth<<" estimated_cov= "<<temp_cov<<endl;
            if (it_per_id.estimated_depth < 0.1)
            {
                it_per_id.estimated_depth = INIT_DEPTH;
                it_per_id.estimated_cov = 100;
            }
            //cout<<" cov = "<<(FOCAL_LENGTH -segma * sqrt(it_per_id.estimated_cov))/1.5<<endl;
        }
    }
    //feature.sort(CMP());
}

void FeatureManager::removeOutlier(set<int> &outlierIndex)
{
    std::set<int>::iterator itSet;
    for (auto it = feature.begin(), it_next = feature.begin();
         it != feature.end(); it = it_next)
    {
        it_next++;
        int index = it->feature_id;
        itSet = outlierIndex.find(index);
        if(itSet != outlierIndex.end())
        {
            feature.erase(it);
            //printf("remove outlier %d \n", index);
        }
    }
}

void FeatureManager::removeBackShiftDepth(Eigen::Matrix3d back_R0, Eigen::Vector3d back_P0, Eigen::Matrix3d new_R0, Eigen::Vector3d new_P0, Eigen::Matrix3d ric[], Eigen::Vector3d tic[])//todo 多目扩充
{
    for (auto it = feature.begin(), it_next = feature.begin();
         it != feature.end(); it = it_next)
    {
        it_next++;

        if (it->start_frame != 0)//如果起始帧不是第一帧，则直接--
            it->start_frame--;
        else
        {
            Eigen::Vector3d uv_i = it->feature_per_frame[0].point;  
            it->feature_per_frame.erase(it->feature_per_frame.begin());
            if (it->feature_per_frame.size() < 2)
            {
                feature.erase(it);
                continue;
            }
            else
            {
                Matrix3d marg_R, new_R;
                Vector3d marg_P, new_P;
                marg_R = back_R0 * ric[it->camera_id];
                new_R = new_R0 * ric[it->camera_id];
                marg_P = back_P0 + back_R0 * tic[it->camera_id];
                new_P = new_P0 + new_R0 * tic[it->camera_id];

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

void FeatureManager::removeBack()
{
    for (auto it = feature.begin(), it_next = feature.begin();
         it != feature.end(); it = it_next)
    {
        it_next++;

        if (it->start_frame != 0)
            it->start_frame--;
        else
        {
            it->feature_per_frame.erase(it->feature_per_frame.begin());
            if (it->feature_per_frame.size() == 0)
                feature.erase(it);
        }
    }
}

void FeatureManager::removeFront(int frame_count)
{
    for (auto it = feature.begin(), it_next = feature.begin(); it != feature.end(); it = it_next)
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
                feature.erase(it);
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