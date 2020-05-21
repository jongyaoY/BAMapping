//
// Created by jojo on 25.03.20.
//

#include "Alignment3D_ransac.h"
#include <random>
#include <Eigen/SVD>
#include <iostream>
using namespace BAMapping::FrontEnd;

bool Alignment3D_ransac::Align(
        const std::vector<Vec3>& source,
        const std::vector<Vec3>& target,
        Mat4& Tst,
        std::vector<size_t>& inliers,
        const Params params)
{
    std::vector<size_t> inliers_max;
    Tst.setIdentity();
    if(source.size() != target.size())
    {
        return false;
    }
    if(source.size() < 5)
    {
        std::cout<<"less then 5 correspondences"<<std::endl;
        return false;
    }
    if(source.empty())
    {
        return false;
    }

    //ransac find set of inliers
    for(int it = 0; it < params.iterations; it++)
    {
        std::vector<size_t> inliers_cur;
        std::vector<Vec3> p_cur;
        std::vector<Vec3> q_cur;
        auto rand_ids = generateRandomIds(4,source.size()-1);
        for(auto id : rand_ids)
        {
            p_cur.push_back(source[id]);
            q_cur.push_back(target[id]);
        }
        Mat3 R;
        Vec3 t;
        bool success = estimate(p_cur,q_cur,R,t);

        if(!success)
            continue;

        for(int i = 0; i < source.size(); i++)
        {
            bool in = isInlier(source[i],target[i],R,t,params.distance_threshold);
            if(in)
                inliers_cur.push_back(i);
        }
        if(inliers_cur.size() > inliers_max.size())
        {
            inliers_max = inliers_cur;
        }
        if((float)inliers_max.size()/source.size() > params.inlier_threshold)
            break;
    }
    //estimate R | t with inliers
    Mat3 R_est;
    Vec3 t_est;
    auto inliers_curr = inliers_max;
    auto inliers_rate_cur = (float) inliers_curr.size() / source.size();
    auto inliers_rate = (float) inliers_max.size()/source.size();
    for(int it = 0; it < 10 ; it++)
    {
        std::vector<Vec3> p_est;
        std::vector<Vec3> q_est;
        for(auto id : inliers_curr)
        {
            p_est.push_back(source[id]);
            q_est.push_back(target[id]);
        }
        bool success = estimate(p_est,q_est,R_est,t_est);

        if(!success)
        {
            return false;
        }

        inliers_curr.clear();
        for(int i = 0; i < source.size(); i++)
        {
            bool in = isInlier(source[i],target[i],R_est,t_est,params.distance_threshold);
            if(in)
                inliers_curr.push_back(i);
        }
        inliers_rate_cur = (float) inliers_curr.size() / source.size();
        if((inliers_rate_cur - inliers_rate) < 0.1)
            break;
        else
        {
            inliers_rate = inliers_rate_cur;
        }
    }
    inliers_max = inliers_curr;
//    if(success)
    {
        Tst.block<3,3>(0,0) = R_est;
        Tst.block<3,1>(0,3) = t_est;
    }

    inliers = inliers_max;
    return true;
}



std::vector<size_t> Alignment3D_ransac::generateRandomIds(int num, int max_id)
{
    std::vector<size_t> ids;
    std::random_device dev;
    std::mt19937 rng(dev());
    std::uniform_int_distribution<std::mt19937::result_type> dist(0,max_id);
    for(int i = 0; i < num; i++)
    {
        for(int j = 0; j < 10; j++)
        {
            size_t rand_id = dist(rng);
            auto it = std::find(ids.begin(),ids.end(),rand_id);
            if(it == std::end(ids))
            {
                ids.push_back(rand_id);
                break;
            }
        }
    }

    return ids;
}

bool Alignment3D_ransac::isInlier(Vec3 p, Vec3 q, Mat3 R, Vec3 t,double thres)
{
    Vec3 p_;
    p_ = R*p + t;
    auto diff = p_ - q;

    auto diff_norm = sqrt(diff.dot(diff));
    if(diff_norm < thres)
        return true;
    else
        return false;
}

bool Alignment3D_ransac::estimate(
        const std::vector<Vec3> &p,
        const std::vector<Vec3> &q,
        Mat3 &R, Vec3 &t)
{
    if(p.size() != q.size())
    {
        return false;
    }
    if(p.empty())
    {
        return false;
    }
    if(p.size() < 3)
    {
        return false;
    }

    Mat3 H;
    H.setZero();
    Vec3 p_cen(0,0,0),q_cen(0,0,0);
    p_cen.setZero();
    q_cen.setZero();
    for(int i = 0; i < p.size(); i++)
    {
        const auto& p_ = p[i];
        const auto& q_ = q[i];
        p_cen += p_;
        q_cen += q_;
    }
    p_cen = p_cen / (double) p.size();
    q_cen = q_cen / (double) q.size();

    for(int i = 0; i < p.size(); i++)
    {
        auto p_c_ = p[i] - p_cen;
        auto q_c_ = q[i] - q_cen;
        Mat3 H_ = p_c_ * q_c_.transpose();

        H += H_;
    }

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(H, Eigen::ComputeThinU | Eigen::ComputeThinV);
    auto U = svd.matrixU();
    auto V = svd.matrixV();

    Mat3 R_;
    R_ = V * U.transpose();
    if(R_.determinant() > 0)
    {
        R = R_;
        t = q_cen - R_ * p_cen;
        return true;
    }
    else
    {
        return false;
    }

}