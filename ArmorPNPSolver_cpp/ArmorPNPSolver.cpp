//
// Created by Leyan on 2025/5/22.
//

#include "ArmorPNPSolver.h"


void ArmorPNPSolver::pnp(TargetData &point4data)
{
    // 准备pnp所需四点的世界坐标3D坐标，小装甲灯柱对中宽134mm，大装甲灯柱对中宽229mm
    // 世界坐标系中心建立在装甲板中心
    vector<cv::Point3f> Points3D;
    cv::RotatedRect R1 = point4data.light1;
    cv::RotatedRect R2 = point4data.light2;

    if (point4data.is_big == 0)
    {
        // 小装甲板的相关参数
        Points3D.push_back(cv::Point3f(-67, -27.5, 0));   // P1 三维坐标的单位是毫米
        Points3D.push_back(cv::Point3f(-67,  27.5, 0));   // P2
        Points3D.push_back(cv::Point3f( 67, -27.5, 0));   // P3
        Points3D.push_back(cv::Point3f( 67,  27.5, 0));   // P4
    }
    else
    {
        // 大装甲板的相关参数
        Points3D.push_back(cv::Point3f(-114.5, -27.5, 0)); // P1 三维坐标的单位是毫米
        Points3D.push_back(cv::Point3f(-114.5,  27.5, 0)); // P2
        Points3D.push_back(cv::Point3f( 114.5, -27.5, 0)); // P3
        Points3D.push_back(cv::Point3f( 114.5,  27.5, 0)); // P4
    }

    // 准备pnp所需四点在2D图中对应的坐标
    // 可以根据匹配的两个灯条计算出装甲板四个角点的位置
    vector<cv::Point2f> Points2D;
    Points2D.push_back(cv::Point2f(R1.center.x + 0.5 * R1.size.height * sin(R1.angle), R1.center.y));
    Points2D.push_back(cv::Point2f(R1.center.x - 0.5 * R1.size.height * sin(R1.angle), R1.center.y));
    Points2D.push_back(cv::Point2f(R2.center.x + 0.5 * R2.size.height * sin(R2.angle), R2.center.y));
    Points2D.push_back(cv::Point2f(R2.center.x - 0.5 * R2.size.height * sin(R2.angle), R2.center.y));

    // 初始化输出矩阵
    cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);
    cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64FC1);
    cv::Mat rvec1 = cv::Mat::zeros(3, 1, CV_64FC1);
    cv::Mat tvec1 = cv::Mat::zeros(3, 1, CV_64FC1);

    float RatioX = 1.0f;
    float RatioY = 1.0f;
    float RatioZ = 1.0f;

    /* Calculate bndbox. */
    Point2f rpt1[4], rpt2[4];
    R1.points(rpt1);
    R2.points(rpt2);

    std::vector<float> vx, vy;
    for (int i = 0; i < 4; i++)
    {
        vx.push_back(rpt1[i].x);
        vy.push_back(rpt1[i].y);
        vx.push_back(rpt2[i].x);
        vy.push_back(rpt2[i].y);
    }

    std::vector<float>::iterator result;

    result = std::max_element(vx.begin(), vx.end());
    float xmax = vx[std::distance(vx.begin(), result)];

    result = std::max_element(vx.begin(), vx.end(), min_compare);
    float xmin = vx[std::distance(vx.begin(), result)];

    result = std::max_element(vy.begin(), vy.end());
    float ymax = vy[std::distance(vy.begin(), result)];

    result = std::max_element(vy.begin(), vy.end(), min_compare);
    float ymin = vy[std::distance(vy.begin(), result)];

    point4data.x_min = xmin;
    point4data.y_min = ymin;
    point4data.width  = xmax - xmin;
    point4data.height = ymax - ymin;

    // 执行 PnP 解算
    solvePnP(Points3D, Points2D, intrinsic_matrix, distortion_vec, rvec1, tvec1, false, SOLVEPNP_AP3P);

    // Rodrigues 变换
    cv::Mat rotate_mat_cv2;
    Eigen::Matrix3d rotate_mat_eigen;
    cv::Rodrigues(rvec1, rotate_mat_cv2);
    cv2eigen(rotate_mat_cv2, rotate_mat_eigen);

    // 欧拉角解算并转为角度制
    point4data.pnp_ypr = rotate_mat_eigen.eulerAngles(2, 1, 0) / CV_PI * 180;
    if (point4data.pnp_ypr[0] > 140)
        point4data.pnp_ypr[0] -= 180;

    // 相机坐标系转换到云台坐标系
    // ypitch: g, zpitch: gun muzzle
    double y_pitch = point4data.y + this->delta_y_pitch;
    double z_pitch = point4data.z + delta_z_pitch;
    double x_yaw   = point4data.x + delta_x_yaw;
    double z_yaw   = point4data.z + delta_z_yaw;

    point4data.yaw_angle   = atan(x_yaw / z_yaw) * 180 / CV_PI;
    point4data.pitch_angle = atan(y_pitch / z_pitch) * 180 / CV_PI;
}
