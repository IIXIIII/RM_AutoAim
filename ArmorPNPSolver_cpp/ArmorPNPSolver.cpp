//
// Created by Leyan on 2025/5/22.
//

#include "ArmorPNPSolver.h"



void ArmorPNPSolver::pnp(TargetData &point4data) {
    // 准备 3D 世界坐标点
    vector<cv::Point3f> Points3D;
    cv::RotatedRect R1 = point4data.light1;
    cv::RotatedRect R2 = point4data.light2;

    if (point4data.is_big == 0) {
        // 小装甲板坐标
        Points3D.push_back(cv::Point3f(-67, -27.5, 0));
        Points3D.push_back(cv::Point3f(-67,  27.5, 0));
        Points3D.push_back(cv::Point3f( 67, -27.5, 0));
        Points3D.push_back(cv::Point3f( 67,  27.5, 0));
    } else {
        // 大装甲板坐标
        Points3D.push_back(cv::Point3f(-114.5, -27.5, 0));
        Points3D.push_back(cv::Point3f(-114.5,  27.5, 0));
        Points3D.push_back(cv::Point3f( 114.5, -27.5, 0));
        Points3D.push_back(cv::Point3f( 114.5,  27.5, 0));
    }

    // 准备图像 2D 点坐标
    vector<cv::Point2f> Points2D;
    Points2D.push_back(cv::Point2f(R1.center.x + 0.5 * R1.size.height * sin(R1.angle), R1.center.y));
    Points2D.push_back(cv::Point2f(R1.center.x - 0.5 * R1.size.height * sin(R1.angle), R1.center.y));
    Points2D.push_back(cv::Point2f(R2.center.x + 0.5 * R2.size.height * sin(R2.angle), R2.center.y));
    Points2D.push_back(cv::Point2f(R2.center.x - 0.5 * R2.size.height * sin(R2.angle), R2.center.y));

    // 初始化输出向量
    cv::Mat rvec1 = cv::Mat::zeros(3, 1, CV_64FC1);
    cv::Mat tvec1 = cv::Mat::zeros(3, 1, CV_64FC1);
    float RatioX = 1.0f, RatioY = 1.0f, RatioZ = 1.0f;

    // 计算边界框（x_min, x_max, y_min, y_max）
    Point2f rpt1[4], rpt2[4];
    R1.points(rpt1);
    R2.points(rpt2);

    std::vector<float> vx, vy;
    for (int i = 0; i < 4; ++i) {
        vx.push_back(rpt1[i].x); vy.push_back(rpt1[i].y);
        vx.push_back(rpt2[i].x); vy.push_back(rpt2[i].y);
    }

    auto max_cmp = [](float a, float b) { return a < b; };
    auto min_cmp = [](float a, float b) { return a > b; };

    float xmax = *std::max_element(vx.begin(), vx.end(), max_cmp);
    float xmin = *std::max_element(vx.begin(), vx.end(), min_cmp);
    float ymax = *std::max_element(vy.begin(), vy.end(), max_cmp);
    float ymin = *std::max_element(vy.begin(), vy.end(), min_cmp);

    point4data.x_min = xmin;
    point4data.y_min = ymin;
    point4data.width  = xmax - xmin;
    point4data.height = ymax - ymin;

    // PnP 解算（使用 AP3P 方法）
    cv::solvePnP(Points3D, Points2D, intrinsic_matrix, distortion_vec, rvec1, tvec1, false, SOLVEPNP_AP3P);

    // 获取旋转矩阵 → 欧拉角
    cv::Mat rotate_mat_cv2;
    Eigen::Matrix3d rotate_mat_eigen;
    cv::Rodrigues(rvec1, rotate_mat_cv2);
    cv2eigen(rotate_mat_cv2, rotate_mat_eigen);
    point4data.pnp_ypr = rotate_mat_eigen.eulerAngles(2, 1, 0) * 180.0 / CV_PI;

    if (point4data.pnp_ypr[0] > 140)
        point4data.pnp_ypr[0] -= 180;

    // 坐标系转换：从相机 → 云台（考虑偏移）
    double y_pitch = point4data.y + this->delta_y_pitch;
    double z_pitch = point4data.z + delta_z_pitch;
    double x_yaw   = point4data.x + delta_x_yaw;
    double z_yaw   = point4data.z + delta_z_yaw;

    point4data.yaw_angle   = atan(x_yaw / z_yaw) * 180.0 / CV_PI;
    point4data.pitch_angle = atan(y_pitch / z_pitch) * 180.0 / CV_PI;
}
