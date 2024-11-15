#include"comman.hpp"



Eigen::VectorXd computeVelocity(
    const Eigen::VectorXd& pose_prev, 
    const Eigen::VectorXd& pose_cur, 
    double delta_t, 
    const std::string& frame) 
{
    if (pose_prev.size() != 6 || pose_cur.size() != 6) {
        throw std::invalid_argument("Both pose_prev and pose_cur must be 6D vectors.");
    }
    
    Eigen::VectorXd velocity(6);
    
    // 1. 线速度计算
    Eigen::Vector3d pos_prev = pose_prev.head<3>();
    Eigen::Vector3d pos_cur = pose_cur.head<3>();
    Eigen::Vector3d linear_velocity = (pos_cur - pos_prev) / delta_t;

    // 2. 角度变化 (轴角形式)
    Eigen::Vector3d axis_angle_prev = pose_prev.tail<3>();
    Eigen::Vector3d axis_angle_cur = pose_cur.tail<3>();

    // 将轴角转换为旋转矩阵
    Eigen::AngleAxisd rot_prev(axis_angle_prev.norm(), axis_angle_prev.normalized());
    Eigen::AngleAxisd rot_cur(axis_angle_cur.norm(), axis_angle_cur.normalized());

    Eigen::Matrix3d R_delta;
    if (frame == "local") {
        R_delta = rot_prev.inverse() * rot_cur; // 局部坐标系
    } else if (frame == "world") {
        R_delta = rot_cur * rot_prev.inverse(); // 世界坐标系
    } else {
        throw std::invalid_argument("Invalid frame specified. Use 'world' or 'local'.");
    }

    // 3. 从 R_delta 提取角速度（轴角形式）
    Eigen::AngleAxisd angle_axis_delta(R_delta);
    Eigen::Vector3d angular_velocity = angle_axis_delta.axis() * angle_axis_delta.angle() / delta_t;

    // 4. 将线速度和角速度组合成六维向量
    velocity.head<3>() = linear_velocity;
    velocity.tail<3>() = angular_velocity;

    return velocity;
}