#ifndef MPINNOCCHIOIK_H
#define MPINNOCCHIOIK_H


#include <pinocchio/spatial/se3.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <Eigen/Dense>

bool mPinocchioIk(pinocchio::Model &model, pinocchio::Data &data, const pinocchio::SE3 &oMdes, int jointId, Eigen::VectorXd &q) {
  // 逆向运动学求解参数
  const double eps = 1e-4;
  const int IT_MAX = 1000;
  const double DT = 1e-3;
  const double damp = 1e-6;

  // 初始化雅可比矩阵和误差向量
  pinocchio::Data::Matrix6x J(6, model.nv);
  J.setZero();
  Eigen::VectorXd v(model.nv); // 速度向量

  bool success = false;
  typedef Eigen::Matrix<double, 6, 1> Vector6d;
  Vector6d err;

  for (int i = 0; i < IT_MAX; ++i) {
    // 正向运动学，计算当前关节位姿
    pinocchio::forwardKinematics(model, data, q);

    // 计算误差，使用 log6 获取 SE3 误差的向量形式
    const pinocchio::SE3 iMd = data.oMi[jointId].actInv(oMdes);
    err = pinocchio::log6(iMd).toVector();

    // 检查误差是否在收敛范围内
    if (err.norm() < eps) {
      success = true;
      break;
    }

    // 计算雅可比
    pinocchio::computeJointJacobian(model, data, q, jointId, J);

    // 调整雅可比矩阵
    pinocchio::Data::Matrix6 Jlog;
    pinocchio::Jlog6(iMd.inverse(), Jlog);
    J = -Jlog * J;

    // 计算 JJ^T 并加入阻尼项
    pinocchio::Data::Matrix6 JJt = J * J.transpose();
    JJt.diagonal().array() += damp;

    // 计算速度向量 v 并更新 q
    v.noalias() = -J.transpose() * JJt.ldlt().solve(err);
    q = pinocchio::integrate(model, q, v * DT);
  }

  return success;
}

#endif