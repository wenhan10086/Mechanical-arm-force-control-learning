// File:          my_controller.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include<webots/Robot.hpp>
#include<webots/Motor.hpp>
#include <webots/Supervisor.hpp>
#include<webots/Node.hpp>
#include<stdio.h>
#include<iostream>

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
// All the webots classes are defined in the "webots" namespace
using namespace webots;
using namespace pinocchio;

#ifndef PINOCCHIO_MODEL_DIR
  #define PINOCCHIO_MODEL_DIR "/home/wenhan/下载/kuka_iiwa_test/src/iiwa_stack/iiwa_description/urdf/"
#endif

int main(int argc, char **argv) {
  // create the Robot instance.
  Supervisor *supervisor= new Supervisor();
  Node* target_ball=supervisor->getFromDef("ball");
  int timeStep = (int)supervisor->getBasicTimeStep();
    const std::string urdf_filename =
    (argc <= 1) ? PINOCCHIO_MODEL_DIR
                    + std::string("iiwa14.urdf")
                : argv[1];
 Model model;
pinocchio::urdf::buildModel(urdf_filename,model);
  std::cout << "model name: " << model.name << std::endl;
  //设定一个关节角
  Eigen::VectorXd q(7);
  q << 0, 0, 1.57, 0, 0, 0, 0;
  Data data(model);
  //定义质量矩阵
  Eigen::MatrixXd M(model.nv, model.nv);
  //计算惯性矩阵存储在data
  crba(model, data, q);

  //将关节角速度和角加速度设为0，获取重力力矩
  Eigen::VectorXd tau_gravity = rnea(model, data, q, Eigen::VectorXd::Zero(model.nv), Eigen::VectorXd::Zero(model.nv));
  std::cout << "Gravity torques:\n" << tau_gravity.transpose() << std::endl;

  //设定随机关节角加速度
  Eigen::VectorXd qdd = Eigen::VectorXd::Random(model.nv);  
  std::cout << "Expect qdd" << qdd.transpose()<<std::endl;
  //设定关节角速度为0，角加速度随机，计算每个关节的力矩
   //rnea(model,data,q,dq,ddq)
  Eigen::VectorXd tau = rnea(model, data, q, Eigen::VectorXd::Zero(model.nv), qdd);
  std::cout<<"tau="<<tau<<std::endl;
  //计算关节角加速度
  //aba(model,data,q,dq,tau)
  aba(model, data, q, Eigen::VectorXd::Zero(model.nv), tau);
  std::cout << "Forward dynamics result (joint accelerations):\n" << data.ddq.transpose() << std::endl;

  //  while (supervisor->step(timeStep) != -1)
  //  {
  //   const double* position=target_ball->getPosition();
  //   // std::cout << "Ball position: " << position[0] << " " << position[1] << " " << position[2] << std::endl;
  //     printf("Ball position: (%f, %f, %f)\n", position[0], position[1], position[2]);
  //  }
   

  delete supervisor;
  return 0;
}
