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
#include<webots/Supervisor.hpp>
#include<webots/PositionSensor.hpp>
#include<webots/Node.hpp>
#include<stdio.h>
#include<iostream>

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/jacobian.hpp"


#ifndef PINOCCHIO_MODEL_DIR
  #define PINOCCHIO_MODEL_DIR "/home/wenhan/kuka_ws/src/kuka_experimental/kuka_lbr_iiwa_support/urdf/"
#endif
// All the webots classes are defined in the "webots" namespace
using namespace webots;
using namespace pinocchio;


int main(int argc, char **argv) {
  // create the Robot instance.
  Supervisor *supervisor= new Supervisor();
  Node* target_ball=supervisor->getFromDef("ball");
  int timeStep = (int)supervisor->getBasicTimeStep();
  const std::string urdf_filename =
    (argc <= 1) ? PINOCCHIO_MODEL_DIR
                    + std::string("model.urdf")
                : argv[1];
 
  Load the urdf model
  Model model;
  pinocchio::urdf::buildModel(urdf_filename, model);
  std::cout << "model name: " << model.name << std::endl;

  //实例化7个电机与位置传感器对象
  Motor *joints[7];
  PositionSensor *positionSensor[7];

  char jointsMotorNames[7][10]={
    "joint_a1",
    "joint_a2",
    "joint_a3",
    "joint_a4",
    "joint_a5",
    "joint_a6",
    "joint_a7", 
  };
  char psSensorName[7][20]={
    "joint_a1_sensor",
    "joint_a2_sensor",
    "joint_a3_sensor",
    "joint_a4_sensor",
    "joint_a5_sensor",
    "joint_a6_sensor",
    "joint_a7_sensor",
  };

  for(int i=0;i<7;i++)
  {
    joints[i]=supervisor->getMotor(jointsMotorNames[i]);
  }
  
  for(int i=0;i<7;i++)
  {
    positionSensor[i]=supervisor->getPositionSensor(psSensorName[i]);
    positionSensor[i]->enable(timeStep);
  }
  
  载入模型
  Model model;
  pinocchio::urdf::buildModel(urdf_filename, model);
  std::cout << "model name: " << model.name << std::endl;
  Eigen::VectorXd q = pinocchio::neutral(model);

  //初始化参数
  const double eps = 1e-4;
  const int IT_MAX = 1000;
  const double DT = 1e-1;
  const double damp = 1e-6;

  // 建立数据
  Data data(model);
 
  const int jointId=7;

  // const SE3 oMdes(Eigen::Matrix3d::identity(),target_ball->getPosition());
  const SE3 oMdes(Eigen::Matrix3d::identity(),psSensor[6]->getValue());


  //显示当前小球位置
  const double* position=target_ball->getPosition();
  std::cout << "Ball position: " << position[0] << " " << position[1] << " " << position[2] << std::endl;
      printf("Ball position: (%f, %f, %f)\n", position[0], position[1], position[2]);

  //初始化雅可比矩阵和误差向量
  pinocchio::Data::Matrix6x J(6, model.nv);
  J.setZero();

  bool success=false;

  typedef Eigen::Matrix<double, 6, 1> Vector6d;

  Vector6d err;
  Eigen::VectorXd v(model.nv);
  
  for （int i=0；；i++）
  {
    //正向运动学计算当前指定关节的位姿，并将其与目标位姿相计算，得到一个SO(3)，在转回so（3），
    //得到其6维的误差向量

    forwardKinematics(model,data,q);
    const SE3 imd=data.omi[jointId].actInv(oMdes);
    err = log6(imd).toVector();

    //判断是否收敛，或是超出最大迭代次数
    if(err.norm()<eps)
    {
      success = true;
      break;
    }
    if(i>=IT_MAX)
    {
      success = false;
      break ;
    }
    //计算此时的雅可比
    computeJointJacobians(model,data,q,jointId,J);
    Data::Matrix6 Jlog;
    //将末端误差从世界坐标系，转为关节坐标系
    Jlog6(imd.inverse(),Jlog);
    //对雅可比矩阵进行修正
    J=-Jlog*J;
    Data::Matrix6 JJt;
    //求jjt
    JJt.noalias()=J*J.transpose();
    //加上阻尼参数，就看成强化学习里的更新率
    JJt.diagnoal().array() += damp;
    //-j*(jjt)-1将jjt这种对称正定矩阵ld分解，求jjt*x=err的解，结合就是jjt的逆乘以err
    //最终就是qdot
    v.noalias() = -J.transpose() * JJt.ldlt().solve(err);
    //积分上去
    q = pinocchio::integrate(model, q, v * DT);
  }

  Sample a random configuration
  Eigen::VectorXd q = randomConfiguration(model);
  std::cout << "q: " << q.transpose() << std::endl;
 
  Perform the forward kinematics over the kinematic tree
  forwardKinematics(model, data, q);
 
  Print out the placement of each joint of the kinematic tree
  for (JointIndex joint_id = 0; joint_id < (JointIndex)model.njoints; ++joint_id)
    std::cout << std::setw(24) << std::left << model.names[joint_id] << ": " << std::fixed
              << std::setprecision(2) << data.oMi[joint_id].translation().transpose() << std::endl;
   while (supervisor->step(timeStep) != -1)
   {
    // const double* position=target_ball->getPosition();
    // // std::cout << "Ball position: " << position[0] << " " << position[1] << " " << position[2] << std::endl;
    //   printf("Ball position: (%f, %f, %f)\n", position[0], position[1], position[2]);
    
   }
 

  delete supervisor;
  return 0;
}
