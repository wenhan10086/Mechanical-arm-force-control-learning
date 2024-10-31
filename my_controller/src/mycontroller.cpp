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
#include<Eigen/Dense>

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/aba.hpp"

#include "mPinocchioIK.hpp"

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


  double kp=10;
  double kv=1;

  const std::string urdf_filename =
    (argc <= 1) ? PINOCCHIO_MODEL_DIR
                    + std::string("model.urdf")
                : argv[1];
 
  //Load the urdf model
  Model model;
  pinocchio::urdf::buildModel(urdf_filename, model);
  pinocchio::Data data(model);
  int jointId=7;
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
  
  // 初始化
  Eigen::VectorXd q_temp = pinocchio::neutral(model);
  Eigen::VectorXd v_temp = Eigen::VectorXd::Zero(model.nv);  // nv 是模型的自由度数量
  Eigen::VectorXd acc_temp = Eigen::VectorXd::Zero(model.nv);  // nv 是模型的自由度数量
  Eigen::VectorXd q_used  = pinocchio::neutral(model);
  Eigen::VectorXd v_used = Eigen::VectorXd::Zero(model.nv);  // nv 是模型的自由度数量
  Eigen::VectorXd acc_used = Eigen::VectorXd::Zero(model.nv);  // nv 是模型的自由度数量
  
  Eigen::VectorXd qd_temp  = pinocchio::neutral(model);
  Eigen::VectorXd vd_temp = Eigen::VectorXd::Zero(model.nv);  // nv 是模型的自由度数量
  Eigen::VectorXd accd_temp = Eigen::VectorXd::Zero(model.nv);  // nv 是模型的自由度数量
  Eigen::VectorXd qd_used  = pinocchio::neutral(model);
  Eigen::VectorXd vd_used = Eigen::VectorXd::Zero(model.nv);  // nv 是模型的自由度数量
  Eigen::VectorXd accd_used = Eigen::VectorXd::Zero(model.nv);  // nv 是模型的自由度数量

  Eigen::VectorXd Eq = Eigen::VectorXd::Zero(model.nv);
  Eigen::VectorXd Ev = Eigen::VectorXd::Zero(model.nv);  // nv 是模型的自由度数量


  Eigen::MatrixXd M(model.nv,model.nv);
  M.setZero();

  Eigen::MatrixXd C(model.nv,model.nv);
  C.setZero();
  Eigen::VectorXd tau(model.nv);


 
/****************************************************************** */

   while (supervisor->step(timeStep) != -1)
   {
    
    const double* position=target_ball->getPosition();
    printf("Ball position: (%f, %f, %f)\n", position[0], position[1], position[2]);

    const pinocchio::SE3 target(Eigen::Matrix3d::Identity(),Eigen::Vector3d(position[0], position[1], position[2]));

    mPinocchioIk(model,data,target,jointId,qd_temp);

    vd_temp=(qd_temp-qd_used)*1e+3/timeStep;

    accd_temp=(vd_temp-vd_used)*1e+3/timeStep;

    Eigen::VectorXd tau_f = rnea(model, data,qd_temp, vd_temp, accd_temp);

    for(int i=0;i<7;i++)
    {
      q_temp[i]=positionSensor[i]->getValue();
      v_temp[i]=(q_temp[i]-q_used[i])*1e+3/timeStep;
      acc_temp[i]=(v_temp[i]-v_used[i])*1e+3/timeStep;
      Eq[i]=qd_temp[i]-q_temp[i];
      Ev[i]=vd_temp[i]-v_temp[i];
    }

    Eigen::VectorXd tau=tau_f+kp*Eq+kv*Ev;
    
    for(int i=0;i<7;i++)
    {
      joints[i]->setTorque(tau[i]);
    }

    for(int i=0;i<7;i++)
    {
      q_used=q_temp;
      qd_used=qd_temp;
      v_used=v_temp;
      vd_used=vd_temp;
      acc_used=acc_temp;
      accd_used=accd_temp;
    }

    // // std::cout << "Ball position: " << position[0] << " " << position[1] << " " << position[2] << std::endl;
    //   printf("Ball position: (%f, %f, %f)\n", position[0], position[1], position[2]);
    
   }
 

  delete supervisor;
  return 0;
}
