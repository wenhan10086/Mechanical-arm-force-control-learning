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
  #define PINOCCHIO_MODEL_DIR "/home/wenhan/下载/kuka_iiwa_test/src/iiwa_stack/iiwa_description/urdf/"
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
  double kv=10000;
  

  const std::string urdf_filename =
    (argc <= 1) ? PINOCCHIO_MODEL_DIR
                    + std::string("iiwa14.urdf")
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
    usleep(10);
  }
  
  // 初始化
  Eigen::VectorXd q_temp = Eigen::VectorXd::Zero(model.nv);
  Eigen::VectorXd v_temp = Eigen::VectorXd::Zero(model.nv);  // nv 是模型的自由度数量
  Eigen::VectorXd acc_temp = Eigen::VectorXd::Zero(model.nv);  // nv 是模型的自由度数量
  Eigen::VectorXd q_used  = Eigen::VectorXd::Zero(model.nv);
  Eigen::VectorXd v_used = Eigen::VectorXd::Zero(model.nv);  // nv 是模型的自由度数量
  Eigen::VectorXd acc_used = Eigen::VectorXd::Zero(model.nv);  // nv 是模型的自由度数量
  
  Eigen::VectorXd qd_temp  = Eigen::VectorXd::Zero(model.nv);
  Eigen::VectorXd vd_temp = Eigen::VectorXd::Zero(model.nv);  // nv 是模型的自由度数量
  Eigen::VectorXd accd_temp = Eigen::VectorXd::Zero(model.nv);  // nv 是模型的自由度数量
  Eigen::VectorXd qd_used  = Eigen::VectorXd::Zero(model.nv);
  Eigen::VectorXd vd_used = Eigen::VectorXd::Zero(model.nv);  // nv 是模型的自由度数量
  Eigen::VectorXd accd_used = Eigen::VectorXd::Zero(model.nv);  // nv 是模型的自由度数量

  Eigen::VectorXd Eq = Eigen::VectorXd::Zero(model.nv);
  Eigen::VectorXd Ev = Eigen::VectorXd::Zero(model.nv);  // nv 是模型的自由度数量

  Eigen::MatrixXd M(model.nv,model.nv);
  M.setZero();

  Eigen::MatrixXd C(model.nv,model.nv);
  C.setZero();
  Eigen::VectorXd tau(model.nv);

  int init_flag=0;

  while(supervisor->step(timeStep)!=-1){

  const double* position=target_ball->getPosition();
  const pinocchio::SE3 target(Eigen::Matrix3d::Identity(),Eigen::Vector3d(position[0], position[1], position[2]));
  
  printf("Ball position: (%f, %f, %f)\n", position[0], position[1], position[2]);
  for(int i=0;i<7;i++)
  {
    q_temp[i]=positionSensor[i]->getValue();
    usleep(10);
  }
  pinocchio::forwardKinematics(model,data,q_temp);

  if(init_flag==0)
  {
    
    init_flag=1;
    
    std::cout<<"当前位姿"<<data.oMi[jointId].translation().transpose()<<std::endl;

    qd_temp=q_temp;

    std::cout<<mPinocchioIk(model,data,target,jointId,qd_temp)<<std::endl;
    std::cout<<"末端位置是："<<data.oMi[7].translation().transpose()<<std::endl;
    std::cout<<"01_qd_temp:"<<qd_temp.transpose()<<std::endl;
    std::cout<<"01_q_temp:"<<q_temp.transpose()<<std::endl;
    vd_temp=(qd_temp-q_temp)/timeStep;
    accd_temp=(vd_temp-vd_used)/timeStep;
    std::cout<<"vd_temp:"<<vd_temp.transpose()<<std::endl;
    std::cout<<"accd_temp:"<<accd_temp.transpose()<<std::endl;
    qd_used=qd_temp;
    vd_used=vd_temp;
    accd_used=accd_temp;
    pinocchio::forwardKinematics(model,data,q_temp);
    static Eigen::VectorXd position=q_temp;

    std::cout<<"init success!"<<std::endl;
  }
  else
  {
    mPinocchioIk(model,data,target,jointId,qd_temp);
    
    vd_temp=(qd_temp-qd_used)/timeStep;

    accd_temp=(vd_temp-vd_used)/timeStep;

    Eigen::VectorXd tau_f = rnea(model, data,qd_temp, vd_temp, accd_temp);

    for(int i=0;i<7;i++)
    {
      q_temp[i]=positionSensor[i]->getValue();
      v_temp[i]=(q_temp[i]-q_used[i])/timeStep;
      Eq[i]=qd_temp[i]-q_temp[i];
      Ev[i]=vd_temp[i]-v_temp[i];
    }
    
    acc_temp=kp*Eq+kv*Ev+accd_temp;
    std::cout<<"加速度为："<<acc_temp<<std::endl<<"q误差为"<<Eq.transpose()<<std::endl<<"q为："<<q_temp.transpose()<<std::endl;
    Eigen::VectorXd qdd=Eigen::VectorXd::Random(model.nv);
    Eigen::VectorXd tau=rnea(model,data,q_temp,v_temp,acc_temp);
    
    
    for(int i=0;i<7;i++)
    {
      joints[i]->setTorque(tau[i]);
    }
    std::cout<<"力矩为:"<<tau.transpose()<<std::endl;

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

  }


  delete supervisor;
  return 0;
}
