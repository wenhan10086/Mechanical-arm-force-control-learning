#ifndef WORLD_C_H
#define WORLD_C_H

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
#include "comman.hpp"


#include<yaml-cpp/yaml.h>


#ifndef PINOCCHIO_MODEL_DIR
  #define PINOCCHIO_MODEL_DIR "/home/wenhan/下载/kuka_iiwa_test/src/iiwa_stack/iiwa_description/urdf/"
#endif
using namespace webots;
using namespace pinocchio;

class worldInit_C
{
    public:

    worldInit_C(pinocchio::Model& model,pinocchio::Data& data,Supervisor* supervisor,webots::Node* target_ball)
    {
     
        int timeStep = (int)supervisor->getBasicTimeStep();

        YAML::Node config;
        try{
        config=YAML::LoadFile("./config/kukaiiwa_config.yaml");
        }
        catch(YAML::BadFile &e){
        std::cout<<"read error!"<<std::endl;
        }

        if (config["jointsMotorNames"]) 
        {
            int i=0;
            for (const auto& motor : config["jointsMotorNames"]) 
            {
                joints[i++]=supervisor->getMotor(motor.as<std::string>());
            }
        }

        if (config["positionSensorNames"]) 
        {
            int i=0;
            for (const auto& sensor : config["positionSensorNames"]) {
                sensors[i]=supervisor->getPositionSensor(sensor.as<std::string>());
                sensors[i++]->enable(timeStep);
                usleep(1e+3);
            }
        }
        for(int i=0;i<7;i++){
        supervisor->step(timeStep); 
        }
        
        const double* target_position=target_ball->getPosition();
        const double* target_oriention=target_ball->getOrientation();
        

        Eigen::Matrix3d orientation_matrix = Eigen::Map<const Eigen::Matrix3d>(target_oriention);
        const pinocchio::SE3 target(orientation_matrix,Eigen::Vector3d(target_position[0], target_position[1], target_position[2]));
        Eigen::AngleAxisd target_angle_axis(target.rotation());
        Eigen::Vector3d target_orientation = target_angle_axis.angle() * target_angle_axis.axis();
        Eigen::Vector3d target_translation = target.translation();
            // 将位置和平移部分组合成六维向量
        Eigen::Matrix<double, 6, 1> target_pose;
        target_pose.head<3>() = target_translation;
        target_pose.tail<3>() = target_orientation;
        xt_used=target_pose;
        
        std::cout<<"xt_used :"<<xt_used.transpose()<<std::endl;

        for(int i=0;i<7;i++)
        {
          q_used[i]=sensors[i]->getValue();
        }
        x_used=getSe3FromJointVariables(model,data,q_used);
        forwardKinematics(model,data,q_used);
        const pinocchio::SE3 iMd = data.oMi[7].actInv(target);
        Eq = pinocchio::log6(iMd).toVector();

        std::cout<<"init success !"<<std::endl;

    }
        int nv=7;
        Eigen::VectorXd q_cur = Eigen::VectorXd::Zero(nv);
        Eigen::VectorXd qd_cur = Eigen::VectorXd::Zero(nv); 
        Eigen::VectorXd q_used = Eigen::VectorXd::Zero(nv);  
        Eigen::VectorXd qd_used  = Eigen::VectorXd::Zero(nv);
        Eigen::VectorXd qdd_cur = Eigen::VectorXd::Zero(nv); 

        Eigen::VectorXd x_cur = Eigen::VectorXd::Zero(6);
        Eigen::VectorXd v_cur = Eigen::VectorXd::Zero(6); 
        Eigen::VectorXd a_cur = Eigen::VectorXd::Zero(6);  
        Eigen::VectorXd x_used  = Eigen::VectorXd::Zero(6);
        Eigen::VectorXd v_used = Eigen::VectorXd::Zero(6); 

        Eigen::VectorXd xt_cur=Eigen::VectorXd::Zero(6);
        Eigen::VectorXd vt_cur=Eigen::VectorXd::Zero(6);
        Eigen::VectorXd at_cur=Eigen::VectorXd::Zero(6);
        Eigen::VectorXd xt_used=Eigen::VectorXd::Zero(6);
        Eigen::VectorXd vt_used=Eigen::VectorXd::Zero(6);

        Eigen::VectorXd Eq = Eigen::VectorXd::Zero(6);
        Eigen::VectorXd Ev = Eigen::VectorXd::Zero(6); 

        Motor *joints[7];
        PositionSensor *sensors[7];

        Eigen::VectorXd getSe3FromJointVariables(const pinocchio::Model& model, pinocchio::Data& data, const Eigen::VectorXd& q);
    
};
        Eigen::VectorXd worldInit_C::getSe3FromJointVariables(const pinocchio::Model& model, pinocchio::Data& data, const Eigen::VectorXd& q)
        {
        // 计算末端位姿
        pinocchio::forwardKinematics(model, data, q);

        // 提取末端的SE3（假设是最后一个关节或特定关节索引）
        pinocchio::SE3 end_effector_pose = data.oMi[model.njoints - 1]; 

        // 获取位姿的平移和旋转（轴角）
        Eigen::Vector3d position = end_effector_pose.translation();
        Eigen::AngleAxisd angle_axis(end_effector_pose.rotation());

        // 创建6维向量并赋值
        Eigen::VectorXd se3(6);
        se3.head<3>() = position; // 前3维为位置
        se3.tail<3>() = angle_axis.angle() * angle_axis.axis(); // 后3维为轴角旋转

        return se3;
        }
#endif