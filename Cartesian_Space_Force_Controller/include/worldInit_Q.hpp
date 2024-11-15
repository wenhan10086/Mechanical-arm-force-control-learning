#ifndef WORLDINIT_Q_H
#define WORLDINIT_Q_H

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
using namespace webots;
using namespace pinocchio;
class worldInit_Q
{
    public:

    worldInit_Q()
    {   
        Supervisor *supervisor= new Supervisor();
        Node* target_ball=supervisor->getFromDef("ball");
        int timeStep = (int)supervisor->getBasicTimeStep();
        Eigen::VectorXd q_cur = Eigen::VectorXd::Zero(model.nv);
        Eigen::VectorXd v_cur = Eigen::VectorXd::Zero(model.nv);  // nv 是模型的自由度数量
        Eigen::VectorXd acc_cur = Eigen::VectorXd::Zero(model.nv);  // nv 是模型的自由度数量
        Eigen::VectorXd q_used  = Eigen::VectorXd::Zero(model.nv);
        Eigen::VectorXd v_used = Eigen::VectorXd::Zero(model.nv);  // nv 是模型的自由度数量
        
        
        Eigen::VectorXd qd_cur  = Eigen::VectorXd::Zero(model.nv);
        Eigen::VectorXd vd_cur = Eigen::VectorXd::Zero(model.nv);  // nv 是模型的自由度数量
        Eigen::VectorXd accd_cur = Eigen::VectorXd::Zero(model.nv);  // nv 是模型的自由度数量
        Eigen::VectorXd qd_used  = Eigen::VectorXd::Zero(model.nv);
        Eigen::VectorXd vd_used = Eigen::VectorXd::Zero(model.nv);  // nv 是模型的自由度数量

        const std::string urdf_filename =PINOCCHIO_MODEL_DIR+ std::string("iiwa14.urdf");

        Model model;
        pinocchio::urdf::buildModel(urdf_filename, model);
        pinocchio::Data data(model);
        std::cout << "model name: " << model.name << std::endl;

        Motor *joints[model.nv];
        PositionSensor *positionSensor[model.nv];

        YAML::Node config;
        try{
        config=YAML::LoadFile("./config/kukaiiwa_config.yaml");
        }
        catch(YAML::BadFile &e){
        std::cout<<"read error!"<<std::endl;
        return -1;
        }

        Motor* joints[model.nv];
        PositionSensor *sensors[model.nv];

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
                sensors[i++]=supervisor->getPositionSensor(sensor.as<std::string>());
                sensors[i]->enable(timeStep);
                usleep(10);
            }
        }

        supervisor->step(timeStep);

        for(int i=0;i<model.nv;i++)
        {
            q_cur[i]=sensors[i]->getValue();
            usleep(5);
        }

        const double* target_position=target_ball->getPosition();

        const pinocchio::SE3 target(Eigen::Matrix3d::Identity(),Eigen::Vector3d(target_position[0], target_position[1], target_position[2]));
        printf("Ball position: (%f, %f, %f)\n", target_position[0], target_position[1], target_position[2]);
        for(int i=0;i<7;i++)
        {
            q_temp[i]=sensors[i]->getValue();
            usleep(10);
        }
        pinocchio::forwardKinematics(model,data,q_cur);
        std::cout<<"当前位姿"<<data.oMi[model.nv].translation().transpose()<<std::endl;

        qd_cur=q_cur;

        vd_cur=(qd_cur-q_cur)/timeStep;
        accd_cur=(vd_cur-vd_used)/timeStep;
 
        qd_used=qd_cur;
        vd_used=vd_cur;
        pinocchio::forwardKinematics(model,data,q_cur);
        std::cout<<"init success!"<<std::endl;
    }


}



#endif