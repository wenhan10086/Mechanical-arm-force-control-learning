#include<webots/Robot.hpp>
#include<webots/Motor.hpp>
#include<webots/Supervisor.hpp>
#include<webots/PositionSensor.hpp>
#include<webots/Node.hpp>
#include<stdio.h>
#include<iostream>
#include<fstream>
#include<Eigen/Dense>

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/aba.hpp"
#include "worldInit_C.hpp"

#include<yaml-cpp/yaml.h>

#include <qpOASES.hpp>

#include "mPinocchioIK.hpp"
#include "comman.hpp"


#ifndef PINOCCHIO_MODEL_DIR
  #define PINOCCHIO_MODEL_DIR "/home/wenhan/下载/kuka_iiwa_test/src/iiwa_stack/iiwa_description/urdf/"
#endif

using namespace webots;
using namespace pinocchio;




int main(int argc,char ** argv)
{
    USING_NAMESPACE_QPOASES

    double kp;
    double ki;
    double kd;

    Supervisor *supervisor= new Supervisor();
    Node* target_ball=supervisor->getFromDef("ball");
    int timeStep = (int)supervisor->getBasicTimeStep();
    /*初始化model*/
     const std::string urdf_filename =
    (argc <= 1) ? PINOCCHIO_MODEL_DIR
                    + std::string("iiwa14.urdf")
                : argv[1];
    
    Model model;

    pinocchio::urdf::buildModel(urdf_filename,model);

    pinocchio::Data data(model);

    int jointID=7;

    std::cout << "model name: " << model.name << std::endl;


    //提取控制器的参数
    YAML::Node controllerConfig;

    try{
        controllerConfig=YAML::LoadFile("./config/controller_config.yaml");
    }
    catch(YAML::BadFile &e){
        std::cout<<"read error!"<<std::endl;
        return -1;
    }


    double w;

    if (controllerConfig["PID"]) {
         kp = controllerConfig["PID"]["p"].as<double>();
         ki = controllerConfig["PID"]["i"].as<double>();
         kd = controllerConfig["PID"]["d"].as<double>();
         w = controllerConfig["PID"]["w"].as<double>();
    }
    //初始化
    worldInit_C _world(model,data,supervisor,target_ball);


    //主循环
    /*结构  1，更新data
            2， */
    while(supervisor->step(timeStep)!=-1)
    {       


        Eigen::VectorXd q_test = Eigen::VectorXd::Zero(model.nv);
  
        //读取此次关节的位置
        for(int i=0;i<7;i++)
        {
          _world.q_cur[i]=_world.sensors[i]->getValue();

        }

        _world.qd_cur=(_world.q_cur-_world.q_used)*1e3/timeStep;
        _world.q_used=_world.q_cur;
        _world.qd_used=_world.qd_cur;
        forwardKinematics(model,data,_world.q_cur);
        /*将机械臂末端的位姿读出*/
        const pinocchio::SE3& end_effector_pose = data.oMi[7];
            // 提取平移部分
        Eigen::Vector3d position = end_effector_pose.translation();

            // 提取旋转矩阵并转换为轴角表示
        Eigen::AngleAxisd angle_axis(end_effector_pose.rotation());
        Eigen::Vector3d orientation = angle_axis.angle() * angle_axis.axis();

            // 将位置和平移部分组合成六维向量
        Eigen::Matrix<double, 6, 1> pose;
        pose.head<3>() = position;
        pose.tail<3>() = orientation;
        _world.x_cur=pose;
        _world.v_cur=computeVelocity(_world.x_used,_world.x_cur,timeStep*1e-3,"world");
        _world.x_used=_world.x_cur;
 
        /*将此时小球的位姿读出*/
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
        _world.xt_cur=target_pose;
        _world.vt_cur=computeVelocity(_world.xt_used,_world.xt_cur,timeStep*1e-3,"world");
        _world.xt_used=_world.xt_cur;
        /*求出Eq与Ev*/

        _world.Eq=computeVelocity(_world.x_cur,_world.xt_cur,1.0,"world");
        _world.Ev=_world.vt_cur-_world.v_cur;

        
        //计算雅可比矩阵与雅可比矩阵的导，注意参考坐标系
        Eigen::MatrixXd J=pinocchio::computeJointJacobians(model,data,_world.q_cur);
        Eigen::MatrixXd Jdot=pinocchio::computeJointJacobiansTimeVariation(model,data,_world.q_cur,_world.qd_cur);
        // Eigen::VectorXd test1=J*_world.qd_cur-_world.v_cur;
        // std::cout<<"test1:"<<test1.transpose()<<std::endl;


        // //根据误差生成vdot_des

        _world.at_cur=-kp*_world.Eq-kd*(_world.Ev);
        // _world.at_cur.tail(3).setZero();
        // _world.at_cur=kp*world.Eq;
      
        // //优化求解力矩
        // /*优化问题1*/
        Eigen::MatrixXd H=2*J.transpose()*J;
        H.diagonal().array() += w;

        Eigen::VectorXd f=-2*(Jdot*_world.qd_cur-_world.at_cur).transpose()*J;
        // Eigen::VectorXd f=-2*(Jdot*_world.qd_cur-_world.at_cur).transpose()*J;
        Eigen::MatrixXd A=Eigen::MatrixXd::Identity(model.nv, model.nv);

        Eigen::VectorXd ub=Eigen::VectorXd::Constant(model.nv,1e+8);
        Eigen::VectorXd lb=Eigen::VectorXd::Constant(model.nv,-1e+8);

        
        /*优化问题2*/
        
        /*emmmmm,不想写了，直接把优化得到的qdd算到tau吧*/

        /*将得到的H,F，等组合起来*/
        real_t Hqp[model.nv*model.nv];
        for (int i = 0; i < model.nv * model.nv; ++i) {
            Hqp[i] = H.data()[i];
        }       
        
        real_t fqp[model.nv];
        for (int i = 0; i < model.nv ; ++i) {
            fqp[i] = f.data()[i];
        } 
        real_t ubqp[model.nv];
        for (int i = 0; i < model.nv ; ++i) {
            ubqp[i] = ub.data()[i];
        } 
        real_t lbqp[model.nv];
        for (int i = 0; i < model.nv ; ++i) {
            lbqp[i] = lb.data()[i];
        } 
        real_t Aqp[model.nv*model.nv];
        for (int i = 0; i < model.nv * model.nv; ++i) {
            Aqp[i] = A.data()[i];
        }       
        

/******************************************* */
        //根据优化解qdot

        QProblem targetPoseQp(model.nv,4);

        Options options;
	    targetPoseQp.setOptions( options );
        int_t nWSR = 100;

        targetPoseQp.init(Hqp,fqp,Aqp,lbqp,ubqp,NULL,NULL,nWSR);
        real_t xOpt[model.nv];

        targetPoseQp.getPrimalSolution(xOpt);

        // std::cout<<"Jdot*_world.qd_cur: "<<Jdot*_world.qd_cur<<std::endl;
        // std::cout<<"H:"<<H<<std::endl;
        // std::cout<<"f:"<<f<<std::endl;
        printf( "\nxOpt = [ %e, %e %e, %e,%e, %e,%e,  ];   objVal = %e\n\n", 
			xOpt[0],xOpt[1],xOpt[2],xOpt[3],xOpt[4],xOpt[5],xOpt[6],xOpt[7],targetPoseQp.getObjVal() );	


        std::cout<<"ok!"<<std::endl;
        
        // targetPoseQp.printOptions();
	    // targetPoseQp.printProperties();

       //将求解的qdd直接放入逆动力学中，求解力矩

        _world.qdd_cur = Eigen::Map<Eigen::VectorXd>(xOpt,model.nv);

        Eigen::VectorXd tau=rnea(model,data,_world.q_cur,_world.qd_cur,_world.qdd_cur);

        std::cout<<"_world.q_cur:"<<_world.q_cur.transpose()<<std::endl;
        std::cout<<"_world.qd_cur:"<<_world.qd_cur.transpose()<<std::endl;
        std::cout<<"_world.qdd_cur:"<<_world.qdd_cur.transpose()<<std::endl;
         std::cout<<"target_xt:"<<_world.xt_cur.transpose()<<std::endl;
        std::cout<<"target_vt:"<<_world.vt_cur.transpose()<<std::endl;
        std::cout<<"x_cur:"<<_world.x_cur.transpose()<<std::endl;
        std::cout<<"v_cur:"<<_world.v_cur.transpose()<<std::endl;
        std::cout<<"_world.Eq:"<<_world.Eq.transpose()<<std::endl;
        std::cout<<"_world.Ev:"<<_world.Ev.transpose()<<std::endl;
        std::cout<<"_world.at_cur:"<<_world.at_cur.transpose()<<std::endl;
        std::cout<<"J:"<<J<<std::endl;
        std::cout<<"Jdot:"<<Jdot<<std::endl;

        for(int i=0;i<model.nv;i++)
        {
            _world.joints[i]->setTorque(tau[i]);
        }
        std::cout<<"力矩为："<<tau.transpose()<<std::endl;
        for(int i=0;i<7;i++)
        {
          q_test[i]=_world.sensors[i]->getValue();

        }
        
        
    }
    return 0;
}   

