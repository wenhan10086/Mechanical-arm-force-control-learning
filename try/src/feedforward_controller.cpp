#include <webots/Robot.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Motor.hpp>
#include <webots/LED.hpp>
#include <webots/Speaker.hpp>
#include <webots/Camera.hpp>
#include <webots/Keyboard.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include "pinocchio/parsers/urdf.hpp"
#include <Eigen/Dense>

#include <iostream>

using namespace webots;
using namespace std;
using namespace pinocchio;

static const char *motorNames[7] = {
    "joint_a1", "joint_a2", "joint_a3", "joint_a4", "joint_a5", "joint_a6", "joint_a7"};

class KukaLBRIIWAPlayer : public Robot
{
public:
  KukaLBRIIWAPlayer();
  virtual ~KukaLBRIIWAPlayer();
  void myStep();
  void wait(int ms);
  void run();
  void computeFeedforwardTorques();
  void computeFeedforwardPID();

private:
  int mTimeStep;
  Motor *mMotors[7];
  PositionSensor *mPositionSensors[7];

  Model model;            // Pinocchio 模型
  Data data;              // Pinocchio 数据
  Eigen::VectorXd q;      // 机器人关节配置（角度）
  Eigen::VectorXd q_dot;  // 关节速度
  Eigen::VectorXd q_ddot; // 关节加速度
  Eigen::VectorXd tau_ff; // 前馈力矩
  double Kp[7], Ki[7], Kd[7];
  double integral[7], previous_error[7];
  double targetAngles[7]={0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  double targetVelocity[7]={0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
};

KukaLBRIIWAPlayer::KukaLBRIIWAPlayer() : Robot()
{
  // 获取时间步长
  std::cout<<"okok"<<std::endl;
  mTimeStep = getBasicTimeStep();
  // 初始化Pinocchio模型和数据
  const std::string urdf_filename = std::string("/home/wenhan/下载/kuka_iiwa_test/src/iiwa_stack/iiwa_description/urdf/lbr_iiwa_14_r820.urdf");
  pinocchio::urdf::buildModel(urdf_filename, model);
  std::cout << "model name: " << model.name << std::endl;
  cout << "--- feedforward controller of KUKA LBR iiwa ---" << endl;
  data = pinocchio::Data(model);
  
  // 获取电机和位置传感器
  for (int i = 0; i < 7; i++)
  {
    mMotors[i] = getMotor(motorNames[i]);
    mPositionSensors[i] = getPositionSensor(motorNames[i] + string("_sensor"));
    mPositionSensors[i]->enable(mTimeStep);

    // 初始化 PID 参数
    Kp[i] = 1.0;  
    Ki[i] = 0.1;  
    Kd[i] = 0.01; 

    // 初始化误差和积分值
    integral[i] = 0.0;
    previous_error[i] = 0.0;
  }

  // 初始化关节配置、速度和加速度
  q = Eigen::VectorXd::Zero(7);
  q_dot = Eigen::VectorXd::Zero(7);
  q_ddot = Eigen::VectorXd::Zero(7);
  tau_ff = Eigen::VectorXd::Zero(7);
}

KukaLBRIIWAPlayer::~KukaLBRIIWAPlayer()
{
  // 这里可以添加清理代码，如果有资源需要释放
}

void KukaLBRIIWAPlayer::myStep()
{
  if (step(mTimeStep) == -1)
    exit(EXIT_SUCCESS);
}

void KukaLBRIIWAPlayer::wait(int ms)
{
  double startTime = getTime();
  double s = (double)ms / 1000.0;
  while (s + startTime >= getTime())
    myStep();
}

void KukaLBRIIWAPlayer::computeFeedforwardTorques()
{
  // 读取机器人当前的关节角度和速度
  for (int i = 0; i < 7; i++)
  {
    q(i) = mPositionSensors[i]->getValue();
    q_dot(i) = mMotors[i]->getVelocity();
  }

  // 假设一个简单的加速度参考（你可以将其替换为实际的轨迹生成函数）
  q_ddot.setZero(); // 设置为零表示没有加速度变化，你可以根据需要添加实际的加速度参考

  // 使用反向动力学（RNEA）计算前馈力矩
  tau_ff = rnea(model, data, q, q_dot, q_ddot);

  // 将前馈力矩输出到电机控制器
  for (int i = 0; i < 7; i++)
  {
    mMotors[i]->setPosition(tau_ff(i)); // 将力矩作为位置输入给电机，通常是转动的目标
    mMotors[i]->setVelocity(0.5);       // 控制速度，调节此值以实现平稳运动
  }
}

void KukaLBRIIWAPlayer::computeFeedforwardPID()
{
  // 读取关节角度和速度
  for (int i = 0; i < 7; i++)
  {
    q(i) = mPositionSensors[i]->getValue();
    q_dot(i) = mMotors[i]->getVelocity();
    cout << "q: " << q << endl
         << "q_dot: " << q_dot << endl;
  }

  q_ddot.setZero(); 
  tau_ff = rnea(model, data, q, q_dot, q_ddot);

  // PID 控制
  for (int i = 0; i < 7; i++)
  {
    double error = targetAngles[i] - q(i); // 计算目标位置与当前的位置误差
    double derivative = targetVelocity[i] - q_dot(i);

    double tau_pid = Kp[i] * error + Kd[i] * derivative;

    double total_tau = tau_ff(i) + tau_pid;

    mMotors[i]->setTorque(total_tau);
  }
}

void KukaLBRIIWAPlayer::run()
{
  // 启动前馈PID控制
  computeFeedforwardPID();

  wait(5000);

  // 控制移动至另一组目标位置
  double newTargetAngles[7] = {0.1, -0.2, 0.6, -0.1, 0.5, -0.1, 0.5}; // 新的目标角度
  double newTargetVelocity[7] = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1}; // 新的目标速度
  for (int i = 0; i < 7; i++)
  {
    targetAngles[i] = newTargetAngles[i];
    targetVelocity[i] = newTargetVelocity[i];
  }

  computeFeedforwardPID();

  wait(5000);

  // 继续执行更多动作，或者执行其他任务
  while (true)
  {
    for (int i = 0; i < 7; i++)
    {
      double position = mPositionSensors[i]->getValue();
      cout << "Joint " << i + 1 << " Position: " << position << endl;
    }

    // myStep();
  }
}
