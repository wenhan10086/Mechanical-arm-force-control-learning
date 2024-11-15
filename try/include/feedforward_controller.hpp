// Copyright 1996-2023 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef KUKALBRIIWAPLAYER_HPP
#define KUKALBRIIWAPLAYER_HPP

#include <webots/Robot.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Motor.hpp>
#include <webots/LED.hpp>
#include <webots/Speaker.hpp>
#include <webots/Camera.hpp>
#include <webots/Keyboard.hpp>
#include <pinocchio/multibody/sample-models.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>


#include <iostream>

using namespace webots;
using namespace std;

class KukaLBRIIWAPlayer : public Robot {
public:
  KukaLBRIIWAPlayer();   // 构造函数
  virtual ~KukaLBRIIWAPlayer();  // 析构函数
  void myStep();   // 执行单步操作
  void wait(int ms);  // 等待指定毫秒数
  void run();  // 运行机器人

private:
  int mTimeStep;  // 时间步长
  Motor *mMotors[7];  // 电机数组
  PositionSensor *mPositionSensors[7];  // 位置传感器数组
  LED *mHeadLED;  // 头部LED
  Camera *mCamera;  // 摄像头
  Speaker *mSpeaker;  // 扬声器
};

#endif  // KUKALBRIIWAPLAYER_HPP
