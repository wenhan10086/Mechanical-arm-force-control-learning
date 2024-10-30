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
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
// All the webots classes are defined in the "webots" namespace
using namespace webots;

// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
int main(int argc, char **argv) {
  // create the Robot instance.
  Supervisor *supervisor= new Supervisor();
  Node* target_ball=supervisor->getFromDef("ball");
  int timeStep = (int)supervisor->getBasicTimeStep();
  // const double* position=target_ball->getPosition();
  
  // get the time step of the current world.
  // const Field* target_position=target_ball->get_field("translation");
  // You should insert a getDevice-like function in order to get the
  // instance of a device of the robot. Something like:
  //  Motor *motor = robot->getMotor("motorname");
  //  DistanceSensor *ds = robot->getDistanceSensor("dsname");
  //  ds->enable(timeStep);
   while (supervisor->step(timeStep) != -1)
   {
    const double* position=target_ball->getPosition();
    // std::cout << "Ball position: " << position[0] << " " << position[1] << " " << position[2] << std::endl;
      printf("Ball position: (%f, %f, %f)\n", position[0], position[1], position[2]);
   }
    // Enter here functions to send actuator commands, like:
    //  motor->setPosition(10.0);
  

  // Enter here exit cleanup code.

  delete supervisor;
  return 0;
}
