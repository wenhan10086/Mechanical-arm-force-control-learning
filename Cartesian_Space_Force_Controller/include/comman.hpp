#ifndef COMMAN_H
#define COMMAN_H

#include <Eigen/Dense>
#include <iostream>



Eigen::VectorXd computeVelocity(
    const Eigen::VectorXd& pose_prev, 
    const Eigen::VectorXd& pose_cur, 
    double delta_t, 
    const std::string& frame = "world") ;

#endif