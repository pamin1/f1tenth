/**
 * @brief This file implements an MPCC controller on an Ackermann Drive Vehicle.
 *          First we get the current state, then compensate for delay and
 *          augment our old QP. Additionally we need to adjust our borders and
 *          account for obstacles/other vehicles on track. From there we can
 *          linearize + discretize the system to build the model and cost function,
 *          as well as solve the QP. From there we have the information to send a
 *          control.
 * @author Prachit Amin
 */

#include <mpcc/state_estimator.h>
#include <rclcpp/rclcpp.hpp>

#include <iostream>
#include <vector>
#include <random>
#include <fstream>
using namespace Eigen;

int main() {
    return 0;
}