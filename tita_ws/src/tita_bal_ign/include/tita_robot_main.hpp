#ifndef TITA_ROBOT_MAIN_HPP
#define TITA_ROBOT_MAIN_HPP

#include <iostream>
#include <vector>
#include <chrono>
#include <cmath>
#include <csignal>
#include <atomic>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <Eigen/Dense>
#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>
#include <tita.hpp>


#define DEG_TO_RAD(x) ((x) * M_PI / 180.0)


enum Mode {
    INIT,
    GO,
    BACK,
    LEFT,
    RIGHT
};

#endif // TITA_ROBOT_MAIN_HPP
