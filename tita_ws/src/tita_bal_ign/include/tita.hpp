#ifndef TITA_HPP
#define TITA_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>
#include <csignal>
#include <iostream>
#include <atomic>

struct JointData {
    double Hip;
    double Sholder;
    double Knee;
    double Wheel;
    double Tau;
};

struct Leg {
    JointData Pos;
    JointData Vel;
    JointData Tau;
};

struct Tita {
    Leg Left;
    Leg Right;
};

extern std::atomic<bool> keepRunning;
extern Tita tita_data;

class tita : public rclcpp::Node {
public:
    tita();
    ~tita();

    void JoyCallBack(const sensor_msgs::msg::Joy::SharedPtr JoyMsg);
    void JointStateCallBack(const sensor_msgs::msg::JointState::SharedPtr JointMsg);
    void ImuCallBack(const sensor_msgs::msg::Imu::SharedPtr ImuMsg);
    void QuaternionToEuler(double qx, double qy, double qz, double qw, double& roll, double& pitch, double& yaw);
    void GenProfile(double v_ref, double dt, double Amax, double *vout);

public:
    double JOYdx, JOYdyaw, JOYpitch, JOYroll;
    bool JOYBtnA, JOYBtnB, JOYBtnX, JOYBtnY;
    double quater_x, quater_y, quater_z, quater_w;
    double ang_vel_x, ang_vel_y, ang_vel_z;
    double lin_acc_x, lin_acc_y, lin_acc_z;
    double roll, pitch, yaw;
    double max_dx = 1.0;
    double dpad_lr = 0.0;
    double dpad_ud = 0.0;

public:
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr Lctrl;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr Rctrl;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr JoySub;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointdata;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr ImuData;
};

class RobotModel {
public:
    RobotModel(const std::string& urdf_path);
    ~RobotModel();
    void printModelInfo();
    RigidBodyDynamics::Model model;
};

class pdcontroller {
public:
    pdcontroller(double kp, double kd);
    double pdcalculate(double th_ref, double dth_ref, double th, double dth);

private:
    double Kp;
    double Kd;
    double previous_error;
};

void signalHandler(int sig);
void installSignalHandler();

#endif // TITA_HPP
