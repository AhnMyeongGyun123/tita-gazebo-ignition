#include "tita.hpp"
#include "tita_robot_main.hpp"

using namespace std;
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;
using namespace RigidBodyDynamics::Utils;

std::atomic<bool> keepRunning{true};

void signalHandler(int sig) {
    std::cout << "\nInterrupt signal (" << sig << ") received.\n";
    keepRunning = false;
}

void installSignalHandler() {
    std::signal(SIGINT, signalHandler);
}

Tita tita_data;

tita::tita() : Node("TitaControl") {
    this->Lctrl = this->create_publisher<std_msgs::msg::Float64MultiArray>("/L_effort_controller/commands", 1);
    this->Rctrl = this->create_publisher<std_msgs::msg::Float64MultiArray>("/R_effort_controller/commands", 1);

    this->JoySub      = this->create_subscription<sensor_msgs::msg::Joy>("/joy", 1, std::bind(&tita::JoyCallBack, this, std::placeholders::_1));

    this->jointdata   = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 1, std::bind(&tita::JointStateCallBack, this, std::placeholders::_1));

    this->ImuData     = this->create_subscription<sensor_msgs::msg::Imu>("/imu_sensor_broadcaster/imu", 1, std::bind(&tita::ImuCallBack, this, std::placeholders::_1));
}

tita::~tita() {
    std::cout << "FINISH TITA" << std::endl;
}

void tita::JoyCallBack(const sensor_msgs::msg::Joy::SharedPtr JoyMsg) {
    JOYdx       = JoyMsg->axes[1] * max_dx;
    JOYdyaw     = JoyMsg->axes[0] * max_dx;
    JOYpitch    = JoyMsg->axes[4] * max_dx;
    JOYroll     = JoyMsg->axes[3] * max_dx;

    JOYBtnA     = JoyMsg->buttons[0];
    JOYBtnB     = JoyMsg->buttons[1];
    JOYBtnX     = JoyMsg->buttons[2];
    JOYBtnY     = JoyMsg->buttons[3];

    if (JoyMsg->axes.size() >= 8) {
        dpad_lr = JoyMsg->axes[6];
        dpad_ud = JoyMsg->axes[7];
    } else {
        dpad_lr = 0.0;
        dpad_ud = 0.0;
    }
}

void tita::JointStateCallBack(const sensor_msgs::msg::JointState::SharedPtr JointMsg) {

    std::unordered_map<std::string, double> pos_map;
    std::unordered_map<std::string, double> vel_map;

    for (size_t i = 0; i < JointMsg->name.size(); ++i) {
        pos_map[JointMsg->name[i]] = JointMsg->position[i];
        vel_map[JointMsg->name[i]] = JointMsg->velocity[i];
    }

    tita_data.Left.Pos.Hip       = pos_map["LHR_JOINT"];
    tita_data.Left.Pos.Sholder   = pos_map["LHP_JOINT"];
    tita_data.Left.Pos.Knee      = pos_map["LKP_JOINT"];
    tita_data.Left.Pos.Wheel     = pos_map["LW_JOINT"];

    tita_data.Left.Vel.Hip       = vel_map["LHR_JOINT"];
    tita_data.Left.Vel.Sholder   = vel_map["LHP_JOINT"];
    tita_data.Left.Vel.Knee      = vel_map["LKP_JOINT"];
    tita_data.Left.Vel.Wheel     = vel_map["LW_JOINT"];

    tita_data.Right.Pos.Hip       = pos_map["RHR_JOINT"];
    tita_data.Right.Pos.Sholder   = pos_map["RHP_JOINT"];
    tita_data.Right.Pos.Knee      = pos_map["RKP_JOINT"];
    tita_data.Right.Pos.Wheel     = pos_map["RW_JOINT"];

    tita_data.Right.Vel.Hip       = vel_map["RHR_JOINT"];
    tita_data.Right.Vel.Sholder   = vel_map["RHP_JOINT"];
    tita_data.Right.Vel.Knee      = vel_map["RKP_JOINT"];
    tita_data.Right.Vel.Wheel     = vel_map["RW_JOINT"];

}

void tita::ImuCallBack(const sensor_msgs::msg::Imu::SharedPtr ImuMsg) {
    quater_x = ImuMsg->orientation.x;
    quater_y = ImuMsg->orientation.y;
    quater_z = ImuMsg->orientation.z;
    quater_w = ImuMsg->orientation.w;

    ang_vel_x = ImuMsg->angular_velocity.x;
    ang_vel_y = ImuMsg->angular_velocity.y;
    ang_vel_z = ImuMsg->angular_velocity.z;

    lin_acc_x = ImuMsg->linear_acceleration.x;
    lin_acc_y = ImuMsg->linear_acceleration.y;
    lin_acc_z = ImuMsg->linear_acceleration.z;

    QuaternionToEuler(quater_x, quater_y, quater_z, quater_w, roll, pitch, yaw);
}


void tita::QuaternionToEuler(double qx, double qy, double qz, double qw, double& roll, double& pitch, double& yaw) {
    // ZYX (Yaw-Pitch-Roll) 순서

    // Roll (X-axis rotation)
    double sinr_cosp = 2.0 * (qw * qx + qy * qz);
    double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
    roll = std::atan2(sinr_cosp, cosr_cosp);

    // Pitch (Y-axis rotation)
    double sinp = 2.0 * (qw * qy - qz * qx);
    if (sinp > 1.0) sinp = 1.0;
    else if (sinp < -1.0) sinp = -1.0;

    pitch = std::asin(sinp);


    // Yaw (Z-axis rotation)
    double siny_cosp = 2.0 * (qw * qz + qx * qy);
    double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    yaw = std::atan2(siny_cosp, cosy_cosp);
}

void tita::GenProfile(double v_ref, double dt, double Amax, double *vout){
    double da = 0;
    double dv = 0;

    // Profile
    if(v_ref == *vout) {
        dv = 0;
    }
    else {
        da = (v_ref - *vout)/dt;
        if(fabs(da) >= Amax) {
        if(da>0) da = Amax;
        else 	 da = -Amax;
        }
    }

    dv = da*dt;
    *vout += dv;
}


RobotModel::RobotModel(const std::string& urdf_path) {
    if (!Addons::URDFReadFromFile(urdf_path.c_str(), &model, true)) {
        cerr << "Error loading model: " << urdf_path << endl;
        exit(1);
    }

    model.gravity = Vector3d(0, 0, -9.81);
}

RobotModel::~RobotModel() {
    cout << "FINISH MODEL" << endl;
}

void RobotModel::printModelInfo() {
    cout << "model dof : " << model.dof_count << endl;
    for (unsigned int i = 0; i <= model.dof_count; i++) {
        // string body_name = model.GetBodyName(i);
        // unsigned int body_id = model.GetBodyId(body_name.c_str());
        // cout << "Name : " << body_name << endl;
        // cout << "ID : " << body_id << endl;
        // cout << "Inertia : " << model.mBodies[i].mInertia << endl;
        // cout << "Mass : " << model.mBodies[i].mMass << endl;
        // cout << "CoM : " << model.mBodies[i].mCenterOfMass.transpose() << endl;
        // cout << "-----------------------------------------" << endl;
        // cout << "Joint " << i << " Name: " << model.GetBodyName(i) << endl;
        // cout << "JOINT : " << model.mJoints[i].mJointType << endl;
        // cout << "Parent Link: " << model.GetBodyName(model.lambda[i]) << endl;
        // cout << "-----------------------------------------" << endl;
        unsigned int joint_id = model.mJointUpdateOrder[i];
        string joint_body_name = model.GetBodyName(joint_id);
        unsigned int body_id = model.GetBodyId(joint_body_name.c_str());
        string body_name = model.GetBodyName(body_id);

        cout << "Joint " << i << " Joint body: " << joint_body_name << " Joint ID " << joint_id << endl;
        cout << "Joint " << i << " link  body: " << body_name << " Body ID " << body_id << endl;
        cout << "  q_index: " << model.mJoints[joint_id].q_index << endl;
        // cout << "  dof count: " << model.mJoints[i].mDoFCount << endl;
        cout << "-----------------------------------------" << endl;
    }
}

// PD Controller
pdcontroller::pdcontroller(double kp, double kd)
    : Kp(kp), Kd(kd), previous_error(0) {}

double pdcontroller::pdcalculate(double th_ref, double dth_ref, double th, double dth) {
    // P Calculate
    double th_error = th_ref - th;

    // D Calculate
    double dth_error = dth_ref - dth;

    double output = Kp*th_error + Kd*dth_error;

    return output;
}
