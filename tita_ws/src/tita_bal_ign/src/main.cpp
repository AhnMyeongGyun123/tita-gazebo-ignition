#include "tita_robot_main.hpp"
#include "tita.hpp"
#include <std_msgs/msg/float64_multi_array.hpp>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <unsupported/Eigen/MatrixFunctions>
#include <OsqpEigen/OsqpEigen.h>
#include <osqp.h>
#include <iomanip>
#include <fstream>

constexpr double g = 9.81;
constexpr int    H  = 40; // MPC 높이
constexpr double wheel_radius = 0.1;
constexpr double v_target = 2; // m/s
constexpr double tau_v = 0.8;
constexpr double torque_lim = 10.0;
constexpr double turn_torque = 0.3;
constexpr double K_ff_turn   = 1.0;

static double g_mass = 0.0; // total_mass
static double h = 0.0; // com 높이
static double Iy = 0.0;
static double v_ref_cmd = 0.0;
static double x_int = 0.0;
double dt = 0.002;
double yaw_ref = 0.0;
double u_yaw = 0.0;
double LW_vel = 0.0;
double RW_vel = 0.0;

static double LW_x_prev = 0.0;
static double RW_x_prev = 0.0;
static double LW_pos_prev = 0.0;
static double RW_pos_prev = 0.0;
static bool wheel_vel_init = false;

static bool mpc_ready = false;
static int  imu_warmup_count = 0;
constexpr double AXIS_DEAD   = 0.12;
constexpr double ACC_MAX = 0.2;

using namespace std;
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;
using namespace RigidBodyDynamics::Utils;
rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr tita_wheel_controller_pub;
bool wheels_enables = false;

Eigen::SparseMatrix<double> Hs, C;
Eigen::VectorXd grad, lb, ub;
Eigen::Matrix<double,4,4> A;
Eigen::Matrix<double,4,1> B;
Eigen::Matrix<double,4,1> xMax, xMin;
Eigen::Matrix<double,1,1> uMax, uMin;
Eigen::DiagonalMatrix<double,4> Qw;
Eigen::DiagonalMatrix<double,1> Rw;

void computeCOMJacobian(Model &model, const VectorNd &q, MatrixNd &J_com) {
    J_com = MatrixNd::Zero(3, model.q_size);
    double total_mass = 0.0;

    for (unsigned int i = 1; i < model.mBodies.size(); ++i) {
        double m_i = model.mBodies[i].mMass;
        Vector3d com_local = model.mBodies[i].mCenterOfMass;

        MatrixNd J_i = MatrixNd::Zero(3, model.q_size);
        CalcPointJacobian(model, q, i, com_local, J_i, true);

        J_com += m_i * J_i;
        total_mass += m_i;
    }

    J_com /= total_mass;
}

struct SE3wB {
  RigidBodyDynamics::Math::Matrix3d R_WB;
  RigidBodyDynamics::Math::Vector3d p_WB;
};

SE3wB world_T_B(RigidBodyDynamics::Model& model,
                const RigidBodyDynamics::Math::VectorNd& q,
                unsigned int body_B_id) {
  using namespace RigidBodyDynamics::Math;
  Matrix3d R_BW = CalcBodyWorldOrientation(model, q, body_B_id, true);
  Matrix3d R_WB = R_BW.transpose();
  Vector3d p_WB = CalcBodyToBaseCoordinates(model, q, body_B_id, Vector3d::Zero(), true);
  return SE3wB{R_WB, p_WB};
}

RigidBodyDynamics::Math::Vector3d world_to_B(const SE3wB& WT_B,
                                             const RigidBodyDynamics::Math::Vector3d& p_W) {
  return WT_B.R_WB.transpose() * (p_W - WT_B.p_WB);
}

static double getPitchInertiaAtCoM(RigidBodyDynamics::Model& model,
                                   const RigidBodyDynamics::Math::VectorNd& Q,
                                   const Eigen::Vector3d& com_world)
{
    using namespace RigidBodyDynamics;
    using namespace RigidBodyDynamics::Math;

    Eigen::Matrix3d I_total = Eigen::Matrix3d::Zero();

    for (unsigned int i = 1; i < model.mBodies.size(); ++i) {
        const Body& Bi = model.mBodies[i];
        const double mi = static_cast<double>(Bi.mMass);

        Vector3d p_i_rbdl = CalcBodyToBaseCoordinates(model, Q, i, Bi.mCenterOfMass, true);
        Eigen::Vector3d p_i(p_i_rbdl[0], p_i_rbdl[1], p_i_rbdl[2]);

        Matrix3d R_bw = CalcBodyWorldOrientation(model, Q, i, false);
        Eigen::Matrix3d R = R_bw.transpose();
        Eigen::Matrix3d Ic_body = Eigen::Matrix3d::Zero();
        Ic_body(0,0)=Bi.mInertia(0,0); Ic_body(0,1)=Bi.mInertia(0,1); Ic_body(0,2)=Bi.mInertia(0,2);
        Ic_body(1,0)=Bi.mInertia(1,0); Ic_body(1,1)=Bi.mInertia(1,1); Ic_body(1,2)=Bi.mInertia(1,2);
        Ic_body(2,0)=Bi.mInertia(2,0); Ic_body(2,1)=Bi.mInertia(2,1); Ic_body(2,2)=Bi.mInertia(2,2);

        Eigen::Matrix3d Ic_world = R * Ic_body * R.transpose();

        Eigen::Vector3d r = p_i - com_world;
        double r2 = r.squaredNorm();
        Eigen::Matrix3d I_shift = mi * (r2 * Eigen::Matrix3d::Identity() - r * r.transpose());

        I_total += Ic_world + I_shift;
    }
    return I_total(1,1);

}

void setDynamicsMatrices(Eigen::Matrix<double, 4, 4> &a, Eigen::Matrix<double, 4, 1> &b, double dt)
{

    double denom = g_mass * h * h + Iy;
    double alpha_x = (g_mass * g * h) / denom;
    double alpha_th = (g_mass * g * h) / Iy;
    double beta  = 1.0 / g_mass;
    double gamma = h / denom;

    // a << 0., 0., 1., 0.,
    //      0., 0., 0., 1.,
    //      0., -22.697, 0., 0.,
    //      0., -23.8832, 0., 0.;

    // b << 0.,
    //      0.,
    //      0.0361144 * (2.0 / wheel_radius),
    //      -0.0835564 * (2.0 / wheel_radius);

    a << 0., 0., 1., 0.,
         0., 0., 0., 1.,
         0., -21.8173, 0., 0.,
         0., -24.196, 0., 0.;

    b << 0.,
         0.,
         0.0361144 * (2.0 / wheel_radius),
         -0.0803178 * (2.0 / wheel_radius);

    // a << 0., 0., 1., 0.,
    //      0., 0., 0., 1.,
    //      0., alpha_x, 0., 0.,
    //      0., alpha_th, 0., 0.;

    // b << 0.,
    //      0.,
    //      beta * (2.0 / wheel_radius),
    //      gamma * (2.0 / wheel_radius);

    Eigen::MatrixXd c = Eigen::MatrixXd::Identity(4,4);
    Eigen::MatrixXd d = Eigen::MatrixXd::Zero(4,1);

    Eigen::MatrixXd M = Eigen::MatrixXd::Zero(a.rows() + b.cols(), a.rows() + b.cols());
    M.topLeftCorner(a.rows(), a.cols()) = a * dt;
    M.topRightCorner(a.rows(), b.cols()) = b * dt;

    Eigen::MatrixXd M_exp = M.exp();

    Eigen::MatrixXd Ad = M_exp.topLeftCorner(a.rows(), a.cols());
    Eigen::MatrixXd Bd = M_exp.topRightCorner(a.rows(), b.cols());

    Eigen::MatrixXd Cd = c;
    Eigen::MatrixXd Dd = d;

    a = Ad;
    b = Bd;
}

void setInequalityConstraints(Eigen::Matrix<double, 4, 1> &xMax, Eigen::Matrix<double, 4, 1> &xMin,
                              Eigen::Matrix<double, 1, 1> &uMax, Eigen::Matrix<double, 1, 1> &uMin)
{
    uMin << -torque_lim;
    uMax << torque_lim;
    xMin << -OsqpEigen::INFTY, -OsqpEigen::INFTY, -2.0, -OsqpEigen::INFTY;
    xMax << OsqpEigen::INFTY, OsqpEigen::INFTY, 2.0, OsqpEigen::INFTY;
}

void setWeightMatrices(Eigen::DiagonalMatrix<double, 4> &Q, Eigen::DiagonalMatrix<double, 1> &R)
{
    Q.diagonal() << 2000, 8000, 400, 50;  // x theta dx dtheta  170 4000 400 50
    R.diagonal() << 0.05;
}

void castMPCToQPHessian(const Eigen::DiagonalMatrix<double, 4> &Q, const Eigen::DiagonalMatrix<double, 1> &R, int mpcWindow,
                        Eigen::SparseMatrix<double> &hessianMatrix)
{
    hessianMatrix.resize(4*(mpcWindow+1) + 1 * mpcWindow, 4*(mpcWindow+1) + 1 * mpcWindow);
    for(int i = 0; i<4*(mpcWindow+1) + 1 * mpcWindow; i++){
        if(i < 4*(mpcWindow+1)){
            int posQ=i%4;
            float value = Q.diagonal()[posQ];
            if(value != 0)
                hessianMatrix.insert(i,i) = value;
        }
        else{
            int posR=i%1;
            float value = R.diagonal()[posR];
            if(value != 0)
                hessianMatrix.insert(i,i) = value;
        }
    }
}

void castMPCToQPGradient(const Eigen::DiagonalMatrix<double, 4> &Q, const Eigen::Matrix<double, 4, 1> &xRef, int mpcWindow,
                         Eigen::VectorXd &gradient)
{
    Eigen::Matrix<double,4,1> Qx_ref;
    Qx_ref = Q * (-xRef);
    gradient = Eigen::VectorXd::Zero(4*(mpcWindow+1) +  1*mpcWindow, 1);
    for(int i = 0; i<4*(mpcWindow+1); i++){
        int posQ=i%4;
        float value = Qx_ref(posQ,0);
        gradient(i,0) = value;
    }
}

void castMPCToQPConstraintMatrix(const Eigen::Matrix<double, 4, 4> &dynamicMatrix, const Eigen::Matrix<double, 4, 1> &controlMatrix,
                                 int mpcWindow, Eigen::SparseMatrix<double> &constraintMatrix)
{
    constraintMatrix.resize(4*(mpcWindow+1)  + 4*(mpcWindow+1) + 1 * mpcWindow, 4*(mpcWindow+1) + 1 * mpcWindow);
    for(int i = 0; i<4*(mpcWindow+1); i++){
        constraintMatrix.insert(i,i) = -1;
    }
    for(int i = 0; i < mpcWindow; i++)
        for(int j = 0; j<4; j++)
            for(int k = 0; k<4; k++){
                float value = dynamicMatrix(j,k);
                if(value != 0){
                    constraintMatrix.insert(4 * (i+1) + j, 4 * i + k) = value;
                }
            }
    for(int i = 0; i < mpcWindow; i++)
        for(int j = 0; j < 4; j++)
            for(int k = 0; k < 1; k++){
                float value = controlMatrix(j,k);
                if(value != 0){
                    constraintMatrix.insert(4*(i+1)+j, 1*i+k+4*(mpcWindow + 1)) = value;
                }
            }
    for(int i = 0; i<4*(mpcWindow+1) + 1*mpcWindow; i++){
        constraintMatrix.insert(i+(mpcWindow+1)*4,i) = 1;
    }
}

void castMPCToQPConstraintVectors(const Eigen::Matrix<double, 4, 1> &xMax, const Eigen::Matrix<double, 4, 1> &xMin,
                                   const Eigen::Matrix<double, 1, 1> &uMax, const Eigen::Matrix<double, 1, 1> &uMin,
                                   const Eigen::Matrix<double, 4, 1> &x0,
                                   int mpcWindow, Eigen::VectorXd &lowerBound, Eigen::VectorXd &upperBound)
{
    Eigen::VectorXd lowerInequality = Eigen::MatrixXd::Zero(4*(mpcWindow+1) +  1 * mpcWindow, 1);
    Eigen::VectorXd upperInequality = Eigen::MatrixXd::Zero(4*(mpcWindow+1) +  1 * mpcWindow, 1);
    for(int i=0; i<mpcWindow+1; i++){
        lowerInequality.block(4*i,0,4,1) = xMin;
        upperInequality.block(4*i,0,4,1) = xMax;
    }
    for(int i=0; i<mpcWindow; i++){
        lowerInequality.block(1 * i + 4 * (mpcWindow + 1), 0, 1, 1) = uMin;
        upperInequality.block(1 * i + 4 * (mpcWindow + 1), 0, 1, 1) = uMax;
    }
    Eigen::VectorXd lowerEquality = Eigen::MatrixXd::Zero(4*(mpcWindow+1),1 );
    Eigen::VectorXd upperEquality;
    lowerEquality.block(0,0,4,1) = -x0;
    upperEquality = lowerEquality;
    lowerEquality = lowerEquality;
    lowerBound = Eigen::MatrixXd::Zero(2*4*(mpcWindow+1) +  1*mpcWindow,1 );
    lowerBound << lowerEquality,
        lowerInequality;
    upperBound = Eigen::MatrixXd::Zero(2*4*(mpcWindow+1) +  1*mpcWindow,1 );
    upperBound << upperEquality,
        upperInequality;
}

int main(int argc, char **argv) {
    bool dataLogging = true;
    std::vector<std::string> DataLog_append;
    std::ostringstream data_log_dir;
    std::chrono::system_clock::time_point nowTime = std::chrono::system_clock::now();
    std::time_t in_time_t = std::chrono::system_clock::to_time_t(nowTime);

    RobotModel robot("/home/amg/tita_0309_ws/tita_ws/src/tita_bal_ign/model/urdf/tita2.urdf");

    Mode mode = INIT;
    Mode target_mode = INIT;
    // static Mode prev_mode = Mode::INIT;
    // if (mode != prev_mode) {
    //     //v_ref_cmd = 0.0;
    //     if (mode ==INIT){
    //     v_target_raw = 0.0;
    //     v_ref_cmd    = 0.0;

    //     // 현 상태 캡처
    //     x_hold  = x0(0);
    //     v_enter = x0(2);

    //     // (유지) 제약과 warm-start 초기화
    //     xMin(2) = -0.12; xMax(2) = +0.12;
    //     uMin(0) = -6.0;  uMax(0) = +6.0;
    //     solver.settings()->setWarmStart(false);
    //     solver.initSolver();
    //     solver.settings()->setWarmStart(true);

    //     std::cout << "[MODE CHANGE → INIT] freeze x to current" << std::endl;
    //     }
    //     prev_mode = mode;
    // }

    VectorNd Q = VectorNd::Zero(robot.model.q_size);
    VectorNd QDot = VectorNd::Zero(robot.model.qdot_size);
    VectorNd QDDot = VectorNd::Zero(robot.model.q_size);
    VectorNd Q_ref = VectorNd::Zero(robot.model.q_size);
    VectorNd QDot_ref = VectorNd::Zero(robot.model.qdot_size);
    VectorNd QDDot_ref = VectorNd::Zero(robot.model.q_size);
    RigidBodyDynamics::Math::Scalar mass;
    RigidBodyDynamics::Math::Vector3d com_pos, com_vel;
    RigidBodyDynamics::Math::Scalar total_mass = 0.0;
    RigidBodyDynamics::Math::VectorNd Qdot_zero = RigidBodyDynamics::Math::VectorNd::Zero(robot.model.qdot_size);

    // double LW_vel = 0.0;
    // double RW_vel = 0.0;
    // double prev_LW_pos = 0.0;
    // double prev_RW_pos = 0.0;
    // bool has_prev_wheel = false;
    // RigidBodyDynamics::Utils::CalcCenterOfMass(robot.model, Q, QDot, nullptr, total_mass, com_pos, &com_vel);

    rclcpp::init(argc, argv);
    auto node = std::make_shared<tita>();
    rclcpp::Rate loop_rate(1.0 / dt);

    tita_wheel_controller_pub = node->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/tita_wheel_controller/commands",
        rclcpp::QoS(10).best_effort());

    installSignalHandler();

    auto last_time = std::chrono::steady_clock::now();
    double t_global;
    double current_forward_vel = 0.0;
    double v_target_raw = 0.0;

    Eigen::Matrix<double,4,1> x0;
    x0 << 0.0, 0.0, 0.0, 0.0;

    Eigen::Matrix<double,4,1> xRef;
    xRef << 0.0, 0.0, 0.0, 0.0;

    setDynamicsMatrices(A, B, dt);
    setWeightMatrices(Qw, Rw);
    setInequalityConstraints(xMax, xMin, uMax, uMin);

    castMPCToQPHessian(Qw, Rw, H, Hs);
    castMPCToQPGradient(Qw, xRef, H, grad);
    castMPCToQPConstraintMatrix(A, B, H, C);
    castMPCToQPConstraintVectors(xMax, xMin, uMax, uMin, x0, H, lb, ub);

    OsqpEigen::Solver solver;
    solver.settings()->setWarmStart(true);
    solver.settings()->setVerbosity(false);
    solver.settings()->setMaxIteration(200);

    const int nx = 4, nu = 1;
    const int nvar = nx*(H+1) + nu*H;
    const int ncon = nx*(H+1) + nx*(H+1) + nu*H;

    solver.data()->setNumberOfVariables(nvar);
    solver.data()->setNumberOfConstraints(ncon);
    solver.data()->setHessianMatrix(Hs);
    solver.data()->setGradient(grad);
    solver.data()->setLinearConstraintsMatrix(C);
    solver.data()->setLowerBound(lb);
    solver.data()->setUpperBound(ub);

    if (!solver.initSolver()) {
        RCLCPP_ERROR(node->get_logger(), "OSQP initSolver failed");
    }
    //enum Mode { INIT, GO, BACK };
    enum InitPhase { BRAKE, HOLD };
    static int prevA = 0, prevB = 0, prevX = 0;
    static int prevUp = 0, prevDown = 0;
    static int prevLeft = 0, prevRight = 0;
    int joyA = 0, joyB = 0, joyX = 0, joyup = 0, joydown = 0, joyleft = 0, joyright = 0;

    static InitPhase init_phase = HOLD;
    static double t_init_enter = 0.0;
    static double v_enter = 0.0;
    static double x_hold = 0.0;
    static bool braking_from_go   = false;
    static bool braking_from_back = false;

    const int imu_warmup_period = 1500; // 3 seconds at 500Hz
    static double pitch_offset = 0.0;
    bool calibrated = false;

    RigidBodyDynamics::Math::Vector3d LW_pos;
    RigidBodyDynamics::Math::Vector3d RW_pos;

    // RigidBodyDynamics::Math::Vector3d LW_vel;
    // RigidBodyDynamics::Math::Vector3d RW_vel;

    while (rclcpp::ok() && keepRunning)
    {
        double LW_pos_now = tita_data.Left.Pos.Wheel;
        double RW_pos_now = tita_data.Right.Pos.Wheel;

        double LW_omega = (LW_pos_now - LW_pos_prev) / dt;  
        double RW_omega = (RW_pos_now - RW_pos_prev) / dt;  
        LW_vel = wheel_radius * LW_omega;   // m/s
        RW_vel = wheel_radius * RW_omega;   // m/s

        LW_pos_prev = LW_pos_now;
        RW_pos_prev = RW_pos_now;
        // std::cout << "Q : " << robot.model.q_size << std::endl;
        Q[0] = LW_pos_now;
        Q[1] = RW_pos_now;
        Q[2] = 0;
        Q[3] = node->roll;
        Q[4] = node->pitch;
        Q[5] = node->yaw;
        
        Q[6] = 0;
        Q[7] = 0;
        
        QDot[0] = LW_omega;
        QDot[1] = RW_omega;
        QDot[2] = 0;
        QDot[3] = node->ang_vel_x;
        QDot[4] = node->ang_vel_y;
        QDot[5] = node->ang_vel_z;
        
        QDot[6] = 0;
        QDot[7] = 0;

        CalcBaseToBodyCoordinates(robot.model, Q, robot.model.GetBodyId("LW_LINK"), LW_pos, true);
        CalcBaseToBodyCoordinates(robot.model, Q, robot.model.GetBodyId("RW_LINK"), RW_pos, true);
        RigidBodyDynamics::Utils::CalcCenterOfMass(robot.model, Q, QDot, nullptr, total_mass, com_pos, &com_vel);
        
        // Vector3d LW_vel = CalcPointVelocity(robot.model, Q, QDot, robot.model.GetBodyId("LW_LINK"), Vector3d::Zero(), true);
        // Vector3d RW_vel = CalcPointVelocity(robot.model, Q, QDot, robot.model.GetBodyId("RW_LINK"), Vector3d::Zero(), true);
        // Vector3d LW_vel;
        // Vector3d RW_vel;
        const double LW_x = LW_pos.x();
        const double RW_x = RW_pos.x();

        // if (!wheel_vel_init) {
        //     // 첫 스텝: 이전 위치만 초기화하고 속도는 0으로
        //     LW_vel = 0.0;
        //     RW_vel = 0.0;
        //     LW_x_prev = LW_x;
        //     RW_x_prev = RW_x;
        //     wheel_vel_init = true;
        // } else {
        //     // v = Δx / dt
        //     LW_vel = (LW_x - LW_x_prev) / dt;
        //     RW_vel = (RW_x - RW_x_prev) / dt;

        //     LW_x_prev = LW_x;
        //     RW_x_prev = RW_x;
        // }
        // LW_vel = (LW_x - LW_x_prev) / dt;
        // RW_vel = (RW_x - RW_x_prev) / dt;

        // LW_x_prev = LW_x;
        // RW_x_prev = RW_x;
        double v_L_ref = 0.0;
        double v_R_ref = 0.0;
        double joy_norm = 0.0;
        double joy_lr = 0.0;
        if (node->max_dx > 0.0) joy_norm = std::clamp(node->JOYdx / node->max_dx, -1.0, 1.0);
        if (node->max_dx > 0.0) joy_lr = std::clamp(node->JOYpitch / node->max_dx, -1.0, 1.0);
        if (std::abs(joy_norm) < AXIS_DEAD) joy_norm = 0.0;
        if (std::abs(joy_lr) < AXIS_DEAD) joy_lr = 0.0;
        int neutral = (std::abs(joy_norm) < AXIS_DEAD) ? 1 : 0;
        int rotate = (std::abs(joy_lr) < AXIS_DEAD) ? 1 : 0;
        bool yaw_cmd_active = (std::abs(joy_lr) >= AXIS_DEAD);
        static bool prev_yaw_cmd_active = false;

        if (!yaw_cmd_active && prev_yaw_cmd_active &&
            (mode == GO || mode == BACK)) {
            yaw_ref = node->yaw;
        }
        prev_yaw_cmd_active = yaw_cmd_active;

        joyA = node->JOYBtnA;
        joyB = node->JOYBtnB;
        joyX = node->JOYBtnX;
        joyup = ( joy_norm > +0.6 ) ? 1 : 0;
        joydown = ( joy_norm < -0.6 ) ? 1 : 0;
        joyleft = ( joy_lr > +0.6 ) ? 1 : 0;
        joyright = ( joy_lr < -0.6 ) ? 1 : 0;

        if (!calibrated && imu_warmup_count < imu_warmup_period) {
            pitch_offset += node->pitch;
            imu_warmup_count++;
            if (imu_warmup_count == imu_warmup_period) {
                pitch_offset /= imu_warmup_period;
                calibrated = true;
                RCLCPP_INFO(node->get_logger(), "IMU calibration finished. Pitch offset: %f", pitch_offset);
            }
        }

        // Q.setZero();
        // QDot.setZero();

        // if (Q.size() >= 2) {
        //     Q[0]    = tita_data.Left.Pos.Wheel;
        //     QDot[0] = tita_data.Left.Vel.Wheel;
        //     Q[1]    = tita_data.Right.Pos.Wheel;
        //     QDot[1] = tita_data.Right.Vel.Wheel;
        // }

        // if (QDot.size() >= 2) {
        //     current_forward_vel = wheel_radius * 0.5 * (QDot[0] + QDot[1]);
        //     LW_vel = wheel_radius * QDot[0];
        //     RW_vel = wheel_radius * QDot[1];
        // }
        // if (Q.size() >= 2) {
        //     double LW_pos_now = tita_data.Left.Pos.Wheel;   // rad
        //     double RW_pos_now = tita_data.Right.Pos.Wheel;  // rad

        //     Q[0] = LW_pos_now;
        //     Q[1] = RW_pos_now;

        //     if (!has_prev_wheel) {
        //         // 첫 루프는 이전 값이 없으니 0으로 시작
        //         LW_vel = 0.0;
        //         RW_vel = 0.0;
        //         has_prev_wheel = true;
        //     } else {
        //         // 각속도(rad/s)
        //         double LW_omega = (LW_pos_now - prev_LW_pos) / dt;
        //         double RW_omega = (RW_pos_now - prev_RW_pos) / dt;

        //         // 선속도(m/s) = r * ω
        //         LW_vel = wheel_radius * LW_omega;
        //         RW_vel = wheel_radius * RW_omega;

        //         // QDot에는 각속도(rad/s)를 넣어준다 (RBDL용)
        //         QDot[0] = LW_omega;
        //         QDot[1] = RW_omega;
        //     }

        //     prev_LW_pos = LW_pos_now;
        //     prev_RW_pos = RW_pos_now;
        // }

        // if (QDot.size() >= 2) {
        //     // 로봇 전진 속도(두 바퀴 평균)
        //     current_forward_vel = 0.5 * (LW_vel + RW_vel);   
        // }
        current_forward_vel = 0.5 * (LW_vel + RW_vel);
        x_int += current_forward_vel * dt;

        // LW_vel = wheel_radius * QDot[0];  
        // RW_vel = wheel_radius * QDot[1];  
        // std::cout << "LW_vel: " << LW_vel <<std::endl;
        // std::cout << "RW_vel: " << RW_vel <<std::endl;
        // std::cout << "QDot[0]: " << QDot[0] <<std::endl;
        // std::cout << "QDot[1]: " << QDot[1] <<std::endl;
        // std::cout << "Pos.L: " << tita_data.Left.Pos.Wheel << "  Pos.R: " << tita_data.Right.Pos.Wheel << std::endl;
        const double tilt_gain = std::clamp(1.0 - std::abs(node->pitch)/0.35, 0.0, 1.0);

        x0 << x_int,
            node->pitch,
            current_forward_vel,
            node->ang_vel_y;

        const double alpha_v = dt / (tau_v + dt);

        static int prev_mode = mode;
        static int prevNeutral = 0;
        static int prevRotate = 0;
        const double WHEEL_BASE = 0.28;   // 휠 간 거리 (m) 대략값 넣어줘
        const double W_YAW_MAX  = 2.0;    // 최대 yaw 속도 (rad/s) 튜닝용
        double w_yaw_ref = joy_lr * W_YAW_MAX;

        // 기존 모드 갱신 후
        if (mode != prev_mode) {
                if (mode == GO || mode == BACK) {
                yaw_ref = node->yaw;
            }

            if (mode == INIT) {
                // // INIT 진입 시 모든 속도 관련 변수 초기화
                // v_ref_cmd = 0.0;
                // v_target_raw = 0.0;
                // current_forward_vel = 0.0;
                // x_int = x0(0);   // 현재 위치를 기준점으로 고정

                // // MPC 참조 갱신
                // xRef << x0(0), 0.0, 0.0, 0.0;

                // // 제약(속도, 토크) 다시 좁게
                // xMin(2) = -0.10; xMax(2) = +0.10;
                // uMin(0) = -5.0;  uMax(0) = +5.0;
                // -------1------
                if (prev_mode == GO) {
                    braking_from_go   = true;
                    braking_from_back = false;
                } else if (prev_mode == BACK) {
                    braking_from_go   = false;
                    braking_from_back = true;
                } else {
                    braking_from_go   = false;
                    braking_from_back = false;
                }

                x_hold    = x0(0);              // 제자리 목표
                v_enter   = x0(2);              // 진입 당시 전진속도
                t_init_enter = t_global;        // 시간 스탬프
                init_phase = BRAKE;

                v_target_raw = 0.0;
                v_ref_cmd    = 0.0;


                const double h_used = 0.28;
                const double w0     = std::sqrt(9.81 / h_used);
                const double xi     = x0(0) + x0(2) / w0;   // capture point
                double dx_ref       = -0.8*xi - 0.2*x0(2);  // CP + 속도 감쇠 섞기
                dx_ref = std::clamp(dx_ref, -0.25, 0.25);   // 과속 브레이크 방지

                xRef << x0(0), 0.0, dx_ref, 0.0;            // 위치는 현재값 고정, 자세 0, 속도는 위 dx_ref
                xMin(2) = -0.12; xMax(2) = +0.12;          // 속도는 작게만 허용(균형용 최소자유도)
                uMin(0) = -6.0;  uMax(0) = +6.0;           // 토크도 작게

                // --- Warm-start 끊어서 이전 해 영향 제거 (한 번만) ---
                solver.settings()->setWarmStart(false);
                solver.initSolver();
                solver.settings()->setWarmStart(true);

                std::cout << "[MODE CHANGE → INIT] Reset velocity & xRef" << std::endl;
            } else {
                braking_from_go = false;
                braking_from_back = false;
            }

            prev_mode = mode;
        }

        if (mode == INIT && target_mode != INIT) {
            const double v_eps     = 0.03;   // m/s 이하이면 거의 정지로 간주
            const double pitch_eps = 0.15;   // rad (약 8.6도) 이하이면 자세 OK

            double v_now = current_forward_vel;
            double pitch_err = node->pitch - pitch_offset;

            if (std::abs(v_now) < v_eps &&
                std::abs(pitch_err) < pitch_eps) {

                mode = target_mode;
                std::cout << "AUTO SWITCH → "
                          << (target_mode == GO ? "GO" : "BACK")
                          << std::endl;
            }
        }

        switch (mode)
        {
        case INIT:{
            v_target_raw = 0.0;
            v_ref_cmd = 0.0;
            xMin(2) = -0.03;
            xMax(2) = +0.03;
            // uMin(0) = -5.0;
            // uMax(0) = +5.0;
            if (braking_from_go || braking_from_back) {
                uMin(0) = -torque_lim;   // 예: -10.0
                uMax(0) = +torque_lim;   // 예: +10.0
            } else {
                uMin(0) = -5.0;
                uMax(0) = +5.0;
            }
            if (calibrated) {
                xRef << x_hold, pitch_offset, 0.0, 0.0;
            } else {
                xRef << x_hold, 0.0, 0.06, 0.0;
            }
            v_L_ref = v_ref_cmd;
            v_R_ref = v_ref_cmd;
            break;
        }
        // case GO:{
        //     xMin(2) = -2;
        //     xMax(2) = +2;
        //     v_target_raw =  -v_target;
        //     v_ref_cmd += alpha_v * (v_target_raw - v_ref_cmd);
        //     v_ref_cmd = std::clamp(v_ref_cmd, -0.2, 0.2);

        //     const double dx_ref = v_ref_cmd * tilt_gain;
        //     xRef << x_int, pitch_offset, dx_ref, 0.0;
        //     break;
        // }
        // case BACK:{
        //     xMin(2) = -2;
        //     xMax(2) = +2;
        //     v_target_raw =  +v_target;
        //     v_ref_cmd += alpha_v * (v_target_raw - v_ref_cmd);
        //     v_ref_cmd = std::clamp(v_ref_cmd, -0.2, 0.2);

        //     const double dx_ref = v_ref_cmd * tilt_gain;
        //     xRef << x_int, pitch_offset, dx_ref, 0.0;
        //     break;
        //     }
        case GO:{
            xMin(2) = -2.0;
            xMax(2) = +2.0;

            // 최종 목표 속도는 -v_target (예: -2.0 m/s)
            v_target_raw = -v_target;

            // 1차 필터로 한 번 부드럽게
            double v_ref_candidate = v_ref_cmd + alpha_v * (v_target_raw - v_ref_cmd);

            // 이번 스텝에서 변경 가능한 최대 속도 (가속도 제한)
            double dv      = v_ref_candidate - v_ref_cmd;
            double dv_max  = ACC_MAX * dt;           // 예: 0.8 * 0.002 = 0.0016 m/s
            dv = std::clamp(dv, -dv_max, dv_max);

            // 가속 제한 적용
            v_ref_cmd += dv;

            // 최종 속도는 ±v_target 안으로만
            v_ref_cmd = std::clamp(v_ref_cmd, -v_target, v_target);

            const double dx_ref = v_ref_cmd * tilt_gain;
            xRef << x_int, pitch_offset, dx_ref, 0.0;
            v_L_ref = v_ref_cmd;
            v_R_ref = v_ref_cmd;
            break;
        }
        case BACK:{
            xMin(2) = -2.0;
            xMax(2) = +2.0;

            v_target_raw = +v_target;

            double v_ref_candidate = v_ref_cmd + alpha_v * (v_target_raw - v_ref_cmd);

            double dv      = v_ref_candidate - v_ref_cmd;
            double dv_max  = ACC_MAX * dt;
            dv = std::clamp(dv, -dv_max, dv_max);

            v_ref_cmd += dv;

            v_ref_cmd = std::clamp(v_ref_cmd, -v_target, v_target);

            const double dx_ref = v_ref_cmd * tilt_gain;
            xRef << x_int, pitch_offset, dx_ref, 0.0;
            v_L_ref = v_ref_cmd;
            v_R_ref = v_ref_cmd;
            break;
        }
        case LEFT:{
            v_target_raw = 0.0;
            v_ref_cmd    = 0.0;

            xMin(2) = -0.03;
            xMax(2) = +0.03;
            uMin(0) = -torque_lim;
            uMax(0) =  torque_lim;

            if (calibrated) {
                xRef << x_int, pitch_offset, 0.0, 0.0;
            } else {
                xRef << x_int, 0.0, 0.058, 0.0;
            }
            v_L_ref = v_ref_cmd - 0.5 * WHEEL_BASE * w_yaw_ref;
            v_R_ref = v_ref_cmd + 0.5 * WHEEL_BASE * w_yaw_ref;
            break;
        }
        case RIGHT:{
            v_target_raw = 0.0;
            v_ref_cmd    = 0.0;

            xMin(2) = -0.03;
            xMax(2) = +0.03;
            uMin(0) = -torque_lim;
            uMax(0) =  torque_lim;

            if (calibrated) {
                xRef << x_int, pitch_offset, 0.0, 0.0;
            } else {
                xRef << x_int, 0.0, 0.058, 0.0;
            }
            v_L_ref = v_ref_cmd - 0.5 * WHEEL_BASE * w_yaw_ref;
            v_R_ref = v_ref_cmd + 0.5 * WHEEL_BASE * w_yaw_ref;
            break;
        }
        }

        if (neutral && !prevNeutral) {
            target_mode = INIT;
            mode = INIT;
            // x_int = 0.0;
            std::cout << "INIT" <<std::endl;
        }
        if (joyup && !prevUp) {
            target_mode = GO;

            if (mode == BACK) {
                // 뒤로 가던 중에 바로 앞으로 → INIT에서 먼저 브레이크
                mode = INIT;
                std::cout << "INIT (brake before GO)" << std::endl;
            } else {
                // INIT 이거나 이미 GO였으면 바로 GO
                mode = GO;
                std::cout << "GO" << std::endl;
            }
        }
        if (joydown && !prevDown) {
            target_mode = BACK;

            if (mode == GO) {
                // 앞으로 가던 중에 바로 뒤로 → INIT에서 먼저 브레이크
                mode = INIT;
                std::cout << "INIT (brake before BACK)" << std::endl;
            } else {
                // INIT 이거나 이미 BACK이었으면 바로 BACK
                mode = BACK;
                std::cout << "BACK" << std::endl;
            }
        }
        if (joyleft && !prevLeft) {
            mode = LEFT;
            std::cout << "TURN_LEFT" << std::endl;
        }
        if (joyright && !prevRight) {
            mode = RIGHT;
            std::cout << "TURN_RIGHT" << std::endl;
        }
        // if (rotate && !prevRotate) {
        //     mode = INIT;
        //     std::cout << "INIT (from rotate)" << std::endl;
        // }
        if ((mode == LEFT || mode == RIGHT) && rotate && !prevRotate) {
            mode = INIT;
            std::cout << "INIT (from rotate)" << std::endl;
        }

        prevA = joyA;
        prevB = joyB;
        prevX = joyX;
        prevUp = joyup;
        prevDown = joydown;
        prevLeft = joyleft;
        prevRight = joyright;
        prevNeutral = neutral;
        prevRotate = rotate;

        castMPCToQPGradient(Qw, xRef, H, grad);
        castMPCToQPConstraintVectors(xMax, xMin, uMax, uMin, x0, H, lb, ub);

        solver.updateGradient(grad);
        solver.updateBounds(lb, ub);

        auto current_time = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(current_time - last_time).count();
        t_global += elapsed;
        last_time = current_time;

        if (solver.solveProblem() == OsqpEigen::ErrorExitFlag::NoError) {
            const Eigen::VectorXd z = solver.getSolution();
            const int nx = 4;
            const int idx_u0 = nx*(H+1);

            // 1) QP에서 나온 기본 토크
            double u_cmd = z(idx_u0);

            // 2) INIT에서 (GO/BACK → INIT 직후에만) 속도에 비례한 브레이크 토크 추가
            if (mode == INIT && (braking_from_go || braking_from_back)) {
                const double v_stop_eps = 0.02;   // m/s
                if (std::abs(current_forward_vel) < v_stop_eps) {
                    // 거의 정지하면 브레이크 플래그 끔
                    braking_from_go   = false;
                    braking_from_back = false;
                } else {
                    const double k_v_brake = 40.0;    // 브레이크 게인 (튜닝)
                    double u_brake = -k_v_brake * current_forward_vel;  // 속도 반대방향으로 토크

                    // 브레이크 토크 자체도 uMin/uMax 안에 클램프
                    u_brake = std::clamp(u_brake, uMin(0), uMax(0));

                    u_cmd += u_brake;
                }
            }

            // 3) 공통 토크를 모드별 제약(uMin/uMax)에 맞게 클램프
            u_cmd = std::clamp(u_cmd, uMin(0), uMax(0));

            // INIT에서 아주 작은 토크는 떨림 방지용 데드존
            if (mode == INIT && std::abs(u_cmd) < 0.02) {
                u_cmd = 0.0;
            }

            // 4) yaw 보정 계산 (LEFT/RIGHT는 제외)
            if (mode == GO || mode == BACK) {
                // yaw_err  = node->yaw - yaw_ref;
                double yaw_rate = node->ang_vel_z;

                const double k_yaw   = 1.0;
                const double k_yaw_d = 0.3;

                //double u_yaw_local = -k_yaw * yaw_err - k_yaw_d * yaw_rate;
                double u_yaw_local = -k_yaw_d * yaw_rate;
                // GO/BACK에서는 속도가 느릴 때만 yaw 제어를 강하게

                if (!yaw_cmd_active){
                    double yaw_err = node->yaw - yaw_ref;
                    u_yaw_local += -k_yaw * yaw_err;
                }

                const double v_slow = 0.25; // m/s
                 double speed = std::abs(current_forward_vel);
                double scale = std::clamp((v_slow - speed) / v_slow, 0.0, 1.0);
                u_yaw_local *= scale;
                u_yaw = u_yaw_local;

            }
            else if (mode == INIT) {
                // ★ INIT에서는 방향을 억지로 되돌리지 않고,
                //    그냥 회전 속도만 천천히 죽이는 "댐핑"만 사용
                double yaw_rate = node->ang_vel_z;

                const double k_yaw_d_init = 0.3;   // 필요하면 0.2~0.5 사이에서 튜닝
                double u_yaw_local = -k_yaw_d_init * yaw_rate;

                u_yaw = u_yaw_local;
            }
            else {
                u_yaw = 0.0;
            }


            double u_yaw_ff = 0.0;
            if (mode == GO || mode == BACK) {
                u_yaw_ff = K_ff_turn * joy_lr;
            } else if (mode == LEFT) {
                u_yaw_ff = +turn_torque;
            } else if (mode == RIGHT) {
                u_yaw_ff = -turn_torque;
            }
            const double u_yaw_ff_max = 0.7 * torque_lim;   // 예: ±7Nm
            u_yaw_ff = std::clamp(u_yaw_ff, -u_yaw_ff_max, u_yaw_ff_max);

            double u_yaw_total = u_yaw + u_yaw_ff;
            double u_L = u_cmd - u_yaw_total;
            double u_R = u_cmd + u_yaw_total;

            // 7) 최종 좌/우 토크 한 번 더 제한 (모드별 uMin/uMax 기준)
            u_L = std::clamp(u_L, uMin(0), uMax(0));
            u_R = std::clamp(u_R, uMin(0), uMax(0));

            // 8) INIT에서 너무 작은 토크는 데드존 처리
            if (mode == INIT && std::abs(u_L) < 0.03) u_L = 0.0;
            if (mode == INIT && std::abs(u_R) < 0.03) u_R = 0.0;

            // 9) 휠 명령 publish
            std_msgs::msg::Float64MultiArray wheel_cmd;
            wheel_cmd.data = {u_L, u_R};
            tita_wheel_controller_pub->publish(wheel_cmd);
        } else {
            std_msgs::msg::Float64MultiArray wheel_cmd;
            wheel_cmd.data = {0.0, 0.0};
            tita_wheel_controller_pub->publish(wheel_cmd);
        }

        rclcpp::spin_some(node);

        DataLog_append.push_back(std::to_string(com_pos.x()) + ";" + std::to_string(com_pos.y()) + ";" + std::to_string(com_pos.z()) + ";" +
                                 std::to_string(node->roll) + ";" + std::to_string(node->pitch) + ";" + std::to_string(node->yaw) + ";" +
                                //  std::to_string(LW_pos.x()) + ";" + std::to_string(LW_pos.y()) + ";" + std::to_string(LW_pos.z()) + ";" +
                                //  std::to_string(RW_pos.x()) + ";" + std::to_string(RW_pos.y()) + ";" + std::to_string(RW_pos.z()) + ";" +
                                 std::to_string(-v_ref_cmd) + ";" + std::to_string(v_L_ref) + ";" + std::to_string(v_R_ref));

        loop_rate.sleep();
    }


        if(dataLogging) {
        data_log_dir << "/home/amg/tita_0309_ws/tita_ws/src/tita_bal_ign/data/dataLog_data_" << std::put_time(std::localtime(&in_time_t), "%Y%m%d_%H%M%S") << ".csv";
        std::ofstream data_log_file(data_log_dir.str());
        data_log_file << "com_x;com_y;com_z;roll;pitch;yaw;velocity;LW_vel;RW_vel";
        for (const auto& row : DataLog_append) {
            data_log_file << row << std::endl;
        }
        data_log_file.close();
    }

    rclcpp::shutdown();

    return 0;
}