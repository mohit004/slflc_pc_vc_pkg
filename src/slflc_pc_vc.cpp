/**
 * @file   SLFL_PC_VC.cpp
 * @author Mohit Mehndiratta
 * @date   December 2017
 *
 * @copyright
 * Copyright (C) 2017.
 */

#include <slflc_pc_vc.h>
#include <boost/shared_ptr.hpp>

using namespace Eigen;
using namespace ros;

// using Eigen::Vector2d;
// using Eigen::Vector3d;

double sampleTime = 0.01;

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

 std::vector<double> current_pos;
 std::vector<double> current_att_quat;
void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    current_pos = {msg->pose.position.x, msg->pose.position.y,
                   msg->pose.position.z};
    current_att_quat = {msg->pose.orientation.x, msg->pose.orientation.y,
                        msg->pose.orientation.z, msg->pose.orientation.w};
}

Vector3d ref_trajectory;
void local_traj_cb(const geometry_msgs::Vector3::ConstPtr& msg)
{
    ref_trajectory << msg->x, msg->y, msg->z;
}

Vector3d ref_trajectory_vel;
void local_traj_vel_cb(const geometry_msgs::Vector3::ConstPtr& msg)
{
    ref_trajectory_vel << msg->x, msg->y, msg->z;
}

std::vector<double> current_vel_rates(6);

void local_vel_rates_cb(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    current_vel_rates = {msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z,
                         msg->twist.angular.x, msg->twist.angular.y, msg->twist.angular.z};
}

std::vector<double> current_acc(3);
void imu_cb(const sensor_msgs::Imu::ConstPtr& msg)
{
    current_acc = {msg->linear_acceleration.x, msg->linear_acceleration.y,
                   msg->linear_acceleration.z};
}


// -------------------------------------
// SLFL_PC_VC class
// -------------------------------------

SLFL_PC_VC::SLFL_PC_VC()
{
    simple_learning_flag = true;

    /* ************************************** */
    /* ROS communication
    /* ************************************** */

//    m = 1.442;
//    m = 1.412;
    m = 1.795;
    g = 9.81;
//    max_Fz = 27;
    min_Fz_scale = 0* m*g;
//    max_Fz_scale = 1.4* m*g;
    max_Fz_scale = 1.95* m*g;
    max_Fz = max_Fz_scale;
    filter_weights_5 = {0.1, 0.1, 0.2, 0.3, 0.3};
    filter_weights_10 = {0.05, 0.05, 0.05, 0.075, 0.075,
                         0.1, 0.1, 0.1, 0.2, 0.2};

//    K1_pos_des = {4.8, 4.9, 11.0};                 // Talon tricopter backup Gazebo
//    K2_vel_des = {3.0, 2.7, 4.3};                 // Talon tricopter backup Gazebo
//    K1_pos_des = {4.8, 4.9, 14.0};                 // Talon tricopter backup
//    K2_vel_des = {3.0, 2.7, 3.5};                 // Talon tricopter backup
//    K1_pos_des = {4.8, 4.9, 4.5};                 // White tricopter backup
//    K2_vel_des = {3.0, 2.7, 3.8};                 // White tricopter backup

    K1_pos_des = {4.5, 4.7, 8.0};                   // Without and with learning
    K2_vel_des = {4.0, 4.1, 3.8};                   // Without and with learning
    alpha_K1_pos = {0.02, 0.02, 0.03};
    alpha_K2_vel = {0.02, 0.02, 0.03};
    alpha_d_dist = {0.2, 0.2, 0.25};

    // ----------
    // Subscribers
    // ----------
    local_traj_sub = private_nh.subscribe<geometry_msgs::Vector3>("ref_trajectory/pose", 1, local_traj_cb);
    local_traj_vel_sub = private_nh.subscribe<geometry_msgs::Vector3>("ref_trajectory/velocity", 1, local_traj_vel_cb);
    local_pos_sub = private_nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 1, local_pos_cb);
    local_vel_rates_sub = private_nh.subscribe<geometry_msgs::TwistStamped>("mavros/local_position/velocity", 1, local_vel_rates_cb);
//    local_vel_rates_sub = private_nh.subscribe<geometry_msgs::TwistStamped>("mavros/mocap/velocity", 1, local_vel_rates_cb);
    imu_sub = private_nh.subscribe<sensor_msgs::Imu>("mavros/imu/data", 1, imu_cb);

    // ----------
    // Publishers
    // ----------
//    att_throttle_pub = private_nh.advertise<std_msgs::Float64>("mavros/setpoint_attitude/att_throttle", 1, true);
    att_throttle_pub = private_nh.advertise<mavros_msgs::Thrust>("mavros/setpoint_attitude/thrust", 1, true);
    attitude_pub = private_nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_attitude/attitude", 1, true);
    slfl_cmd_xyz_vel_pub = private_nh.advertise<std_msgs::Float64MultiArray>("pos_velController_slfl_cmd/xyz_vel", 1, true);
    slfl_cmd_rpy_pub = private_nh.advertise<std_msgs::Float64MultiArray>("pos_velController_slfl_cmd/rpy", 1, true);
    slfl_cmd_Fz_pub = private_nh.advertise<std_msgs::Float64MultiArray>("pos_velController_slfl_cmd/Fz_FzScaled", 1, true);
    slfl_cmd_d_dist_pub = private_nh.advertise<std_msgs::Float64MultiArray>("pos_velController_slfl_cmd/d_dist", 1, true);
    slfl_cmd_K1_pos_pub = private_nh.advertise<std_msgs::Float64MultiArray>("pos_velController_slfl_cmd/K1_pos", 1, true);
    slfl_cmd_K2_vel_pub = private_nh.advertise<std_msgs::Float64MultiArray>("pos_velController_slfl_cmd/K2_vel", 1, true);
    slfl_cmd_exeTime_pub = private_nh.advertise<std_msgs::Float64>("pos_velController_slfl_cmd/exeTime", 1, true);

    std::vector<double> zeros_vector_5(5,0.0);
    std::vector<double> zeros_vector_10(10,0.0);
    std::vector<double> zeros_vector_15(15,0.0);
    std::vector<double> zeros_vector_20(20,0.0);
    std::vector<double> zeros_vector_30(30,0.0);

    meas_struct.curr_x_pos = 0.0;
    meas_struct.curr_y_pos = 0.0;
    meas_struct.curr_z_pos = 0.0;
    meas_struct.curr_x_vel = 0.0;
    meas_struct.curr_y_vel = 0.0;
    meas_struct.curr_z_vel = 0.0;
    meas_struct.last_x_vel = 0.0;
    meas_struct.last_y_vel = 0.0;
    meas_struct.last_z_vel = 0.0;
    meas_struct.x_acc_unfiltered = zeros_vector_10;
    meas_struct.y_acc_unfiltered = zeros_vector_10;
    meas_struct.z_acc_unfiltered = zeros_vector_10;
    meas_struct.curr_x_acc = 0.0;
    meas_struct.curr_y_acc = 0.0;
    meas_struct.curr_z_acc = 0.0;

    ref_struct.des_x_pos = 0.0;
    ref_struct.des_y_pos = 0.0;
    ref_struct.des_z_pos = 0.0;
    ref_struct.last_des_x_pos = 0.0;
    ref_struct.last_des_y_pos = 0.0;
    ref_struct.last_des_z_pos = 0.0;
    ref_struct.des_x_vel_unfiltered = zeros_vector_10;
    ref_struct.des_y_vel_unfiltered = zeros_vector_10;
    ref_struct.des_z_vel_unfiltered = zeros_vector_10;
    ref_struct.des_x_vel = 0.0;
    ref_struct.des_y_vel = 0.0;
    ref_struct.des_z_vel = 0.0;
    ref_struct.last_des_x_vel = 0.0;
    ref_struct.last_des_y_vel = 0.0;
    ref_struct.last_des_z_vel = 0.0;
    ref_struct.des_x_vel_dot_unfiltered = zeros_vector_10;
    ref_struct.des_y_vel_dot_unfiltered = zeros_vector_10;
    ref_struct.des_z_vel_dot_unfiltered = zeros_vector_10;
    ref_struct.des_x_vel_dot = 0.0;
    ref_struct.des_y_vel_dot = 0.0;
    ref_struct.des_z_vel_dot = 0.0;

    sl_struct.error1_pos = {0.0, 0.0, 0.0};
    sl_struct.error2_vel = {0.0, 0.0, 0.0};
    sl_struct.error2_vel_dot = {0.0, 0.0, 0.0};
    sl_struct.cost = {0.0, 0.0, 0.0};
    sl_struct.B = {1, 1, 1};

    slfl_cmd_struct.roll_ang_unfiltered = zeros_vector_15;
    slfl_cmd_struct.pitch_ang_unfiltered = zeros_vector_15;
    slfl_cmd_struct.Fz_unfiltered = zeros_vector_10;
//    slfl_cmd_struct.filter_weights = {0.05, 0.05, 0.05, 0.075, 0.075,
//                                      0.1, 0.1, 0.1, 0.2, 0.2};
    slfl_cmd_struct.roll_ang = 0.0;
    slfl_cmd_struct.pitch_ang = 0.0;
    slfl_cmd_struct.yaw_ang = 0.0;
    slfl_cmd_struct.Fz = m*g;
//    slfl_cmd_struct.Fz_scaled = 0.0;
    slfl_cmd_struct.Fz_scaled = ( (1 - 0)/(max_Fz_scale - min_Fz_scale) ) * (slfl_cmd_struct.Fz - min_Fz_scale);
    if (simple_learning_flag)
    {
//        slfl_cmd_struct.d_dist = {-0.1, -0.5, -0.1};			// Talon Tricopter
//        slfl_cmd_struct.K1_pos = {4.55, 4.75, 13.5};                   	// Talon Tricopter
//        slfl_cmd_struct.K2_vel = {3.3, 3.0, 3.5};                     	// Talon Tricopter
        slfl_cmd_struct.d_dist = {-0.08, -1.0, -4.6};                 // White Tricopter Gazebo
        slfl_cmd_struct.K1_pos = {4.55, 4.75, 4.2};                   // White Tricopter Gazebo
        slfl_cmd_struct.K2_vel = {3.3, 3.0, 3.5};                     // White Tricopter Gazebo
//        slfl_cmd_struct.d_dist = {0.15, -0.02, -0.3};                 // White Tricopter
//        slfl_cmd_struct.K1_pos = {4.3, 4.5, 7.7};                   // White Tricopter
//        slfl_cmd_struct.K2_vel = {3.8, 3.9, 3.6};                     // White Tricopter
    }
    else
    {
//        slfl_cmd_struct.d_dist = {-0.1, -0.5, -0.1};			// Talon
        slfl_cmd_struct.d_dist = {-0.08, -1.0, -4.6};			// White Tricopter Gazebo
//        slfl_cmd_struct.d_dist = {0.15, -0.02, -0.3};			// White tricopter
        slfl_cmd_struct.K1_pos = K1_pos_des;
        slfl_cmd_struct.K2_vel = K2_vel_des;
    }
    slfl_cmd_struct.exe_time = 0.0;

    ROS_INFO_STREAM("Constructor of the class SLFL_PC_VC is created");

}

SLFL_PC_VC::~SLFL_PC_VC()
{
    ROS_INFO_STREAM("Destructor of the class SLFL_PC_VC");
}

void SLFL_PC_VC::slfl_core(struct meas_struct_& measstruct, struct command_struct& commandstruct,
                           struct ref_struct_& refstruct, struct sl_struct_& slstruct,
                           Vector3d reftrajectory, Vector3d reftrajectoryvel, std::vector<double> currentpos, std::vector<double> current_velrates,
                           std::vector<double> currentacc)
{

    // set the current state feedback
    set_measurements(measstruct, currentpos, current_velrates, currentacc);

    // set the reference path
    set_reftrajectory(refstruct, reftrajectory, reftrajectoryvel);

    // SLFL: calculate and apply control for each direction
    // ---------------------------------------------------

    // Execute Calculation (SLFL control)
    ros::Time stopwatch = ros::Time::now();

    slstruct.error1_pos = {refstruct.des_x_pos - measstruct.curr_x_pos,
                           refstruct.des_y_pos - measstruct.curr_y_pos,
                           refstruct.des_z_pos - measstruct.curr_z_pos};
//    std::cout<<"error1_pos = ";
//    for (int i=0; i<slstruct.error1_pos.size(); ++i)
//        std::cout<<slstruct.error1_pos[i]<<", ";
//    std::cout<<"\n";

    slstruct.error2_vel = {refstruct.des_x_vel - measstruct.curr_x_vel,
                           refstruct.des_y_vel - measstruct.curr_y_vel,
                           refstruct.des_z_vel - measstruct.curr_z_vel};
//    std::cout<<"error2_vel = ";
//    for (int i=0; i<slstruct.error2_vel.size(); ++i)
//        std::cout<<slstruct.error2_vel[i]<<", ";
//    std::cout<<"\n";

    slstruct.error2_vel_dot = {refstruct.des_x_vel_dot - measstruct.curr_x_acc,
                               refstruct.des_y_vel_dot - measstruct.curr_y_acc,
                               refstruct.des_z_vel_dot - measstruct.curr_z_acc};
//    std::cout<<"error2_vel_dot = ";
//    for (int i=0; i<slstruct.error2_vel_dot.size(); ++i)
//        std::cout<<slstruct.error2_vel_dot[i]<<", ";
//    std::cout<<"\n";

    if (simple_learning_flag)
        slfl_calculate(slstruct, commandstruct);
    else
    {
        std::cout<<"d_dist = ";
        for (int i=0; i<commandstruct.d_dist.size(); ++i)
            std::cout<<commandstruct.d_dist[i]<<", ";
        std::cout<<"\n";
        std::cout<<"K1_pos = ";
        for (int i=0; i<commandstruct.K1_pos.size(); ++i)
            std::cout<<commandstruct.K1_pos[i]<<", ";
        std::cout<<"\n";
        std::cout<<"K2_vel = ";
        for (int i=0; i<commandstruct.K2_vel.size(); ++i)
            std::cout<<commandstruct.K2_vel[i]<<", ";
        std::cout<<"\n";
    }

    int i = 0;
    while (i < commandstruct.roll_ang_unfiltered.size()-1)
    {
        commandstruct.roll_ang_unfiltered[i] = commandstruct.roll_ang_unfiltered[i+1];
        commandstruct.pitch_ang_unfiltered[i] = commandstruct.pitch_ang_unfiltered[i+1];
//        commandstruct.Fz_unfiltered[i] = commandstruct.Fz_unfiltered[i+1];
        ++i;
    }

    double pitch_temp = (current_velrates[4]*current_velrates[2] - current_velrates[5]*current_velrates[1] -
                         refstruct.des_x_vel_dot + commandstruct.K2_vel[0]*slstruct.error2_vel[0]
                                                 + commandstruct.K1_pos[0]*slstruct.error1_pos[0]
                                                 - commandstruct.d_dist[0]) * (1/g);
//    std::cout<<"pitch_temp = "<<pitch_temp<<"\n";
    if (std::abs(pitch_temp) <= 0.5736)
        commandstruct.pitch_ang_unfiltered[i] = asin(pitch_temp);
    else if(pitch_temp < -0.5736)
        commandstruct.pitch_ang_unfiltered[i] = -0.6109;              // constraints of +-35 degs
    else if(pitch_temp > 0.5736)
        commandstruct.pitch_ang_unfiltered[i] = 0.6109;

    double roll_temp = (current_velrates[3]*current_velrates[2] - current_velrates[5]*current_velrates[0] -
                        refstruct.des_y_vel_dot - commandstruct.K2_vel[1]*slstruct.error2_vel[1]
                                                - commandstruct.K1_pos[1]*slstruct.error1_pos[1]
                                                + commandstruct.d_dist[1]) * (1/g*cos(commandstruct.pitch_ang_unfiltered[i]));
//    std::cout<<"roll_temp = "<<roll_temp<<"\n";
    if (std::abs(roll_temp) <= 0.5736)
        commandstruct.roll_ang_unfiltered[i] = asin(roll_temp);
    else if(roll_temp < -0.5736)
        commandstruct.roll_ang_unfiltered[i] = -0.6109;              // constraints of +-35 degs
    else if(roll_temp > 0.5736)
        commandstruct.roll_ang_unfiltered[i] = 0.6109;

    i = 0;
    while (i < commandstruct.Fz_unfiltered.size()-1)
    {
        commandstruct.Fz_unfiltered[i] = commandstruct.Fz_unfiltered[i+1];
        ++i;
    }

    double Fz_temp = m*(current_velrates[3]*current_velrates[1] - current_velrates[4]*current_velrates[0] +
                        g*cos(commandstruct.roll_ang_unfiltered[i])*cos(commandstruct.pitch_ang_unfiltered[i]) +
                        refstruct.des_z_vel_dot + commandstruct.K2_vel[2]*slstruct.error2_vel[2]
                                                + commandstruct.K1_pos[2]*slstruct.error1_pos[2]
                                                - commandstruct.d_dist[2]);
    if (Fz_temp > min_Fz_scale && Fz_temp <= max_Fz)
        commandstruct.Fz_unfiltered[i] = Fz_temp;
    else if(Fz_temp < min_Fz_scale)
        commandstruct.Fz_unfiltered[i] = min_Fz_scale;
    else if(Fz_temp > max_Fz)
        commandstruct.Fz_unfiltered[i] = max_Fz;
/*
    commandstruct.roll_ang = 0;
    commandstruct.pitch_ang = 0;
    commandstruct.Fz = 0;
    for (int i=0; i<commandstruct.roll_ang_unfiltered.size(); ++i)
    {
        commandstruct.roll_ang += filter_weights_10[i]*commandstruct.roll_ang_unfiltered[i];
        commandstruct.pitch_ang += filter_weights_10[i]*commandstruct.pitch_ang_unfiltered[i];
        commandstruct.Fz += filter_weights_10[i]*commandstruct.Fz_unfiltered[i];
    }
*/
    commandstruct.roll_ang = (std::accumulate(std::begin(commandstruct.roll_ang_unfiltered), std::end(commandstruct.roll_ang_unfiltered), 0.0)) /
                             commandstruct.roll_ang_unfiltered.size();
    commandstruct.pitch_ang = (std::accumulate(std::begin(commandstruct.pitch_ang_unfiltered), std::end(commandstruct.pitch_ang_unfiltered), 0.0)) /
                              commandstruct.pitch_ang_unfiltered.size();
    commandstruct.Fz = (std::accumulate(std::begin(commandstruct.Fz_unfiltered), std::end(commandstruct.Fz_unfiltered), 0.0)) /
                       commandstruct.Fz_unfiltered.size();
//    std::cout<<"roll_ang = "<<rad2pi*commandstruct.roll_ang<<"\n";
//    std::cout<<"pitch_ang = "<<rad2pi*commandstruct.pitch_ang<<"\n";
    std::cout<<"Fz = "<<commandstruct.Fz<<"\n";
    std::cout<<"------------------------------------------------\n";

    commandstruct.yaw_ang = 0.0;
//    commandstruct.yaw_ang = 1.57;

    commandstruct.Fz_scaled = ( (1 - 0)/(max_Fz_scale - min_Fz_scale) ) * (commandstruct.Fz - min_Fz_scale);

    commandstruct.exe_time = ros::Time::now().toSec() - stopwatch.toSec();

    // Publish SLFL output
    publish_rpyFz(commandstruct, refstruct);

//    ROS_INFO_STREAM("Stoptime velocityController SLFL: " << ros::Time::now().toSec() - stopwatch.toSec() << " (sec)");
}

double SLFL_PC_VC::slfl_calculate(struct sl_struct_& slstruct, struct command_struct& commandstruct)
{
    std::vector<double> error1_pos(slstruct.error1_pos.size(),0.0),
                        error2_vel(slstruct.error2_vel.size(),0.0);
    // Dead zone for weight update
    for (int i=0;i<slstruct.error1_pos.size(); ++i)
    {
        if ((std::abs(slstruct.error1_pos[i]) >= 0.2) || (std::abs(slstruct.error1_pos[i]) >= 0.15) && i != 0)
            error1_pos[i] = slstruct.error1_pos[i];
        else
            error1_pos[i] = 0.0;

        if (std::abs(slstruct.error2_vel[i]) >= 0.07)
            error2_vel[i] = slstruct.error2_vel[i];
        else
            error2_vel[i] = 0.0;
    }

    for(int i=0; i<slstruct.error1_pos.size(); ++i)
        slstruct.cost[i] = slstruct.error2_vel_dot[i] + K2_vel_des[i]*slstruct.error2_vel[i]
                                                      + K1_pos_des[i]*slstruct.error1_pos[i];
//        slstruct.cost[i] = slstruct.error2_vel_dot[i] + (K2_vel_des[i] - commandstruct.K2_vel[i])*slstruct.error2_vel[i]
//                                                      + (K1_pos_des[i] - commandstruct.K1_pos[i])*slstruct.error1_pos[i];               // other cost expression

    std::cout<<"cost = ";
    for (int i=0; i<slstruct.cost.size(); ++i)
        std::cout<<slstruct.cost[i]<<", ";
    std::cout<<"\n";

    double delta = sampleTime;

    // -------------------------
    // self learning in SLFL controller
    // -------------------------

    for (int i=0; i<slstruct.error1_pos.size(); ++i)
    {
//        commandstruct.d_dist[i] = commandstruct.d_dist[i] - delta*alpha_d_dist[i]*slstruct.cost[i]*slstruct.B[i];                         // Old WRONG expression
//        commandstruct.K1_pos[i] = commandstruct.K1_pos[i] + delta*alpha_K1_pos[i]*slstruct.cost[i]*slstruct.B[i]*error1_pos[i];           // Old WRONG expression
//        commandstruct.K2_vel[i] = commandstruct.K2_vel[i] + delta*alpha_K2_vel[i]*slstruct.cost[i]*slstruct.B[i]*error2_vel[i];           // Old WRONG expression

        commandstruct.d_dist[i] = commandstruct.d_dist[i] - delta*alpha_d_dist[i]*slstruct.cost[i];
        commandstruct.K1_pos[i] = commandstruct.K1_pos[i] + delta*alpha_K1_pos[i]*slstruct.cost[i]*error1_pos[i];
        commandstruct.K2_vel[i] = commandstruct.K2_vel[i] + delta*alpha_K2_vel[i]*slstruct.cost[i]*error2_vel[i];

    }

    std::cout<<"d_dist = ";
    for (int i=0; i<commandstruct.d_dist.size(); ++i)
        std::cout<<commandstruct.d_dist[i]<<", ";
    std::cout<<"\n";
    std::cout<<"K1_pos = ";
    for (int i=0; i<commandstruct.K1_pos.size(); ++i)
        std::cout<<commandstruct.K1_pos[i]<<", ";
    std::cout<<"\n";
    std::cout<<"K2_vel = ";
    for (int i=0; i<commandstruct.K2_vel.size(); ++i)
        std::cout<<commandstruct.K2_vel[i]<<", ";
    std::cout<<"\n";

}

void SLFL_PC_VC::set_measurements(struct meas_struct_& measstruct, std::vector<double> currentpos,
                                  std::vector<double> current_velrates, std::vector<double> currentacc)
{
    measstruct.curr_x_pos = currentpos[0];
    measstruct.curr_y_pos = currentpos[1];
    measstruct.curr_z_pos = currentpos[2];
    measstruct.curr_x_vel = current_velrates[0];
    measstruct.curr_y_vel = current_velrates[1];
    measstruct.curr_z_vel = current_velrates[2];

//    std::cout<<"curr_vel = "<<measstruct.curr_x_vel<<", "<<measstruct.curr_y_vel<<", "<<measstruct.curr_z_vel<<"\n";
/*
    measstruct.curr_x_acc = currentacc[0];
    measstruct.curr_y_acc = currentacc[1];
    measstruct.curr_z_acc = currentacc[2];
*/
    int i = 0;
    while (i < measstruct.x_acc_unfiltered.size()-1)
    {
        measstruct.x_acc_unfiltered[i] = measstruct.x_acc_unfiltered[i+1];
        measstruct.y_acc_unfiltered[i] = measstruct.y_acc_unfiltered[i+1];
        measstruct.z_acc_unfiltered[i] = measstruct.z_acc_unfiltered[i+1];
        ++i;
    }
    measstruct.x_acc_unfiltered[i] = (measstruct.curr_x_vel - measstruct.last_x_vel)/sampleTime;
    measstruct.y_acc_unfiltered[i] = (measstruct.curr_y_vel - measstruct.last_y_vel)/sampleTime;
    measstruct.z_acc_unfiltered[i] = (measstruct.curr_z_vel - measstruct.last_z_vel)/sampleTime;

    measstruct.curr_x_acc = (std::accumulate(std::begin(measstruct.x_acc_unfiltered), std::end(measstruct.x_acc_unfiltered), 0.0)) /
                            measstruct.x_acc_unfiltered.size();
    measstruct.curr_y_acc = (std::accumulate(std::begin(measstruct.y_acc_unfiltered), std::end(measstruct.y_acc_unfiltered), 0.0)) /
                            measstruct.y_acc_unfiltered.size();
    measstruct.curr_z_acc = (std::accumulate(std::begin(measstruct.z_acc_unfiltered), std::end(measstruct.z_acc_unfiltered), 0.0)) /
                            measstruct.z_acc_unfiltered.size();
//    std::cout<<"curr_acc = "<<measstruct.curr_x_acc<<", "<<measstruct.curr_y_acc<<", "<<measstruct.curr_z_acc<<"\n";

    measstruct.last_x_vel = measstruct.curr_x_vel;
    measstruct.last_y_vel = measstruct.curr_y_vel;
    measstruct.last_z_vel = measstruct.curr_z_vel;

}

void SLFL_PC_VC::set_reftrajectory(struct ref_struct_& refstruct, Vector3d reftrajectory, Vector3d reftrajectoryvel)
{
    refstruct.des_x_pos = reftrajectory(0);
    refstruct.des_y_pos = reftrajectory(1);
    refstruct.des_z_pos = reftrajectory(2);
    int i = 0;
    while (i < refstruct.des_x_vel_unfiltered.size()-1)
    {
        refstruct.des_x_vel_unfiltered[i] = refstruct.des_x_vel_unfiltered[i+1];
        refstruct.des_y_vel_unfiltered[i] = refstruct.des_y_vel_unfiltered[i+1];
        refstruct.des_z_vel_unfiltered[i] = refstruct.des_z_vel_unfiltered[i+1];
        ++i;
    }
//    refstruct.des_x_vel_unfiltered[i] = (refstruct.des_x_pos - refstruct.last_des_x_pos)/sampleTime;
//    refstruct.des_y_vel_unfiltered[i] = (refstruct.des_y_pos - refstruct.last_des_y_pos)/sampleTime;
//    refstruct.des_z_vel_unfiltered[i] = (refstruct.des_z_pos - refstruct.last_des_z_pos)/sampleTime;
    refstruct.des_x_vel_unfiltered[i] = reftrajectoryvel(0);
    refstruct.des_y_vel_unfiltered[i] = reftrajectoryvel(1);
    refstruct.des_z_vel_unfiltered[i] = reftrajectoryvel(2);

/*
    refstruct.des_x_vel = (std::accumulate(std::begin(refstruct.des_x_vel_unfiltered), std::end(refstruct.des_x_vel_unfiltered), 0.0)) /
                          refstruct.des_x_vel_unfiltered.size();
    refstruct.des_y_vel = (std::accumulate(std::begin(refstruct.des_y_vel_unfiltered), std::end(refstruct.des_y_vel_unfiltered), 0.0)) /
                          refstruct.des_y_vel_unfiltered.size();
    refstruct.des_z_vel = (std::accumulate(std::begin(refstruct.des_z_vel_unfiltered), std::end(refstruct.des_z_vel_unfiltered), 0.0)) /
                          refstruct.des_z_vel_unfiltered.size();
*/

    refstruct.des_x_vel = 0.0;
    refstruct.des_y_vel = 0.0;
    refstruct.des_z_vel = 0.0;
    for (int i=0; i<refstruct.des_x_vel_unfiltered.size(); ++i)
    {
        refstruct.des_x_vel += refstruct.des_x_vel_unfiltered[i];
        refstruct.des_y_vel += refstruct.des_y_vel_unfiltered[i];
        refstruct.des_z_vel += refstruct.des_z_vel_unfiltered[i];
    }
    refstruct.des_x_vel = refstruct.des_x_vel/refstruct.des_x_vel_unfiltered.size();
    refstruct.des_y_vel = refstruct.des_y_vel/refstruct.des_y_vel_unfiltered.size();
    refstruct.des_z_vel = refstruct.des_z_vel/refstruct.des_z_vel_unfiltered.size();
//    std::cout<<"des_vel = "<<refstruct.des_x_vel<<", "<<refstruct.des_y_vel<<", "<<refstruct.des_z_vel<<"\n";

    i = 0;
    while (i < refstruct.des_x_vel_dot_unfiltered.size()-1)
    {
        refstruct.des_x_vel_dot_unfiltered[i] = refstruct.des_x_vel_dot_unfiltered[i+1];
        refstruct.des_y_vel_dot_unfiltered[i] = refstruct.des_y_vel_dot_unfiltered[i+1];
        refstruct.des_z_vel_dot_unfiltered[i] = refstruct.des_z_vel_dot_unfiltered[i+1];
        ++i;
    }
    refstruct.des_x_vel_dot_unfiltered[i] = (refstruct.des_x_vel - refstruct.last_des_x_vel)/sampleTime;
    refstruct.des_y_vel_dot_unfiltered[i] = (refstruct.des_y_vel - refstruct.last_des_y_vel)/sampleTime;
    refstruct.des_z_vel_dot_unfiltered[i] = (refstruct.des_z_vel - refstruct.last_des_z_vel)/sampleTime;
/*
    refstruct.des_x_vel_dot = (std::accumulate(std::begin(refstruct.des_x_vel_dot_unfiltered), std::end(refstruct.des_x_vel_dot_unfiltered), 0.0)) /
                                  refstruct.des_x_vel_dot_unfiltered.size();
    refstruct.des_y_vel_dot = (std::accumulate(std::begin(refstruct.des_y_vel_dot_unfiltered), std::end(refstruct.des_y_vel_dot_unfiltered), 0.0)) /
                                  refstruct.des_y_vel_dot_unfiltered.size();
    refstruct.des_z_vel_dot = (std::accumulate(std::begin(refstruct.des_z_vel_dot_unfiltered), std::end(refstruct.des_z_vel_dot_unfiltered), 0.0)) /
                                  refstruct.des_z_vel_dot_unfiltered.size();
*/
    refstruct.des_x_vel_dot = 0.0;
    refstruct.des_y_vel_dot = 0.0;
    refstruct.des_z_vel_dot = 0.0;
    for (int i=0; i<refstruct.des_x_vel_dot_unfiltered.size(); ++i)
    {
        refstruct.des_x_vel_dot += refstruct.des_x_vel_dot_unfiltered[i];
        refstruct.des_y_vel_dot += refstruct.des_y_vel_dot_unfiltered[i];
        refstruct.des_z_vel_dot += refstruct.des_z_vel_dot_unfiltered[i];
    }
    refstruct.des_x_vel_dot = refstruct.des_x_vel_dot/refstruct.des_x_vel_dot_unfiltered.size();
    refstruct.des_y_vel_dot = refstruct.des_y_vel_dot/refstruct.des_y_vel_dot_unfiltered.size();
    refstruct.des_z_vel_dot = refstruct.des_z_vel_dot/refstruct.des_z_vel_dot_unfiltered.size();
//    std::cout<<"des_vel_dot = "<<refstruct.des_x_vel_dot<<", "<<refstruct.des_y_vel_dot<<", "<<refstruct.des_z_vel_dot<<"\n";

    refstruct.last_des_x_pos = refstruct.des_x_pos;
    refstruct.last_des_y_pos = refstruct.des_y_pos;
    refstruct.last_des_z_pos = refstruct.des_z_pos;
    refstruct.last_des_x_vel = refstruct.des_x_vel;
    refstruct.last_des_y_vel = refstruct.des_y_vel;
    refstruct.last_des_z_vel = refstruct.des_z_vel;
}

void SLFL_PC_VC::publish_rpyFz(struct command_struct& commandstruct, struct ref_struct_& refstruct)
{
//    std_msgs::Float64 att_thro_cmd;
//    att_thro_cmd.data = commandstruct.Fz_scaled;
    mavros_msgs::Thrust att_thro_cmd;
    att_thro_cmd.header.frame_id = "";
    att_thro_cmd.header.stamp = ros::Time::now();
    att_thro_cmd.thrust = commandstruct.Fz_scaled;
    att_throttle_pub.publish(att_thro_cmd);

    tf::Quaternion q(0.0, 0.0, 0.0, 1.0);
    q.setRPY(commandstruct.roll_ang, commandstruct.pitch_ang, commandstruct.yaw_ang);

    geometry_msgs::PoseStamped attitude_cmd;
    attitude_cmd.header.frame_id = "";
    attitude_cmd.header.stamp = ros::Time::now();
    attitude_cmd.pose.orientation.x = q.getX();
    attitude_cmd.pose.orientation.y = q.getY();
    attitude_cmd.pose.orientation.z = q.getZ();
    attitude_cmd.pose.orientation.w = q.getW();
    attitude_pub.publish(attitude_cmd);

    std::vector<double> xyz_vel_vec = {refstruct.des_x_vel, refstruct.des_y_vel, refstruct.des_z_vel};
    std_msgs::Float64MultiArray xyz_vel_cmd;
    xyz_vel_cmd.layout.dim.push_back(std_msgs::MultiArrayDimension());
    xyz_vel_cmd.layout.dim[0].size = xyz_vel_vec.size();
    xyz_vel_cmd.layout.dim[0].stride = 1;
    xyz_vel_cmd.layout.dim[0].label = "x,y,z vels (m/sec)";
    xyz_vel_cmd.data.clear();
    xyz_vel_cmd.data.insert(xyz_vel_cmd.data.end(), xyz_vel_vec.begin(), xyz_vel_vec.end());
    slfl_cmd_xyz_vel_pub.publish(xyz_vel_cmd);

    std::vector<double> rpy_vec = {commandstruct.roll_ang, commandstruct.pitch_ang, commandstruct.yaw_ang};
    std_msgs::Float64MultiArray rpy_cmd;
    rpy_cmd.layout.dim.push_back(std_msgs::MultiArrayDimension());
    rpy_cmd.layout.dim[0].size = rpy_vec.size();
    rpy_cmd.layout.dim[0].stride = 1;
    rpy_cmd.layout.dim[0].label = "Roll, Pitch, Yaw (rad)";
    rpy_cmd.data.clear();
    rpy_cmd.data.insert(rpy_cmd.data.end(), rpy_vec.begin(), rpy_vec.end());
    slfl_cmd_rpy_pub.publish(rpy_cmd);

    std::vector<double> Fz_vec = {commandstruct.Fz, commandstruct.Fz_scaled};
    std_msgs::Float64MultiArray Fz_cmd;
    Fz_cmd.layout.dim.push_back(std_msgs::MultiArrayDimension());
    Fz_cmd.layout.dim[0].size = Fz_vec.size();
    Fz_cmd.layout.dim[0].stride = 1;
    Fz_cmd.layout.dim[0].label = "Fz (N), Fz_scaled";
    Fz_cmd.data.clear();
    Fz_cmd.data.insert(Fz_cmd.data.end(), Fz_vec.begin(), Fz_vec.end());
    slfl_cmd_Fz_pub.publish(Fz_cmd);

    std_msgs::Float64MultiArray d_dist_cmd;
    d_dist_cmd.layout.dim.push_back(std_msgs::MultiArrayDimension());
    d_dist_cmd.layout.dim[0].size = commandstruct.d_dist.size();
    d_dist_cmd.layout.dim[0].stride = 1;
    d_dist_cmd.layout.dim[0].label = "Dist coeff (along x,y,z)";
    d_dist_cmd.data.clear();
    d_dist_cmd.data.insert(d_dist_cmd.data.end(), commandstruct.d_dist.begin(), commandstruct.d_dist.end());
    slfl_cmd_d_dist_pub.publish(d_dist_cmd);

    std_msgs::Float64MultiArray K1_pos_cmd;
    K1_pos_cmd.layout.dim.push_back(std_msgs::MultiArrayDimension());
    K1_pos_cmd.layout.dim[0].size = commandstruct.K1_pos.size();
    K1_pos_cmd.layout.dim[0].stride = 1;
    K1_pos_cmd.layout.dim[0].label = "Position coeff (along x,y,z)";
    K1_pos_cmd.data.clear();
    K1_pos_cmd.data.insert(K1_pos_cmd.data.end(), commandstruct.K1_pos.begin(), commandstruct.K1_pos.end());
    slfl_cmd_K1_pos_pub.publish(K1_pos_cmd);

    std_msgs::Float64MultiArray K2_vel_cmd;
    K2_vel_cmd.layout.dim.push_back(std_msgs::MultiArrayDimension());
    K2_vel_cmd.layout.dim[0].size = commandstruct.K2_vel.size();
    K2_vel_cmd.layout.dim[0].stride = 1;
    K2_vel_cmd.layout.dim[0].label = "Velocity coeff (along x,y,z)";
    K2_vel_cmd.data.clear();
    K2_vel_cmd.data.insert(K2_vel_cmd.data.end(), commandstruct.K2_vel.begin(), commandstruct.K2_vel.end());
    slfl_cmd_K2_vel_pub.publish(K2_vel_cmd);

    std_msgs::Float64 exe_time_cmd;
    exe_time_cmd.data = commandstruct.exe_time;
    slfl_cmd_exeTime_pub.publish(exe_time_cmd);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "posCont_velCont_SLFL");
    ros::NodeHandle nh;

    state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 1, state_cb);

    SLFL_PC_VC *slfl_pc_vc = new SLFL_PC_VC();
//    boost::shared_ptr<SLFL_PC_VC> slfl_pc_vc = new SLFL_PC_VC();

    ros::Rate rate(1/sampleTime);

    std::vector<double> current_pos_B(3);
    std::vector<double> current_vel_rates_B(6);
    Vector3d ref_trajectory_B;

    double curr_roll, curr_pitch, curr_yaw;

    int print_flag_offboard = 1;
    int print_flag_arm = 1;
    int print_flag_altctl = 1;

    bool control_stop = false;

    while(ros::ok() && !control_stop)
    {
        t = ros::Time::now().toSec();

        if( current_state.mode != "OFFBOARD" && print_flag_offboard == 1)
        {
            ROS_INFO("OFBOARD mode is not enabled!");
            print_flag_offboard = 0;
        }
        if( !current_state.armed && print_flag_arm == 1)
        {
            ROS_INFO("Vehicle is not armed!");
            print_flag_arm = 2;
        }
        else if(current_state.armed && print_flag_arm == 2)
        {
            ROS_INFO("Vehicle is armed!");
            print_flag_arm = 0;
        }

        if( current_state.mode == "ALTCTL")
        {
            if(print_flag_altctl == 1)
            {
                ROS_INFO("ALTCTL mode is enabled!");
                print_flag_altctl = 0;
            }
        }

        while(ros::ok() && current_state.mode == "OFFBOARD" && !control_stop)
        {
            tf::Quaternion q(current_att_quat[0], current_att_quat[1],
                             current_att_quat[2], current_att_quat[3]);

            tf::Matrix3x3 dcm_z(q);
            dcm_z.getRPY(curr_roll, curr_pitch, curr_yaw);
            dcm_z.setEulerYPR(curr_yaw, 0, 0);
            dcm_z = dcm_z.transpose();

            for (int i=0; i<3; ++i)
            {
                ref_trajectory_B(i) = dcm_z[i][0]*ref_trajectory(0) +
                                      dcm_z[i][1]*ref_trajectory(1) +
                                      dcm_z[i][2]*ref_trajectory(2);

                current_pos_B[i] = dcm_z[i][0]*current_pos[0] +
                                   dcm_z[i][1]*current_pos[1] +
                                   dcm_z[i][2]*current_pos[2];
            }

            tf::Matrix3x3 R_BW(q);
            R_BW = R_BW.transpose();
            for (int i=0; i<3; ++i)
            {
                current_vel_rates_B[i] = R_BW[i][0]*current_vel_rates[0] +
                                         R_BW[i][1]*current_vel_rates[1] +
                                         R_BW[i][2]*current_vel_rates[2];
            }

//            slfl_pc_vc->slfl_core(slfl_pc_vc->meas_struct, slfl_pc_vc->slfl_cmd_struct, slfl_pc_vc->ref_struct,
//                                  slfl_pc_vc->sl_struct, ref_trajectory, ref_trajectory_vel, current_pos, current_vel_rates, current_acc);

            slfl_pc_vc->slfl_core(slfl_pc_vc->meas_struct, slfl_pc_vc->slfl_cmd_struct, slfl_pc_vc->ref_struct,
                                  slfl_pc_vc->sl_struct, ref_trajectory_B, ref_trajectory_vel, current_pos_B, current_vel_rates_B, current_acc);

            t_vc_loop = ros::Time::now().toSec() - t;
            if(std::isnan(slfl_pc_vc->slfl_cmd_struct.roll_ang) == true || std::isnan(slfl_pc_vc->slfl_cmd_struct.pitch_ang) == true ||
               std::isnan(slfl_pc_vc->slfl_cmd_struct.Fz) == true)
            {
                ROS_ERROR_STREAM("positionvelocityController_SLFL ERROR at time = " << t_vc_loop <<" (sec)" );
                control_stop = true;
                exit;
            }

//            ROS_INFO("t_ac_loop = %f",t_ac_loop);

            print_flag_offboard = 1;
            print_flag_arm = 1;

            ros::spinOnce();
            rate.sleep();
        }

        slfl_pc_vc->publish_rpyFz(slfl_pc_vc->slfl_cmd_struct, slfl_pc_vc->ref_struct);

        ros::spinOnce();
        rate.sleep();

    }

    return 0;
}

