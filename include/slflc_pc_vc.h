#pragma once

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/State.h>
// #include<mavros_msgs/ExtendedState.h>
#include <mavros_msgs/ActuatorControl.h>
#include <mavros_msgs/Thrust.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <math.h>
#include <Eigen/Dense>
#include <tf/tf.h>
#include <numeric>

extern double sampleTime;
const double rad2pi = 180/M_PI;

class SLFL_PC_VC
{
	private:

		bool simple_learning_flag;

		double m;
		double g;
		double max_Fz;
		double min_Fz_scale;
		double max_Fz_scale;
		std::vector<double> filter_weights_5;
		std::vector<double> filter_weights_10;
		std::vector<double> filter_weights_15;

		std::vector<double> K1_pos_des;
		std::vector<double> K2_vel_des;
		std::vector<double> alpha_K1_pos;
		std::vector<double> alpha_K2_vel;
		std::vector<double> alpha_d_dist;

	public:

		struct meas_struct_
		{
			double curr_x_pos;
			double curr_y_pos;
			double curr_z_pos;
			double curr_x_vel;
			double curr_y_vel;
			double curr_z_vel;
			double last_x_vel;
			double last_y_vel;
			double last_z_vel;
			std::vector<double> x_acc_unfiltered;
			std::vector<double> y_acc_unfiltered;
			std::vector<double> z_acc_unfiltered;
			double curr_x_acc;
			double curr_y_acc;
			double curr_z_acc;
		} meas_struct;
		
		struct ref_struct_
		{
			double des_x_pos;
			double des_y_pos;
			double des_z_pos;
			double last_des_x_pos;
			double last_des_y_pos;
			double last_des_z_pos;
			std::vector<double> des_x_vel_unfiltered;
			std::vector<double> des_y_vel_unfiltered;
			std::vector<double> des_z_vel_unfiltered;
			double des_x_vel;
			double des_y_vel;
			double des_z_vel;
			double last_des_x_vel;
			double last_des_y_vel;
			double last_des_z_vel;
			std::vector<double> des_x_vel_dot_unfiltered;
			std::vector<double> des_y_vel_dot_unfiltered;
			std::vector<double> des_z_vel_dot_unfiltered;
			double des_x_vel_dot;
			double des_y_vel_dot;
			double des_z_vel_dot;
		} ref_struct;
	
		struct sl_struct_
		{
			std::vector<double> error1_pos;
			std::vector<double> error2_vel;
			std::vector<double> error2_vel_dot;
			std::vector<double> cost;
			std::vector<double> B;
		} sl_struct;

		struct command_struct
		{
			std::vector<double> roll_ang_unfiltered;
			std::vector<double> pitch_ang_unfiltered;
			std::vector<double> Fz_unfiltered;
			double roll_ang;
			double pitch_ang;
			double yaw_ang;
			double Fz;
			double Fz_scaled;	// between 0 and 1
			std::vector<double> K1_pos;
			std::vector<double> K2_vel;
			std::vector<double> d_dist;
			double exe_time;

		} slfl_cmd_struct;

		SLFL_PC_VC();
		~SLFL_PC_VC();

		void slfl_core(struct meas_struct_& measstruct, struct command_struct& commandstruct, struct ref_struct_& refstruct, struct sl_struct_& slstruct, Eigen::Vector3d reftrajectory, Eigen::Vector3d reftrajectoryvel, std::vector<double> currentpos, std::vector<double> current_velrates, std::vector<double> currentacc);

		double slfl_calculate(struct sl_struct_& slstruct, struct command_struct& commandstruct);

		void publish_rpyFz(struct command_struct& commandstruct, struct ref_struct_& refstruct);

	protected:
		ros::NodeHandle private_nh;

		void set_measurements(struct meas_struct_& measstruct, std::vector<double> currentpos, std::vector<double> current_velrates, std::vector<double> currentacc);

		void set_reftrajectory(struct ref_struct_& refstruct, Eigen::Vector3d reftrajectory, Eigen::Vector3d reftrajectoryvel);

		// Subscribers
		ros::Subscriber local_traj_sub;
		ros::Subscriber local_traj_vel_sub;
		ros::Subscriber local_pos_sub;
		ros::Subscriber local_vel_rates_sub;
		ros::Subscriber imu_sub;
//		ros::Subscriber posControl_pd_cmd_uvw_sub;

		// Publishers
		ros::Publisher att_throttle_pub;
		ros::Publisher attitude_pub;
		ros::Publisher slfl_cmd_xyz_vel_pub;
		ros::Publisher slfl_cmd_rpy_pub;
		ros::Publisher slfl_cmd_Fz_pub;
		ros::Publisher slfl_cmd_d_dist_pub;
		ros::Publisher slfl_cmd_K1_pos_pub;
		ros::Publisher slfl_cmd_K2_vel_pub;
		ros::Publisher slfl_cmd_exeTime_pub;
};

ros::Subscriber state_sub;
// ros::Subscriber ext_state_sub;

double t, t_pc_loop, t_vc_loop;
