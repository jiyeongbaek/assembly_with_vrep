#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#define Hz 100.0

#include <iostream>
#include <Eigen/Dense>
#include <memory>
#include "plannar.h"

#include <trac_ik/trac_ik.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <ros/ros.h>
#include <ros/node_handle.h>

using namespace RigidBodyDynamics;
using namespace std;
using namespace Eigen;


class Controller
{

public: 
	Controller(ros::NodeHandle nh_);
	~Controller();
	enum ARM_CONTROL_MODE
	{
		INIT,
		APPROACH1,
		APPROACH2,
		APPROACH3,
		APPROACH4,
		DEFAULT
	};
public:
	void compute();
	void initModel();
	bool JointPIDControl(double duration, bool left);
	void setMode(ARM_CONTROL_MODE mode);
	void readdata(VectorXd& ql, VectorXd& qldot);
	void writedata(VectorXd& ql);
	void initPosition(VectorXd &ql, VectorXd &qr);

	
// Math library

// variable Setting
	struct jointState {
		VectorXd qInit_;
		VectorXd qGoal_;
		VectorXd q_;
		VectorXd qdot_;
	};
	struct JointLimit {
		VectorXd lower;
		VectorXd upper;
		VectorXd lower_rad;
		VectorXd upper_rad;
	};

	struct taskState {
		Vector3d xInit_;
		Vector3d x_;
		Vector3d x_des_;
		Vector3d x_cubic_;
		Vector3d x_prev_;
		VectorXd x_dot;
		VectorXd x_dotd;
		VectorXd x_error;

		Matrix3d rot_;
		Matrix3d rotInit_;
		Vector3d phi_;

		VectorXd xdot_; // 6d
		VectorXd x_error_; // 6d
		VectorXd torque_;
		MatrixXd J_;
		MatrixXd J_inv_;
		MatrixXd JT_;
		MatrixXd lambda_;
		MatrixXd lambda_inv_;

		MatrixXd A_;
		MatrixXd A_inv_;
		MatrixXd Lambda_;
	};

	struct jointTarget {
		VectorXd q_;
		VectorXd cubic_q_;
		VectorXd desired_q_;
	};
	struct Obstacle {
		int num;
		VectorXd length[10];
		VectorXd com[10]; // ³Ë³ËÇÏ°Ô.
	};

	double mass_[dof];
	Math::Vector3d inertia_[dof];

	shared_ptr<Model> model_l, model_r;
	unsigned int body_id_l[dof], body_id_r[dof];
	Body body_l[dof], body_r[dof];
	Joint joint_l[dof], joint_r[dof];

	jointState joint_left_, joint_right_;
	taskState task_left_, task_right_;
	jointTarget joint_target_left_, joint_target_right_;
	JointLimit joint_limit_left_, joint_limit_right_;
	Obstacle obj_;

	Vector3d axis_l[dof];
	Math::Vector3d joint_position_global_l[dof];
	Math::Vector3d joint_position_local_l[dof];
	Math::Vector3d com_position_l[dof];



	double playTime_;
	double Hz_;
	double controlStartTime_;
	double current_time;

	Vector3d object_position;

	ARM_CONTROL_MODE controlMode_;
	bool isModeChanged;

	rrt _rrt;
	
	Robotmodel _robot_left, _robot_right;
	int target_num, target_state;
	MatrixXd _joint_target;
	
	Eigen::VectorXd init_pose_l, init_pose_r;
	Matrix3d Rot_l;
	Matrix6d Rot_l_tot;

	void RRT_planning(VectorXd q_start, VectorXd q_goal, MatrixXd& joint_target, bool left);
	void CRRT_planning(VectorXd q_start, VectorXd q_goal, MatrixXd& joint_target, bool left);

	void TRAC_IK_solver(Vector4d x_pos, Vector3d rot_angle, bool left);
	TRAC_IK::TRAC_IK ik_solver(KDL::Chain chain, KDL::JntArray lower_joint_limits, KDL::JntArray upper_joint_limits, double timeout_in_secs=0.005, double error=1e-5, TRAC_IK::SolveType type=TRAC_IK::Speed);  
	std::string urdf_param;
	Matrix4d trans;
	
	double fRand(double min, double max)
	{
	double f = (double)rand() / RAND_MAX;
	return min + f * (max - min);
	};

	static const Matrix3d Vec_2_Rot(const Vector3d& vec) {
		Matrix3d Rot;
		double angle = vec.norm();

		if (angle == 0.0)
			Rot = Matrix3d::Identity();
		else {
			Vector3d axis = vec / angle;
			Matrix3d V;
			V.setZero();
			V(0, 1) = -axis(2);
			V(0, 2) = axis(1);
			V(1, 0) = axis(2);
			V(1, 2) = -axis(0);
			V(2, 0) = -axis(1);
			V(2, 1) = axis(0);

			Rot = Matrix3d::Identity() + V * sin(angle) + (1 - cos(angle))* V* V;
		}
		return Rot;
	};
};
#endif