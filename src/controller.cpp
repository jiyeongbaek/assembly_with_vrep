#include "controller.h"
#include <iostream>
#include <fstream>
#include <cmath>
#include <boost/bind.hpp>
#include <boost/algorithm/string.hpp>

#define DEGREE M_PI/180.0

using std::ofstream;

Controller::Controller(ros::NodeHandle nh_)
{
	joint_left_.qdot_.resize(dof);
	joint_left_.q_.resize(dof);
	joint_left_.qInit_.resize(dof);
	joint_left_.qGoal_.resize(dof);


	joint_left_.qdot_.setZero();
	joint_left_.q_.setZero();
	joint_left_.qInit_.setZero();
	joint_left_.qGoal_.setZero();

	joint_right_.qdot_.resize(dof);
	joint_right_.q_.resize(dof);
	joint_right_.qInit_.resize(dof);
	joint_right_.qGoal_.resize(dof);

	joint_right_.qdot_.setZero();
	joint_right_.q_.setZero();
	joint_right_.qInit_.setZero();
	joint_right_.qGoal_.setZero();

	joint_target_left_.q_.resize(dof);
	joint_target_left_.q_.setZero();
	joint_target_left_.cubic_q_.resize(dof);
	joint_target_left_.desired_q_.resize(dof);
	joint_target_left_.desired_q_.setZero();

	joint_target_right_.q_.resize(dof);
	joint_target_right_.q_.setZero();
	joint_target_right_.cubic_q_.resize(dof);
	joint_target_right_.desired_q_.resize(dof);
	joint_target_right_.desired_q_.setZero();

	joint_limit_right_.upper.resize(dof);
	joint_limit_right_.lower.resize(dof);
	joint_limit_left_.upper.resize(dof);
	joint_limit_left_.lower.resize(dof);


	joint_limit_right_.upper_rad.resize(dof);
	joint_limit_right_.lower_rad.resize(dof);
	joint_limit_left_.upper_rad.resize(dof);
	joint_limit_left_.lower_rad.resize(dof);

	for (int i = 0; i < dof; i++) {
		joint_limit_left_.lower(i) = -166.0;
		joint_limit_left_.upper(i) = 166.0;
	}
	for (int i = 0; i < dof; i++) {
		joint_limit_right_.lower(i) = -166.0;
		joint_limit_right_.upper(i) = 166.0;
	}

	joint_limit_left_.lower(1) = -101.0;
	joint_limit_left_.upper(1) = 101.0;

	joint_limit_left_.lower(3) = -176.0;
	joint_limit_left_.upper(3) = 4.0;

	joint_limit_left_.lower(5) = -1.0;
	joint_limit_left_.upper(5) = 215;


	joint_limit_right_.lower(1) = -101.0;
	joint_limit_right_.upper(1) = 101.0;

	joint_limit_right_.lower(3) = -176.0;
	joint_limit_right_.upper(3) = 4.0;

	joint_limit_right_.lower(5) = -1.0;
	joint_limit_right_.upper(5) = 215;

	joint_limit_left_.upper_rad = joint_limit_left_.upper / 180.0*M_PI;
	joint_limit_left_.lower_rad = joint_limit_left_.lower / 180.0*M_PI;

	joint_limit_right_.upper_rad = joint_limit_right_.upper / 180.0*M_PI;
	joint_limit_right_.lower_rad = joint_limit_right_.lower / 180.0*M_PI;
	
	model_l = make_shared<Model>();
	model_l->gravity = Vector3d(0., 0, -9.81);

	playTime_ = 0.0;
	threshold = 0.0;
	controlMode_ = DEFAULT;

}
Controller::~Controller()
{       
  }
void Controller::initModel() {
	model_l = make_shared<Model>();
	
	model_l->gravity = Vector3d(0., 0, -9.81);
	
	Rot_l = Matrix3d::Identity();

	for (int i = 0;i < 2;i++) {
		Rot_l_tot.block(3 * i, 3 * i, 3, 3) = Rot_l;
	
	}

	axis_l[0] = Rot_l * Eigen::Vector3d::UnitZ();
	axis_l[1] = Rot_l * Eigen::Vector3d::UnitY();
	axis_l[2] = Rot_l * Eigen::Vector3d::UnitZ();
	axis_l[3] = Rot_l * (-1.0*Eigen::Vector3d::UnitY());
	axis_l[4] = Rot_l * Eigen::Vector3d::UnitZ();
	axis_l[5] = Rot_l * (-1.0*Eigen::Vector3d::UnitY());
	axis_l[6] = Rot_l * (-1.0*Eigen::Vector3d::UnitZ());

	for (int i = 0; i < dof; i++) {
		mass_[i] = 1.0;
		inertia_[i] = Vector3d(1.0, 1.0, 1.0);
	}


	joint_position_global_l[0] = Vector3d(0, 0, 0.33);
	joint_position_global_l[1] = Vector3d(0, 0, 0.33);
	joint_position_global_l[2] = Vector3d(0, 0, 0.649);
	joint_position_global_l[3] = Vector3d(0.0825, 0, 0.649);
	joint_position_global_l[4] = Vector3d(0.0, -0.0, 1.033);
	joint_position_global_l[5] = Vector3d(0.0, -0.0, 1.033);
	joint_position_global_l[6] = Vector3d(0.0762, -0.0, 1.077);


	joint_position_local_l[0] = joint_position_global_l[0];
	for (int i = 1; i < dof; i++)
		joint_position_local_l[i] = joint_position_global_l[i] - joint_position_global_l[i - 1];

	com_position_l[0] = Vector3d(0.0, -0.0346, 0.2575);
	com_position_l[1] = Vector3d(0.0, 0.0344, 0.4094);
	com_position_l[2] = Vector3d(0.0334, 0.0266, 0.6076);
	com_position_l[3] = Vector3d(0.0331, -0.0266, 0.6914);
	com_position_l[4] = Vector3d(0.0, 0.0423, 0.9243);
	com_position_l[5] = Vector3d(0.0421, -0.0103, 1.0482);
//	com_position_l[6] = Vector3d(0.4090, -0.5400, 1.5129); // link7_respondable
	com_position_l[6] = Vector3d(0.088, -0.0, 0.8676); // panda_finger_joint 6
	for (int i = 0; i < dof; i++)
		com_position_l[i] -= joint_position_global_l[i];



	for (int i = 0; i < dof; i++) {
		body_l[i] = Body(mass_[i], com_position_l[i], inertia_[i]);
		joint_l[i] = Joint(JointTypeRevolute, axis_l[i]);

		if (i == 0) {
			body_id_l[i] = model_l->AddBody(0, Math::Xtrans(joint_position_local_l[i]), joint_l[i], body_l[i]);
		}
		else {
			body_id_l[i] = model_l->AddBody(body_id_l[i - 1], Math::Xtrans(joint_position_local_l[i]), joint_l[i], body_l[i]);
		
		}
	}
}

void Controller::setMode(ARM_CONTROL_MODE mode) {
	isModeChanged = true;
	controlMode_ = mode;
}
void Controller::compute() {
	_robot_left.model = model_l;
	_robot_left.q = joint_left_.q_;
	for (int i = 0;i < dof;i++) {
		_robot_left.body_id[i] = body_id_l[i];
		_robot_left.com_id[i] = com_position_l[i];
	}
	_robot_left.Rot = Rot_l;

	bool joint_flag = false, task_flag = false;
	
	if (isModeChanged)
	{
		isModeChanged = false;
		controlStartTime_ = playTime_;

		joint_left_.qInit_ = joint_left_.q_;
		target_state = 1;

	}
	switch (controlMode_)
	{
	case INIT:
		//JointPIDControl(0.1*Hz, false); // false : Right Arm Control, true : Left Arm Control
		joint_target_left_.q_(0) = 0.0 * DEGREE;
		joint_target_left_.q_(1) = -45.0 * DEGREE;
		joint_target_left_.q_(2) = 0.0 * DEGREE;
		joint_target_left_.q_(3) = -0.0 * DEGREE;
		joint_target_left_.q_(4) = 0.0 * DEGREE;
		joint_target_left_.q_(5) = 0.0 * DEGREE;
		joint_target_left_.q_(6) = 0.0 * DEGREE;
		JointPIDControl(0.1 * Hz, true);

		break;
	
	case APPROACH1:
		if (controlStartTime_ == playTime_) {
			_rrt.box_num = 3;
			_rrt.box_num2 = 1; 	

			_rrt.Box1[0].fAxis = Vector3d(0.01, 0.025, 0.135); // axis length 
			_rrt.Box1[0].vAxis[0] = Vector3d(1, 0, 0);
			_rrt.Box1[0].vAxis[1] = Vector3d(0, 1, 0);
			_rrt.Box1[0].vAxis[2] = Vector3d(0, 0, 1);
			_rrt.Box1[0].vPos = Vector3d(0.5, -0.25, 0.135); // axis center pos

			// box
			_rrt.Box1[1].fAxis = Vector3d(2.0, 2.0, 0.02);
			_rrt.Box1[1].vAxis[0] = Vector3d(1, 0, 0);
			_rrt.Box1[1].vAxis[1] = Vector3d(0, 1, 0);
			_rrt.Box1[1].vAxis[2] = Vector3d(0, 0, 1);
			_rrt.Box1[1].vPos = Vector3d(0.0, -0.0, -0.00);

			_rrt.Box1[2].fAxis = Vector3d(0.01, 0.025, 0.135); // axis length 
			_rrt.Box1[2].vAxis[0] = Vector3d(1, 0, 0);
			_rrt.Box1[2].vAxis[1] = Vector3d(0, 1, 0);
			_rrt.Box1[2].vAxis[2] = Vector3d(0, 0, 1);
			_rrt.Box1[2].vPos = Vector3d(0.6, -0.25, 0.135);


			_rrt.Box2[1].fAxis = Vector3d(0.02, 0.02, 0.20);
			_rrt.Box2[2].fAxis = Vector3d(0.02, 0.02, 0.20);

			TRAC_IK_solver(Vector4d(0.40, -0.10, 0.14, 1), Vector3d(-M_PI/2.0,0.0, M_PI/4.0), true);
			RRT_planning(joint_left_.qInit_, joint_left_.qGoal_, _joint_target, true); // true -> left planning

			_rrt.box_num = 0;
			_rrt.box_num2 = 0; 

			joint_left_.qInit_ = joint_left_.qGoal_;
			TRAC_IK_solver(Vector4d(0.4, -0.17, 0.14, 1), Vector3d(-M_PI/2.0,0.0, M_PI/4.0), true);
			RRT_planning(joint_left_.qInit_, joint_left_.qGoal_, _joint_target2, true); // true -> left planning
		}

		row1 = _joint_target.rows();
		row2 = _joint_target2.rows();
		target.resize(row1+row2, 7);
		target.topRows(row1) = _joint_target;
		target.bottomRows(row2) = _joint_target2;

		// target.topLeftCorner(row1, 7) = _joint_target;
		// target.bottomLeftCorner(row2,7) = _joint_target2;

		joint_flag = false;
		target_num = target.rows();
		
		if (target_num != target_state)
			joint_target_left_.q_ = target.row(target_state) * DEGREE;

		joint_flag = JointPIDControl(1.0 * Hz, true);

		if (joint_flag && target_state < target_num) {
			controlStartTime_ = playTime_;
			joint_left_.qInit_ = joint_left_.q_; // why update qInit ? Because PID controller consider initial state
			target_state++;
			
		}
		break;



	case APPROACH2:
		if (controlStartTime_ == playTime_) {
			_rrt.box_num = 0;
			_rrt.box_num2 = 0; 

			// _rrt.Box1[0].fAxis = Vector3d(2.0, 2.0, 0.02);
			// _rrt.Box1[0].vAxis[0] = Vector3d(1, 0, 0);
			// _rrt.Box1[0].vAxis[1] = Vector3d(0, 1, 0);
			// _rrt.Box1[0].vAxis[2] = Vector3d(0, 0, 1);
			// _rrt.Box1[0].vPos = Vector3d(0.0, -0.0, -0.025);	
			TRAC_IK_solver(Vector4d(0.35, -0.121, 0.2, 1), Vector3d(M_PI/2.0, 0.0, M_PI/4.0), true);
			RRT_planning(joint_left_.qInit_, joint_left_.qGoal_, _joint_target, true); // true -> left planning
			// _joint_target : Matrix of configurations (degrees) 
		}
		joint_flag = false;
		target_num = _joint_target.rows();

		if (target_num != target_state)
			joint_target_left_.q_ = _joint_target.row(target_state) * DEGREE;

		joint_flag = JointPIDControl(1.0 * Hz, true);

		if (joint_flag && target_state < target_num) {
			controlStartTime_ = playTime_;
			joint_left_.qInit_ = joint_left_.q_; // why update qInit ? Because PID controller consider initial state
			target_state++;
			
		}

		break;

	case APPROACH3:
		if (controlStartTime_ == playTime_) {
			_rrt.box_num = 2;
			_rrt.box_num2 = 1; 	

			// _rrt.Box1[0].fAxis = Vector3d(0.01, 0.025, 0.135); // axis length 
			// _rrt.Box1[0].vAxis[0] = Vector3d(1, 0, 0);
			// _rrt.Box1[0].vAxis[1] = Vector3d(0, 1, 0);
			// _rrt.Box1[0].vAxis[2] = Vector3d(0, 0, 1);
			// _rrt.Box1[0].vPos = Vector3d(0.6, -0.25, 0.135); // axis center pos

			// box
			_rrt.Box1[0].fAxis = Vector3d(2.0, 2.0, 0.02);
			_rrt.Box1[0].vAxis[0] = Vector3d(1, 0, 0);
			_rrt.Box1[0].vAxis[1] = Vector3d(0, 1, 0);
			_rrt.Box1[0].vAxis[2] = Vector3d(0, 0, 1);
			_rrt.Box1[0].vPos = Vector3d(0.0, -0.0, -0.00);

			_rrt.Box1[1].fAxis = Vector3d(0.01, 0.025, 0.135); // axis length 
			_rrt.Box1[1].vAxis[0] = Vector3d(1, 0, 0);
			_rrt.Box1[1].vAxis[1] = Vector3d(0, 1, 0);
			_rrt.Box1[1].vAxis[2] = Vector3d(0, 0, 1);
			_rrt.Box1[1].vPos = Vector3d(0.6, -0.25, 0.135);

			TRAC_IK_solver(Vector4d(0.60, -0.1, 0.17, 1), Vector3d(-M_PI/2.0,0.0, M_PI/4.0), true);
			RRT_planning(joint_left_.qInit_, joint_left_.qGoal_, _joint_target, true); // true -> left planning
			// _joint_target : Matrix of configurations (degrees) 
			
			_rrt.box_num = 0;
			_rrt.box_num2 = 0; 
			joint_left_.qInit_ = joint_left_.qGoal_;
			TRAC_IK_solver(Vector4d(0.60, -0.17, 0.17, 1), Vector3d(-M_PI/2.0,0.0, M_PI/4.0), true);
			RRT_planning(joint_left_.qInit_, joint_left_.qGoal_, _joint_target2, true); // true -> left planning
			// _joint_target : Matrix of configurations (degrees) 
		}
		row1 = _joint_target.rows();
		row2 = _joint_target2.rows();
		target.resize(row1+row2, 7);

		target.topRows(row1) = _joint_target;
		target.bottomRows(row2) = _joint_target2;

		joint_flag = false;
		target_num = target.rows();

		//distance = sqrt(pow((_joint_target.row(target_state)(0) - joint_left_.q_(0)), 2) + pow((_joint_target.row(target_state)(1) - joint_left_.q_(1)), 2) + pow((_joint_target.row(target_state)(2) - joint_left_.q_(2)), 2));
		if (target_num != target_state)
			joint_target_left_.q_ = target.row(target_state) * DEGREE;

		joint_flag = JointPIDControl(1.0 * Hz, true);

		if (joint_flag && target_state < target_num) {
			controlStartTime_ = playTime_;
			joint_left_.qInit_ = joint_left_.q_; // why update qInit ? Because PID controller consider initial state
			target_state++;
			
		}

		break;

	case APPROACH4:
		if (controlStartTime_ == playTime_) {

			_rrt.box_num = 0;
			_rrt.box_num2 = 0; 
			TRAC_IK_solver(Vector4d(0.60, -0.17, 0.27, 1), Vector3d(-M_PI/2.0,0.0, M_PI/4.0), true);
			RRT_planning(joint_left_.qInit_, joint_left_.qGoal_, _joint_target, true);

			_rrt.box_num = 1;
			_rrt.box_num2 = 1; 

			_rrt.Box1[0].fAxis = Vector3d(2.0, 2.0, 0.02);
			_rrt.Box1[0].vAxis[0] = Vector3d(1, 0, 0);
			_rrt.Box1[0].vAxis[1] = Vector3d(0, 1, 0);
			_rrt.Box1[0].vAxis[2] = Vector3d(0, 0, 1);
			_rrt.Box1[0].vPos = Vector3d(0.0, -0.0, -0.025);	

			joint_left_.qInit_ = joint_left_.qGoal_;
			TRAC_IK_solver(Vector4d(0.72, -0.03, 0.14, 1), Vector3d(M_PI/2.0, 0.0, M_PI/4.0), true);
			RRT_planning(joint_left_.qInit_, joint_left_.qGoal_, _joint_target2, true); // true -> left planning
			// _joint_target : Matrix of configurations (degrees) 
		}
		row1 = _joint_target.rows();
		row2 = _joint_target2.rows();
		target.resize(row1+row2, 7);

		target.topRows(row1) = _joint_target;
		target.bottomRows(row2) = _joint_target2;

		joint_flag = false;
		target_num = target.rows();

		//distance = sqrt(pow((_joint_target.row(target_state)(0) - joint_left_.q_(0)), 2) + pow((_joint_target.row(target_state)(1) - joint_left_.q_(1)), 2) + pow((_joint_target.row(target_state)(2) - joint_left_.q_(2)), 2));
		if (target_num != target_state)
			joint_target_left_.q_ = target.row(target_state) * DEGREE;

		joint_flag = JointPIDControl(1.0 * Hz, true);

		if (joint_flag && target_state < target_num) {
			controlStartTime_ = playTime_;
			joint_left_.qInit_ = joint_left_.q_; // why update qInit ? Because PID controller consider initial state
			target_state++;
			
		}

		break;


	case APPROACH5:
		if (controlStartTime_ == playTime_) {
			_rrt.box_num = 0;
			_rrt.box_num2 = 0; 

			_rrt.Box1[0].fAxis = Vector3d(2.0, 2.0, 0.02);
			_rrt.Box1[0].vAxis[0] = Vector3d(1, 0, 0);
			_rrt.Box1[0].vAxis[1] = Vector3d(0, 1, 0);
			_rrt.Box1[0].vAxis[2] = Vector3d(0, 0, 1);
			_rrt.Box1[0].vPos = Vector3d(0.0, -0.0, -0.025);	

			TRAC_IK_solver(Vector4d(0.45, 0.275, 0.13, 1), Vector3d(0.0, 0.0, M_PI/4.0), true);
			RRT_planning(joint_left_.qInit_, joint_left_.qGoal_, _joint_target, true); // true -> left planning

			joint_left_.qInit_ = joint_left_.qGoal_;
			
			TRAC_IK_solver(Vector4d(0.45, 0.275, 0.08, 1), Vector3d(0.0, 0.0, M_PI/4.0), true);
			RRT_planning(joint_left_.qInit_, joint_left_.qGoal_, _joint_target2, true); // true -> left planning

			// _joint_target : Matrix of configurations (degrees) 
		}
		row1 = _joint_target.rows();
		row2 = _joint_target2.rows();
		target.resize(row1+row2, 7);

		target.topRows(row1) = _joint_target;
		target.bottomRows(row2) = _joint_target2;

		joint_flag = false;
		target_num = target.rows();

		//distance = sqrt(pow((_joint_target.row(target_state)(0) - joint_left_.q_(0)), 2) + pow((_joint_target.row(target_state)(1) - joint_left_.q_(1)), 2) + pow((_joint_target.row(target_state)(2) - joint_left_.q_(2)), 2));
		if (target_num != target_state)
			joint_target_left_.q_ = target.row(target_state) * DEGREE;

		joint_flag = JointPIDControl(2.0 * Hz, true);

		if (joint_flag && target_state < target_num) {
			controlStartTime_ = playTime_;
			joint_left_.qInit_ = joint_left_.q_; // why update qInit ? Because PID controller consider initial state
			target_state++;
			
		}
	
	
		break;
		case APPROACH6:
		if (controlStartTime_ == playTime_) {
			_rrt.box_num = 0;
			_rrt.box_num2 = 0; 

			_rrt.Box1[0].fAxis = Vector3d(2.0, 2.0, 0.02);
			_rrt.Box1[0].vAxis[0] = Vector3d(1, 0, 0);
			_rrt.Box1[0].vAxis[1] = Vector3d(0, 1, 0);
			_rrt.Box1[0].vAxis[2] = Vector3d(0, 0, 1);
			_rrt.Box1[0].vPos = Vector3d(0.0, -0.0, -0.025);	

			_rrt.C.resize(6, 2);
			_rrt.C.setZero(); // max , min
			_rrt.C(0, 0) = 0.01;
			_rrt.C(1, 0) = 0.02;
			_rrt.C(2, 0) = 0.02;
			_rrt.C(3, 0) = 0.1;
			_rrt.C(4, 0) = 0.1;
			_rrt.C(5, 0) = 0.1;

			_rrt.C(0, 1) = -0.01;
			_rrt.C(1, 1) = -0.01;
			_rrt.C(2, 1) = -0.02;
			_rrt.C(3, 1) = -0.1;
			_rrt.C(4, 1) = -0.1;
			_rrt.C(5, 1) = -0.1;

			TRAC_IK_solver(Vector4d(0.45, 0.275, 0.20, 1), Vector3d(0.0, 0.0, M_PI/4.0), true);

			_robot_left.refer_pos(0) = 0.45;
			_robot_left.refer_pos(1) = 0.275;
			_robot_left.refer_pos(2) = 0.08;
			_robot_left.refer_rot = Rotate_with_X(0.0)*Rotate_with_Y(0.0)*Rotate_with_Z(M_PI/4.0);
			
			CRRT_planning(joint_left_.qInit_, joint_left_.qGoal_, _joint_target, true);
			
			// _rrt.box_num = 1;
			// _rrt.box_num2 = 0; 

			// _rrt.Box1[0].fAxis = Vector3d(2.0, 2.0, 0.02);
			// _rrt.Box1[0].vAxis[0] = Vector3d(1, 0, 0);
			// _rrt.Box1[0].vAxis[1] = Vector3d(0, 1, 0);
			// _rrt.Box1[0].vAxis[2] = Vector3d(0, 0, 1);
			// _rrt.Box1[0].vPos = Vector3d(0.0, -0.0, -0.025);	
			// joint_left_.qInit_ = joint_left_.qGoal_;
			// TRAC_IK_solver(Vector4d(0.27, 0.425, 0.08, 1), Vector3d(0.0, M_PI/2.0, M_PI/4.0), true);
			// RRT_planning(joint_left_.qInit_, joint_left_.qGoal_, _joint_target2, true); // true -> left planning
			// _joint_target : Matrix of configurations (degrees) 
		}
		// row1 = _joint_target.rows();
		// row2 = _joint_target2.rows();
		// target.resize(row1+row2, 7);

		// target.topRows(row1) = _joint_target;
		// target.bottomRows(row2) = _joint_target2;

		joint_flag = false;
		target_num = _joint_target.rows();

		//distance = sqrt(pow((_joint_target.row(target_state)(0) - joint_left_.q_(0)), 2) + pow((_joint_target.row(target_state)(1) - joint_left_.q_(1)), 2) + pow((_joint_target.row(target_state)(2) - joint_left_.q_(2)), 2));
		if (target_num != target_state)
			joint_target_left_.q_ = _joint_target.row(target_state) * DEGREE;

		joint_flag = JointPIDControl(0.1* Hz, true);

		if (joint_flag && target_state < target_num) {
			controlStartTime_ = playTime_;
			joint_left_.qInit_ = joint_left_.q_; // why update qInit ? Because PID controller consider initial state
			target_state++;
			
		}
	
		break;
	case APPROACH7:
		if (controlStartTime_ == playTime_) {
			_rrt.box_num = 0;
			_rrt.box_num2 = 0; 
			TRAC_IK_solver(Vector4d(-0.25, 0.5, 0.18, 1), Vector3d(0.0, 0.0, M_PI/4.0), true);
			RRT_planning(joint_left_.qInit_, joint_left_.qGoal_, _joint_target, true); // true -> left planning
			
			joint_left_.qInit_ = joint_left_.qGoal_;
			TRAC_IK_solver(Vector4d(-0.25, 0.5, 0.08, 1), Vector3d(0.0, 0.0, M_PI/4.0), true);
			RRT_planning(joint_left_.qInit_, joint_left_.qGoal_, _joint_target2, true);
		}
		row1 = _joint_target.rows();
		row2 = _joint_target2.rows();
		target.resize(row1+row2, 7);

		target.topRows(row1) = _joint_target;
		target.bottomRows(row2) = _joint_target2;

		joint_flag = false;
		target_num = target.rows();

		//distance = sqrt(pow((_joint_target.row(target_state)(0) - joint_left_.q_(0)), 2) + pow((_joint_target.row(target_state)(1) - joint_left_.q_(1)), 2) + pow((_joint_target.row(target_state)(2) - joint_left_.q_(2)), 2));
		if (target_num != target_state)
			joint_target_left_.q_ = target.row(target_state) * DEGREE;

		joint_flag = JointPIDControl(2.0 * Hz, true);

		if (joint_flag && target_state < target_num) {
			controlStartTime_ = playTime_;
			joint_left_.qInit_ = joint_left_.q_; // why update qInit ? Because PID controller consider initial state
			target_state++;
			
		}
	

		break;

	case APPROACH8:
		if (controlStartTime_ == playTime_) {
			_rrt.box_num = 0;
			_rrt.box_num2 = 0; 
			TRAC_IK_solver(Vector4d(-0.25, 0.5, 0.3, 1), Vector3d(0.0, 0.0, M_PI/4.0), true);
			RRT_planning(joint_left_.qInit_, joint_left_.qGoal_, _joint_target, true);

			joint_left_.qInit_ = joint_left_.qGoal_;
			TRAC_IK_solver(Vector4d(0.5, 0.0, 0.3, 1), Vector3d(0.0, 0.0, -M_PI/4.0), true);
			RRT_planning(joint_left_.qInit_, joint_left_.qGoal_, _joint_target2, true);
		}
		joint_flag = false;
		target_num = _joint_target.rows();

		if (target_num != target_state)
			joint_target_left_.q_ = _joint_target.row(target_state) * DEGREE;

		joint_flag = JointPIDControl(1.0 * Hz, true);

		if (joint_flag && target_state < target_num) {
			controlStartTime_ = playTime_;
			joint_left_.qInit_ = joint_left_.q_; // why update qInit ? Because PID controller consider initial state
			target_state++;
			
		}

		break;
	case DEFAULT:
		joint_target_left_.desired_q_ = joint_left_.qInit_;
		joint_target_right_.desired_q_ = joint_right_.qInit_;
		//std::cout << joint_left_.qInit_.transpose() << std::endl;

		break;
	}

	playTime_++;
}

// \B3\BB\B9\AB function
bool Controller::JointPIDControl(double duration, bool left) {
	bool res = false;
	if (left == true) {
		for (int i = 0; i < dof; i++)
			joint_target_left_.cubic_q_(i) = JCubic(playTime_, controlStartTime_, controlStartTime_ + duration, joint_left_.qInit_(i), 0.0, joint_target_left_.q_(i), 0.0);
		joint_target_left_.desired_q_ = joint_target_left_.cubic_q_;
	}
	else {
		for (int i = 0; i < dof; i++)
			joint_target_right_.cubic_q_(i) = JCubic(playTime_, controlStartTime_, controlStartTime_ + duration, joint_right_.qInit_(i), 0.0, joint_target_right_.q_(i), 0.0);
		joint_target_right_.desired_q_ = joint_target_right_.cubic_q_;
	}
	if (controlStartTime_ + duration < playTime_)
		res = true;

	return res;
}

void Controller::readdata(VectorXd &ql, VectorXd &qldot)
{
	joint_left_.q_ = ql;
	joint_left_.qdot_ = qldot;
	// joint_right_.q_ = qr;
	// joint_right_.qdot_ = qrdot;

}
void Controller::writedata(VectorXd& ql) {
	ql = joint_target_left_.desired_q_;
	//qr = joint_target_right_.desired_q_;
	
}



void Controller::TRAC_IK_solver(Vector4d x_pos, Vector3d rot_angle, bool left)
{
	double eps = 1e-7;
	double num_samples = 1000;
	double timeout = 0.005;
	std::string chain_start = "panda_link0";
	std::string chain_end = "panda_link8";
	TRAC_IK::TRAC_IK tracik_solver(chain_start, chain_end, urdf_param, timeout, eps);

	KDL::Chain chain;
	KDL::JntArray ll, ul; //lower joint limits, upper joint limits

	bool valid = tracik_solver.getKDLChain(chain);

	if (!valid)
	{
		ROS_ERROR("there was no valid KDL chain found");
		return;
	}

	valid = tracik_solver.getKDLLimits(ll, ul);

	if (!valid)
	{
		ROS_ERROR("there was no valid KDL joint limits found");
		return;
	}

	assert(chain.getNrOfJoints() == ll.data.size());
	assert(chain.getNrOfJoints() == ul.data.size());

	ROS_INFO("Using %d joints", chain.getNrOfJoints());

	// Set up KDL IK
	KDL::ChainFkSolverPos_recursive fk_solver(chain); // Forward kin. solver
	KDL::ChainIkSolverVel_pinv vik_solver(chain); // PseudoInverse vel solver
	KDL::ChainIkSolverPos_NR_JL kdl_solver(chain, ll, ul, fk_solver, vik_solver, 1, eps); // Joint Limit Solver
  	// 1 iteration per solve (will wrap in timed loop to compare with TRAC-IK)

	// Create Nominal chain configuration midway between all joint limits
  	KDL::JntArray nominal(chain.getNrOfJoints());
	  for (size_t j = 0; j < nominal.data.size(); j++)
  	{	
    	nominal(j) = (ll(j) + ul(j)) / 2.0;
  	}

	// Create desired number of valid, random joint configurations
 	std::vector<KDL::JntArray> JointList;
  	KDL::JntArray q(chain.getNrOfJoints());
	
	for (uint i = 0; i < num_samples; i++)
  	{
    	for (uint j = 0; j < ll.data.size(); j++)
    	{
      		q(j) = fRand(ll(j), ul(j));
    	}
    	JointList.push_back(q);
 	}

	boost::posix_time::ptime start_time;
	boost::posix_time::time_duration diff;

	KDL::JntArray result;
	KDL::Frame end_effector_pose;

	ROS_INFO_STREAM("Target position(global) : " << x_pos.transpose());
		
	for (int i = 0; i < 3; i ++){
		end_effector_pose.p(i) = x_pos(i);
	}
	Matrix3d Rot_d;
	Rot_d = Rotate_with_X(rot_angle(0))*Rotate_with_Y(rot_angle(1))*Rotate_with_Z(rot_angle(2));
	
	KDL::Rotation A;
	A.data[0] = Rot_d(0,0); 
	A.data[1] = Rot_d(0,1);
	A.data[2] = Rot_d(0,2);
	A.data[3] = Rot_d(1,0);
	A.data[4] = Rot_d(1,1);
	A.data[5] = Rot_d(1,2);
	A.data[6] = Rot_d(2,0); 
	A.data[7] = Rot_d(2,1);
	A.data[8] = Rot_d(2,2);
	end_effector_pose.M = A;

	int rc;

	double total_time = 0;
	uint success = 0;

  	ROS_INFO_STREAM("*** Testing TRAC-IK with " << num_samples << " random samples");

	for (uint i = 0; i < num_samples; i++)
	{
		//fk_solver.JntToCart(JointList[i], end_effector_pose);
		double elapsed = 0;
		start_time = boost::posix_time::microsec_clock::local_time();
		rc = tracik_solver.CartToJnt(nominal, end_effector_pose, result);
		diff = boost::posix_time::microsec_clock::local_time() - start_time;
		elapsed = diff.total_nanoseconds() / 1e9;
		total_time += elapsed;
		if (rc >= 0){
			success++;
		}

	}
	ROS_INFO_STREAM("TRAC-IK found " << success << " solutions (" << 100.0 * success / num_samples << "\%) with an average of " << total_time / num_samples << " secs per sample");
	ROS_INFO_STREAM("Result data : " << result.data.transpose() *180/ M_PI   );

	if (left) {joint_left_.qGoal_ = result.data;}
	else {joint_right_.qGoal_ = result.data;}
}

void Controller::RRT_planning(VectorXd q_start, VectorXd q_goal, MatrixXd& joint_target, bool left)
{
	//for (int i = 0; i < dof; i++) {
	//	joint_limit_left_.lower(i) = -166.0;
	//	joint_limit_left_.upper(i) = 166.0;
	//}
	//for (int i = 0; i < dof; i++) {
	//	joint_limit_right_.lower(i) = -166.0;
	//	joint_limit_right_.upper(i) = 166.0;
	//}
	//joint_limit_left_.lower(1) = -101.0;
	//joint_limit_left_.upper(1) = 101.0;

	//joint_limit_left_.lower(3) = -176.0;
	//joint_limit_left_.upper(3) = 4.0;

	//joint_limit_left_.lower(5) = -1.0;
	//joint_limit_left_.upper(5) = 215;


	//joint_limit_right_.lower(1) = -101.0;
	//joint_limit_right_.upper(1) = 101.0;

	//joint_limit_right_.lower(3) = -176.0;
	//joint_limit_right_.upper(3) = 4.0;

	//joint_limit_right_.lower(5) = -1.0;
	//joint_limit_right_.upper(5) = 215;

	/////// deg -> rad
	//joint_limit_left_.upper_rad = joint_limit_left_.upper / 180.0*M_PI;
	//joint_limit_left_.lower_rad = joint_limit_left_.lower / 180.0*M_PI;

	//joint_limit_right_.upper_rad = joint_limit_right_.upper / 180.0*M_PI;
	//joint_limit_right_.lower_rad = joint_limit_right_.lower / 180.0*M_PI;

	_rrt.left = left;
	if (left) {
		_rrt.lower_limit = joint_limit_left_.lower;
		_rrt.upper_limit = joint_limit_left_.upper;
		_rrt.DoF_size = dof;
	}
	else
	{
		_rrt.lower_limit = joint_limit_right_.lower;
		_rrt.upper_limit = joint_limit_right_.upper;
		_rrt.DoF_size = dof;
	}
	_rrt.qinit = q_start;
	_rrt.qgoal = q_goal;
	
	//////////////// until here okay 
	
	std::ofstream outFile("path_result.txt", ios::out); // open file for "writing"
	bool a = false;
	
	while (!a)
		if (left)
		{	
			
			a = _rrt.StartRRT2(_robot_left, outFile);
			}
		else
			a = _rrt.StartRRT2(_robot_right, outFile);
	ofstream outFile2("path_result2.txt", ios::out); // "writing" 
	// path_result -> Smooth -> path_result2
	ifstream inFile("path_result.txt"); // "reading"
	_rrt.SmoothPath(outFile2, inFile);

	outFile2.close();
	MatrixXd joint_temp(100, dof);
	if (!left)
		joint_temp.resize(100, dof);
	
	ifstream inFile2("path_result2.txt"); // "reading"
	int size = 0;
	std::vector<std::string> parameters;
	char inputString[1000];
	while (!inFile2.eof()) { // eof : end of file
		inFile2.getline(inputString, 1000);
		boost::split(parameters, inputString, boost::is_any_of(","));
		if (left) {
			if (parameters.size() == dof) {
				for (int j = 0; j < parameters.size(); j++)
					joint_temp(size, j) = atof(parameters[j].c_str()); // c_str() : string -> char* // atof : char* -> float
				size++;
			}
		}
		else {
			if (parameters.size() == dof) {
				for (int j = 0; j < parameters.size(); j++)
					joint_temp(size, j) = atof(parameters[j].c_str());
				size++;
			}
		}
	}
	cout << "trajectory size" << size << endl;
	inFile2.close();
	if (left)
		joint_target = joint_temp.topLeftCorner(size, dof);
	else
		joint_target = joint_temp.topLeftCorner(size, dof);
}
void Controller::CRRT_planning(VectorXd q_start, VectorXd q_goal, MatrixXd& joint_target, bool left)
{
		for (int i = 0; i < dof; i++) {
		joint_limit_left_.lower(i) = -166.0;
		joint_limit_left_.upper(i) = 166.0;
	}
	for (int i = 0; i < dof; i++) {
		joint_limit_right_.lower(i) = -166.0;
		joint_limit_right_.upper(i) = 166.0;
	}
	joint_limit_left_.lower(1) = -101.0;
	joint_limit_left_.upper(1) = 101.0;

	joint_limit_left_.lower(3) = -176.0;
	joint_limit_left_.upper(3) = 4.0;

	joint_limit_left_.lower(5) = -1.0;
	joint_limit_left_.upper(5) = 215;


	joint_limit_right_.lower(1) = -101.0;
	joint_limit_right_.upper(1) = 101.0;

	joint_limit_right_.lower(3) = -176.0;
	joint_limit_right_.upper(3) = 4.0;

	joint_limit_right_.lower(5) = -1.0;
	joint_limit_right_.upper(5) = 215;

	///// deg -> rad


	_rrt.left = left;
	if (left) {
		for (int i = 0;i < dof;i++) {
			_rrt.lower_limit(i) = joint_limit_left_.lower(i) / 180.0*M_PI;
			_rrt.upper_limit(i) = joint_limit_left_.upper(i) / 180.0*M_PI;
		}
		_rrt.DoF_size = dof;
	}
	else
	{
		for (int i = 0;i < dof;i++) {
			_rrt.lower_limit(i) = joint_limit_left_.lower(i) / 180.0*M_PI;
			_rrt.upper_limit(i) = joint_limit_left_.upper(i) / 180.0*M_PI;
		}
		_rrt.DoF_size = dof;
	}
	_rrt.qinit = q_start;
	_rrt.qgoal = q_goal;
	ofstream outFile("path_result.txt", ios::out);
	bool a = false;
	while (!a)
		if (left)
			a = _rrt.StartCRRT(_robot_left, outFile);
		else
			a = _rrt.StartCRRT(_robot_right, outFile);
	// ofstream outFile2("path_result2.txt", ios::out);

	// ifstream inFile("path_result.txt");
	//_rrt.SmoothCPath(outFile2, inFile);
	//inFile.close();
	//cout << "3" << endl;
	outFile.close();
	MatrixXd joint_temp(5000, dof);
	if (!left)
		joint_temp.resize(5000, dof);

	ifstream inFile2("path_result.txt");
	int size = 0;
	std::vector<std::string> parameters;
	char inputString[50000];
	while (!inFile2.eof()) {
		inFile2.getline(inputString, 50000);
		boost::split(parameters, inputString, boost::is_any_of(","));
		if (left) {
			if (parameters.size() == dof) {
				for (int j = 0; j < parameters.size(); j++)
					joint_temp(size, j) = atof(parameters[j].c_str());
				size++;
			}
		}
		else {
			if (parameters.size() == dof) {
				for (int j = 0; j < parameters.size(); j++)
					joint_temp(size, j) = atof(parameters[j].c_str());
				size++;
			}
		}
	}
	inFile2.close();
	cout << "size" << size << endl;

	if (left)
		joint_target = joint_temp.topLeftCorner(size, dof);
	else
		joint_target = joint_temp.topLeftCorner(size, dof);
}

