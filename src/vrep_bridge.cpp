#include "vrep_bridge.hpp"

VrepBridge::VrepBridge(ros::NodeHandle nh_,double hz_):
    rate_(hz_)
  {    
    is_first_run = true;
    tick = 0;
    sim_step_done_ = false;
    sim_time_ = 0.0f;

    current_ql_.resize(dof);
    current_ql_dot_.resize(dof);
    desired_ql_.resize(dof);
    desired_ql_.setZero();
    target_x_.resize(3);

    desired_grisping_l = 0.04;
    desired_grisping_r = 0.04;
    
    current_qr_.resize(dof);
    current_qr_dot_.resize(dof);
    desired_qr_.resize(dof);
    desired_qr_.setZero();
    target_xr_.resize(3);

    
    // current_base_.setZero();

    ljoint_cmd_.name.resize(dof);
    ljoint_cmd_.position.resize(dof);

    rjoint_cmd_.name.resize(dof);
    rjoint_cmd_.position.resize(dof);

    gripper_cmd_.name.resize(2);
    gripper_cmd_.position.resize(2);


	// velocity_l = 3.0;
	// velocity_r = 3.0;
	// desired_grisping_l = 0.5;
	// desired_grisping_r = 0.5;

    for(size_t i=0; i<dof; i++)
    {
      ljoint_cmd_.name[i]= L_JOINT_NAME[i];
	    rjoint_cmd_.name[i]= R_JOINT_NAME[i];
    }

    for (size_t i = 0; i <2; i++){
      gripper_cmd_.name[i] = GRIPPER_NAME[i];
    }

    vrep_sim_start_pub_ = nh_.advertise<std_msgs::Bool>("/startSimulation", 5);
    vrep_sim_stop_pub_ = nh_.advertise<std_msgs::Bool>("/stopSimulation", 5);
    vrep_sim_step_trigger_pub_ = nh_.advertise<std_msgs::Bool>("/triggerNextStep", 100);
    vrep_sim_enable_syncmode_pub_ = nh_.advertise<std_msgs::Bool>("/enableSyncMode", 5);
    vrep_success_pub_ = nh_.advertise<std_msgs::Int32>("/success", 5);

    // Queue_size = 발행하는 메세지를 몇 개까지 저장해 둘 것인지
    vrep_ljoint_set_pub_ = nh_.advertise<sensor_msgs::JointState>("/panda/left_joint_set", 1);
    //vrep_rjoint_set_pub_ = nh_.advertise<sensor_msgs::JointState>("/panda/right_joint_set", 1);
    vrep_gripper_set_pub = nh_.advertise<sensor_msgs::JointState>("/panda/gripper_set", 1);

    vrep_ljoint_state_sub_ = nh_.subscribe("/panda/left_joint_states", 100, &VrepBridge::ljoint_cb, this);
    //vrep_rjoint_state_sub_ = nh_.subscribe("/panda/right_joint_states", 100, &VrepBridge::rjoint_cb, this);
   
    vrep_sim_step_done_sub_ = nh_.subscribe("/simulationStepDone", 100, &VrepBridge::sim_step_done_cb, this);
    vrep_sim_time_sub_ = nh_.subscribe("/simulationTime",100,&VrepBridge::sim_time_cb,this);
    vrep_sim_status_sub_ = nh_.subscribe("/simulationState",100,&VrepBridge::sim_status_cb,this);

  }
  VrepBridge::~VrepBridge()
  {       
  }

  void VrepBridge::set_exec_time(float t)
  {
    exec_time_ = t;
  }
  
  void VrepBridge::ljoint_cb(const sensor_msgs::JointStateConstPtr& msg)
  {
	  for(size_t i=0; i< msg->name.size(); i++)
      {
        current_ql_[i] = msg->position[i];
        current_ql_dot_[i] = msg->velocity[i];        
      }
}

  void VrepBridge::rjoint_cb(const sensor_msgs::JointStateConstPtr& msg)
  {
	  for(size_t i=0; i< msg->name.size(); i++)
    {
      current_qr_[i] = msg->position[i];
      current_qr_dot_[i] = msg->velocity[i];        
    }
  }

 
  void VrepBridge::sim_status_cb(const std_msgs::Int32ConstPtr& msg)
  {
    vrep_sim_status = msg->data;
  }

  void VrepBridge::read_vrep()
  {
    ros::spinOnce();
  }
  

  void VrepBridge::write_vrep()
  {
    for(size_t i=0;i<dof;i++) {
        ljoint_cmd_.position[i] = desired_ql_(i);
        //rjoint_cmd_.position[i] = desired_qr_(i);
    } 
    gripper_cmd_.position[0] = desired_grisping_l;
    gripper_cmd_.position[1] = desired_grisping_l;
    
    vrep_ljoint_set_pub_.publish(ljoint_cmd_);
	  // vrep_rjoint_set_pub_.publish(rjoint_cmd_);
    vrep_gripper_set_pub.publish(gripper_cmd_);
    
    vrepStepTrigger();
  }

  void VrepBridge::wait()
  {
    while(ros::ok() && !sim_step_done_)
    {
      ros::spinOnce();
    }
    sim_step_done_ = false;
    rate_.sleep();
  }

  void VrepBridge::sim_time_cb(const std_msgs::Float32ConstPtr& msg)
  {
    sim_time_ = msg->data;
    tick = (sim_time_*100)/(SIM_DT*100);    
  }

  void VrepBridge::sim_step_done_cb(const std_msgs::BoolConstPtr &msg)
  {
    sim_step_done_ = msg->data;
  }

  void VrepBridge::vrepStart()
  {
    ROS_INFO("Starting V-REP Simulation");
    std_msgs::Bool msg;
    msg.data = true;
    // std_msgs::Bool msg2;
    // msg2.data=false;
    vrep_sim_start_pub_.publish(msg);
    // vrep_sim_enable_syncmode_pub_.publish(msg2);
  }
  void VrepBridge::vrepStop()
  {
    ROS_INFO("Stopping V-REP Simulation");
    std_msgs::Bool msg;
    msg.data = true;
    vrep_sim_stop_pub_.publish(msg);
  }
  void VrepBridge::vrepStepTrigger()
  {
    std_msgs::Bool msg;
    msg.data = true;
    vrep_sim_step_trigger_pub_.publish(msg);
  }
  void VrepBridge::vrepEnableSyncMode()
  {
    ROS_INFO("Sync Mode On");
    std_msgs::Bool msg;
    msg.data = true;
    vrep_sim_enable_syncmode_pub_.publish(msg);
  }

  void VrepBridge::success(std_msgs::Int32 msg)
  {
    vrep_success_pub_.publish(msg);
  }

  