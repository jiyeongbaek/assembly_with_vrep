#include <iostream>
#include <string>
#include "vrep_bridge.hpp"
#include "controller.h"
#include <std_msgs/Int32.h>
#include <sensor_msgs/JointState.h>
// for vrep keyboard event
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

using namespace std;
std::string state_;

void state_cb(const std_msgs::String::ConstPtr& msg)
  {
	
    state_ = msg->data;

  }
static constexpr uint32_t HashCode(const char *p) {

    return *p ? static_cast<uint32_t>(*p) + 33 * HashCode(p + 1) :  5381;

  }
int main(int argc, char** argv)
{
	ros::init(argc, argv, "assembly_vrep");
	ros::NodeHandle nh("~");
	const double hz = 100;
	VrepBridge vb(nh, hz);
	
	ros::Subscriber state_trans_sub = nh.subscribe("/assembly/state_transition", 100, &state_cb);
	
	Controller ac(nh);
	nh.param("urdf_param", ac.urdf_param, std::string("/robot_description"));

	ac.initModel();
	
	sleep(1);
	vb.vrepStart();
	sleep(1);
	vb.vrepEnableSyncMode();
	sleep(1);

	bool isSimulationRun = true;
	bool exitFlag = false;	
	std_msgs::Int32 msg;
	
	while (ros::ok() && !exitFlag)
	{
		vb.read_vrep();
		ac.readdata(vb.current_ql_, vb.current_ql_dot_);
		//cout << state_ << endl;
		uint32_t hash = HashCode(state_.c_str());
		switch (hash)
		{
			case HashCode("grasp"): // go to short part1
				ROS_INFO("GRASPING");
				vb.desired_grisping_l = 0.00;
				break;
			case HashCode("open"): // go to short part1
				ROS_INFO("Gripper Open");
				vb.desired_grisping_l = 0.04;
				break;
			case HashCode("attach"): // go to short part1
				ROS_INFO("Attach Object");
				vb.success(msg);
				break;
			case HashCode("detach"): // go to short part1
				ROS_INFO("Detach Object");
				vb.success(msg);
				break;

			case HashCode("approach1-1"): // go to short part1
				ROS_INFO("APPROACH1");
				ac.setMode(Controller::APPROACH1);
				msg.data = 1;
				break;
			case HashCode("approach1-2"): // go to short part1
				ROS_INFO("APPROACH2");
				ac.setMode(Controller::APPROACH2);
				msg.data = 2;
				break;
			case HashCode("approach2-1"): // go to short part1
				ROS_INFO("APPROACH3");
				ac.setMode(Controller::APPROACH3);
				msg.data = 3;
				break;
			case HashCode("approach2-2"): // go to short part1
				ROS_INFO("APPROACH4");
				ac.setMode(Controller::APPROACH4);
				msg.data =4;
				break;										
			case HashCode("approach3-1"): // go to short part1
				ROS_INFO("APPROACH5");
				ac.setMode(Controller::APPROACH5);
				msg.data = 5;
				break;	
			case HashCode("approach3-2"): // go to short part1
				ROS_INFO("APPROACH6");
				ac.setMode(Controller::APPROACH6);
				msg.data = 6;
				break;	
			
			case HashCode("approach4-1"): // go to short part1
				ROS_INFO("APPROACH7");
				ac.setMode(Controller::APPROACH7);
				msg.data = 7;
				break;	
			case HashCode("approach4-2"): // go to short part1
				ROS_INFO("APPROACH8");
				ac.setMode(Controller::APPROACH8);
				msg.data = 8;
				break;			
		}
		
		
		if (isSimulationRun) {
			state_ = 'default';
			ac.compute();
			ac.writedata(vb.desired_ql_);
			vb.write_vrep(); // publish desired q, success
			vb.wait();		
		}
	
	}
	
	return 0;
}