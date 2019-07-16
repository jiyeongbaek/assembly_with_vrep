#include <iostream>
#include <string>
#include "vrep_bridge.hpp"
#include "controller.h"

#include <sensor_msgs/JointState.h>

// for vrep keyboard event
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

using namespace std;

bool _kbhit()
{
    termios term;
    tcgetattr(0, &term);

    termios term2 = term;
    term2.c_lflag &= ~ICANON;
    tcsetattr(0, TCSANOW, &term2);

    int byteswaiting;
    ioctl(0, FIONREAD, &byteswaiting);

    tcsetattr(0, TCSANOW, &term);

    return byteswaiting > 0;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "assembly_vrep");
	ros::NodeHandle nh("~");
	
	const double hz = 100;
	VrepBridge vb(nh, hz);
	
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

	//ros::Publisher desired_joint_pub = nh.advertise<sensor_msgs::JointState>("/desired_joint_", 100);
	while (ros::ok() && !exitFlag)
	{
		vb.read_vrep();
		ac.readdata(vb.current_ql_, vb.current_ql_dot_);
		
		if (_kbhit())
		{
			int key = getchar();
			switch (key)
			{
			case 'i':
				cout << "Initial Posture (Joint Space)" << endl;
				ac.setMode(Controller::INIT);
				
				break;
			case '1':
				cout << "Approach1" << endl;
				ac.setMode(Controller::APPROACH1);
				break;
			
			case '2':
				cout << "Approach2" << endl;
				ac.setMode(Controller::APPROACH2);
				break;
			case '3':
				cout << "Approach3" << endl;
				ac.setMode(Controller::APPROACH3);
			
				break;
			case '4':
				cout << "Approach4" << endl;
				ac.setMode(Controller::APPROACH4);
				break;
			case 'g':
				cout << "Grasping Milk" << endl;
				vb.desired_grisping_l = 0.02;
				break;
			case '\t':
				if (isSimulationRun) {
					cout << "Simulation Pause" << endl;
					isSimulationRun = false;
				}
				else {
					cout << "Simulation Run" << endl;
					isSimulationRun = true;
					//ac.setMode(Controller::DEFAULT);
				}
				break;
			case 'q':
				isSimulationRun = false;
				exitFlag = true;
				break;
			
			}
			// svb.wait();
		}
		
		if (isSimulationRun) {
			ac.compute();
			ac.writedata(vb.desired_ql_);
			vb.write_vrep();
			vb.wait();
			
		}
	
	}
	
	return 0;
}