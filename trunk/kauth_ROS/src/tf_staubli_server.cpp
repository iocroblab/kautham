
// Included to use the ROS messaging system
#include "ros/ros.h"
// Include to use the mesage definition in Kautham
#include <kautham/tf_server.h>
#include <sensor_msgs/JointState.h>
//#include <tf/transform_datatypes.h>
#include <mt/rotation.h>
#include <mt/scalar.h>

#include <TX90.h>

#include <sstream>
#include <string>
#include <algorithm>


using namespace kautham;
TX90          _robotSoap;
std::string   STAUBLI_IP;

bool robotCallback(kautham::tf_server::Request  &tcpRob,
                   kautham::tf_server::Response &stateRob){

//	mt::Quaternion quat;
  mt::Scalar yaw, pitch, roll;

//	tf::quaternionMsgToTF 	( tcpRob.tcp.transform.rotation, quat);
//	btMatrix3x3(quat).getRPY( roll,pitch,yaw);
	mt::Rotation rot;
	rot.setValue(tcpRob.tcp.transform.rotation.x,
							 tcpRob.tcp.transform.rotation.y,
							 tcpRob.tcp.transform.rotation.z,
							 tcpRob.tcp.transform.rotation.w);
	rot.getYpr(yaw, pitch, roll);

	std::vector<double> target_pos(6, 0.);
	target_pos[0] = tcpRob.tcp.transform.translation.x;
	target_pos[1] = tcpRob.tcp.transform.translation.y;
	target_pos[2] = tcpRob.tcp.transform.translation.z;
	target_pos[3] = roll;
	target_pos[4] = pitch;
	target_pos[5] = yaw;


	std::stringstream ss;
	std::copy(target_pos.begin(), target_pos.end(), std::ostream_iterator<float>(ss, " "));
	ROS_INFO("I receive the data: [%s]", ss.str().c_str());

//	_robotSoap.Login(STAUBLI_IP, "default", "");
//	if( _robotSoap.Power(true) ){
//		_robotSoap.ResetMotion();
//		if(_robotSoap.MoveCart(target_pos)){
//			_robotSoap.GetRobotJoints(target_pos);
			stateRob.joints.position.resize(target_pos.size());
			for( size_t i = 0; i < target_pos.size(); i++)
				stateRob.joints.position[i] = target_pos[i];
			return true;
//		}
//	}
//	return false;
}

int main(int argc, char **argv)
{
  try{
  ros::init(argc, argv, "tf_staubli_server");

  // what controller should be handle?
  // firts remove any parameter added by the roslaunch or another program that adds parameters.
  ros::V_string _vargv;
  _vargv.clear();
  ros::removeROSArgs(argc, argv, _vargv);

  if( _vargv.size() > 1)
    STAUBLI_IP = _vargv[1] ;
  else
    STAUBLI_IP = "147.83.37.76:5653";

  ros::NodeHandle n;
  ros::ServiceServer service = n.advertiseService("tf_staubli_soap", robotCallback);
  ROS_INFO("Ready to move the Staubli Robot %s using moveCar.", STAUBLI_IP.c_str());

  ros::spin();

  return 0;
 
 }catch(...){
      //std::cout << ex.what() << std::endl;
      return 1;
 }

}

