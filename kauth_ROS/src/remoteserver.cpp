// Included to use the ROS messaging system
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

// Include the memory share object definition
//#include "data_ioc_cell.hpp"

// Include the Soap Client for the CS8 controler.
#include <CS8Robot.h>

// Include the VAL client.
#include "connection.hpp"

#include <sstream>
#include <iostream>
#include <algorithm>
#include <iterator>


// Creatin the two VAL clients;
Connection    valClientR1;
Connection    valClientR2;

// Creating the two SOAP Cleints.
CS8Robot      _robotSoap1;
CS8Robot      _robotSoap2;

std::string   STAUBLI_IP_R1;
std::string   STAUBLI_IP_R2;
std::string   SOAP_IP_R1;
std::string   SOAP_IP_R2;
enum connType{VAL, SOAP} _activeConn;

std::string   PORT="";

void robot1Callback(const sensor_msgs::JointState::ConstPtr& r1des)
{
  if( _activeConn == VAL ){
  // Now I send this data to the CS8 using val program and simple udp socket.
  std::string smsg;
  for (unsigned int i=0; i<r1des->position.size(); i++){
      smsg.append(boost::lexical_cast<std::string>(r1des->position[i]));
      smsg.append(" ");
  }
  smsg.append("@");

  valClientR1.send(smsg);
  }else{
	try{
	  RobotJointPos joints;
	  joints.resize(6);
		for (size_t i=0; i<r1des->position.size(); ++i)
		  joints.at(i) = r1des->position.at(i);
			
		_robotSoap1.moveJoints(joints);
	}
    catch (const CS8Error& err) {
      std::cout << err.what() << std::endl;
      _robotSoap1.setPower(false);
      
    }
  }

#ifdef _DEBUG
//  std::stringstream ss;
//  std::copy(r1des->position.begin(), r1des->position.end(), std::ostream_iterator<float>(ss, " "));
//  ROS_INFO("I heard: [%s]", ss.str().c_str());
  ROS_INFO( "Received R1_Command" );
#endif

}

void robot2Callback(const sensor_msgs::JointState::ConstPtr& r2des){
if( _activeConn == VAL ){
  // Now I send this data to the CS8 using val program and simple udp socket.
  std::string smsg;
  for (unsigned int i=0; i<r2des->position.size(); i++){
      smsg.append(boost::lexical_cast<std::string>(r2des->position[i]));
      smsg.append(" ");
  }
  smsg.append("@");

  valClientR2.send(smsg);
  }else{
	try{
	  RobotJointPos joints;
	  joints.resize(6);
		for (size_t i=0; i<r2des->position.size(); ++i)
		  joints.at(i) = r2des->position.at(i);
			
		_robotSoap1.moveJoints(joints);
	}
    catch (const CS8Error& err) {
      std::cout << err.what() << std::endl;
      _robotSoap1.setPower(false);
      
    }
  }
  


#ifdef _DEBUG
//  std::stringstream ss;
//  std::copy(r2des->position.begin(), r2des->position.end(), std::ostream_iterator<float>(ss, " "));
//  ROS_INFO("I heard: [%s]", ss.str().c_str());
  ROS_INFO( "Received R2_Command");
#endif
}

int main(int argc, char **argv){

  ros::V_string _vargv;
  _vargv.clear();
  ros::removeROSArgs(argc, argv, _vargv);

  if( _vargv.size() == 5 ){
	if( _vargv[2] == VAL )
		_activeConn = VAL;
	else if ( _vargv[2] == SOAP )
		_activeConn = SOAP;
	else{
		std::cout << "remoteServer VAL/SOAP ip_R1 ip_R2 freq.\nRemember it.\n";
		return -1;
	}
		
    STAUBLI_IP_R1 = _vargv[3];
    STAUBLI_IP_R2 = _vargv[4];
    SOAP_IP_R1="http://";
    SOAP_IP_R1.append(STAUBLI_IP_R1);
    SOAP_IP_R1.append(":5653");
    SOAP_IP_R2="http://";
    SOAP_IP_R2.append(STAUBLI_IP_R2);
    SOAP_IP_R2.append(":5653");

  }else{
    std::cout << "remoteServer VAL/SOAP ip_R1 ip_R2 freq.\nRemember it.\n";
    return -1;
  }

  // The connection only modifies the send data to the CS8 controller.
  // The data always are readed using SOAP.
  if( _activeConn == VAL ){
	valClientR1.connect_to(STAUBLI_IP_R1,"5558");
	valClientR2.connect_to(STAUBLI_IP_R2,"5558");
	try{
	  _robotSoap1.setURL(SOAP_IP_R1);
	  _robotSoap1.login("default","");
	  _robotSoap2.setURL(SOAP_IP_R2);
	  _robotSoap2.login("default","");
	}catch(const CS8Error& err) {
	  std::cerr << err.what() << std::endl;
	  _robotSoap1.setPower(false) ;
	  _robotSoap2.setPower(false) ;
	}
  }else{ // Here use only the SOAP
	try{
	  _robotSoap1.setURL(SOAP_IP_R1);
	  _robotSoap1.login("default","");
	  _robotSoap1.setPower(true) ;
	  _robotSoap1.resetMotion();
	  _robotSoap2.setURL(SOAP_IP_R2);
	  _robotSoap2.login("default","");
	  _robotSoap2.setPower(true) ;
	  _robotSoap2.resetMotion();
	}catch(const CS8Error& err) {
	  std::cerr << err.what() << std::endl;
	  _robotSoap1.setPower(false) ;
	  _robotSoap2.setPower(false) ;
	}
  }
	
	

  std::cout << "We have connected to the two Stäubli TX" << std::endl;

  ros::init(argc, argv, "remoteServer");

  ros::NodeHandle n;
  ros::Publisher _robot1_pub = n.advertise<sensor_msgs::JointState>("R1_state", 1000);
  ros::Publisher _robot2_pub = n.advertise<sensor_msgs::JointState>("R2_state", 1000);
  ros::Subscriber _robot1_subs = n.subscribe("R1_command", 1000, robot1Callback);
  ros::Subscriber _robot2_subs = n.subscribe("R2_command", 1000, robot2Callback);

  ros::Rate loop_rate(atoi(argv[3]));

  std::vector<double> target_pos(6, 0.);

  while (ros::ok())
  {
    sensor_msgs::JointState _r1_state_message;
    _r1_state_message.header.stamp = ros::Time::now();
    _r1_state_message.position.resize(6);

    _robotSoap1.GetRobotJoints(target_pos);
    std::copy( target_pos.begin(),target_pos.end(), _r1_state_message.position.begin() );

#ifdef _DEBUG
    _r1_state_message.name.resize(6);
    _r1_state_message.name[0] ="shoulder";
    _r1_state_message.name[1] ="forearm";
    _r1_state_message.name[2] ="elbow";
    _r1_state_message.name[3] ="arm";
    _r1_state_message.name[4] ="wrist";
    _r1_state_message.name[5] ="tcp";
#endif

    sensor_msgs::JointState _r2_state_message;
    _r2_state_message.header.stamp = ros::Time::now();
    _r2_state_message.position.resize(6);

    _robotSoap2.GetRobotJoints(target_pos);

    std::copy( target_pos.begin(),target_pos.end(), _r2_state_message.position.begin() );

#ifdef _DEBUG
    _r2_state_message.name.resize(6);
    _r2_state_message.name[0] ="shoulder";
    _r2_state_message.name[1] ="forearm";
    _r2_state_message.name[2] ="elbow";
    _r2_state_message.name[3] ="arm";
    _r2_state_message.name[4] ="wrist";
    _r2_state_message.name[5] ="tcp";
#endif

    _robot1_pub.publish(_r1_state_message);
    _robot2_pub.publish(_r2_state_message);

#ifdef _DEBUG
    ROS_INFO( "Published R1_state and R2_state" );
#endif

    ros::spinOnce();

    loop_rate.sleep();
  }
  return 0;
}

