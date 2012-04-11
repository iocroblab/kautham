// Included to use the ROS messaging system
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

// Include the memory share object definition
//#include "data_ioc_cell.hpp"

// Include the Soap Client for the CS8 controler.
#include <CS8Robot.h>
//#include <CS8Types.h>

// Include the VAL client.
#include "connection.hpp"

#include <sstream>
#include <iostream>
#include <algorithm>
#include <iterator>
#include <vector>


// Creatin the two VAL clients;
Connection    valClientR1;
Connection    valClientR2;

// Creating the two SOAP Clients.
CS8Robot      _robotSoap1;
CS8Robot      _robotSoap2;

// The emulation variables.
std::vector<double> _emuRobot1;
std::vector<double> _emuRobot2;

std::string   STAUBLI_IP_R1;
std::string   STAUBLI_IP_R2;
std::string   SOAP_IP_R1;
std::string   SOAP_IP_R2;
enum connType{VAL, SOAP, EMU} _activeConn;

std::string   PORT="";

void robot1Callback(const sensor_msgs::JointState::ConstPtr& r1des)
{
  std::string smsg;
  switch( _activeConn ){
  case VAL:
    // Now I send this data to the CS8 using val program and simple udp socket.
    for (unsigned int i=0; i<r1des->position.size(); i++){
      smsg.append(boost::lexical_cast<std::string>(r1des->position[i]));
      smsg.append(" ");
    }
    smsg.append("@");

    valClientR1.send(smsg);
    break;
    
  case SOAP:
    try{
      RobotJointPos joints;
      joints.resize(6);
      for (size_t i=0; i<r1des->position.size(); ++i)
	joints.at(i) = r1des->position.at(i);
      
      _robotSoap1.moveJoints(joints);
    }catch (const CS8Error& err) {
      std::cout << err.what() << std::endl;
      _robotSoap1.setPower(false);
      
    }
    break;
    
  case EMU:
    for (size_t i=0; i<r1des->position.size(); ++i)
	  _emuRobot1.at(i) = r1des->position.at(i);
  }

#ifdef _DEBUG
//  std::stringstream ss;
//  std::copy(r1des->position.begin(), r1des->position.end(), std::ostream_iterator<float>(ss, " "));
//  ROS_INFO("I heard: [%s]", ss.str().c_str());
  ROS_INFO( "Received R1_Command" );
#endif

}

void robot2Callback(const sensor_msgs::JointState::ConstPtr& r2des){
  
    std::string smsg;
  switch( _activeConn ){
    case VAL:
      // Now I send this data to the CS8 using val program and simple udp socket.
      for (unsigned int i=0; i<r2des->position.size(); i++){
	smsg.append(boost::lexical_cast<std::string>(r2des->position[i]));
	smsg.append(" ");
      }
      smsg.append("@");

      valClientR2.send(smsg);
      break;
	    
    case SOAP:
      try{
	RobotJointPos joints;
	joints.resize(6);
	for (size_t i=0; i<r2des->position.size(); ++i)
	  joints.at(i) = r2des->position.at(i);
		      
	_robotSoap2.moveJoints(joints);
      }catch (const CS8Error& err) {
	std::cout << err.what() << std::endl;
	_robotSoap2.setPower(false);
	
      }
      break;
	  
    case EMU:
      for(size_t i=0; i<r2des->position.size(); ++i)
	    _emuRobot2.at(i) = r2des->position.at(i);
  }

#ifdef _DEBUG
//  std::stringstream ss;
//  std::copy(r2des->position.begin(), r2des->position.end(), std::ostream_iterator<float>(ss, " "));
//  ROS_INFO("I heard: [%s]", ss.str().c_str());
  ROS_INFO( "Received R2_Command");
#endif
}


// This program have been designed to run in the Quasar machine in the IOC lab.
// This program is part of the ROS nodes used by the Kautham project in order to 
// take the capability to teleoperate the Staubli robots and to have haptic 
// feedback of this teleoperation procedure.
// The program could be connected to the CS8 controller using either VAL or 
// SOAP protocols. Furthermore, the program can be used in EMU mode. This 
// execution modality provides the emulation capability in order to connect 
// two simulation nodes with the remote Cell avoiding the robot movements.

int main(int argc, char **argv){

  ros::V_string _vargv;
  _vargv.clear();
  ros::removeROSArgs(argc, argv, _vargv);

  if( _vargv.size() == 5 ){
    if( _vargv[1] == "VAL" )
      _activeConn = VAL;
    else if ( _vargv[1] == "SOAP" )
      _activeConn = SOAP;
    else{
      std::cout << "remoteServer VAL/SOAP/EMU ip_R1 ip_R2 freq.\nRemember it.\n";
      return -1;
    }
		
    STAUBLI_IP_R1 = _vargv[2];
    STAUBLI_IP_R2 = _vargv[2];
    SOAP_IP_R1="http://";
    SOAP_IP_R1.append(STAUBLI_IP_R1);
    SOAP_IP_R1.append(":5653");
    SOAP_IP_R2="http://";
    SOAP_IP_R2.append(STAUBLI_IP_R2);
    SOAP_IP_R2.append(":5653");

  }else{
    if ( _vargv.size() > 1 && _vargv[1] == "EMU" )
      _activeConn = EMU;
    else{
      for(size_t i =0; i< _vargv.size(); ++i )
	std::cout << _vargv.at(i) << "\t" ;
      std::cout << "\nremoteServer VAL/SOAP/EMU ip_R1 ip_R2 freq.\nRemember it.\n";
      return -1;
    }
  }

  // The connection only modifies the send data to the CS8 controller.
  // The data always are readed using SOAP.
  switch( _activeConn ){
  case VAL:
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
	break;
	
  case SOAP: // Here use only the SOAP
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
	break;
	
  case EMU:
	_emuRobot1.resize(6);
	_emuRobot2.resize(6);
		
  }
	
  std::cout << "We have connected to the two Stäubli TX ";
  if( _activeConn == EMU ) std::cout << "in Emulation mode." ;
  std::cout  << std::endl;
  ros::init(argc, argv, "remoteServer");

  ros::NodeHandle n;
  ros::Publisher _robot1_pub = n.advertise<sensor_msgs::JointState>("R1_state", 1000);
  ros::Publisher _robot2_pub = n.advertise<sensor_msgs::JointState>("R2_state", 1000);
  ros::Subscriber _robot1_subs = n.subscribe("R1_command", 1000, robot1Callback);
  ros::Subscriber _robot2_subs = n.subscribe("R2_command", 1000, robot2Callback);

  int rate=250;
  if( _activeConn != EMU )
    rate = atoi( _vargv[4].c_str() );
  

  ros::Rate loop_rate( rate );
  std::vector<double> target_pos(6, 0.);

  while (ros::ok())
  {
    sensor_msgs::JointState _r1_state_message;
    _r1_state_message.header.stamp = ros::Time::now();
    _r1_state_message.position.resize(6);

	if( _activeConn == EMU ){
	  std::copy( _emuRobot1.begin(),_emuRobot1.end(), _r1_state_message.position.begin() );
	}else{
		_robotSoap1.getRobotJointPos(target_pos);
		std::copy( target_pos.begin(),target_pos.end(), _r1_state_message.position.begin() );
	}

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

	if( _activeConn == EMU ){
	  std::copy( _emuRobot2.begin(),_emuRobot2.end(), _r2_state_message.position.begin() );
	}else{
      _robotSoap2.getRobotJointPos(target_pos);
      std::copy( target_pos.begin(),target_pos.end(), _r2_state_message.position.begin() );
	}

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
  
  try{
    _robotSoap1.setPower(false) ;
    _robotSoap2.setPower(false) ;
  }catch(...){}
  
  std::cout << "The connection with robots is leaving.\n";
  return 0;
  
}

