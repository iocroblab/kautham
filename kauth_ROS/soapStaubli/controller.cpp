
/*
 * controller.cpp
 *
 *  Created on: Jan 29, 2010
 *      Author: joseph
 */

#include "TX90.h"
#include <iostream>
#include <algorithm>
// #include <ros/ros.h>
// #include <std_msgs/String.h>
#include <string>
#include <sstream>
#include <cstdlib>
// #include "staubli/command.h"		//command service of staubli
#include "controller.h"
#include <cmath>

using namespace std;

#if !defined( toRad )
#define toRad M_PI/180.
#endif

#ifdef WIN32
    #include <windows.h>
    #define SLEEP(n) Sleep(n);
#else
    #include <unistd.h>
    #define SLEEP(n) usleep(n);
#endif


void print(double e){
	std::cout << " " << e;
}

//int GetPosition(staubli::command::Response *res)
int GetPosition(){

	TX90 robot;
	robot.Login(STAUBLI_IP, "default", "");
	robot.Power(true);
	// ROS_INFO("Staubli is connected !!.");

	std::vector<double> position;
	position.resize(6);

	if(robot.GetRobotCartesianPosition(position)){
		for_each(position.begin(), position.end(), print);
		std::cout <<"\n";
		// res->x = position[0];
		// res->y = position[1];
		// res->z = position[2];
		// res->theta_x  = position[3];
		// res->theta_y  = position[4];
		// res->theta_z  = position[5];
	}else
		std::cout <<"wrong\n";

	robot.Logoff();
	return 0;
}

// int GetJoints(staubli::command::Response *res)
  int GetJoints( void )
{
	TX90 robot;
	robot.Login(STAUBLI_IP, "default", "");
	robot.Power(true);
	// ROS_INFO("Staubli is connected !!.");

	std::vector<double> joints;
	joints.resize(6);

	if(robot.GetRobotJoints(joints)){
		for_each(joints.begin(), joints.end(), print);
		std::cout <<"\n";
		// res->joint1 = joints[0];
		// res->joint2 = joints[1];
		// res->joint3 = joints[2];
		// res->joint4  = joints[3];
		// res->joint5  = joints[4];
		// res->joint6  = joints[5];
	}else
		std::cout <<"wrong\n";

	robot.Logoff();
	return 0;
}

// int MoveJoints(staubli::command::Request  *req,
			   // staubli::command::Response *res)
  int MoveJoints( std::vector<double> target_joints )
{
	TX90 robot;
	robot.Login(STAUBLI_IP, "default", "");
	robot.Power(true);
	// ROS_INFO("Staubli is connected !!.");

	//Move joints
	// std::vector<double> target_joints;
	// target_joints.push_back(req->joint1);
	// target_joints.push_back(req->joint2);
	// target_joints.push_back(req->joint3);
	// target_joints.push_back(req->joint4);
	// target_joints.push_back(req->joint5);
	// target_joints.push_back(req->joint6);
	robot.ResetMotion();
	robot.MoveJoints(target_joints);

	//Get Current joints
	for_each(target_joints.begin(), target_joints.end(), print);
	std::cout <<"\n";
	// res->joint1 = target_joints[0];
	// res->joint2 = target_joints[1];
	// res->joint3 = target_joints[2];
	// res->joint4  = target_joints[3];
	// res->joint5  = target_joints[4];
	// res->joint6  = target_joints[5];

	//Log off staubli
	robot.Logoff();
  return 0;
}

// int MoveStraightLine(staubli::command::Request  *req,
					 // staubli::command::Response *res)
  int MoveStraightLine( std::vector<double> target_pos )
{
	TX90 robot;
	robot.Login(STAUBLI_IP, "default", "");
	robot.Power(true);
	//ROS_INFO("Staubli is connected !!.");

	//Move joints
	// std::vector<double> target_pos;
	// target_pos.push_back(req->x);
	// target_pos.push_back(req->y);
	// target_pos.push_back(req->z);
	// target_pos.push_back(req->theta_x);
	// target_pos.push_back(req->theta_y);
	// target_pos.push_back(req->theta_z);
	robot.ResetMotion();
	robot.MoveLine(target_pos);

	//Get Current joints
	for_each(target_pos.begin(), target_pos.end(), print);
	std::cout <<"\n";
	// res->x = target_pos[0];
	// res->y = target_pos[1];
	// res->z = target_pos[2];
	// res->theta_x  = target_pos[3];
	// res->theta_y  = target_pos[4];
	// res->theta_z  = target_pos[5];

	//Log off staubli
	robot.Logoff();
  return 0;
}

// int MoveCart(staubli::command::Request  *req,
					 // staubli::command::Response *res)
  int MoveCart( std::vector<double> target_pos )
{
	TX90 robot;
	robot.Login(STAUBLI_IP, "default", "");
	robot.Power(true);
	//ROS_INFO("Staubli is connected !!.");

	//Move joints
	// std::vector<double> target_pos;
	// target_pos.push_back(req->x);
	// target_pos.push_back(req->y);
	// target_pos.push_back(req->z);
	// target_pos.push_back(req->theta_x);
	// target_pos.push_back(req->theta_y);
	// target_pos.push_back(req->theta_z);
	robot.ResetMotion();
	robot.MoveCart(target_pos);

	//Get Current joints
	for_each(target_pos.begin(), target_pos.end(), print);
	std::cout <<"\n";
	// res->x = target_pos[0];
	// res->y = target_pos[1];
	// res->z = target_pos[2];
	// res->theta_x  = target_pos[3];
	// res->theta_y  = target_pos[4];
	// res->theta_z  = target_pos[5];

	//Log off staubli
	robot.Logoff();
  return 0;
}

// Service  for forward kinematics
// int ForwardKinematics(staubli::command::Request  *req,
					 // staubli::command::Response *res)
  int ForwardKinematics( std::vector<double> target_joints )
{
	TX90 robot;
	robot.Login(STAUBLI_IP, "default", "");
	robot.Power(true);
	//ROS_INFO("Staubli is connected !!.");

	std::vector<double> result_pos;
	// std::vector<double> target_joints;
	// target_joints.push_back(req->joint1);
	// target_joints.push_back(req->joint2);
	// target_joints.push_back(req->joint3);
	// target_joints.push_back(req->joint4);
	// target_joints.push_back(req->joint5);
	// target_joints.push_back(req->joint6);
  for_each(target_joints.begin(), target_joints.end(), print);
  std::cout <<"\n";

	robot.ForwardKinematics(target_joints,result_pos);

	//save the result position into response of the command service
	// Unit : meter, degree
  for_each(result_pos.begin(), result_pos.end(), print);
	std::cout <<"\n";
  
	// res->x = result_pos[0];
	// res->y = result_pos[1];
	// res->z = result_pos[2];
	// res->theta_x  = result_pos[3];
	// res->theta_y  = result_pos[4];
	// res->theta_z  = result_pos[5];

	robot.Logoff();
	return 0;

}


// Service  for forward kinematics
// int InverseKinematics(staubli::command::Request  *req,
					 // staubli::command::Response *res)
  int InverseKinematics( std::vector<double> target_pos )
{
	TX90 robot;
	robot.Login(STAUBLI_IP, "default", "");
	robot.Power(true);
	//ROS_INFO("Staubli is connected !!.");

	// std::vector<double> target_pos, current_joints, result_joints;
  std::vector<double> current_joints, result_joints;
	// target_pos.push_back(req->x);
	// target_pos.push_back(req->y);
	// target_pos.push_back(req->z);
	// target_pos.push_back(req->theta_x);
	// target_pos.push_back(req->theta_y);
	// target_pos.push_back(req->theta_z);

	robot.GetRobotJoints(current_joints);
	robot.InverseKinematics(target_pos,current_joints, result_joints);

	//save the result position into response of the command service
  for_each(result_joints.begin(), result_joints.end(), print);
	std::cout <<"\n";
	// res->joint1 = result_joints[0];
	// res->joint2 = result_joints[1];
	// res->joint3 = result_joints[2];
	// res->joint4  = result_joints[3];
	// res->joint5  = result_joints[4];
	// res->joint6  = result_joints[5];

	robot.Logoff();
	return 0;

}

void drawcircle(int numsegm, int seg){
  TX90 robot;
  bool canToggle= false;
  
  double totaltime = seg;// * 1000.0 ; // asumes seconds to complete the circle
  double rad = 0.200;  // Radius = 100 mm.
  double deltaTheta = 2 * M_PI / numsegm;
  double deltaTime = totaltime / numsegm;
  vector<double> position;
  position.resize(6);
  robot.Login(STAUBLI_IP, "default", "");
  robot.Power(true);

  position.at(0) = 0.700; // X coordinate
  position.at(3) = 0. * toRad;  //  alpha
  position.at(4) = 90. * toRad;  //  beta
  position.at(5) = 0. * toRad;  //  gamma
  for(int i = 0; i < numsegm; ++i){
    position.at(1) = rad * cos(i*deltaTheta) + 0.100;
    position.at(2) = rad * sin(i*deltaTheta) + 0.280;

    while(MoveCart(position)){
      SLEEP( deltaTime );
    }

  }

  robot.Power(false);
}

bool fillTarget(std::vector<double>& target ){
  int i = 0;
  double val;
  target.clear();
  do{
    try{
      cin >> val;
      target.push_back(val);
      i++;
    }catch(...){}
  }while(i < 6 );
  return target.size() == 6;
}

bool ping(string text){
  TX90 robot;
	robot.Login(STAUBLI_IP, "default", "");
	return robot.Ping(text);
}

bool resetRobot(){
  TX90 robot;
  robot.Login(STAUBLI_IP, "default", "");
  robot.Stop();
  robot.Restart(); 
  return robot.ResetMotion() ;
    
}

bool togglePower(){
  TX90 robot;
  bool canToggle= false;
  robot.Login(STAUBLI_IP, "default", "");
  if( robot.Power(true) ){
    canToggle= true;
    robot.Power(false);
  }else
    canToggle= false;

	return canToggle;
}

// handler for command service
// See command.srv
// bool commandhandler(staubli::command::Request  &req,
					// staubli::command::Response &res )
   bool commandhandler( int command ){

	// ROS_INFO("Command #%d.",req.command_number);
  std::vector<double> target;
	//switch(req.command_number)
  
  int i = 0;
  int j = 0;
  switch( command ){
		// case 1 : GetPosition(&res); break;
		// case 2 : GetJoints(&res);break;
		// case 3 : MoveJoints(&req,&res);break;
		// case 4 : MoveStraightLine(&req,&res);break;
		// case 5 : ForwardKinematics(&req,&res);break;
		// case 6 : InverseKinematics(&req,&res);break;
		// default: break;
    case 1 : GetPosition(); break;
		case 2 : GetJoints();break;
		case 3 : 
      cout << "\nProvide the q1, q2, ... q6 values.\n";
      if( fillTarget( target ) )
        MoveJoints( target );
      else 
        cout << "Please provide the right information.\n";
      break;
		case 4 : 
      cout << "\nProvide the x, y, z, alpha, beta, gamma values.\n";
      if( fillTarget( target ) )
        MoveStraightLine( target );
      else 
        cout << "Please provide the right information.\n";
      break;
		case 5 : 
      cout << "\nProvide the q1, q2, ... q6 values.\n";
      if( fillTarget( target ) )
        ForwardKinematics( target );
      else 
        cout << "Please provide the right information.\n";
      break;
		case 6 : 
      cout << "\nProvide the x, y, z, alpha, beta, gamma values.\n";
      if( fillTarget( target ) )
          InverseKinematics( target );
      else 
        cout << "Please provide the right information.\n";    
      break;
    case 7:
      cout << "Sending ""hello"" to server!!\n";
      if( ping("hello") )
        cout << "Everything is ok with the server!!\n";
      else
        cout << "Something is wrong with the server!!\n";

      break;
//     case 8:
//       cout << "Switching the power of the motors\n";
//       if( togglePower() )
//         cout << "The motors can be turn on remotely!\n";
//       else
//         cout << "The motors can not be turn on remotely!\n";
//       break;
    case 8:
      cout << "Reset the previous movements\n";
      if( resetRobot() )
        cout << "The robot movements has been cleared.\n";
      else
        cout << "The robot movements has not been cleared\n";
      break;
    case 9:
      cout << "Ready to draw a circle, be careful\n";
      cout << "How many segments should have the circle? (4-999)\n";
      do{
        cin >> i ;
      }while(i < 4 || i > 1000);
      cout << "How long time may be used to draw the circle? (5-99)\n";
      do{
        cin >> j ;
      }while(j < 5 || i > 100);
      drawcircle(i,j);
      break;
    default: break;
  }
  return true;
}

int main(int argc, char** argv){

	// ros::init(argc, argv, "staubli_server");
	// ros::NodeHandle n;
	// ros::ServiceServer service = n.advertiseService("command", commandhandler);
	// ROS_INFO("Ready to access staubli.");
	// ros::spin();
  if( argc > 1 )
    STAUBLI_IP = argv[1];
  
  if( resetRobot() )
    cout << " The robot at " << STAUBLI_IP << " is ready to be used.\n" ;
  else{
    cout << " The robot at " << STAUBLI_IP << " is not cleared and has previous movement remained.\n" ;
    exit(1);
  }
  
    
  int i = 0;
//  char opt;
  do{
      cout << "\n\nBasic Menu:\n";
      cout << " 0 : Exit\n";
      cout << " 1 : GetPosition\n";
      cout << " 2 : GetJoints\n";
      cout << " 3 : MoveJoints\n";
      cout << " 4 : MoveStraightLine\n";
      cout << " 5 : ForwardKinematics\n";
      cout << " 6 : InverseKinematics\n";
      cout << " 7 : Ping\n";
      //cout << " 8 : Switch Power\n";
      cout << " 8 : Reset Movements\n";
      cout << " 9 : Draw circle\n";
      cout << "Choose an option: ";
      cin >> i;
      //i = atoi(opt);
      if( i > 0 && i <= 9 )
        commandhandler( i );
  }while( i != 0 );

	return 0;
}
