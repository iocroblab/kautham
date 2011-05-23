// Included to use the ROS messaging system
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
// Included to use the shared memory between the Kautham and the publisher
//#include <boost/interprocess/shared_memory_object.hpp>
//#include <boost/interprocess/mapped_region.hpp>
//#include <boost/interprocess/sync/scoped_lock.hpp>

// Include the memory share object definition
//#include "data_ioc_cell.hpp"

// Include de Soap Client for the CS8 controler.
#include <TX90.h>
#include "connection.hpp"

#include <sstream>
#include <iostream>
#include <algorithm>
#include <iterator>

//using namespace kautham;
//using namespace boost::interprocess;

//data_ioc_cell* _data = NULL;
Connection    valClientR1;
Connection    valClientR2;

TX90          _robotSoap;
std::string   STAUBLI_IP_R1;
std::string   STAUBLI_IP_R2;
std::string   PORT="";

void robot1Callback(const sensor_msgs::JointState::ConstPtr& r1des)
{
  // Now I send this data to the CS8 using val program and simple udp socket.
  std::string smsg;
  for (unsigned int i=0; i<r1des->position.size(); i++){
      smsg.append(boost::lexical_cast<std::string>(r1des->position[i]));
      smsg.append(" ");
  }
  smsg.append("@");

  valClientR1.send(smsg);

   // Limiting the scope of the mutex lock
   //Lock the mutex
//  if(_data != NULL){
//    scoped_lock<interprocess_mutex> lock(_data->mutex_in);

//    for(size_t i = 0; i < r1des->position.size(); ++i)
//      _data->r1_desired.joint[i] = r1des->position[i];

//  }

#ifdef _DEBUG
//  std::stringstream ss;
//  std::copy(r1des->position.begin(), r1des->position.end(), std::ostream_iterator<float>(ss, " "));
//  ROS_INFO("I heard: [%s]", ss.str().c_str());
  ROS_INFO( "Received R1_Command" );
#endif

}

void robot2Callback(const sensor_msgs::JointState::ConstPtr& r2des){
  // Now I send this data to the CS8 using val program and simple udp socket.
  std::string smsg;
  for (unsigned int i=0; i<r2des->position.size(); i++){
      smsg.append(boost::lexical_cast<std::string>(r2des->position[i]));
      smsg.append(" ");
  }
  smsg.append("@");

  valClientR2.send(smsg);

//  Limiting the scope of the mutex lock
//  Lock the mutex
//  if(_data != NULL){
//    scoped_lock<interprocess_mutex> lock(_data->mutex_in);

//    for(size_t i = 0; i < r2des->position.size(); ++i)
//      _data->r2_desired.joint[i] = r2des->position[i];

//  }

#ifdef _DEBUG
//  std::stringstream ss;
//  std::copy(r2des->position.begin(), r2des->position.end(), std::ostream_iterator<float>(ss, " "));
//  ROS_INFO("I heard: [%s]", ss.str().c_str());
  ROS_INFO( "Received R2_Command");
#endif
}

int main(int argc, char **argv){

  //Remove shared memory on construction and destruction
//  struct shm_remove{
//      shm_remove() { shared_memory_object::remove("KauthamSharedMemory"); }
//      ~shm_remove(){ shared_memory_object::remove("KauthamSharedMemory"); }
//  } remover;

//  //Create a shared memory object.
//   shared_memory_object shm
//         (create_only               //only create
//         ,"KauthamSharedMemory"          //name
//         ,read_write   //read-write mode
//         );

//  //Set size
//  shm.truncate(sizeof(data_ioc_cell));

//  //Map the whole shared memory in this process
//  mapped_region region
//         (shm          //What to map
//         ,read_write   //Map it as read-write
//         );

//  //Get the address of the mapped region
//  void * _addr       = region.get_address();

//  //Construct the shared structure in memory
//  _data = new (_addr) data_ioc_cell;

  ros::V_string _vargv;
  _vargv.clear();
  ros::removeROSArgs(argc, argv, _vargv);

  if(_vargv.size() > 1 && _vargv.size()< 4){
    STAUBLI_IP_R1 = _vargv[1];
    STAUBLI_IP_R2 = _vargv[2];

  }else{
    std::cout << "remoteServer ip_R1 ip_R2 freq.\nRemember it.\n";
    return -1;
  }

  valClientR1.connect_to(STAUBLI_IP_R1,"5558");
  valClientR2.connect_to(STAUBLI_IP_R2,"5558");

  std::cout << "We have connected and the two Stäubli TX" << std::endl;

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

    _robotSoap.Login(STAUBLI_IP_R1, "default", "");
    _robotSoap.GetRobotJoints(target_pos);
    _robotSoap.Logoff();

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

    _robotSoap.Login(STAUBLI_IP_R2, "default", "");
    _robotSoap.GetRobotJoints(target_pos);
    _robotSoap.Logoff();

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

//    {  // Limiting the scope of the mutex lock
//      //Lock the mutex
//      scoped_lock<interprocess_mutex> lock(_data->mutex_out);

//      std::copy( _data->r1_state.joint,_data->r1_state.joint+sizeof(_data->r1_state.joint)/sizeof(_data->r1_state.joint[0]),
//                 _r1_state_message.position.begin() );

//      std::copy( _data->r2_state.joint,_data->r2_state.joint+sizeof(_data->r2_state.joint)/sizeof(_data->r2_state.joint[0]),
//                 _r2_state_message.position.begin() );
//    }

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

