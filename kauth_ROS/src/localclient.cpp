
// Included to use the ROS messaging system
#include "ros/ros.h"
// Include to use the mesage definition in Kautham
//#include <kautham/R1_pos.h>
#include <sensor_msgs/JointState.h>
// Included to use the shared memory between the Kautham and the publisher
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>
// Include the memory share object definition
#include "data_ioc_cell.hpp"

#include <sstream>
#include <string>
#include <algorithm>

using namespace boost::interprocess;
using namespace kautham;
using namespace std;

data_ioc_cell* _data = NULL;
bool   _robot1 = true;

void robotCallback(const sensor_msgs::JointState::ConstPtr& robpos)
{
   // Limiting the scope of the mutex lock
     //Lock the mutex
  if(_data != NULL){
    scoped_lock<interprocess_mutex> lock(_data->mutex_in);

    if( _robot1 )
      for(size_t i = 0; i < robpos->position.size(); ++i)
        _data->r1_state.joint[i] = robpos->position[i];
    else
      for(size_t i = 0; i < robpos->position.size(); ++i)
        _data->r2_state.joint[i] = robpos->position[i];

  }

#ifdef _DEBUG
  if( _robot1 )
    ROS_INFO( "R1_State received.");
  else
    ROS_INFO( "R2_State received.");
#endif
  
}

void otherRobotCallback(const sensor_msgs::JointState::ConstPtr& robpos){
   // Limiting the scope of the mutex lock
     //Lock the mutex
  if(_data != NULL){
    scoped_lock<interprocess_mutex> lock(_data->mutex_in);

    if( _robot1 )
      for(size_t i = 0; i < robpos->position.size(); ++i)
        _data->r2_state.joint[i] = robpos->position[i];
    else
      for(size_t i = 0; i < robpos->position.size(); ++i)
        _data->r1_state.joint[i] = robpos->position[i];
  }

#ifdef _DEBUG
  if( _robot1 )
    ROS_INFO( "R2_State received.");
  else
    ROS_INFO( "R1_State received.");
#endif
}

int main(int argc, char **argv){
 try{
  //Open the shared memory object.
  shared_memory_object shm( open_only,                    //open create
                            "KauthamSharedMemory",        //name
                            read_write );                 //read-write mode


  //Map the whole shared memory in this process
  mapped_region region( shm,                       //What to map
                        read_write);               //Map it as read-write


  //Get the address of the mapped region
  void* addr       = region.get_address();

  //Construct the shared structure in memory
  _data = static_cast<data_ioc_cell*>(addr);

  // what client is it?
  // firts remove any parameter added by the roslaunch or another program that adds parameters.
  ros::V_string _vargv;
  _vargv.clear();
  ros::removeROSArgs(argc, argv, _vargv);
  string _pubName = "";
  string _subName = "";
  string _subOther = "";
  string _nodeName = "";

  if( _vargv.size() > 1 && _vargv.size() < 5){
    string rob = _vargv[3] ;
    _pubName.append( rob );
    _pubName.append("_command");
    _subName.append( rob );
    _subName.append("_state");
    _nodeName = "localClient_" ;
    _nodeName.append(rob);

  }else{
    // Error because this program will run in windows and the user should provide the master url, the
    // local ip address, the node name and the freq to publish the data.
    return -1;
  }

  if( _pubName == "R1_command" ){
    _robot1 = true;
    _subOther = "R2_state";
  }else{
    _robot1 = false;
    _subOther = "R1_state";
  }


#ifdef _DEBUG
    stringstream ss;
    ss << "rob = " <<  rob << endl;
    ss << "_nodeName = " <<  _nodeName << endl;
    ss << "_pubName = " <<  _pubName << endl;
    ss << "_subName = " <<  _subName << endl;
    ROS_INFO("%s", ss.str().c_str());
#endif

  // Now, the node is initialized with the respective remapping.
  std::map<std::string,std::string> remappings;
  remappings["__master"] = _vargv[1];
  remappings["__hostname"] = _vargv[2];

  ros::init(remappings, _nodeName);

  ros::NodeHandle n;

  ros::Publisher _robot_pub = n.advertise<sensor_msgs::JointState>(_pubName, 1000);
  ros::Subscriber _robot_sub = n.subscribe(_subName, 1000, robotCallback);
  ros::Subscriber _otherRobot_sub = n.subscribe(_subOther, 1000, otherRobotCallback);


  ros::Rate loop_rate(atoi(_vargv[4].c_str()));
  bool _sync = false;

  while (ros::ok()){
    { // Limiting the scope of the mutex lock
      //Lock the mutex
      scoped_lock<interprocess_mutex> lock(_data->mutex_out);
      _sync = _data->synchronized;
    }

    if( _sync ){
      sensor_msgs::JointState _robot_desired_message;
      _robot_desired_message.header.stamp = ros::Time::now();
      _robot_desired_message.position.resize(6);
  //    _robot_desired_message.name.resize(6);
  //    _robot_desired_message.name[0] ="shoulder";
  //    _robot_desired_message.name[1] ="forearm";
  //    _robot_desired_message.name[2] ="elbow";
  //    _robot_desired_message.name[0] ="arm";
  //    _robot_desired_message.name[1] ="wrist";
  //    _robot_desired_message.name[2] ="tcp";

      {  // Limiting the scope of the mutex lock
      //Lock the mutex
      scoped_lock<interprocess_mutex> lock(_data->mutex_out);

      if( _robot1 )
        std::copy( _data->r1_desired.joint,_data->r1_desired.joint+sizeof(_data->r1_desired.joint)/sizeof(_data->r1_desired.joint[0]),
                 _robot_desired_message.position.begin() );
      else
        std::copy( _data->r2_desired.joint,_data->r2_desired.joint+sizeof(_data->r2_desired.joint)/sizeof(_data->r2_desired.joint[0]),
                 _robot_desired_message.position.begin() );
      }



      _robot_pub.publish(_robot_desired_message);

#ifdef _DEBUG
      std::stringstream ss;

      std::copy(_robot_desired_message.position.begin(), _robot_desired_message.position.end(), std::ostream_iterator<float>(ss, " "));
      ROS_INFO("Command has been published: %s", ss.str().c_str() );
#endif
    } // if (_data->synchronized)

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
 
 }catch(interprocess_exception &ex){
      std::cout << ex.what() << std::endl;
      return 1;
 }catch(...){

 }

}

