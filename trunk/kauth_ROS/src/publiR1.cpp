// Included to use the ROS messaging system
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
// Included to use the shared memory between the Kautham and the publisher
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>

// Include the memory share object definition
#include "data_ioc_cell.hpp"

#include <sstream>
#include <iostream>
#include <algorithm>
#include <iterator>

using namespace kautham;
using namespace boost::interprocess;

int main(int argc, char **argv)
{
  //Remove shared memory on construction and destruction
  struct shm_remove 
  {
      shm_remove() { shared_memory_object::remove("KauthamSharedMemory"); }
      ~shm_remove(){ shared_memory_object::remove("KauthamSharedMemory"); }
  } remover;

  //Create a shared memory object.
   shared_memory_object shm
         (create_only               //only create
         ,"KauthamSharedMemory"          //name
         ,read_write   //read-write mode
         );

  //Set size
  shm.truncate(sizeof(data_ioc_cell));

  //Map the whole shared memory in this process
  mapped_region region
         (shm          //What to map
         ,read_write   //Map it as read-write
         );

  //Get the address of the mapped region
  void * addr       = region.get_address();

  //Construct the shared structure in memory
  data_ioc_cell* data = new (addr) data_ioc_cell;

  ros::init(argc, argv, "remoteR1");

  ros::NodeHandle n;

  ros::Publisher robot1_pub = n.advertise<sensor_msgs::JointState>("R1_State", 1000);

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    sensor_msgs::JointState _r1_state_message;
    _r1_state_message.header.stamp = ros::Time::now();
    _r1_state_message.name.resize(6);
    _r1_state_message.position.resize(6);
    _r1_state_message.name[0] ="shoulder";
    _r1_state_message.name[1] ="forearm";
    _r1_state_message.name[2] ="elbow";
    _r1_state_message.name[0] ="arm";
    _r1_state_message.name[1] ="wrist";
    _r1_state_message.name[2] ="tcp";

    {  // Limiting the scope of the mutex lock
      //Lock the mutex
      scoped_lock<interprocess_mutex> lock(data->mutex_out);

      std::copy( data->r1_state.joint,data->r1_state.joint+sizeof(data->r1_state.joint)/sizeof(data->r1_state.joint[0]),
                 _r1_state_message.position.begin() );
    }
    std::stringstream ss;

    std::copy(_r1_state_message.position.begin(), _r1_state_message.position.end(), std::ostream_iterator<float>(ss, " "));
    ROS_INFO("%s", ss.str().c_str() );

    robot1_pub.publish(_r1_state_message);

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}

