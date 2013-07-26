// Included to use the ROS messaging system
#include <ros/ros.h>
#include <std_msgs/Bool.h>

// Included to use the shared memory between the haptic application and the publisher
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>

// Include the memory share object definition
#include "haptic_data.hpp"

#include <sstream>
#include <iostream>
#include <algorithm>
#include <iterator>

using namespace boost::interprocess;

int main(int argc, char **argv){
  try{
    // Only to test it.
//    struct shm_remove {
//        shm_remove() { shared_memory_object::remove("PhantomSharedMemory"); }
//        ~shm_remove(){ shared_memory_object::remove("PhantomSharedMemory"); }
//    } remover;

    //Open a shared memory object.
    //shared_memory_object shm(  create_only,                 // Open shared memory -- Only to test it.
    shared_memory_object shm(  open_only,                  // Open shared memory
                                "PhantomSharedMemory",      // name
                                read_write );               // read-write mode

    //Set size
    shm.truncate( sizeof( haptic::haptic_data ) );

    //Map the whole shared memory in this process
    mapped_region region ( shm,                     //What to map
                           read_write);             //Map it as read-write


    //Get the address of the mapped region
    void * addr = region.get_address();

    //Construct the shared structure in memory
    haptic::haptic_data* data = new (addr) haptic::haptic_data;

    //  Filter the parameters to obtain the ip_master, ip_node, nodeName, and the frequency of publishing
    ros::V_string   _vargv;
    _vargv.clear();
    ros::removeROSArgs(argc, argv, _vargv);

    std::cout << _vargv[1] << " " << _vargv[2] << " " << _vargv[3] << " " << _vargv[4] << std::endl;

    // Now, the node is initialized with the respective remapping.
    std::map<std::string,std::string> remappings;
    remappings["__master"] = _vargv[1];
    remappings["__hostname"] = _vargv[2];
    ros::init(remappings,_vargv[3]);

    ros::NodeHandle n;

    ros::Publisher button_pub = n.advertise<std_msgs::Bool>(_vargv[3] + "_button", 1000);

    int frq = atoi( _vargv[4].c_str() );

    ros::Rate loop_rate(frq);

    while( ros::ok() ){
      std_msgs::Bool _bool_msg;

      {  // Limiting the scope of the mutex lock
        //Lock the mutex
        scoped_lock<interprocess_mutex> lock(data->mutex_out);

        //set the value
        _bool_msg.data = data->button;
      }

  #ifdef _DEBUG
      ROS_INFO("The %s has been published", _vargv[3] + "_button" );
  #endif

      //publish the message
      button_pub.publish(_bool_msg);

      ros::spinOnce();

      loop_rate.sleep();
    }

    return 0;
  }catch(...){
    std::cerr << "Unexpected error trying to open the button publisher.";
  }

  return -1;
}
