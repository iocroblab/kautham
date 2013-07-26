// Included to use the ROS messaging system
#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>

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

haptic::haptic_data* data = NULL;

void wrenchCallback(const geometry_msgs::WrenchStamped::ConstPtr& wrench_msg)
{
  // Limiting the scope of the mutex lock
  //Lock the mutex
  if(data != NULL){
    scoped_lock<interprocess_mutex> lock(data->mutex_out);

    data->wrench[0] = wrench_msg->wrench.force.x;
    data->wrench[1] = wrench_msg->wrench.force.y;
    data->wrench[2] = wrench_msg->wrench.force.z;
    data->wrench[3] = wrench_msg->wrench.torque.x;
    data->wrench[4] = wrench_msg->wrench.torque.y;
    data->wrench[5] = wrench_msg->wrench.torque.z;

  }

#ifdef _DEBUG
  std::stringstream ss;
  for(int i = 0; i < 6; i++) ss << data->wrench[i] << " ";
  ROS_INFO("Wrench to be applied: [%s]", ss.str().c_str());
#endif

}

int main(int argc, char **argv){
  try{
    // Only to test it.
//    struct shm_remove {
//        shm_remove() { shared_memory_object::remove("PhantomSharedMemory"); }
//        ~shm_remove(){ shared_memory_object::remove("PhantomSharedMemory"); }
//    } remover;

    //Open a shared memory object.
    //shared_memory_object shm(  create_only,                 // Open shared memory -- Only to test it.
    shared_memory_object shm( open_only,                  // Open shared memory
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
    data = new (addr) haptic::haptic_data;

    //  Filter the parameters to obtain the ip_master, ip_node, nodeName, and the frequency of publishing
    ros::V_string   _vargv;
    _vargv.clear();
    ros::removeROSArgs(argc, argv, _vargv);

    // Now, the node is initialized with the respective remapping.
    std::map<std::string,std::string> remappings;
    remappings["__master"] = _vargv[1];
    remappings["__hostname"] = _vargv[2];
    ros::init(remappings,_vargv[3] + "_fdback");

    ros::NodeHandle n;

    ros::Subscriber wrench_sub = n.subscribe(_vargv[3] + "_force", 1000, wrenchCallback);

    ros::spin();


    return 0;
  }catch(...){
    std::cerr << "Unexpected error trying to open the wrench suscriber.";
  }

  return -1;
}
