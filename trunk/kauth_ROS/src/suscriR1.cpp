
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
#include <algorithm>

using namespace boost::interprocess;
using namespace kautham;

data_ioc_cell* data = NULL;

void robot1Callback(const sensor_msgs::JointState::ConstPtr& r1pos)
{
   // Limiting the scope of the mutex lock
     //Lock the mutex
  if(data != NULL){
    scoped_lock<interprocess_mutex> lock(data->mutex_out);

    for(size_t i = 0; i < r1pos->position.size(); ++i)
      data->r1_state.joint[i] = r1pos->position[i] + 0.5;

  }

  std::stringstream ss;
  std::copy(r1pos->position.begin(), r1pos->position.end(), std::ostream_iterator<float>(ss, " "));
  ROS_INFO("I heard: [%s]", ss.str().c_str());
  
}

int main(int argc, char **argv)
{
  try{
   //Open the shared memory object.
   shared_memory_object shm
      (open_only                    //only create
      ,"KauthamSharedMemory"              //name
      ,read_write  //read-write mode
      );

   //Map the whole shared memory in this process
   mapped_region region
      (shm,                       //What to map
       read_write //Map it as read-write
      );

   //Get the address of the mapped region
   void* addr       = region.get_address();

   //Construct the shared structure in memory
   data = static_cast<data_ioc_cell*>(addr);


  ros::init(argc, argv, "suscriR1");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("R1_State", 1000, robot1Callback);

  ros::spin();

  return 0;
 
 }catch(interprocess_exception &ex){
      std::cout << ex.what() << std::endl;
      return 1;
 }

}

