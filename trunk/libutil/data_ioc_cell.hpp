#ifndef _KAUTHAM_IPC_H
#define _KAUTHAM_IPC_H

#include <boost/interprocess/sync/interprocess_mutex.hpp>
//#include <boost/interprocess/managed_shared_memory.hpp>
//#include <boost/interprocess/containers/vector.hpp>
//#include <boost/interprocess/allocators/allocator.hpp>

namespace kautham{
  /////Define an STL compatible allocator of ints that allocates from the managed_shared_memory.
  /////This allocator will allow placing containers in the segment
  //typedef boost::interprocess::allocator<int, boost::interprocess::managed_shared_memory::segment_manager>  ShmemAllo;

  /////Alias a vector that uses the previous STL-like allocator so that allocates
  /////its values from the segment
  //typedef boost::interprocess::vector<int, ShmemAllo> PathVector;
  struct robot_data{
    robot_data(){
      for(int i=0; i < 6; i++){
        joint[i] = 0.;
        velocity[i] = 0.;
      }
    }
    float joint[6];
    float velocity[6];
  };

  struct data_ioc_cell{
    data_ioc_cell(){
      synchronized = false;
      //pathChange = false;
      //path = segment.construct<PathVector>("DesPath")(ShmemAllo);
    }
    //Items to fill
    bool        synchronized;
    robot_data  r1_state;
    robot_data  r1_desired;
    robot_data  r2_state;
    robot_data  r2_desired;
    //Mutex to protect access to the data
    boost::interprocess::interprocess_mutex mutex_in, mutex_out;

    //// Data about the path
    //bool        pathChange;
    //PathVector* path;
    ////Mutex to protect access to the data
    //boost::interprocess::interprocess_mutex mutpath_in, mutpath_out;
  };
}

#endif //_KAUTHAM_IPC_H



