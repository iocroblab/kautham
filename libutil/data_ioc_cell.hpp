#ifndef _KAUTHAM_IPC_H
#define _KAUTHAM_IPC_H

#include <boost/interprocess/sync/interprocess_mutex.hpp>

namespace kautham{
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
    }
    //Items to fill
    bool       synchronized;
    robot_data r1_state;
    robot_data r1_desired;
    robot_data r2_state;
    robot_data r2_desired;
    //Mutex to protect access to the data
    boost::interprocess::interprocess_mutex mutex_in, mutex_out;
  };
}

#endif //_KAUTHAM_IPC_H



