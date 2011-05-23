#ifndef _HAPTIC_DATA_INTERPROCESS
#define _HAPTIC_DATA_INTERPROCESS

#include <boost/interprocess/sync/interprocess_mutex.hpp>

namespace haptic{
  struct haptic_data{
    haptic_data(){
      for(int i=0; i < 6; i++){
        hipPose[i] = 0.;
        hipVel[i] = 0.;
        wrench[i] = 0.;
      }
      hipPose[6] = 0.;
      button = false;
    }

    //Button state
    bool button;
    //! The pose (position + quaternion ) of the hip in this order: x, y, z, wx, wy, wz, ang.
    double hipPose[7];
	  
    //! The twist of the hip in this order: xdot, ydot, xdot, angxdot, angydot, angzdot.
    double hipVel[6];
	  
    //! The wrench to be applied to the HIP in the following order: fx, fy, fz, Tx, Ty, Tz.
    double wrench[6];
	  
    //! Mutex to protect access to the data
    boost::interprocess::interprocess_mutex mutex_in, mutex_out;
  };
}

#endif //_HAPTIC_DATA_INTERPROCESS

