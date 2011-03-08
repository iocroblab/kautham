
#if !defined(_DATA_IOC_CELL)
#define _DATA_IOC_CELL

#include <boost/interprocess/sync/interprocess_mutex.hpp>

struct data_ioc_cell {

   data_ioc_cell(){
	for(int i=0; i < 6; i++){
		r1Pos[i] = 0.;
		r1Vel[i] = 0.;
		r2Pos[i] = 0.;
		r2Vel[i] = 0.;
	}
	r2Pos[6] = 0.;
	r2Vel[6] = 0.;
   }

   //Items to fill
   float r1Pos[6];
   float r1Vel[6];
   float r2Pos[7];
   float r2Vel[7];
};

struct kautham_ioc_cell{
   // Data to be filled by the suscriber and used by the Kautham  application 
   data_ioc_cell in; // data received from the server


   // Data to be filled by the kautham application and read by publisher
   data_ioc_cell out;// data produced here that will be sent to the server.
   

   //Mutex to protect access to the data
   boost::interprocess::interprocess_mutex mutex_in, mutex_out;
};

#endif // _DATA_IOC_CELL

