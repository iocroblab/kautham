

//mt library
#include <mt/mt.h>
#include <server.h>

//haptic library
#include <haptic/haptic.h>

#if defined(WIN32)
  #define _WINSOCKAPI_
  #include <windows.h>
#endif

using namespace ioc_comm;
using namespace mt;
haptic::Haptic* myHaptic;
mt::Transform myPos;
mt::Vector3 pos;
mt::Vector3 rot;
Vect6 _force(6);
//boost::thread* hapThread;
boost::asio::deadline_timer* serverTimer;
ioc_comm::Server* myServ;
unsigned int sendPer;

double fMax, tMax;

//// Create the data to be sent to each client.
    ioc_comm::vecData serverData;

//! Create vector to put the clients data
    std::vector<ioc_comm::clientData> theClients;

	
void readnUpdate(){
	try{
	  double *we = NULL;
	  std::vector<double>* values = NULL;

    //  Reading haptic device and updating the sendingData
    //  local structure 
	  myHaptic->getPosition(myPos);
	  pos=myPos.getTranslation();

	  myPos.getRotation().getYpr(rot.at(0), rot.at(1), rot.at(2));

	  ioc_comm::baseData& dataSer_pt = serverData.at(0);
	  dataSer_pt.time_stamp.assign(ioc_comm::cal_time_stamp());
	  dataSer_pt._data.at(0) = pos[0];
	  dataSer_pt._data.at(1) = pos[1];
	  dataSer_pt._data.at(2) = pos[2];
	  dataSer_pt._data.at(3) = rot[0];
	  dataSer_pt._data.at(4) = rot[1];
	  dataSer_pt._data.at(5) = rot[2];
	  
	  if(myHaptic->getButtom())
  		dataSer_pt._data.at(6) = 1.0;
	  else
  		dataSer_pt._data.at(6) = 0.0;

      serverData.at(0) = dataSer_pt;
      myServ->setSendingData(serverData);

	  // Reading clients information in order to calculate the 
	  // total force to be applied to the haptic device. This is
	  // the summatory of weights * force for each client.

	  for(int i = 0; i < 6; i++)
			_force[i] = 0.0;

	  myServ->getClientsData(theClients);

	  if(theClients.size() >0){
		  std::vector<ioc_comm::clientData>::iterator _aClient;

		  //! summation forces of the clients
		  for(_aClient=theClients.begin(); _aClient != theClients.end(); ++_aClient){
			  // _force = weigh * value for each client.
			  we = &((*_aClient).setts._data.at(1));
			  values = &(((ioc_comm::baseData&)(*_aClient).data.at(0))._data);
			  for(int i = 0; i < 6; i++)
			    _force[i] += (*we) * values->at(i);
		  }  

		  for(unsigned int i = 0; i < 3; i++){
			if(_force[i] > fMax ) _force[i] = fMax ;
			if(_force[i] < -fMax ) _force[i] = -fMax ;
		  }
		  for(unsigned int i = 3; i < 6; i++){
			if(_force[i] > tMax ) _force[i] = tMax ;
			if(_force[i] < -tMax ) _force[i] = -tMax ;
		  }
	  }
	  
	  myHaptic->setForce(_force);

	 // std::stringstream sstring;
	 // sstring.precision(5);

	 // for(int i = 0; i < 6; i++)
		//sstring << _force[i] << "\t" ;

	  //std::cout << sstring.str() << std::endl;
	  //std::cout.flush();
		
  }catch(std::out_of_range e){
    std::cerr << "out of range\n"; //e.message();
  }catch(...){
		myServ->close();
  //  hapThread->interrupt();
		//hapThread->join();
		myHaptic->stop();
		exit(1);
	}
}

int main(int argc, char *argv[]){

try{
    // Check command line arguments.
    if (argc != 5){
      std::cerr << "Usage: haptic_server <port> <server_period> <Fmax> <Tmax>" << std::endl;
      std::cerr << "time period in milliseconds \nFmax equal in all direcions" << std::endl;
      std::cerr << "\n Tmax equal in all axis " << std::endl;
      return 1;
    }
	
    unsigned short port = boost::lexical_cast<unsigned short>(argv[1]);
    sendPer = boost::lexical_cast<unsigned int>(argv[2]);
    fMax = boost::lexical_cast<double>(argv[3]);
    tMax = boost::lexical_cast<double>(argv[4]);

	  _force[0]=0;
	  _force[1]=0;
	  _force[2]=0;
	  _force[3]=0;
	  _force[4]=0;
	  _force[5]=0;


	
    char res = 'n';
  	
    std::cout << "Are you ready to start the haptic device? (y/n)" << std::endl;
    std::cin >> res;

    if( res != 'y' ) return 1;

	bool init=false;

	myHaptic = new haptic::Haptic(init);

	if(init){
		myHaptic->calibrate();
		myHaptic->start();
		
		//// Create the data to be sent to each client.
	    

		ioc_comm::cartesian::position apos;

		// Creating an initializating the server object
		//boost::asio::io_service io_service;
    myServ = new ioc_comm::Server(port, sendPer, ioc_comm::HAPTIC);

    // Now make it runs.
    //hapThread = new boost::thread(boost::bind(&boost::asio::io_service::run, &io_service));
    myServ->start();
		
		myHaptic->getPosition(myPos);
		pos=myPos.getTranslation();

		myPos.getRotation().getYpr(rot.at(0), rot.at(1), rot.at(2));

		apos.time_stamp = ioc_comm::cal_time_stamp();
		apos._data.push_back(pos[0]);
		apos._data.push_back(pos[1]);
		apos._data.push_back(pos[2]);
		apos._data.push_back(rot[0]);
		apos._data.push_back(rot[1]);
		apos._data.push_back(rot[2]);
		if(myHaptic->getButtom())
		  apos._data.push_back(1.0);
		else
		  apos._data.push_back(0.0);

		serverData.push_back(apos);

		myServ->setSendingData(serverData);

		// Now the infinite cycle
		int i = 0;
		while(true){
			readnUpdate();
			i++;
			if(i%100 == 0 ){
				for(int j = 0; j < 6; j++)
					std::cout << _force[j] << " " ;
				std::cout << std::endl;
				i = 0;
			}
#if defined(WIN32)
			  Sleep(sendPer);
#else
			  usleep(sendPer*1000);
#endif
			

		  }
		  return 0;
	  }else{
		  std::cout << "Something is wrong with the haptic device.\n" <<  
			  "The haptic device is not ready to be used." << std::endl;
	  }

  }catch (std::exception& e) {
	  std::cerr << e.what() << std::endl;
  }
  return 1;
}
