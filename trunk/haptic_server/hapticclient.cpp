
#include <client.h>
#include <boost/asio.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
	
boost::asio::deadline_timer* t;
ioc_comm::Client* _client;
void print(const boost::system::error_code& /*e*/){
      ioc_comm::vecData serverData;
			std::stringstream sstream;
      sstream.precision(3);
		  _client->getServerData(serverData);
		  if(serverData.size() > 0 ){
			  ioc_comm::baseData& tmp = serverData[0];
			  sstream << tmp.time_stamp << "\t";
			  for(unsigned int i = 0; i < tmp._data.size(); i++)
				  sstream << tmp._data[i] << "\t";

			  std::cout << sstream.str() << std::endl;
			  std::cout.flush();
        sstream.clear();
      }
      t->expires_at(t->expires_at() + boost::posix_time::seconds(2));

      t->async_wait(print);
}


int main(int argc, char *argv[]){

try{
    // Check command line arguments.
    if (argc != 3){
      std::cerr << "Usage: haptic_client <server> <port>" << std::endl;
      return 1;
    }

    boost::asio::io_service io;

    t = new boost::asio::deadline_timer(io, boost::posix_time::seconds(2));
    t->async_wait(print);

    _client = new ioc_comm::Client(argv[1], argv[2], ioc_comm::HAPTIC, 1.0, 6);
    _client->start();

    ioc_comm::vecData sendingData;

    ioc_comm::cartesian::force force;

    // Set initial values to the force to be send
    force.time_stamp.assign(ioc_comm::cal_time_stamp());
    force._data.at(0) = 2.0;  force._data.at(1) = 2.0;
    force._data.at(2) = 2.0;  force._data.at(3) = 0.0;
    force._data.at(4) = 0.0;  force._data.at(5) = 0.0;

    sendingData.push_back(force);

    _client->setSendingData(sendingData);

    boost::thread th(boost::bind(&boost::asio::io_service::run, &io));

    for(int i = 0; i < 10; i++){
			  std::cout << i ;

  #if defined(WIN32)
		  Sleep(2000);
  #else
		  usleep(2000*1000);
  #endif
  //		
	  }
  	
	  _client->close();
    th.interrupt();
    t->cancel();
    th.join();
    delete t;
	  return 0;

  }catch (std::exception& e) {
    std::cerr << e.what() << std::endl;

  }
  return 1;
}
