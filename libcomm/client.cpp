

#include "client.h"

namespace ioc_comm {

  /// Constructor starts the asynchronous connect operation.
  //Client::Client(boost::asio::io_service& io_service,
  //    const std::string& host, const std::string& service,
  //    DEVICE dev, double weight, int dim)
  //  : _service(io_service), connection_(io_service)
  //{
  Client::Client(const std::string& host, const std::string& service,
      DEVICE dev, double weight, int dim)
    : connection_(_service)
  {
    //  Configure the setting package to will be send to the server
    comm_Settings._data.push_back((double)dev); //Device to be looking for.
    comm_Settings._data.push_back(weight);      //Weight of component
    comm_Settings._data.push_back((double)dim); // Dimension of articular if needed.

    // Resolve the host name into an IP address.
    boost::asio::ip::tcp::resolver resolver(_service);
    boost::asio::ip::tcp::resolver::query query(host, service);
    boost::asio::ip::tcp::resolver::iterator endpoint_iterator =
      resolver.resolve(query);
    boost::asio::ip::tcp::endpoint endpoint = *endpoint_iterator;

    // Start an asynchronous connect operation.
    connection_.socket().async_connect(endpoint,
        boost::bind(&Client::handle_connect, this,
        boost::asio::placeholders::error, ++endpoint_iterator));
  }

  void Client::start(){
   t = new boost::thread(boost::bind(&boost::asio::io_service::run, &_service));
  }

  void Client::close(){
    try{
      connection_.close();
      t->interrupt();
      _service.stop(); 
      t->join();
    }catch(...){
      t->interrupt();
      t->join();
    }
  }

  //! This method sets the vector to should be send to the server.
  void Client::setSendingData(vecData& data){
      boost::mutex::scoped_lock lk1(out_mutex, boost::adopt_lock);
      sendingData = data;
  }

  bool Client::setSendingData(unsigned int i, baseData& data){
      boost::mutex::scoped_lock lk1(out_mutex, boost::adopt_lock);
      if(i < sendingData.size()){
          sendingData.at(i) = data;
          return true;
      }
      return false;
  }

  //! Returns a particular baseData from the data vector received
  //! from the server
  bool Client::getServerData(unsigned int i, baseData& data){
      boost::mutex::scoped_lock lk1(in_mutex, boost::adopt_lock);
      if(i < serverData.size() ){
        data = serverData.at(i);
        return true;
      }
      return false;
  }

  //! Returns the data vector received from the server
  void Client::getServerData(vecData& data){
      boost::mutex::scoped_lock lk1(in_mutex, boost::adopt_lock);

      data = serverData;
  }

  /// Handle completion of a connect operation.
  void Client::handle_connect(const boost::system::error_code& e,
      boost::asio::ip::tcp::resolver::iterator endpoint_iterator)
  {
        if (!e){
          // Successfully established connection. First the settings is sending to server.
          vecData tmp;
          tmp.push_back(comm_Settings);
          connection_.async_write(tmp,
                  boost::bind(&Client::handle_write, this,
                  boost::asio::placeholders::error));

          // Second the Client is waiting for incomming data.
          connection_.async_read(boost::bind(&Client::handle_read, this,
              boost::asio::placeholders::error));

        }else if (endpoint_iterator != boost::asio::ip::tcp::resolver::iterator()){

          // Try the next endpoint.
          connection_.socket().close();
          boost::asio::ip::tcp::endpoint endpoint = *endpoint_iterator;
          connection_.socket().async_connect(endpoint,
              boost::bind(&Client::handle_connect, this,
                boost::asio::placeholders::error, ++endpoint_iterator));
        
        }else{

          // An error occurred. Log it and return. 
          std::cerr << e.message() << std::endl;
        }
    
  }


  /// Handle completion of a read operation.
  void Client::handle_read(const boost::system::error_code& e)
  {
    if (!e){
      connection_.getReceivedData(receivingData);
//      std::cout << "Id number " << receivingData.at(0).id << std::endl;
//      std::cout << "time: " << receivingData.at(0).time_stamp << std::endl;

      boost::mutex::scoped_lock lk1(in_mutex,boost::adopt_lock);

      switch(receivingData.at(0).id){
          case SETTING:
            // This package configures the connection;
            server_Settings.time_stamp = ((baseData)receivingData.at(0)).time_stamp;
            server_Settings._data.push_back(((baseData)receivingData.at(0))._data.at(0));
            server_Settings._data.push_back(((baseData)receivingData.at(0))._data.at(1));
            server_Settings._data.push_back(((baseData)receivingData.at(0))._data.at(2));
            receivingData.clear();
            break;
          default:
            // This package is a data target of the comunication.
            serverData = receivingData;
            receivingData.clear();
            break;
      }

      boost::mutex::scoped_lock lk2(out_mutex,boost::adopt_lock);
      if(sendingData.size() != 0)
        connection_.async_write(sendingData,
            boost::bind(&Client::handle_write, this,
            boost::asio::placeholders::error));

      connection_.async_read(boost::bind(&Client::handle_read, this,
              boost::asio::placeholders::error));
    } else {
      // An error occurred.
      std::cerr << e.message() << std::endl;
      close();
    }
  }

  /// Handle completion of a write operation.
  void Client::handle_write(const boost::system::error_code& e)  {
    if (!e){
//      std::cout << "Writing data correctly on connection to server: " << std::endl;
    }else{
      ;
    }
  }

} // namespace ioc_comm

//int main(int argc, char* argv[])
//{
//  try{
//    // Check command line arguments.
//    if (argc != 3){
//      std::cerr << "Usage: Client <host> <port>" << std::endl;
//      return 1;
//    }
//
//    boost::asio::io_service io_service;
//    ioc_comm::Client Client(io_service, argv[1], argv[2], ioc_comm::HAPTIC, 1.0, 6);
//
//    boost::thread t(boost::bind(&boost::asio::io_service::run, &io_service));
//
//    while(true){
//        ;
//    }
//
//    Client.close();
//    t.join();
//
//  }
//  catch (std::exception& e){
//
//    std::cerr << e.what() << std::endl;
//  }
//
//  return 0;
//}
