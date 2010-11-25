
#ifndef IOC_COMM_CLIENT_H
#define IOC_COMM_CLIENT_H

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <iostream>
#include <vector>
#include "connection.hpp" // Must come before boost/serialization headers.
#include <boost/serialization/vector.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include "exchanged_data.hpp"

namespace ioc_comm {

/// Downloads data from a server.
class Client
{
public:
  /// Constructor starts the asynchronous connect operation.
  Client(const std::string& host, const std::string& service,
      DEVICE dev, double weight=1.0, int dim=0);
  //Client(boost::asio::io_service& io_service,
  //    const std::string& host, const std::string& service,
  //    DEVICE dev, double weight=1.0, int dim=0);

  void start();

  //! This method close the connection with the server.
  void close();

  //! share the boost::io_service associated.
  boost::asio::io_service& getService(){return _service;}

  //! This method sets the vector to should be send to the server.
  void setSendingData(vecData& data);

  bool setSendingData(unsigned int i, baseData& data);

  //! Returns a particular baseData from the data vector received
  //! from the server
  bool getServerData(unsigned int i, baseData& data);

  //! Returns the data vector received from the server
  void getServerData(vecData& data);

protected:
  boost::thread* t;
    //! io_service reference
  boost::asio::io_service _service;

  /// Handle completion of a connect operation.
  void handle_connect(const boost::system::error_code& e,
      boost::asio::ip::tcp::resolver::iterator endpoint_iterator);


  /// Handle completion of a read operation.
  void handle_read(const boost::system::error_code& e);

  /// Handle completion of a write operation.
  void handle_write(const boost::system::error_code& e);


  //! receiving data mutual exclusion access
  boost::mutex in_mutex;

  //! sending data mutual exclusion access
  boost::mutex out_mutex;

  //! The client communication settings
  ioc_comm::settings comm_Settings;

  //! The server communication settings
  ioc_comm::settings server_Settings;

  //! The data should sending to the server
  vecData sendingData;

  //! The data could be received from the server.
  vecData receivingData;

  //! Server data
  vecData serverData;

  //! The connection to the server.
  connection connection_;

};

} // namespace ioc_comm

#endif //IOC_COMM_CLIENT_H