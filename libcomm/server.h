
#ifndef IOC_COMM_SERVER_H
#define IOC_COMM_SERVER_H

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/lexical_cast.hpp>
#include <iostream>
#include <vector>
#include <set>
#include "connection.hpp" // Must come before boost/serialization headers.
#include <boost/serialization/vector.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>
#include "exchanged_data.hpp"

namespace ioc_comm {
  using namespace boost::asio;

struct clientData{
    settings setts;
    vecData  data;
};

/// Serves stock quote information to any client that connects to it.
class Server {
public:
  /// Constructor opens the acceptor and starts waiting for the first incoming
  /// connection.
  Server( unsigned short port, int mlsec, ioc_comm::DEVICE dev, 
          int conAva=4, int dim=0);

  //! This method starts the server.
  void start();

  //! This method closes all client connections and stop the server.
  void close();

  //! This method sets the vector to should be send to each client.
  void setSendingData(vecData& data);

  //! This method is able to set a particular item on the sendingData
  //! vector.
  bool setSendingData(unsigned int i, baseData& data);

  //! Returns a particular baseData from the data vector received
  //! from the server
  //! Returns a vector with  received from the server
  void getClientsData(std::vector<clientData>& clients);

protected:
  /// Handle completion of a accept operation.
  void handle_accept(const boost::system::error_code& e, connection_ptr conn);

  /// Handle completion of a write operation.
  void handle_write(const boost::system::error_code& e, connection_ptr conn);

  /// Handle completion of a read operation.
  void handle_read(const boost::system::error_code& e, connection_ptr conn);


  //! This method sends the server data to all connected clients.
  void deliver(const boost::system::error_code& e);

private:
  boost::thread*                _thSer;
  //! io_service reference
  boost::asio::io_service       _service;

  //! receiving data mutual exclusion access
  mutable boost::mutex          in_mutex;

  //! sending data mutual exclusion access
  mutable boost::mutex          out_mutex;

  //! Timer to fire the delivering operation.
  boost::asio::deadline_timer*  _sendingTimer;

  //! Time between two delivering operations in milliseconds.
  int                           msPeriod;

  //! Data collected from a client.
  vecData                       receivingData;

  //! The data to should be sent to each connected client.
  //! Each position should have a different data (q, x, v, etc)
  vecData                       sendingData;

  //! Client connections
//  std::set<connection_ptr> _clients;
  //Attaching the information received from each client.
  std::map<connection_ptr, clientData> _clients;

  /// The acceptor object used to accept incoming socket connections.
  boost::asio::ip::tcp::acceptor _acceptor;

  // The communication settings
  ioc_comm::settings comm_Settings;
};

} // namespace ioc_comm

#endif // IOC_COMM_SERVER_H
