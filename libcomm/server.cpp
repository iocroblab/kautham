

#include "server.h"
#include <boost/thread/locks.hpp>

namespace ioc_comm {
  using namespace boost::asio;
  /// Constructor opens the acceptor and starts waiting for the first incoming
  /// connection.
  Server::Server(unsigned short port, int mlsec, DEVICE dev, int conAva, int dim)
                :_acceptor(_service, ip::tcp::endpoint(ip::tcp::v4(), port))
  {
    msPeriod = mlsec;

    // Server communication settings
    comm_Settings._data.push_back((double)dev); //Device served.
    comm_Settings._data.push_back((double)conAva);      //Available connections
    comm_Settings._data.push_back((double)dim); // Dimension of articular if needed.

    // Start an accept operation for a new connection.
    connection_ptr new_conn(new connection(_acceptor.io_service()));
    _acceptor.async_accept(new_conn->socket(),
        boost::bind(&Server::handle_accept, this,
          boost::asio::placeholders::error, new_conn));

    _sendingTimer = new boost::asio::deadline_timer(_service, boost::posix_time::millisec(msPeriod));
  }

  void Server::start(){
    _thSer = new boost::thread(boost::bind(&boost::asio::io_service::run, &_service));
    _sendingTimer->async_wait(boost::bind(&Server::deliver, this, boost::asio::placeholders::error ));
  }

  //! This method closes all client connections.
  void Server::close(){
    boost::mutex::scoped_lock lk1(in_mutex,boost::adopt_lock);

    if(!_clients.empty()){
      std::map<connection_ptr, clientData>::iterator _aClient;
      for(_aClient=_clients.begin(); _aClient !=_clients.end(); ++_aClient){
        ((*_aClient).first)->close();
      }
    }
    _service.stop();
    _thSer->interrupt();
    _thSer->join();
  }

  //! This method sets the vector to should be send to each client.
  void Server::setSendingData(vecData& data){
    boost::mutex::scoped_lock lk1(out_mutex, boost::adopt_lock);
    sendingData = data;
  }

  //! This method is able to set a particular item on the sendingData
  //! vector.
  bool Server::setSendingData(unsigned int i, baseData& data){
    boost::mutex::scoped_lock lk1(out_mutex, boost::adopt_lock);
    if(i < sendingData.size()){
        sendingData.at(i) = data;
        return true;
    }
    return false;
  }

  //! Returns a particular baseData from the data vector received
  //! from the server
  //! Returns a vector with  received from the server
  void Server::getClientsData(std::vector<clientData>& clients){
      // This mutex blocks the access to shared map.
      boost::mutex::scoped_lock lk1(in_mutex, boost::adopt_lock);

      clients.clear();
      if(_clients.size() > 0 ){
        std::map<connection_ptr, clientData>::iterator _aClient;


        for(_aClient=_clients.begin(); _aClient !=_clients.end(); ++_aClient){
            if(_aClient->first->socket().is_open() && _aClient->second.data.size()>0)
              clients.push_back(_aClient->second);

        }

      }

  }

  /// Handle completion of a accept operation.
  void Server::handle_accept(const boost::system::error_code& e, connection_ptr conn)  {
    if (!e){
      // First the settings is sending to the client.
      vecData tmp;
      tmp.push_back(comm_Settings);
      conn->async_write(tmp,
        boost::bind(&Server::handle_write, this,
            boost::asio::placeholders::error, conn));

      // Second, init asincronous reading process.
      conn->async_read( boost::bind(&Server::handle_read, this,
              boost::asio::placeholders::error, conn));

      // Start an accept operation for a new connection.
      connection_ptr new_conn(new connection(_acceptor.io_service()));
      _acceptor.async_accept(new_conn->socket(),
            boost::bind(&Server::handle_accept, this,
            boost::asio::placeholders::error, new_conn));

      // Now adding the client connection data to map
      boost::mutex::scoped_lock lk1(in_mutex,boost::adopt_lock);

      clientData tmpClient;
      _clients.insert(std::pair<connection_ptr, clientData>(conn, tmpClient));

      // print ip address of a client
      std::cout << "Accepting connection from: " <<
              conn->socket().remote_endpoint().address().to_string()
              << std::endl;

    } else {
      // An error occurred. Log it and return. 
        std::cerr << "accept_server: " << e.message() << std::endl;
    }
  }

  /// Handle completion of a write operation.
  void Server::handle_write(const boost::system::error_code& e, connection_ptr conn)  {
    if (!e){
      //std::cout << "Active conections" << _clients.size() << std::endl;
      ;
    }else{
      // Blocking the inbound structure and erasing the closed connection.
      boost::mutex::scoped_lock lk1(in_mutex,boost::adopt_lock);
      std::cout << "Losing connection " << std::endl;

      _clients.erase(conn);
    }
  }

  /// Handle completion of a read operation.
  void Server::handle_read(const boost::system::error_code& e, connection_ptr conn)  {

    boost::mutex::scoped_lock lk1(in_mutex,boost::adopt_lock);

    conn->getReceivedData(receivingData);

    if (!e){
      if(receivingData.size() > 0 ){
        //std::cout << "Id number " << receivingData.at(0).id << std::endl;
        //std::cout << "time: " << receivingData.at(0).time_stamp << std::endl;
        switch(receivingData.at(0).id){
            case SETTING:
              //! This package configures the connection;
              if (_clients.find(conn) != _clients.end()){
                  // Now saving the client connection settings in the
                  // structure provided
                  (_clients[conn].setts).id = receivingData.at(0).id;
                  (_clients[conn].setts).time_stamp = receivingData.at(0).time_stamp;
                  (_clients[conn].setts)._data = receivingData.at(0)._data;
              }else{
                  std::cerr << "Connection not found into the map" << std::endl;
              }
              break;
            default:
              //! This package is a data targert of the comunication.
              //std::cout << "time: " << receivingData.at(0).time_stamp << std::endl;
              if (_clients.find(conn) != _clients.end()){
                  // Now saving the client data in the structure provided
                  _clients[conn].data = receivingData;
              }else{
                  std::cerr << "Connection not found into the map" << std::endl;
              }
              break;
        }
      }
      
      conn->async_read( boost::bind(&Server::handle_read, this,
          boost::asio::placeholders::error, conn));
    }else {
        // Blocking the inbound structure and erasing the closed connection.
        boost::mutex::scoped_lock lk1(in_mutex,boost::adopt_lock);
        std::cout << "Losing connection " << std::endl;

        _clients.erase(conn);

    }
     

  }


  //! This method sends the server data to all connected clients.
  void Server::deliver(const boost::system::error_code& e) {
    // First block the shared memory strucutures then send data.
    boost::lock(in_mutex, out_mutex);
    boost::mutex::scoped_lock lk1(in_mutex,boost::adopt_lock);
    boost::mutex::scoped_lock lk2(out_mutex,boost::adopt_lock);

    if(!_clients.empty()){
      std::map<connection_ptr, clientData>::iterator _aClient;
      for(_aClient=_clients.begin(); _aClient !=_clients.end(); ++_aClient){
        ((*_aClient).first)->async_write(sendingData,
            boost::bind(&Server::handle_write, this,
            boost::asio::placeholders::error, ((*_aClient).first)));
      }      
    }
    
    _sendingTimer->expires_from_now( boost::posix_time::millisec(msPeriod));
    _sendingTimer->async_wait(boost::bind(&Server::deliver, this,
                      boost::asio::placeholders::error ));
	//std::cout << "sending..." << std::endl;
  }


} // namespace ioc_comm




