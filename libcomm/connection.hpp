

#ifndef IOC_COMM_CONNECTION_HPP
#define IOC_COMM_CONNECTION_HPP

#include <boost/asio.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/tuple/tuple.hpp>
#include <iomanip>
#include <string>
#include <sstream>
#include <vector>
#include <boost/enable_shared_from_this.hpp>
#include "exchanged_data.hpp"


namespace ioc_comm {

//! The connection class provides serialization primitives on top of a socket.
/**
 * Each message sent using this class consists of:
 * @li An 8-byte header containing the length of the serialized data in
 * hexadecimal.
 * @li The serialized data.
 * 
 * This is a template class and it is able to drive correctly your serialized data.
 * The connection can be instantiated with the data type to be exchanged between
 * server and clients. 
 */


//template <typename D>
//class connection
//  : public boost::enable_shared_from_this<connection<D>>

class connection
  : public boost::enable_shared_from_this<connection>{
public:

  /// Constructor.
  connection(boost::asio::io_service& io_service)
    : socket_(io_service) {

  }

  void close(){
      socket_.close();
  }

  /// Get the underlying socket. Used for making a connection or for accepting
  /// an incoming connection.
  boost::asio::ip::tcp::socket& socket()
  {
    return socket_;
  }

  /// Asynchronously write a data structure to the socket.
  template <typename T, typename Handler>
  void async_write(const T& t, Handler handler)
  {
    //T _localCopy = t;
    // Serialize the data first so we know how large it is.
    std::ostringstream archive_stream;
    boost::archive::text_oarchive archive(archive_stream);
    archive << t;
    outbound_data_ = archive_stream.str();

    // Format the header.
    std::ostringstream header_stream;
    header_stream << std::setw(header_length)
      << std::hex << outbound_data_.size();
    if (!header_stream || header_stream.str().size() != header_length)
    {
      // Something went wrong, inform the caller.
      boost::system::error_code error(boost::asio::error::invalid_argument);
      socket_.io_service().post(boost::bind(handler, error));
      return;

    }

    outbound_header_ = header_stream.str();

    // Write the serialized data to the socket. We use "gather-write" to send
    // both the header and the data in a single write operation.
    std::vector<boost::asio::const_buffer> buffers;
    buffers.push_back(boost::asio::buffer(outbound_header_));
    buffers.push_back(boost::asio::buffer(outbound_data_));
    boost::asio::async_write(socket_, buffers, handler);

  }

  //! Read the last received data from the connection.
  //template <typename D>
  void getReceivedData(vecData& data){
    data = _receivedData;
  }

  /// Asynchronously read a data structure from the socket.
  //template <typename T, typename Handler>
    template < typename Handler>
  //void async_read(T& t, Handler handler)
  void async_read( Handler handler)
  {
    // Issue a read operation to read exactly the number of bytes in a header.
    void (connection::*f)(
        const boost::system::error_code&,
        vecData&, boost::tuple<Handler>)
      = &connection::handle_read_header<vecData, Handler>;
    boost::asio::async_read(socket_, boost::asio::buffer(inbound_header_),
        boost::bind(f, this, boost::asio::placeholders::error, boost::ref(_receivedData),
          boost::make_tuple(handler)));
  }

  /// Handle a completed read of a message header. The handler is passed using
  /// a tuple since boost::bind seems to have trouble binding a function object
  /// created using boost::bind as a parameter.
  template <typename T, typename Handler>
  void handle_read_header(const boost::system::error_code& e,
      T& t, boost::tuple<Handler> handler)
  {
    if (e)
    {
      boost::get<0>(handler)(e);
    }
    else
    {
      // Determine the length of the serialized data.
      std::istringstream is(std::string(inbound_header_, header_length));
      std::size_t inbound_data_size = 0;
      if (!(is >> std::hex >> inbound_data_size))
      {
        // Header doesn't seem to be valid. Inform the caller.
        boost::system::error_code error(boost::asio::error::invalid_argument);
        boost::get<0>(handler)(error);
        return;
      }

      // Start an asynchronous call to receive the data.
      inbound_data_.resize(inbound_data_size);
      void (connection::*f)(
          const boost::system::error_code&,
          T&, boost::tuple<Handler>)
        = &connection::handle_read_data<T, Handler>;
      boost::asio::async_read(socket_, boost::asio::buffer(inbound_data_),
        boost::bind(f, this,
          boost::asio::placeholders::error, boost::ref(t), handler));
    }
  }

  /// Handle a completed read of message data.
  template <typename T, typename Handler>
  void handle_read_data(const boost::system::error_code& e,
      T& t, boost::tuple<Handler> handler)
  {
    if (e)
    {
      boost::get<0>(handler)(e);
    }
    else
    {
      // Extract the data structure from the data just received.
      try
      {
        std::string archive_data(&inbound_data_[0], inbound_data_.size());
        std::istringstream archive_stream(archive_data);
        boost::archive::text_iarchive archive(archive_stream);
        archive >> t;
      }
      catch (std::exception& )
      {
        // Unable to decode data.
        boost::system::error_code error(boost::asio::error::invalid_argument);
        boost::get<0>(handler)(error);
        return;
      }

      // Inform caller that data has been received ok.
      boost::get<0>(handler)(e);
    }
  }

private:

  vecData _receivedData;
  /// The underlying socket.
  boost::asio::ip::tcp::socket socket_;

  /// The size of a fixed length header.
  enum { header_length = 8 };

  /// Holds an outbound header.
  std::string outbound_header_;

  /// Holds the outbound data.
  std::string outbound_data_;

  /// Holds an inbound header.
  char inbound_header_[header_length];

  /// Holds the inbound data.
  std::vector<char> inbound_data_;
};

  typedef boost::shared_ptr<connection> connection_ptr;
} // namespace ioc_comm

#endif // IOC_COMM_CONNECTION_HPP
