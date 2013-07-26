// Implementation of the connection class.


#include "connection.hpp"
#include <string>
#include <errno.h>
#include <fcntl.h>


using namespace std;

Connection::Connection()
{
}

Connection::~Connection()
{
  close ();
}


bool Connection::send ( const std::string msg ) const
{
  size_t size_msg = msg.size();
 
  int numbytes = ::send(sockfd, msg.c_str(), size_msg, 0);
 
  if(numbytes == size_msg)
  {
      std::cout << "Send " << numbytes << " bytes" << std::endl;
      return true;
  }
  else
  {
      std::cerr << "We cannot send " << numbytes << " bytes" << std::endl;
      return false;
  }
}


int Connection::recv ( std::string& s ) const
{
  size_t numbytes;
 
  char buf [ MAXRECV + 1 ];
  memset ( buf, 0, MAXRECV + 1 );
  s = "";
  if ((numbytes = ::recv(sockfd, buf, MAXDATASIZE-1, 0)) == -1)
  {
    std::cout << "status == -1 errno == " << errno << " in Connection::recv\n";
    return 0;
  }
  else
  {
    s = buf;
    return numbytes;
  }
}



bool Connection::connect_to ( const std::string host, const char *port)
{
    memset(&hints, 0, sizeof hints);
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
//    hints.sin_port = htons ( port );
 
    if ((rv = getaddrinfo(host.c_str(), port, &hints, &servinfo)) != 0) {
        fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(rv));
        return 1;
    }
  // loop through all the results and connect to the first we can
   
    for(p = servinfo; p != NULL; p = p->ai_next) {
        if ((sockfd = socket(p->ai_family, p->ai_socktype,
                p->ai_protocol)) == -1) {
            perror("client: socket");
            continue;
        }

        if (connect(sockfd, p->ai_addr, p->ai_addrlen) == -1) {
            ::close(sockfd);
            perror("client: connect");
            continue;
        }

        break;
    }

    if (p == NULL) {
        fprintf(stderr, "client: failed to connect\n");
        return 2;
    }

    inet_ntop(p->ai_family, get_in_addr((struct sockaddr *)p->ai_addr),
            s, sizeof s);
    printf("client: connecting to %s\n", s);
    hostinfo = (struct sockaddr_in*)servinfo->ai_addr;
    freeaddrinfo(servinfo); // all done with this structure
    return true;
}

void* Connection::get_in_addr(struct sockaddr *sa)
{
    if (sa->sa_family == AF_INET) {
        return &(((struct sockaddr_in*)sa)->sin_addr);
    }

    return &(((struct sockaddr_in6*)sa)->sin6_addr);
}

void Connection::close()
{
  ::close(sockfd);
}

//udp client to
bool Connection::udp_connect_to ( const std::string host, const char *port)
{
    memset(&hints, 0, sizeof hints);
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_DGRAM;
//    hints.sin_port = htons ( port );
 
    if ((rv = getaddrinfo(host.c_str(), port, &hints, &servinfo)) != 0) {
        fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(rv));
        return 1;
    }
  // loop through all the results and connect to the first we can
   
    for(p = servinfo; p != NULL; p = p->ai_next) {
        if ((sockfd = socket(p->ai_family, p->ai_socktype,
                p->ai_protocol)) == -1) {
            perror("client: socket");
            continue;
        }

        break;
    }
 
    if (p == NULL) {
        fprintf(stderr, "talker: failed to bind socket\n");
        return 2;
    }
   
    freeaddrinfo(servinfo); // all done with this structure
    return true;
}

//////////////////////////////////////////////////////////////

bool Connection::sendto ( const std::string msg ) const
{
  size_t size_msg = msg.size();
 
  int numbytes = ::sendto(sockfd, msg.c_str() , size_msg, 0,
             p->ai_addr, p->ai_addrlen);
 
  if(numbytes == size_msg)
  {
      std::cout << "Send " << numbytes << " bytes" << std::endl;
      return true;
  }
  else
  {
      std::cerr << "We cannot send " << numbytes << " bytes" << std::endl;
      return false;
  }
}
  

