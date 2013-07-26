// Definition of the Socket class

#ifndef CONNECTION_H
#define CONNECTION_H


#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <unistd.h>
#include <string>
#include <cstdlib>
#include <cstring>
#include <cstdio>
#include <arpa/inet.h>
#include <iostream>

#define MAXDATASIZE 200

const int MAXHOSTNAME = 200;
const int MAXCONNECTIONS = 5;
const int MAXRECV = 500;


class Connection
{
 public:
  Connection();
  ~Connection();

  // Server initialization
 
// Client initialization
  bool connect_to ( const std::string host, const char* port );

  bool udp_connect_to ( const std::string host, const char* port );
 
  // Data Transimission
  bool send ( const std::string ) const;
  bool sendto ( const std::string ) const;
 
  int recv ( std::string& ) const;
  void close();
  int sockfd;
 private:
   
    int numbytes;
    char buf[MAXDATASIZE];
    struct addrinfo hints, *servinfo, *p;
    int rv;
    char s[INET6_ADDRSTRLEN];
    void* get_in_addr(struct sockaddr *sa);
    struct sockaddr_in *hostinfo;

};
#endif


