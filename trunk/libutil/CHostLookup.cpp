
/*
  * HostLookup.cpp
  *
  * Desc.: Implements the CHostLookup class
  * Start: 17.09.2003
  * Autor: Dyle (aka Oliver Maurhart)
  * Tabw.: 3
  */


// ------------------------------------------------------------
// incs


// we need the gethostname method here
#ifdef WIN32

#include <winsock2.h>
#define GETHOSTXXXX_DEFINED



// check we have gethostname
#ifndef GETHOSTXXXX_DEFINED
#error CHostLookup needs the gethostname and gethostbyname for the OS 
specified!
#endif

// qtnet stuff
#include "CHostLookup.h"


// ------------------------------------------------------------
// vars

/**
  * the IP-addresses found
  */
QHostAddress CHostLookup::m_cAddress;


/**
  * looked up name
  */
QString CHostLookup::m_sName;


/**
  * Already resolved flag
  */
bool CHostLookup::m_bResolved = false;


// ------------------------------------------------------------
// code


/**
  * returns the IP-addresses of the current machine
  *
  * @return:	a QHostAddress object holding the found IP-Address
  */
const QHostAddress& CHostLookup::getHostAddress() {
	if (!m_bResolved) resolve();
	return m_cAddress;
}


/**
  * returns the Name of the current machine
  *
  * @return:	the name of the current machine
  */
const QString& CHostLookup::getName() {
	if (!m_bResolved) resolve();
	return m_sName;
}


/**
  * performs the lookup
  */
void CHostLookup::resolve() {
	// we 've got the startup the windows sockets prior to any socket call
	WSAData cData;
	WSAStartup(MAKEWORD(2, 2), &cData);

	char sHostName[1024];
	if (gethostname(sHostName, sizeof(sHostName)) < 0) {
		qWarning("Warning: can't resolve own host name!");
		return;
	}

	m_sName = sHostName;

	// figure out IP
	HOSTENT* cHostEnt = gethostbyname(sHostName);
	if (cHostEnt) {

		char* cIP = cHostEnt->h_addr_list[0];
		unsigned int nIPv4 = 0;
		unsigned char* nIPv6 = NULL;

		// detect size
		switch (cHostEnt->h_addrtype) {

		case AF_INET:

			nIPv4 = (cIP[0] & 0xFF) << 24;
			nIPv4 |= (cIP[1] & 0xFF) << 16;
			nIPv4 |= (cIP[2] & 0xFF) << 8;
			nIPv4 |= (cIP[3] & 0xFF);

			m_cAddress = QHostAddress(nIPv4);

			break;

		case AF_INET6:

			// TODO: to be tested (don't know if it works on IPv6 as well)
			m_cAddress = QHostAddress(QString(cHostEnt->h_addr_list[0]));
			break;

		}
	}



	m_bResolved = true;
}


#endif  //WIN32

