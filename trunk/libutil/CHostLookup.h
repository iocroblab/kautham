/*
  * CHostLookup.h
  *
  * Desc.: Declares a class which is capable of identifying our own 
IP-Address
  * Start: 17.09.2003
  * Autor: Dyle (aka Oliver Maurhart)
  * Tabw.: 3
  */

#ifndef __CHOSTLOOKUP_H
#define __CHOSTLOOKUP_H


// ------------------------------------------------------------
// incs

// Qt Stuff
#include <QtNetwork/QHostAddress>
#include <QString>


// ------------------------------------------------------------
// declares


/**
  * This class is capable to look up the own IP-Address
  */
class CHostLookup {


private:


	/**
	 * constructor
	 *
	 * Only static methods are present so it is not allowed to instantiate 
this class
	 */
	CHostLookup() {};


public:


	/**
	 * returns the IP-addresses of the current machine
	 *
	 * @return	a QHostAddress object holding the found IP-Address
	 */
	static const QHostAddress& getHostAddress();


	/**
	 * returns the Name of the current machine
	 *
	 * @return	the name of the current machine
	 */
	static const QString& getName();


protected:


	/**
	 * performs the lookup
	 */
	static void resolve();


private:


	/**
	 * the IP-address found
	 */
	static QHostAddress m_cAddress;

	
	/**
	 * looked up name
	 */
	static QString m_sName;


	/**
	 * Already resolved flag
	 */
	static bool m_bResolved;

};



#endif

