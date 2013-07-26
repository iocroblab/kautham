/***************************************************************************
 *   Copyright (C) 2007 by Emmanuel Nu�o                                   *
 *   emmanuel.nuno@upc.edu                                                 *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

#ifndef _H_HAPTIC
#define _H_HAPTIC

#define WIN32_LEAN_AND_MEAN


#ifdef WIN32
#include <conio.h>
#endif

//mt library
#include <mt/mt.h>

//HD headers from the Open Haptics
#include <HD/hd.h>
#include <HDU/hduVector.h>

#include <iostream>


//uBLAS library
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/vector.hpp>

namespace ublas= boost::numeric::ublas;

// typedef for the Six component Vector
typedef ublas::vector<mt::Scalar> Vect6;


namespace haptic
{


//Shared struct for the haptic threads
struct HapticState{
	HapticState(): phForce(0,0,0),phTorqe(0,0,0){};

	HDint phButton;
	HDdouble phPos[16];
	hduVector3Dd phVel;
	HDdouble phAngVel[3];
	hduVector3Dd phForce;
	hduVector3Dd phTorqe;
	HDdouble phBaseJoints[3];
	HDdouble phGimbalJoints[3];
	HDlong phMotorTorque[6];

};


class Haptic
{
public:
	Haptic(bool&);

public:
	bool calibrate();
	bool start();
	bool stop();
	bool getPositionTX(mt::Transform &);
	bool getPosition(mt::Transform &);
	bool getVelocity(Vect6 &);
	bool setForce(const Vect6);
	bool getButtom();
	bool getJointPosition(Vect6 &);
	bool setMotorTorque(const Vect6);
	bool getJacobian(ublas::matrix<mt::Scalar> &);
	bool getJacobianTranspose(ublas::matrix<mt::Scalar> &);
	

public:
	HHD m_hHD;
	HDErrorInfo m_error;
	HapticState *m_phState;
	bool m_init;
	};

HDCallbackCode HDCALLBACK hdState(void *);
}//namespace 

#endif