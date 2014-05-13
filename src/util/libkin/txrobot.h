/*************************************************************************\
   Copyright 2014 Institute of Industrial and Control Engineering (IOC)
                 Universitat Politecnica de Catalunya
                 BarcelonaTech
    All Rights Reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the
    Free Software Foundation, Inc.,
    59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 \*************************************************************************/

/* Author: Leopold Palomo, Ignacio Flores, Adolfo Rodriguez and Emmanuel Nuño               */

/////////////////////////////// PREPROCESSOR DIRECTIVES //////////////////////

// HEADER GUARD
#ifndef TXROBOT_H
#define TXROBOT_H

// MT LIBRARY HEADERS
// This library needs the mtlib
#include <mt/scalar.h>
#include <mt/transform.h>
#include <mt/interval.h>

//uBLAS library
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/vector.hpp>

namespace ublas= boost::numeric::ublas;

// typedef for the Six component Vector
typedef ublas::vector<mt::Scalar> Vect6;

/////////////////////////////// NAMESPACE SCOPE //////////////////////////////

/** \addtogroup libKin
 *  @{
 */
namespace TXrobot
{

/////////////////////////////// CLASS DEFINITION /////////////////////////////

/// \brief Stäubli TX series kinematics class.
///
///
///
enum shoulder{srighty=0, slefty, sfree};
enum elbow{epositive=0, enegative, efree};
enum wrist{wpositive=0, wnegative , wfree};

enum TXtype {TX40, 
            TX60,
            TX60L,
            TX90,
            TX90L,
            TX90XL
};

enum TXerror {SUCCESS, 
              ERROR_J1,
              ERROR_J2,
              ERROR_J3,
              ERROR_J4,
              ERROR_J5,
              ERROR_J6,
              ERROR_UNDEFINED
};

struct config
{
  config() : sh(slefty), el(epositive), wr(wpositive){}
  bool matchMask(config mask){
    if((mask.sh == sfree || mask.sh == sh) &&
       (mask.el == efree || mask.el == el) &&
       (mask.wr == wfree || mask.wr == wr))
      return true;
    else
      return false;
  }

  void set(shoulder s, elbow e, wrist w){
    sh = s; el = e; wr = w;
  }

	shoulder sh;
	elbow el;
	wrist wr;
};

struct limits
{
    mt::Interval range; //in radians
    mt::Scalar speed; //in radians/s
};

class TXRobot
{
    //Staübli mechanical parameters
    mt::Scalar a1;
    mt::Scalar a2;
    mt::Scalar a3;
    mt::Scalar a4;
    mt::Scalar a5;
    mt::Scalar a6;   //Distance from wrist center to tool

   //Staübli mechanical parameter for 'Jacobian'
    mt::Scalar aJ5;

    mt::Scalar TXtolerance;

public:
    TXRobot(TXtype &robot);
    TXRobot();

    limits TXlimits[6];
    config TXconfig;

/// Computes forward kinematics from joint angular values
/// (expressed in radians), Vect6 and put the result in.
/// Pos. The function returns an error code
    TXerror fwdKin(const Vect6& j, mt::Transform &Pos);

/// Computes inverse kinematics from Cartesian pos values
/// (expressed in milimeters) and a configuration. The result goes to 
/// Vect6 (radians). The function returns an error code.
    TXerror invKin(const mt::Transform& p, Vect6&, const config& conf = TXrobot::config(), Vect6 qant=ublas::zero_vector<mt::Scalar>(6));


/// Computes inverse kinematics from Cartesian pos values
/// (expressed in milimeters), the current configuration of the robot and the last
/// position of a path (Vect6 (radians)
/// The result goes to Vect6 (radians). The function returns an error code.
    TXerror invKin(const mt::Transform& p,  Vect6& , const Vect6&, const config& conf = TXrobot::config());

/// Computes inverse kinematics from Cartesian pos values
/// (expressed in milimeters), and after crash your program
    TXerror invKin(const mt::Transform& p,  Vect6& , int foo, const config& conf = TXrobot::config());

//! This method computes the inverse kinematic with the preferred configuration but if it does not
//! provide a solution then it uses the solution configuration with almost one free member to looking for
//! all possible configurations. The function fills q with the successfully articular values closest one to
//! the qNear values and the
//! respective configuration in the solution configuration. This method returns an error code if none of the
//! possible configurations will return a feasible pose.
    TXerror invKin(const mt::Transform& p,  Vect6& q , const config& preferred, config& solution, const Vect6& qNear);

//Coments on getJacobian()

    TXerror getJacobian(const Vect6& , ublas::matrix<mt::Scalar> &) const;

//Function to place a new angle as close as possible with the previous
    TXerror setClosestAngle(mt::Scalar &, const mt::Scalar &);

};
/////////////////////////////// HELPER FUNCTIONS /////////////////////////////

// OPERATORS
std::ostream& operator<<(std::ostream&  os, const config& c);


/////////////////////////////// INLINE HELPER FUNCTIONS /////////////////////////
inline std::ostream& operator<<(std::ostream& os, const config& c)
{
    (c.sh==srighty) ? os<<"righty ":os<<"lefty ";
    (c.el==epositive) ? os<<"epositive ":os<<"enegative ";
    (c.wr==wpositive) ? os<<"wpositive ":os<<"wnegative ";

    return os;
}



} // TXrobot
/** @}   end of Doxygen module "Util */
#endif // TXROBOT_H
