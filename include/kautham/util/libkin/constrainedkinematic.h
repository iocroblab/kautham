
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

/* Author: Alexander Perez, Jan Rosell */

#if !defined(_CONSTRAINEDKINEMATIC_H)
#define _CONSTRAINEDKINEMATIC_H


#include <util/kthutil/kauthamdefs.h>
#include <util/kthutil/kauthamobject.h>
#include <sampling/robconf.h>
#include <sampling/se3conf.h>
#include <sampling/rnconf.h>


namespace Kautham{
/** \defgroup libKin  Kinematics module
 *  \brief contains classes to solve the inverse kinematics of some robots
 *
 *   \todo Add detailed description of Kinematics module
 *
 *  @{
 */
  class Robot;
  class ConstrainedKinematic:public KauthamObject{
  public:
    friend class Robot;
    ConstrainedKinematic(Robot* rob);

    virtual ~ConstrainedKinematic(void);

    virtual bool		  solve()=0;

    virtual bool		  setParameters()=0;

	  //!	This method allow to setup the target.
    void              setTarget(vector<KthReal> &target);
    inline SE3Conf&   getSE3(){return _robConf.getSE3();}
    inline RnConf&    getRn(){return _robConf.getRn();}
    inline RobConf&   getRobConf(){return _robConf;}
    inline Robot&     getRobot(){return *_robot;}
  private:
    ConstrainedKinematic(void);
  protected:
    	  //!	This is a robot pointer. It will be free, chain or tree robot.
    Robot*          _robot;
    RobConf         _robConf;

	  //! This vector contains the target to be solved.
    vector<KthReal> _target;      //!< This is a generic way to set up the target.

  };
  /** @}   end of Doxygen module "Util */
}

#endif //_CONSTRAINEDKINEMATIC_H
