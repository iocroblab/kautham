
/***************************************************************************
*                                                                          *
*           Institute of Industrial and Control Engineering                *
*                 Technical University of Catalunya                        *
*                        Barcelona, Spain                                  *
*                                                                          *
*                Project Name:       Kautham Planner                       *
*                                                                          *
*     Copyright (C) 2007 - 2009 by Alexander P�rez and Jan Rosell          *
*            alexander.perez@upc.edu and jan.rosell@upc.edu                *
*                                                                          *
*             This is a motion planning tool to be used into               *
*             academic environment and it's provided without               *
*                     any warranty by the authors.                         *
*                                                                          *
*          Alexander P�rez is also with the Escuela Colombiana             *
*          de Ingenier��a "Julio Garavito" placed in Bogot� D.C.            *
*             Colombia.  alexander.perez@escuelaing.edu.co                 *
*                                                                          *
***************************************************************************/
/***************************************************************************
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

#if !defined(_CONSTRAINEDKINEMATIC_H)
#define _CONSTRAINEDKINEMATIC_H


#include <libutil/kauthamdefs.h>
#include <libsampling/robconf.h>
#include <libsampling/se3conf.h>
#include <libsampling/rnconf.h>
#include <libutil/kauthamobject.h>

using namespace Kautham;
using namespace libSampling;

namespace libProblem{
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
}

#endif //_CONSTRAINEDKINEMATIC_H