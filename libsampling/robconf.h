
/***************************************************************************
*                                                                          *
*           Institute of Industrial and Control Engineering                *
*                 Technical University of Catalunya                        *
*                        Barcelona, Spain                                  *
*                                                                          *
*                Project Name:       Kautham Planner                       *
*                                                                          *
*     Copyright (C) 2007 - 2009 by Alexander Pérez and Jan Rosell          *
*            alexander.perez@upc.edu and jan.rosell@upc.edu                *
*                                                                          *
*             This is a motion planning tool to be used into               *
*             academic environment and it's provided without               *
*                     any warranty by the authors.                         *
*                                                                          *
*          Alexander Pérez is also with the Escuela Colombiana             *
*          de Ingeniería "Julio Garavito" placed in Bogotá D.C.            *
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




#if !defined(_ROBCONF_H)
#define _ROBCONF_H

#include "se3conf.h"
#include "rnconf.h"
#include "robweight.h"

namespace libSampling{
  class RobConf: public std::pair<SE3Conf,RnConf>{

  public:
    RobConf();
	
	//RobConf(RobConf& robc);

    //~RobConf();
    inline  SE3Conf& getSE3(){return first;}

    //! It initializes the SE3Conf with (0, 0, 0, 0, 0, 0).
    bool    setSE3();

    //! It initializes the SE3Conf with the coords contents.
    bool    setSE3(vector<KthReal>& coords);

    //! It copies the se3 coordinates.
    bool    setSE3(SE3Conf& se3);

    inline  RnConf& getRn(){return second;}

    //! It initializes the RnConf.
    bool    setRn(unsigned int);

    //! It initializes the RnConf with the coords contents.
    bool    setRn(vector<KthReal>& coords);

    //! It copies the rn coordinates.
    bool    setRn(RnConf& rn);

    inline KthReal getDistance(RobConf& robc){return sqrt(getDistance2(robc));}
    inline KthReal getDistance(RobConf& robc, RobWeight& robw){return sqrt(getDistance2(robc,robw));}

    KthReal getDistance2(RobConf& robc);
    KthReal getDistance2(RobConf& robc, RobWeight& robw);

    //! Returns the interpolated configuration based on its configurations.
    RobConf interpolate(RobConf& rbc, KthReal fraction);

  private:


  };

}

#endif //_ROBCONF_H
