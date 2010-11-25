
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

#include "robconf.h"

namespace libSampling{
  RobConf::RobConf(){

  }

  bool RobConf::setSE3(){
    vector<KthReal> coords(6,(KthReal)0.0);

    return setSE3(coords);
  }

  bool RobConf::setSE3(vector<KthReal>& coords){
    try{
      first.setCoordinates(coords);
       return true;
    }catch(...){
      return false;
    }
  }

  bool RobConf::setSE3(SE3Conf& se3){
    try{
        first.setCoordinates(se3.getCoordinates());
       return true;
    }catch(...){
      return false;
    }
    
  }

  bool RobConf::setRn(unsigned int dim){
    if(second.getDim() != dim)
      second.reDim(dim);
    return true;
  }

  bool RobConf::setRn(vector<KthReal>& coords){
    try{
      if(second.getDim() != coords.size())
        second.reDim((unsigned int)coords.size());

      second.setCoordinates(coords);
      return true;
    }catch(...){
      return false;
    }
    
  }

  bool RobConf::setRn(RnConf& rn){
    try{
      if(second.getDim() != rn.getDim())
        second.reDim((unsigned int)rn.getDim());

      second.setCoordinates(rn.getCoordinates());
      return true;
    }catch(...){
      return false;
    }

  }

  KthReal RobConf::getDistance2(RobConf& robc){
    KthReal dist = (KthReal)0.0;
      dist += getSE3().getDistance2(robc.getSE3());

      dist += getRn().getDistance2(robc.getRn());

    return dist;
  }

  KthReal RobConf::getDistance2(RobConf& robc, RobWeight& robw){
    KthReal dist = (KthReal)0.0;

    dist += getSE3().getDistance2( robc.getSE3(), robw.getSE3Weight()[0],
                                  robw.getSE3Weight()[1] );

    dist += getRn().getDistance2(robc.getRn(), robw.getRnWeights());

    return dist;
  }


  RobConf RobConf::interpolate(RobConf& rbc, KthReal fraction){
    RobConf tmpRobConf;
    try{
      SE3Conf tmpS = getSE3().interpolate(rbc.getSE3(), fraction );
      tmpRobConf.setSE3( tmpS );

      RnConf tmpR = getRn().interpolate(rbc.getRn(), fraction );
      tmpRobConf.setRn( tmpR );
    }catch(...){ }

    return tmpRobConf;
      
  }
}
