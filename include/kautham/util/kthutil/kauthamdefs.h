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

/* Author: Alexander Perez, Jan Rosell and Nestor Garcia Hidalgo */


  
 /*	This file contains some useful definitions to be used into Kautham planning
		system.
*/




#if !defined(_KAUTHAMDEFS_H)
#define _KAUTHAMDEFS_H

#include <iostream> 
#include <string>
#include <vector>
#include <map>

#ifdef KAUTHAM_USE_OMPL
#include <ompl/config.h>
#if OMPL_VERSION_VALUE >= 1004000 //1.4.0
#include <Eigen/Core>
#else
#ifndef Q_MOC_RUN //moc problems
#include <ompl/base/ProjectionEvaluator.h>
#endif
#endif //OMPL_VERSION_VALUE
#endif //KAUTHAM_USE_OMPL

using namespace std;

namespace Kautham {
#define KthReal float


  enum ROBOTTYPE {
      FREEFLY,
      CHAIN,
      TREE,
      CLOSEDCHAIN
  };

  enum APPROACH {
      DHSTANDARD,
      DHMODIFIED,
      URDF
  };

  enum NEIGHSEARCHTYPE {
      BRUTEFORCE,
      ANNMETHOD
  };

  enum CONFIGTYPE {
      SE2,
      SE3,
      Rn
  };

  enum SPACETYPE {
      SAMPLEDSPACE,
      CONTROLSPACE,
      CONFIGSPACE
  };

  enum PROBLEMSTATE {
      INITIAL,
      PROBLEMLOADED,
  };

  enum CONSTRAINEDKINEMATICS {
      UNCONSTRAINED,
      BRONCHOSCOPY
  };

  //! This enumeration has the relationship of all Inverse Kinematic models
  //! available to be used as solver of the robot inverse kinematics.
  enum INVKINECLASS {
      NOINVKIN,
      UNIMPLEMENTED,
      RR2D,
      TX90,
      HAND,
      TX90HAND,
      UR5,
      YUMI_RIGHT,
      YUMI_LEFT,
      KUKA_LWR
  };

  enum PLANNERFAMILY {
      NOFAMILY,
      IOCPLANNER,
      OMPLPLANNER,
      OMPLCPLANNER,
      ODEPLANNER
  };

#ifndef INFINITY
#define INFINITY 1.0e40
#endif

  typedef std::map<std::string, KthReal> HASH_S_K;
  typedef std::map<std::string, std::string> HASH_S_S;

#ifdef KAUTHAM_USE_OMPL
#if OMPL_VERSION_VALUE >= 1004000 //1.4.0
  typedef Eigen::VectorXd Vector;
  typedef Eigen::Ref<Eigen::VectorXd> VectorRef;
#else
  typedef ompl::base::EuclideanProjection Vector;
  typedef ompl::base::EuclideanProjection &VectorRef;
#endif //OMPL_VERSION_VALUE
#endif //KAUTHAM_USE_OMPL
	
   //! Structure  that save the object position and orientation in the WSpace.
   /*! This structure save  temporarily the most important information about
   * the object position and orientation in the 3D physical space regardless
   * If the problem is or not 3D.*/
  struct DATA{
      DATA(){
          for(int i=0;i<3;i++)
              pos[i]=ori[i]= (KthReal)0.0;
          ori[3]= (KthReal)0.0;
      }
      KthReal pos[3];
      KthReal ori[4];
  };
}

#endif //_KAUTHAMDEFS_H
