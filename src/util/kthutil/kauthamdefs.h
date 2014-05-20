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


/**
* \mainpage THE KAUTHAM PROJECT
*  A robot simulation toolkit for motion planning
*
* \section Abstract
* The Kautham Project is a software tool used at the Institute of Industrial and
* Control Engineering (IOC-UPC) for teaching and research in robot motion planning.
* The tool allows to cope with problems with one or more robots, being a generic
* robot defined as a kinematic tree with a mobile base. i.e. the tool can plan
* and simulate from simple two degrees of freedom free-flying robots to multi-robot
* scenarios with mobile manipulators equipped with anthropomorphic hands.
* The main core of planners is provided by the Open Motion Planning Library (OMPL).
* Different basic planners can be flexibly used and parameterized, allowing
* students to gain insight into the different planning algorithms. Among the
* advanced features the tool allows to easily define the coupling between degrees of
* freedom, the dynamic simulation and the integration with task planers. It is
* principally being used in the research of motion planning strategies for hand-arm
* robotic systems.
*
* \section Features
* -# Uses OMPL suite of planners (geometric and control based)
* -# Uses PQP collision detection library
* -# Uses Coin3D for visualization
* -# Uses ODE physics engine
* -# Robot models are defined using urdf or DH parameters coded in an XML file
* -# Describes robots as kinematic trees with a mobile base (SE3xRn configuration space)
* -# Geometry is described in VRML files
* -# Allows multi-robot systems
* -# Allows the coupling of degrees of freedom
* -# Can be encapsuled as a ROS node

* \section Credits
*
* Institute of Industrial and Control Engineering (IOC)\n
* Universitat Polirecnica de Catalunya (UPC) - BarcelonaTECH\n
* Barcelona, Spain\n
* <A HREF="http://www.ioc.upc.edu"> www.ioc.upc.edu</A>\n
*
* In collaboration with  the Escuela Colombiana
* de Ingenieria "Julio Garavito" placed in Bogota D.C.
* Colombia
*
* \section Webpage
* <A HREF="http://sir.upc.edu/kautham"> sir.upc.edu/kautham</A>\n
*/



#if !defined(_KAUTHAMDEFS_H)
#define _KAUTHAMDEFS_H

#include <iostream> 
#include <string>
#include <vector>
#include <map>



using namespace std;

namespace Kautham{
  #define KthReal float
  #define MAJOR_VERSION "2"
  #define MINOR_VERSION "4"

	enum ROBOTTYPE{
		FREEFLY,
		CHAIN,
		TREE,
		CLOSEDCHAIN
	};

    enum APPROACH{
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

  enum LIBUSED{
    INVENTOR,
    IVPQP,
    IVSOLID
  };

  enum SPACETYPE{
    SAMPLEDSPACE,
    CONTROLSPACE,
    CONFIGSPACE
  };

  enum PROBLEMSTATE{
	  INITIAL,
	  PROBLEMLOADED,
	  CSPACECREATED,
	  PRMCALCULATED
  };

  enum CONSTRAINEDKINEMATICS{
    UNCONSTRAINED,
    BRONCHOSCOPY
  };

  //! This enumeration has the relationship of all Inverse Kinematic models
  //! available to be used as solver of the robot inverse kinematics.
  enum INVKINECLASS{
    NOINVKIN,
    UNIMPLEMENTED,
    RR2D,
    TX90,
    HAND,
    TX90HAND,
    UR5
  };

#ifndef INFINITY
#define INFINITY 1.0e40
#endif

    typedef std::map<std::string, KthReal> HASH_S_K;
    typedef std::map<std::string, std::string> HASH_S_S;

	
	//!	Structure  that save the object position and orientation in the WSpace.	
	/*!	This structure save  temporarily the most important information about 
	*		the object position and orientation in the 3D physical space regardless
	*		If the problem is or not 3D.*/
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

