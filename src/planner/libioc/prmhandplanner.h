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

/* Author: Alexander Perez, Jan Rosell, Nestor Garcia Hidalgo */

#include "prmplanner.h"
#include <libsutil/lcprng.h>

#if !defined(_PRMHANDPLANNER_H)
#define _PRMHANDPLANNER_H
 namespace Kautham {
 /** \addtogroup libPlanner
  *  @{
  */
  namespace IOC{
		class PRMHandPlanner:public PRMPlanner{
    public:
			PRMHandPlanner(SPACETYPE stype, Sample *init, Sample *goal, SampleSet *samples, Sampler *sampler, 
          WorkSpace *ws,  int cloundSize, KthReal cloudRad);
			~PRMHandPlanner();
      bool            setParameters();
	  void            saveData();
//	  void            setIniGoal();
      virtual bool    trySolve()=0;
      inline void     setCloudSize(int cs){_cloudSize = cs;}
      inline int      cloudSize(){return _cloudSize;}
      inline void     setCloudRad(KthReal cr){_cloudRadius = cr;}
      inline KthReal  cloudRad(){return _cloudRadius;}
	  
	  bool getSampleInGoalRegion();

	protected:
	  LCPRNG*			_gen;
      int               _cloudSize;
      KthReal           _cloudRadius;
	};	
  }
  /** @}   end of Doxygen module "libPlanner */
};
 
#endif  //_PRMPLANNER_H

