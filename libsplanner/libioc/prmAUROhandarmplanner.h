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

#include "prmhandplanner.h"

#if !defined(_PRMAUROHANDPLANNER_H)
#define _PRMAUROHANDPLANNER_H
 namespace Kautham {
 /** \addtogroup libPlanner
  *  @{
  */
  namespace IOC{
      typedef std::pair<KthReal, KthReal> thumbLimits;
	  

	class PRMAUROHandArmPlanner:public PRMHandPlanner{
		public:
			PRMAUROHandArmPlanner(SPACETYPE stype, Sample *init, Sample *goal, SampleSet *samples, Sampler *sampler, 
                WorkSpace *ws,  int cloundSize, KthReal cloudRad, int numHC);
	  
			~PRMAUROHandArmPlanner();
      
			bool  setParameters();
			bool  trySolve();
      bool getHandConfig(vector<KthReal>& coord, bool randhand, int numPMDs);
			bool getSampleInGoalRegion(double tradius, double rradius);  
    bool  getSampleInRegion(SE3Conf  *smpse3, double tradius, double rradius);
			void saveData();//reimplemented
//      void setIniGoal();//reimplemented
      void writeFiles(FILE *fpr, FILE *fph, RobConf* joints);
			void computeMaxSteps(KthReal radius, int *bits, int *steps);

	bool ArmInverseKinematics(vector<KthReal> &carm, Sample *smp, bool maintainSameWrist=true);
 	void setIniGoalSe3();
	 
		private:
			int	_numberHandConf;
			int _incrementalPMDs;
			KthReal _cloudRadiusMax;
			int _exhaustiveCloud;
			SE3Conf _inise3;
			SE3Conf _goalse3;

	};	
  }
  /** @}   end of Doxygen module "libPlanner */
}
 
#endif  //_PRMAUROHANDPLANNER_H

