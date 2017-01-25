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

#if !defined(_PRMHANDPLANNERARMHAND_H)
#define _PRMHANDPLANNERARMHAND_H

#include <kautham/planner/ioc/prmhandplanner.h>


#if defined(KAUTHAM_USE_ARMADILLO)
#include <iostream>
#include <armadillo>
#include <boost/random.hpp>

using namespace arma;
using namespace std;

typedef boost::normal_distribution<double> NormalDistribution;
typedef boost::mt19937 RandomGenerator;
typedef boost::variate_generator<RandomGenerator&, NormalDistribution> GaussianGenerator;
 
 namespace Kautham {
 /** \addtogroup Planner
  *  @{
  */
  namespace IOC{
      typedef std::pair<KthReal, KthReal> thumbLimits;
	  

	class PRMPCAHandArmPlanner:public PRMHandPlanner{
		public:
			PRMPCAHandArmPlanner(SPACETYPE stype, Sample *init, Sample *goal, SampleSet *samples, Sampler *sampler, 
                WorkSpace *ws, int cloundSize,int samplingV, KthReal cloudRad, int samplingR,float distgoal,float distsamplingpcagoal);
	  
			~PRMPCAHandArmPlanner();
      
			bool  setParameters();
			bool  trySolve();
			bool getHandConfig(vector<KthReal>& coord, bool randhand, int numPMDs);
			bool getSampleInGoalRegion(double tradius, double rradius);  
			///////////////////////////////////////////////////////////
			////////////////Process PCA///////////////////////////////
			bool getSampleRandPCA(float R);
			bool SamplerPCASpace(bool spca,float R,float tradius);
			////////////////////////////////////////////////////////
			int getSamplesBetweenInitGoal(int maxinterpolatedpoints, double tdeltaM, double rdeltaM, bool handWholeRange);
			////////////////////////////////////////////////////////
			bool getSampleInGoalRegionRealworld(double tradius, double rradius, bool handWholeRange);
			//////////////////////////////////////////////////////////
			void printPCAComponents();
			//////////////////////////////////////////////////////////
			bool  getSampleInRegion(SE3Conf  *smpse3, double tradius, double rradius);
			void saveData();//reimplemented

			bool ArmInverseKinematics(vector<KthReal> &carm, Sample *smp, bool maintainSameWrist=true);
 			void setIniGoalSe3();

			bool getSampleInGoalRegionRealworldGaussian(double tdeltaM, double rdeltaM, bool handWholeRange);
			bool getSampleInGoalRegionRealworldBridgeTest(double tdeltaM, double rdeltaM, bool handWholeRange);
			bool verifySuccess(clock_t inittime, int nloops, int nPCAcalls);
	 
		private:
			int _incrementalPMDs;
			KthReal _lambda;
			int _samplingV;
			int _samplingR;
			SE3Conf _inise3;
			SE3Conf _goalse3;
			///For Process PCA
			vector<float> _distance;
			vector<int>	_indexpca;
			
			mt::Point3 _goaltrans;
			int callpca;
			float _deltaR;
			float _deltaI;
			int _samplingmethod; //flag to use gaussian instead of sampling in V using the pca. For comparison purposes. 
            mat matPCA;

			FILE *fp;
			//GaussianGenerator *_gaussianGen;
	};	
  }
  /** @}   end of Doxygen module "Planner */
}

#endif // KAUTHAM_USE_ARMADILLO
 
#endif  //_PRMHANDPLANNERARMHAND_H

