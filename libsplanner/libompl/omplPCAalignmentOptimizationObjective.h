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

#if !defined(_omplPCAOBJECTIVE_H)
#define _omplPCAOBJECTIVE_H

#if defined(KAUTHAM_USE_OMPL)

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/objectives/MechanicalWorkOptimizationObjective.h>
#include <ompl/base/ProjectionEvaluator.h>



namespace ob = ompl::base;


namespace Kautham {
/** \addtogroup libPlanner
 *  @{
 */
  namespace omplplanner{

  class PMDalignmentOptimizationObjective:public ob::OptimizationObjective {
      protected:
      typedef boost::numeric::ublas::matrix<double> Matrix;

      ob::ProjectionMatrix PMD; //!< The copupling matrix
      ob::EuclideanProjection lambda;//!< The eignevalues of the PMDs. This is a typedef of boost::numeric::ublas::vector < double >
      int numPMD; //!< Number of PMDs considered for the hand. It corresponds to the number of columns of te copupling matrix
      int numDOF; //!< Number of DOF coupled. It corresponds to the number of rows of the coupling matrix

      double wpenalization; //!< To penalize changes in orientation between consecutive edges of a path
      double wdistance;//!< To weight the distance
      double worientation;//!< To weight the alignment with the PMDs

  public:
      PMDalignmentOptimizationObjective(const ob::SpaceInformationPtr &si, ob::ProjectionMatrix M);
      ~PMDalignmentOptimizationObjective();

      virtual ob::Cost motionCost(const ob::State *s0, const ob::State *s1, const ob::State *s2) const = 0;
      virtual ob::Cost motionCost(const ob::State *s1, const ob::State *s2) const;
      void setPCAdata(ob::ProjectionMatrix M);
      inline double getOrientationPenalization(){return wpenalization;}
      inline void setOrientationPenalization(double w){wpenalization=w;};
      inline double getDistanceWeight(){return wdistance;}
      inline void setDistanceWeight(double w){wdistance=w;};
      inline void setOrientationWeight(double w){worientation=w;};
      inline double getOrientationWeight(){return worientation;};
    };

  class singleRobotPMDalignmentOptimizationObjective:public PMDalignmentOptimizationObjective {
      private:
      typedef boost::numeric::ublas::matrix<double> Matrix;

      double weightSE3;//!< To weight the costs in Rn subspace (cost=distcost+alignmentcost+penalization)
      double weightRn;//!< To weight the cost in SE3 subspace (cost=distance)
      int robotindex;

  public:
      singleRobotPMDalignmentOptimizationObjective(int roboti, const ob::SpaceInformationPtr &si, ob::ProjectionMatrix M);
      ~singleRobotPMDalignmentOptimizationObjective();

      virtual ob::Cost motionCost(const ob::State *s0, const ob::State *s1, const ob::State *s2) const;
      ob::Cost motionCostRn(const ob::State *s0, const ob::State *s1, const ob::State *s2) const;
      ob::Cost motionCostSE3(const ob::State *s1, const ob::State *s2) const;
    };


  class multiRobotSE3PMDalignmentOptimizationObjective:public PMDalignmentOptimizationObjective {
      private:
      typedef boost::numeric::ublas::matrix<double> Matrix;

  public:
      multiRobotSE3PMDalignmentOptimizationObjective(const ob::SpaceInformationPtr &si, ob::ProjectionMatrix M);
      ~multiRobotSE3PMDalignmentOptimizationObjective();

      virtual ob::Cost motionCost(const ob::State *s0, const ob::State *s1, const ob::State *s2) const;
    };


  /*
    class singleRobotPMDalignmentOptimizationObjective:public ob::OptimizationObjective {
        private:
        typedef boost::numeric::ublas::matrix<double> Matrix;

        ob::ProjectionMatrix PMD; //!< The copupling matrix
        ob::EuclideanProjection lambda;//!< The eignevalues of the PMDs. This is a typedef of boost::numeric::ublas::vector < double >
        int numPMD; //!< Number of PMDs considered for the hand. It corresponds to the number of columns of te copupling matrix
        int numDOF; //!< Number of DOF coupled. It corresponds to the number of rows of the coupling matrix

        double wpenalization; //!< To penalize changes in orientation between consecutive edges of a path
        double wdistance;//!< To weight the distance
        double worientation;//!< To weight the alignment with the PMDs
        double weightSE3;//!< To weight the costs in Rn subspace (cost=distcost+alignmentcost+penalization)
        double weightRn;//!< To weight the cost in SE3 subspace (cost=distance)
        int robotindex;

    public:
        singleRobotPMDalignmentOptimizationObjective(int roboti, const ob::SpaceInformationPtr &si, ob::ProjectionMatrix M);
        ~singleRobotPMDalignmentOptimizationObjective();

        virtual ob::Cost motionCost(const ob::State *s1, const ob::State *s2) const;
        virtual ob::Cost motionCost(const ob::State *s0, const ob::State *s1, const ob::State *s2) const;
        ob::Cost motionCostRn(const ob::State *s0, const ob::State *s1, const ob::State *s2) const;
        ob::Cost motionCostSE3(const ob::State *s1, const ob::State *s2) const;
        void setPCAdata(ob::ProjectionMatrix M);
        inline double getOrientationPenalization(){return wpenalization;}
        inline void setOrientationPenalization(double w){wpenalization=w;};
        inline double getDistanceWeight(){return wdistance;}
        inline void setDistanceWeight(double w){wdistance=w;};
        inline void setOrientationWeight(double w){worientation=w;};
        inline double getOrientationWeight(){return worientation;};
      };
      */

    /*
        class PCAalignmentOptimizationObjective:public ob::OptimizationObjective {
            public:
            typedef boost::numeric::ublas::matrix<double> Matrix;

            ob::ProjectionMatrix pcaM;
            ob::ProjectionMatrix pcaMinv;
            ob::EuclideanProjection lambda;//this is a typedef of boost::numeric::ublas::vector < double >
            bool PCAdataset;
            int dimension;
            double wpenalization;
            double wdistance;
            double worientation;

            PCAalignmentOptimizationObjective(const ob::SpaceInformationPtr &si, int dim,ob::ProjectionMatrix M);
            ~PCAalignmentOptimizationObjective();

            virtual ob::Cost motionCost(const ob::State *s1, const ob::State *s2) const;
            virtual ob::Cost motionCost(const ob::State *s0, const ob::State *s1, const ob::State *s2) const;
            void setPCAdata(ob::ProjectionMatrix M);
            inline bool isPCAdataset(){return PCAdataset;};
            inline double getOrientationPenalization(){return wpenalization;}
            inline void setOrientationPenalization(double w){wpenalization=w;};
            inline double getDistanceWeight(){return wdistance;}
            inline void setDistanceWeight(double w){wdistance=w;};
            inline void setOrientationWeight(double w){worientation=w;};
            inline double getOrientationWeight(){return worientation;};
          };
          */

    /*

    class PCAalignmentOptimizationObjective3:public ob::MechanicalWorkOptimizationObjective {
        public:
        typedef boost::numeric::ublas::matrix<double> Matrix;

        ob::ProjectionMatrix pcaM;
        std::vector<double> bari;
        ob::EuclideanProjection lambda;//this is a typedef of boost::numeric::ublas::vector < double >
        bool PCAdataset;
        int dimension;
        double wdistance;
        double wfix;

        PCAalignmentOptimizationObjective3(const ob::SpaceInformationPtr &si, int dim);
        ~PCAalignmentOptimizationObjective3();

        ob::Cost stateCost(const ob::State *s1) const;
        ob::Cost motionCost(const ob::State *s1, const ob::State *s2) const;
        void setPCAdata(int v);//ob::ProjectionMatrix M, ob::EuclideanProjection v);
        inline bool isPCAdataset(){return PCAdataset;};
        inline double getDistanceWeight(){return wdistance;}
        inline void setDistanceWeight(double w){wdistance=w;};
        inline void setFixWeight(double w){wfix=w;};
        inline double getFixWeight(){return wfix;};
      };
      */


  }
  /** @}   end of Doxygen module "libPlanner */
}

#endif // KAUTHAM_USE_OMPL
#endif  //_omplPCAOBJECTIVE_H
