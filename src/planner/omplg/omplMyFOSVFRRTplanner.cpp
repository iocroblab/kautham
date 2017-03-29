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

/* Author: Nestor Garcia Hidalgo */


#if defined(KAUTHAM_USE_OMPL)
#include <boost/bind/mem_fn.hpp>
#include <kautham/planner/omplg/MyFOSVFRRT.h>

#include <kautham/planner/omplg/omplMyFOSVFRRTplanner.h>
#include <kautham/planner/omplg/omplValidityChecker.h>
#include <kautham/planner/omplg/FOSUpstreamCriterionOptimizationObjective.h>
#include <kautham/planner/omplg/vectorField.h>





namespace Kautham {
  namespace omplplanner {
    //! Constructor
    omplMyFOSVFRRTPlanner::omplMyFOSVFRRTPlanner(SPACETYPE stype, Sample *init, Sample *goal,
                                       SampleSet *samples, WorkSpace *ws, og::SimpleSetup *ssptr,
                                             const std::string &synergyTreeFilename):
              omplPlanner(stype,init,goal,samples,ws,ssptr) {
        _guiName = "ompl VFRRT Planner";
        _idName = "omplVFRRT";

        //Create planner
        if (synergyTreeFilename.empty()) {
            st_ = new VectorField();
        } else {
            st_ =  new SynergyTree(synergyTreeFilename);
        }
        ss->getProblemDefinition()->setOptimizationObjective
                (ob::OptimizationObjectivePtr(new ob::FOSUpstreamCriterionOptimizationObjective(si,this,st_)));
        og::MyFOSVFRRT *planner = new og::MyFOSVFRRT(si,this,st_);
        planner->setup();

        //set planner parameters: range and goalbias
        addParameter("Range",planner->getRange());
        addParameter("Goal Bias",planner->getGoalBias());
        if (synergyTreeFilename.empty()) {
            addParameter("Problem",((VectorField*)st_)->getProblem());
        }

        //set the planner
        ss->setPlanner(ob::PlannerPtr(planner));
    }

    //! void destructor
    omplMyFOSVFRRTPlanner::~omplMyFOSVFRRTPlanner(){

    }

    //! function to find a solution path
    bool omplMyFOSVFRRTPlanner::trySolve()
    {
        if (omplPlanner::trySolve()) {
            //evaluate path
            cout << "path with " << ((og::PathGeometric)ss->getSolutionPath()).getStateCount() << " states" << endl;
            ob::Cost pathcost = ((og::PathGeometric)ss->getSolutionPath()).cost(ss->getProblemDefinition()->getOptimizationObjective());
            cout << "Path cost = " << pathcost.value() << endl;

            return true;
        } else {
            cout<<"No solution found"<<endl;

            return true;
        }
    }


    //! setParameters sets the parameters of the planner
    bool omplMyFOSVFRRTPlanner::setParameters() {
        if (!omplPlanner::setParameters()) return false;

        try {
            HASH_S_K::iterator it;
            og::MyFOSVFRRT *planner = ss->getPlanner()->as<og::MyFOSVFRRT>();

            it = _parameters.find("Range");
            if (it == _parameters.end()) return false;
            if (it->second <= 0.) {
                it->second = planner->getRange();
            } else {
                planner->setRange(it->second);
                if (it->second <= ( _validSegmentCount-1)*space->getLongestValidSegmentLength()) {
                    space->setLongestValidSegmentFraction(it->second/_validSegmentCount/space->getMaximumExtent());
                    space->setup();
                }
            }

            it = _parameters.find("Goal Bias");
            if (it == _parameters.end()) return false;
            if (it->second < 0. || it->second > 1.) {
                it->second = planner->getGoalBias();
            } else {
                planner->setGoalBias(it->second);
            }

            if (dynamic_cast<VectorField*>(st_)) {
                it = _parameters.find("Problem");
                if (it == _parameters.end()) return false;
                if (it->second < 0.) {
                    it->second = ((VectorField*)st_)->getProblem();
                } else {
                    ((VectorField*)st_)->setProblem(it->second);
                }
            }
        } catch(...) {
            return false;
        }
        return true;
    }
}
}


#endif // KAUTHAM_USE_OMPL
