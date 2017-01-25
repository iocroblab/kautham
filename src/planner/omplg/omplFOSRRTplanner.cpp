/*************************************************************************\
   Copyright 2014 Institute of Industrial and Control Engineering (IOC)
                 Universitat Politecnica de Catalunya
                 BarcelonaTech
    All Rights Reserved.

    This prompl::geometricram is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This prompl::geometricram is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this prompl::geometricram; if not, write to the
    Free Software Foundation, Inc.,
    59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 \*************************************************************************/

/* Author: Nestor Garcia Hidalgo */


#if defined(KAUTHAM_USE_OMPL)
#include <kautham/problem/workspace.h>
#include <kautham/sampling/sampling.h>

#include <boost/bind/mem_fn.hpp>

#include <kautham/planner/omplg/omplFOSRRTplanner.h>
#include <kautham/planner/omplg/omplValidityChecker.h>
#include <kautham/planner/omplg/FOSRRT.h>

#include <pugixml.hpp>


using namespace pugi;

namespace Kautham {
namespace omplplanner {
void omplFOSRRTPlanner::setSynergyTree(string filename) {
    SynergyTree *tree = new SynergyTree(filename);
    (ss->getPlanner()->as<ompl::geometric::FOSRRT>())->setSynergyTree(tree);
}


//! Constructor
omplFOSRRTPlanner::omplFOSRRTPlanner(SPACETYPE stype, Sample *init, Sample *goal,
                                       SampleSet *samples, WorkSpace *ws, og::SimpleSetup *ssptr):
    omplPlanner(stype,init,goal,samples,ws,ssptr) {
    _guiName = "ompl FOSRRT Planner";
    _idName = "omplFOSRRT";

    //create planner
    ob::PlannerPtr planner(new ompl::geometric::FOSRRT(si,this));
    //set planner parameters: range and goalbias
    _GoalBias = (planner->as<ompl::geometric::FOSRRT>())->getGoalBias();
    addParameter("Goal Bias",_GoalBias);
    _TimeStep = (planner->as<ompl::geometric::FOSRRT>())->getTimeStep();
    addParameter("Time Step",_TimeStep);
    _Range = (planner->as<ompl::geometric::FOSRRT>())->getRange();
    addParameter("Range",_Range);
    _PMDBias = (planner->as<ompl::geometric::FOSRRT>())->getPMDbias();
    addParameter("PMD Bias",_PMDBias);

    //set the planner
    ss->setPlanner(planner);
}


//! void destructor
omplFOSRRTPlanner::~omplFOSRRTPlanner(){

}


//! setParameters sets the parameters of the planner
bool omplFOSRRTPlanner::setParameters(){
    if (!omplPlanner::setParameters()) return false;

    try {
        HASH_S_K::iterator it;


        it = _parameters.find("Goal Bias");
        if (it == _parameters.end()) return false;
        if (it->second < 0. || it->second > 1.) {
            setParameter("Goal Bias",_GoalBias);
        } else {
            _GoalBias = it->second;
            ss->getPlanner()->as<ompl::geometric::FOSRRT>()->setGoalBias(_GoalBias);
        }


        it = _parameters.find("PMD Bias");
        if (it == _parameters.end()) return false;
        if (it->second < 0. || it->second > 1.) {
            setParameter("PMD Bias",_PMDBias);
        } else {
            _PMDBias = it->second;
            ss->getPlanner()->as<ompl::geometric::FOSRRT>()->setPMDbias(_PMDBias);
        }


        it = _parameters.find("Time Step");
        if (it == _parameters.end()) return false;
        if (it->second < 0.) {
            setParameter("Time Step",_TimeStep);
        } else {
            _TimeStep = it->second;
            ss->getPlanner()->as<ompl::geometric::FOSRRT>()->setTimeStep(_TimeStep);
        }


        it = _parameters.find("Range");
        if (it == _parameters.end()) return false;
        if (it->second < 0.) {
            setParameter("Range",_Range);
        } else {
            _Range = it->second;
            ss->getPlanner()->as<ompl::geometric::FOSRRT>()->setRange(_Range);
        }
    } catch(...) {
        return false;
    }
    return true;
}
}
}
#endif // KAUTHAM_USE_OMPL
