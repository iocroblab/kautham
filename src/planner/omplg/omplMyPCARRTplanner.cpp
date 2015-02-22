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
#include <problem/workspace.h>
#include <sampling/sampling.h>

#include <boost/bind/mem_fn.hpp>

#include "omplMyPCARRTplanner.h"
#include "omplValidityChecker.h"
#include "myPCARRT.h"


#include <pugixml.hpp>
#include "pcaresult.h"

using namespace pugi;

namespace Kautham {
namespace omplplanner {
PCAResult *PMDset(xml_node *node, unsigned int dim, bool isPositionPCA) {
    arma::vec barycenter(dim);
    arma::vec eigenvalues(dim);
    arma::mat eigenvectors(dim,dim);

    xml_node barycenterNode = node->child("Offset").child("DOF");
    xml_node eigenvaluesNode = node->child("Control");
    xml_node eigenvectorsNode;

    for (unsigned int i = 0; i < dim; i++) {
        if (!barycenterNode.attribute("value")) return NULL;
        barycenter[i] = barycenterNode.attribute("value").as_double();

        if (!eigenvaluesNode.attribute("eigValue")) return NULL;
        eigenvalues[i] = eigenvaluesNode.attribute("eigValue").as_double();

        eigenvectorsNode = eigenvaluesNode.child("DOF");
        for (unsigned int j = 0; j < dim; j++) {
            if (!eigenvectorsNode.attribute("value")) return NULL;
            eigenvectors.at(i,j) = eigenvectorsNode.attribute("value").as_double();

            eigenvectorsNode = eigenvectorsNode.next_sibling("DOF");
        }

        barycenterNode = barycenterNode.next_sibling("DOF");
        eigenvaluesNode = eigenvaluesNode.next_sibling("Control");
    }

    if (isPositionPCA) {
        return new PositionPCAResult(barycenter,eigenvalues,eigenvectors);
    } else {
        return new VelocityPCAResult(barycenter,eigenvalues,eigenvectors);
    }
}


void omplMyPCARRTPlanner::setPMDset(string filename) {
    xml_document doc;
    xml_parse_result result = doc.load_file(filename.c_str());
    if (result) {
        vector<VelocityPCAResult*> *pmdSet = new vector<VelocityPCAResult*>;
        xml_node node = doc.child("ControlSet");
        while (node) {
            pmdSet->push_back(dynamic_cast<VelocityPCAResult*>(PMDset(&node,2,false)));
            node = node.next_sibling("ControlSet");
        }
        pmdSet->resize(pmdSet->size());
        (ss->getPlanner()->as<myPCARRT>())->setPMDset(pmdSet);
    } else {
        cout << result.description() << endl;
    }
}


//! Constructor
omplMyPCARRTPlanner::omplMyPCARRTPlanner(SPACETYPE stype, Sample *init, Sample *goal,
                                       SampleSet *samples, WorkSpace *ws, og::SimpleSetup *ssptr):
    omplPlanner(stype,init,goal,samples,ws,ssptr) {
    _guiName = "ompl MyPCARRT Planner";
    _idName = "omplMyPCARRT";


    //alloc valid state sampler
    si->setValidStateSamplerAllocator(boost::bind(&omplplanner::allocValidStateSampler,_1,(Planner*)this));
    //alloc state sampler
    space->setStateSamplerAllocator(boost::bind(&omplplanner::allocStateSampler,_1,(Planner*)this));

    //create planner
    ob::PlannerPtr planner(new myPCARRT(si));
    //set planner parameters: range and goalbias
    _GoalBias = (planner->as<myPCARRT>())->getGoalBias();
    addParameter("Goal Bias",_GoalBias);
    _Alfa = (planner->as<myPCARRT>())->getAlfa();
    addParameter("Alfa",_Alfa);
    _Range = (planner->as<myPCARRT>())->getRange();
    addParameter("Range",_Range);
    _PMDBias = (planner->as<myPCARRT>())->getPMDbias();
    addParameter("PMD Bias",_PMDBias);

    //set the planner
    ss->setPlanner(planner);
}


//! void destructor
omplMyPCARRTPlanner::~omplMyPCARRTPlanner(){

}


//! setParameters sets the parameters of the planner
bool omplMyPCARRTPlanner::setParameters(){
    if (!omplPlanner::setParameters()) return false;

    try {
        HASH_S_K::iterator it;


        it = _parameters.find("Goal Bias");
        if (it == _parameters.end()) return false;
        if (it->second < 0. || it->second > 1.) {
            setParameter("Goal Bias",_GoalBias);
        } else {
            _GoalBias = it->second;
            ss->getPlanner()->as<myPCARRT>()->setGoalBias(_GoalBias);
        }


        it = _parameters.find("PMD Bias");
        if (it == _parameters.end()) return false;
        if (it->second < 0. || it->second > 1.) {
            setParameter("PMD Bias",_PMDBias);
        } else {
            _PMDBias = it->second;
            ss->getPlanner()->as<myPCARRT>()->setPMDbias(_PMDBias);
        }


        it = _parameters.find("Alfa");
        if (it == _parameters.end()) return false;
        if (it->second < 0. || it->second > 1.) {
            setParameter("Alfa",_Alfa);
        } else {
            _Alfa = it->second;
            ss->getPlanner()->as<myPCARRT>()->setAlfa(_Alfa);
        }


        it = _parameters.find("Range");
        if (it == _parameters.end()) return false;
        if (it->second < 0.) {
            setParameter("Range",_Range);
        } else {
            _Range = it->second;
            ss->getPlanner()->as<myPCARRT>()->setRange(_Range);
        }
    } catch(...) {
        return false;
    }
    return true;
}
}
}
#endif // KAUTHAM_USE_OMPL
