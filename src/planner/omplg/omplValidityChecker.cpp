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



#if defined(KAUTHAM_USE_OMPL)
#include <kautham/problem/workspace.h>
#include <kautham/sampling/sampling.h>

#include <kautham/planner/omplg/omplValidityChecker.h>
#include <kautham/planner/omplg/omplplanner.h>

namespace Kautham {
namespace omplplanner{

// Returns whether the given state's position overlaps the obstacles
bool ValidityChecker::isValid(const ob::State* state) const
{
    //JAN DEBUG
    //return true;

    //verify bounds
    if(thesi->satisfiesBounds(state)==false)
        return false;
    //create sample
    int d = theplanner->wkSpace()->getNumRobControls();
    Sample *smp = new Sample(d);
    //copy the conf of the init smp. Needed to capture the home positions.
    smp->setMappedConf(theplanner->initSamp()->getMappedConf());
    //load the RobConf of smp form the values of the ompl::state
    ((omplPlanner*)theplanner)->omplState2smp(state,smp);
    //collision-check
    if( theplanner->wkSpace()->collisionCheck(smp) )
        return false;

    //Discards the sample.
    //By default does nothing, i.e. the default filtersample function in class Planner
    //returns always false
    if(theplanner->filtersample(smp))
        return false;

    return true;
}

// Returns the distance from the given state's position to the obstacles
double ValidityChecker::clearance(const ob::State* state) const
{
    //verify bounds
    if(thesi->satisfiesBounds(state)==false)
        return false;
    //create sample
    int d = theplanner->wkSpace()->getNumRobControls();
    Sample *smp = new Sample(d);
    //copy the conf of the init smp. Needed to capture the home positions.
    smp->setMappedConf(theplanner->initSamp()->getMappedConf());
    //load the RobConf of smp form the values of the ompl::state
    ((omplPlanner*)theplanner)->omplState2smp(state,smp);
    //distance-check
    vector<KthReal> *distvect;
    distvect = theplanner->wkSpace()->distanceCheck(smp);
    KthReal dist = FLT_MAX;
    for(unsigned i=0; i<distvect->size(); i++)
        if(dist>distvect->at(i)) dist = distvect->at(i);
    return dist;
}

}
}


#endif // KAUTHAM_USE_OMPL
