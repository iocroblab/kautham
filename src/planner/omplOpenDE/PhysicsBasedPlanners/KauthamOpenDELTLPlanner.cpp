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

/* Author:  Muhayyuddin */

#include "KauthamOpenDELTLPlanner.h"

#include <ompl/control/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/config.h>
#include <iostream>
#include <vector>

#include <ompl/extensions/triangle/PropositionalTriangularDecomposition.h>
#include <ompl/control/planners/ltl/PropositionalDecomposition.h>
#include <ompl/control/planners/ltl/Automaton.h>
#include <ompl/control/planners/ltl/ProductGraph.h>
#include <ompl/control/planners/ltl/LTLPlanner.h>
#include <ompl/control/planners/ltl/LTLProblemDefinition.h>

//#include "../InstantiatedKnowledge/Instantiatedknowledge.h"
namespace ob = ompl::base;
namespace oc = ompl::control;
namespace Kautham {

namespace omplcplanner{
typedef oc::PropositionalTriangularDecomposition::Polygon Polygon;
typedef oc::PropositionalTriangularDecomposition::Vertex Vertex;

// a decomposition is only needed for SyclopRRT and SyclopEST
// use TriangularDecomp
class MyDecomposition : public oc::PropositionalTriangularDecomposition
{
public:
    MyDecomposition(const ob::RealVectorBounds& bounds)
        : oc::PropositionalTriangularDecomposition(bounds) { }
    virtual ~MyDecomposition() { }

    virtual void project(const ob::State* s, std::vector<double>& coord) const
    {
        coord.resize(2);
        coord[0]=s->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[0];
        coord[1]=s->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[1];
    }

    virtual void sampleFullState(const ob::StateSamplerPtr& sampler, const std::vector<double>& coord, ob::State* s) const
    {
       sampler->sampleUniform(s);
       s->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[0] = coord[0];
       s->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[1] = coord[1];

    }

private:
    ompl::RNG rng_;
};

////////////////////LTLProblemSetup//////////////////////

KauthamDELTL2DPlanner::KauthamDELTL2DPlanner(SPACETYPE stype, Sample *init, Sample *goal, SampleSet *samples, WorkSpace *ws):
    KauthamDEPlanner(stype, init, goal, samples, ws)
{
    //set intial values from parent class data
    _wkSpace->moveRobotsTo(init);
    ob::RealVectorBounds bounds1(2);

    _guiName = "ompl ODE LTL 2D Planner";
    _idName = "omplODELTL2DPlanner";
    bounds1.resize(2);
    dInitODE2(0);

    oc::OpenDEEnvironmentPtr envPtr(new ConstraintAware2DRobotEnvironment (ws,_maxspeed,_maxContacts,_minControlSteps,_maxControlSteps, _erp, _cfm));
    stateSpace = new ConstraintAwaretwoDRobotStateSpace(envPtr);
    stateSpacePtr = ob::StateSpacePtr(stateSpace);

    ss = new oc::OpenDESimpleSetup(stateSpacePtr);
    oc::SpaceInformationPtr si=ss->getSpaceInformation();

    bounds1.low[0] = _wkSpace->getRobot(0)->getLimits(0)[0];
    bounds1.low[1] = _wkSpace->getRobot(0)->getLimits(1)[0];
    bounds1.high[0] = _wkSpace->getRobot(0)->getLimits(0)[1];
    bounds1.high[1] = _wkSpace->getRobot(0)->getLimits(1)[1];
    ob::RealVectorBounds vb(3);
    ob::RealVectorBounds bounds(3);
    vb.low[0] = _wkSpace->getRobot(0)->getLimits(0)[0];
    vb.low[1] = _wkSpace->getRobot(0)->getLimits(1)[0];
    vb.low[2] = _wkSpace->getRobot(0)->getLimits(2)[0];
    vb.high[0] = _wkSpace->getRobot(0)->getLimits(0)[1];
    vb.high[1] = _wkSpace->getRobot(0)->getLimits(1)[1];
    vb.high[2] = _wkSpace->getRobot(0)->getLimits(2)[1];

    stateSpace->setVolumeBounds(vb);
    bounds.setLow(-50);
    bounds.setHigh(50);
    stateSpace->setLinearVelocityBounds(bounds);
    bounds.setLow(-20);
    bounds.setHigh(20);
    stateSpace->setAngularVelocityBounds(bounds);

    // create triangulation that ignores obstacle and respects propositions
    MyDecomposition* ptd = new MyDecomposition(bounds1);
    // helper method that add propositions
    LTLProblemSetup LTLProblem(ptd,_wkSpace);
    //Propositions(ptd);
    ptd->setup();
    oc::PropositionalDecompositionPtr pd(ptd);

     si->setStateValidityChecker(ob::StateValidityCheckerPtr(new oc::OpenDEStateValidityChecker(si)));
     si->setStatePropagator(oc::StatePropagatorPtr(new oc::OpenDEStatePropagator(si)));

     std::vector<unsigned int> sequence(LTLProblem.sequence.size());
//     sequence[0] = 0;
//     sequence[1] = 1;
//     sequence[2] = 2;
    sequence=LTLProblem.sequence;

     oc::AutomatonPtr cosafety ;
     oc::AutomatonPtr safety;
     for(int i=0;i<LTLProblem.TypeOfAutomaton.size();i++)
     {
         if(LTLProblem.TypeOfAutomaton[i]=="DisjunctionAutomaton")
         {
             cosafety= oc::Automaton::DisjunctionAutomaton(LTLProblem.PropsCouunt,LTLProblem.sequence);
         }
         if(LTLProblem.TypeOfAutomaton[i]=="SequenceAutomaton")
         {
             cosafety= oc::Automaton::SequenceAutomaton(LTLProblem.PropsCouunt,LTLProblem.sequence);
         }
         if(LTLProblem.TypeOfAutomaton[i]=="AvoidanceAutomaton")
         {
             safety= oc::Automaton::AvoidanceAutomaton(LTLProblem.PropsCouunt,LTLProblem.sequence);
             std::vector<unsigned int> avoid(LTLProblem.avoidance.size());
             avoid = LTLProblem.avoidance;
             safety = oc::Automaton::CoverageAutomaton(LTLProblem.PropsCouunt, avoid);

         }
     }
      oc::ProductGraphPtr product;
      if(LTLProblem.TypeOfAutomaton.size()==2)
      {
          //construct product graph (propDecomp x A_{cosafety} x A_{safety})
          product=oc::ProductGraphPtr(new oc::ProductGraph(pd, cosafety,safety));
      }
      else
      {
          product=oc::ProductGraphPtr(new oc::ProductGraph(pd, cosafety));
      }

     // LTLSpaceInformation creates a hybrid space of robot state space x product graph.
     // It takes the validity checker from SpaceInformation and expands it to one that also
     // rejects any hybrid state containing rejecting automaton states.
     // It takes the state propagator from SpaceInformation and expands it to one that
     // follows continuous propagation with setting the next decomposition region
     // and automaton states accordingly.
     //
     // The robot state space, given by SpaceInformation, is referred to as the "lower space".
    ltlsi= oc::LTLSpaceInformationPtr(new oc::LTLSpaceInformation(si, product));

     // LTLProblemDefinition creates a goal in hybrid space, corresponding to any
     // state in which both automata are accepting
     pDefp = oc::LTLProblemDefinitionPtr(new oc::LTLProblemDefinition(ltlsi));
     //oc::LTLProblemDefinitionPtr pDefp(new oc::LTLProblemDefinition(ltlsi));

    // create a start state
         ob::ScopedState<oc::OpenDEStateSpace> start(stateSpacePtr);
        start=ss->getCurrentState();

         // addLowerStartState accepts a state in lower space, expands it to its
         // corresponding hybrid state (decomposition region containing the state, and
         // starting states in both automata), and adds that as an official start state.
         pDefp->addLowerStartState(start.get());

         //LTL planner (input: LTL space information, product automaton)

        ltlplanner = new oc::LTLPlanner(ltlsi, product);
        ltlplanner->setProblemDefinition(pDefp);

}
//! void destructor
KauthamDELTL2DPlanner::~KauthamDELTL2DPlanner(){

}
//! this function set the necessary parameters for KAPIECE Planner.
bool KauthamDELTL2DPlanner::setParameters()
{
    KauthamDEPlanner::setParameters();

    return true;

}

}

}

/*void Propositions(oc::PropositionalTriangularDecomposition* decomp)
{

//    Polygon p0(4);
//    p0.pts[0] = Vertex(135,115);
//    p0.pts[1] = Vertex(135,155);
//    p0.pts[2] = Vertex(95,155);
//    p0.pts[3] = Vertex(95,115);
//    decomp->addProposition(p0);

        Polygon p1(4);
        p1.pts[0] = Vertex(-170,-150);
        p1.pts[1] = Vertex(-190,-150);
        p1.pts[2] = Vertex(-190,-170);
        p1.pts[3] = Vertex(-170,-170);
        decomp->addProposition(p1);

        Polygon p2(4);
        p2.pts[0] = Vertex(200,-200);
        p2.pts[1] = Vertex(220,-200);
        p2.pts[2] = Vertex(220,-180);
        p2.pts[3] = Vertex(200,-180);
        decomp->addProposition(p2);

//        Polygon p3(4);
//        p3.pts[0] = Vertex(0,0);
//        p3.pts[1] = Vertex(-5,0);
//        p3.pts[2] = Vertex(-5,5);
//        p3.pts[3] = Vertex(0,5);
//        decomp->addProposition(p3);
//whole for wall1
//        Polygon hole1(4);
//        hole1.pts[0]=Vertex(130,-5);
//        hole1.pts[1]=Vertex(130,-35);
//        hole1.pts[2]=Vertex(-130,-35);
//        hole1.pts[3]=Vertex(-130,-5);
//        decomp->addHole(hole1);
//whole for wall2

//        Polygon hole2(4);
//        hole2.pts[0]=Vertex(130,80);
//        hole2.pts[1]=Vertex(160,80);
//        hole2.pts[2]=Vertex(160,220);
//        hole2.pts[3]=Vertex(130,220);
//        decomp->addHole(hole2);

//        Polygon hole3(4);
//        hole3.pts[0]=Vertex(180,-120);
//        hole3.pts[1]=Vertex(180,-80);
//        hole3.pts[2]=Vertex(140,-80);
//        hole3.pts[3]=Vertex(140,-120);
//        decomp->addHole(hole3);

//        Polygon hole4(4);
//        hole4.pts[0]=Vertex(-95,180);
//        hole4.pts[1]=Vertex(-95,220);
//        hole4.pts[2]=Vertex(-135,220);
//        hole4.pts[3]=Vertex(-135,180);
//        decomp->addHole(hole4);

//        Polygon hole5(4);
//        hole5.pts[0]=Vertex(-185,-150);
//        hole5.pts[1]=Vertex(-185,-110);
//        hole5.pts[2]=Vertex(-225,-110);
//        hole5.pts[3]=Vertex(-225,-150);
//        decomp->addHole(hole5);

//        Polygon hole6(4);
//        hole6.pts[0]=Vertex(-95,-250);
//        hole6.pts[1]=Vertex(-95,-210);
//        hole6.pts[2]=Vertex(-135,-210);
//        hole6.pts[3]=Vertex(-135,-250);
//        decomp->addHole(hole6);


        //        Polygon p3(4);
        //        p3.pts[0] = Vertex(0,0);
        //        p3.pts[1] = Vertex(-5,0);
        //        p3.pts[2] = Vertex(-5,5);
        //        p3.pts[3] = Vertex(0,5);
        //        decomp->addProposition(p3);
        //whole for wall1 scene 2
                Polygon hole1(4);
                hole1.pts[0]=Vertex(-25,-5);
                hole1.pts[1]=Vertex(-25,-35);
                hole1.pts[2]=Vertex(-285,-35);
                hole1.pts[3]=Vertex(-285,-5);
                decomp->addHole(hole1);
        //whole for wall2

                Polygon hole2(4);
                hole2.pts[0]=Vertex(-25,140);
                hole2.pts[1]=Vertex(-25,0);
                hole2.pts[2]=Vertex(-55,0);
                hole2.pts[3]=Vertex(-55,140);
                decomp->addHole(hole2);

                Polygon hole3(4);
                hole3.pts[0]=Vertex(180,-120);
                hole3.pts[1]=Vertex(180,-80);
                hole3.pts[2]=Vertex(140,-80);
                hole3.pts[3]=Vertex(140,-120);
                decomp->addHole(hole3);

        //        Polygon hole4(4);
        //        hole4.pts[0]=Vertex(-95,180);
        //        hole4.pts[1]=Vertex(-95,220);
        //        hole4.pts[2]=Vertex(-135,220);
        //        hole4.pts[3]=Vertex(-135,180);
        //        decomp->addHole(hole4);

                Polygon hole5(4);
                hole5.pts[0]=Vertex(-185,-150);
                hole5.pts[1]=Vertex(-185,-110);
                hole5.pts[2]=Vertex(-225,-110);
                hole5.pts[3]=Vertex(-225,-150);
                decomp->addHole(hole5);

                Polygon hole6(4);
                hole6.pts[0]=Vertex(-95,-250);
                hole6.pts[1]=Vertex(-95,-210);
                hole6.pts[2]=Vertex(-135,-210);
                hole6.pts[3]=Vertex(-135,-250);
                decomp->addHole(hole6);

}
*/
