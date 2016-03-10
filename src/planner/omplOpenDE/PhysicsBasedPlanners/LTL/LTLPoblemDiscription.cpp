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

#include "LTLPoblemDiscription.h"

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



////////////////////LTLProblemSetup//////////////////////

LTLProblemSetup::LTLProblemSetup(oc::PropositionalTriangularDecomposition* decomp, WorkSpace* ws)
{
   PropositionsAndHoles(decomp);
   PropsCouunt=3;
   sequence.push_back(0);
   sequence.push_back(1);
   sequence.push_back(2);

   std::cout<<"sequence size "<<sequence.size()<<std::endl;
   TypeOfAutomaton.push_back("SequenceAutomaton");
   //TypeOfAutomaton.push_back("DisjunctionAutomaton");

}
void LTLProblemSetup::PropositionsAndHoles(oc::PropositionalTriangularDecomposition* decomp)
{

    Polygon p0(4);
    p0.pts[0] = Vertex(135,115);
    p0.pts[1] = Vertex(135,155);
    p0.pts[2] = Vertex(95,155);
    p0.pts[3] = Vertex(95,115);
    decomp->addProposition(p0);

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


//whole for wall1
        Polygon hole1(4);
        hole1.pts[0]=Vertex(130,-5);
        hole1.pts[1]=Vertex(130,-35);
        hole1.pts[2]=Vertex(-130,-35);
        hole1.pts[3]=Vertex(-130,-5);
        decomp->addHole(hole1);
//whole for wall2

        Polygon hole2(4);
        hole2.pts[0]=Vertex(130,80);
        hole2.pts[1]=Vertex(160,80);
        hole2.pts[2]=Vertex(160,220);
        hole2.pts[3]=Vertex(130,220);
        decomp->addHole(hole2);

        Polygon hole3(4);
        hole3.pts[0]=Vertex(180,-120);
        hole3.pts[1]=Vertex(180,-80);
        hole3.pts[2]=Vertex(140,-80);
        hole3.pts[3]=Vertex(140,-120);
        decomp->addHole(hole3);

        Polygon hole4(4);
        hole4.pts[0]=Vertex(-95,180);
        hole4.pts[1]=Vertex(-95,220);
        hole4.pts[2]=Vertex(-135,220);
        hole4.pts[3]=Vertex(-135,180);
        decomp->addHole(hole4);

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




//    Polygon p0(4);
//    p0.pts[0] = Vertex(-260,20);
//    p0.pts[1] = Vertex(-260,30);
//    p0.pts[2] = Vertex(-270,30);
//    p0.pts[3] = Vertex(-270,20);
//    decomp->addProposition(p0);

//   Polygon p1(4);
//   p1.pts[0] = Vertex(-170,-150);
//   p1.pts[1] = Vertex(-190,-150);
//   p1.pts[2] = Vertex(-190,-170);
//   p1.pts[3] = Vertex(-170,-170);
//   decomp->addProposition(p1);

//   Polygon p2(4);
//   p2.pts[0] = Vertex(200,-200);
//   p2.pts[1] = Vertex(220,-200);
//   p2.pts[2] = Vertex(220,-180);
//   p2.pts[3] = Vertex(200,-180);
//   decomp->addProposition(p2);
//   //whole for wall1 scene 2
//   Polygon hole1(4);
//   hole1.pts[0]=Vertex(-25,-5);
//   hole1.pts[1]=Vertex(-25,-35);
//   hole1.pts[2]=Vertex(-285,-35);
//   hole1.pts[3]=Vertex(-285,-5);
//   decomp->addHole(hole1);
//   //whole for wall2

//   Polygon hole2(4);
//   hole2.pts[0]=Vertex(-25,140);
//   hole2.pts[1]=Vertex(-25,0);
//   hole2.pts[2]=Vertex(-55,0);
//   hole2.pts[3]=Vertex(-55,140);
//   decomp->addHole(hole2);

//   Polygon hole3(4);
//   hole3.pts[0]=Vertex(180,-120);
//   hole3.pts[1]=Vertex(180,-80);
//   hole3.pts[2]=Vertex(140,-80);
//   hole3.pts[3]=Vertex(140,-120);
//   decomp->addHole(hole3);

//   Polygon hole5(4);
//   hole5.pts[0]=Vertex(-185,-150);
//   hole5.pts[1]=Vertex(-185,-110);
//   hole5.pts[2]=Vertex(-225,-110);
//   hole5.pts[3]=Vertex(-225,-150);
//   decomp->addHole(hole5);

//   Polygon hole6(4);
//   hole6.pts[0]=Vertex(-95,-250);
//   hole6.pts[1]=Vertex(-95,-210);
//   hole6.pts[2]=Vertex(-135,-210);
//   hole6.pts[3]=Vertex(-135,-250);
//   decomp->addHole(hole6);



}

}

}
