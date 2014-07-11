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

/* Author: Joan Fontanals Martinez, Muhayyuddin  */


#if defined(KAUTHAM_USE_OMPL)
#if defined(KAUTHAM_USE_ODE)
#include "KauthamOpenDEauxiliarclasses.h"

class KauthamODEobject;

using namespace std;


/*class KauthamDEGoalRegion: public ob::GoalRegion
{
    public:

    KauthamDEGoalRegion(const ob::SpaceInformationPtr &si) : ob::GoalRegion(si)
    {
    threshold_ = 0.5;
    }
    //a on esta de Kautham el goal??
    double 	distanceGoalRegion (const State *st) const {
       const double *pos = st->as<oc::OpenDEStateSpace::StateType>()->getBodyPosition(0);
    }
};
*/
namespace Kautham {

namespace omplcplanner{

KauthamDEGoal::KauthamDEGoal(const ob::SpaceInformationPtr &si, WorkSpace *ws, bool a,Sample *goal):ob::GoalRegion(si)
{
    threshold_ = 0.5;
    Kauthamodebodies=smp2KauthamOpenDEState(ws,goal);
    onlyend=a;
}

KauthamDEGoal::~KauthamDEGoal()
{
}

vector<KauthamDEGoal::KauthamODEobject> KauthamDEGoal::smp2KauthamOpenDEState(WorkSpace *wkSpace,Sample *goal)
{

    //Extract the mapped configuration of the sample. It is a vector with as many components as robots.
    //each component has the RobConf of the robot (the SE3 and the Rn configurations)

    //std::vector<RobConf>* smpRobotsConf(new std::vector<RobConf>);


    //std::vector<RobConf>& smpRConf = smp->getMappedConf();

    //smpRobotsConf=&smpRConf;
    //vector<KauthamODEobject> y(new vector<KauthamODEobject>);
    //loop for all the robots
    vector<KauthamODEobject> kauthamob;

    //for(int i=0; i<=(int(wkSpace->robotsCount())-1); i++)
    {
        wkSpace->getRobot(0)->Kinematics(goal->getMappedConf()[0]);
        //_wkSpace->getRobot(i)->Kinematics(smp->getMappedConf().at(0).getSE3());
       // for(int j=0; j<= (wkSpace->getRobot(i)->getNumLinks())-1;j++)
        {

            KauthamODEobject odeob;
            //odeob.objectposition[0] = wkSpace->getRobot(i)->getLink(j)->getElement()->getPosition()[0];
            //odeob.objectposition[1] = wkSpace->getRobot(i)->getLink(j)->getElement()->getPosition()[1];
            //odeob.objectposition[2] = wkSpace->getRobot(i)->getLink(j)->getElement()->getPosition()[2];
            //Kauthamodebodies.insert(pair<int,KauthamODEobject>(i,*(_wkSpace->getRobot(i)->getLink(j)->getElement()->getPosition());
            //odeob.objectorientation[0] = wkSpace->getRobot(i)->getLink(j)->getElement()->getOrientation()[0];
            //odeob.objectorientation[1] = wkSpace->getRobot(i)->getLink(j)->getElement()->getOrientation()[1];
            //odeob.objectorientation[2] = wkSpace->getRobot(i)->getLink(j)->getElement()->getOrientation()[2];
            //odeob.objectorientation[3] = wkSpace->getRobot(i)->getLink(j)->getElement()->getOrientation()[3];

            odeob.objectposition[0] =-100;
            odeob.objectposition[1] = -100;
            odeob.objectposition[2] = 55;
            ////Kauthamodebodies.insert(pair<int,KauthamODEobject>(i,*(_wkSpace->getRobot(i)->getLink(j)->getElement()->getPosition());
            odeob.objectorientation[0] = 0;
           odeob.objectorientation[1] = 0;
           odeob.objectorientation[2] = 1;
           odeob.objectorientation[3] = 0;
            kauthamob.push_back(odeob);
        }

   }
    return kauthamob;

}

double KauthamDEGoal::distanceGoal(const ob::State *st) const
{
    //smp2KauthamOpenDEState(Kauthamodebodies,ws);
    KthReal distance;
    distance=0.0;

    if(!onlyend)
    {
        for(int i=0; i < Kauthamodebodies.size(); i++)
        {
            //const double *pos = st->as<oc::OpenDEStateSpace::StateType>()->getBodyPosition(i);
            const double *pos = st->as<oc::OpenDEStateSpace::StateType>()->getBodyPosition(0);
            const double *vel = st->as<oc::OpenDEStateSpace::StateType>()->getBodyLinearVelocity(0);
            double dx = fabs(Kauthamodebodies[i].objectposition[0] - pos[0]);
            double dy = fabs(Kauthamodebodies[i].objectposition[1] - pos[1]);
            //double dz = fabs(Kauthamodebodies[i].objectposition[2] - pos[2]);
            //double dot = dx * vel[0] + dy * vel[1] + dz *vel[2];
            double dot = dx * vel[0] + dy * vel[1];
            if(dot > 0)
                dot=0;
            else
                dot=sqrt(fabs(dot));

           // distance = sqrt(dx*dx + dy*dy + dz*dz) + dot;
            distance = sqrt(dx*dx + dy*dy) + dot;
        }

    }
    else
    {
       // const double *pos = st->as<oc::OpenDEStateSpace::StateType>()->getBodyPosition(Kauthamodebodies.size()-1);
       // distance = distance + (fabs(pos[0]-Kauthamodebodies[Kauthamodebodies.size()-1].objectposition[0]))+(fabs(pos[1]-Kauthamodebodies[Kauthamodebodies.size()-1].objectposition[1]))+(fabs(pos[2]-Kauthamodebodies[Kauthamodebodies.size()-1].objectposition[2]));
    }
    return distance;

//    if(!onlyend)
//    {
//        for (map<int,KauthamODEobject>::iterator it=Kauthamodebodies.begin(); it!=Kauthamodebodies.end(); ++it)
//        {
//            const double *pos = st->as<oc::OpenDEStateSpace::StateType>()->getBodyPosition(it->first);
//            distance = distance + (fabs(pos[0]-Kauthamodebodies[it->first].objectposition[0]))+(fabs(pos[1]-Kauthamodebodies[it->first].objectposition[1]))+(fabs(pos[2]-Kauthamodebodies[it->first].objectposition[2]));
//        }

//    }
//    else
//    {
//        const double *pos = st->as<oc::OpenDEStateSpace::StateType>()->getBodyPosition(Kauthamodebodies.size()-1);
//        distance = distance + (fabs(pos[0]-Kauthamodebodies[Kauthamodebodies.size()-1].objectposition[0]))+(fabs(pos[1]-Kauthamodebodies[Kauthamodebodies.size()-1].objectposition[1]))+(fabs(pos[2]-Kauthamodebodies[Kauthamodebodies.size()-1].objectposition[2]));
//    }
//    return distance;
}

KauthamDEStateProjectionEvaluator::KauthamDEStateProjectionEvaluator(const ob::StateSpace *space) : ob::ProjectionEvaluator(space)
{
}

unsigned int KauthamDEStateProjectionEvaluator::getDimension(void) const
{
    return 2;
    //return 3;
}
void KauthamDEStateProjectionEvaluator :: defaultCellSizes(void)
   {
    cellSizes_.resize(2);
   //cellSizes_.resize(3);
   cellSizes_[0] = 1.0;
   cellSizes_[1] = 1.0;
   //cellSizes_[2] = 1.0;

   }

void KauthamDEStateProjectionEvaluator::project(const ob::State *state, ob::EuclideanProjection &projection) const
{
   const double *pos = state->as<oc::OpenDEStateSpace::StateType>()->getBodyPosition(0);
    projection[0] = pos[0];
    projection[1] = pos[1];
 //   projection[2] = pos[2];

}

 KauthamDEStateSpace::KauthamDEStateSpace(const oc::OpenDEEnvironmentPtr &env) : oc::OpenDEStateSpace(env)
 {
 }
 KauthamDEStateSpace::~KauthamDEStateSpace()
 {
 }

 double KauthamDEStateSpace::distance(const ob::State *s1, const ob::State *s2) const
 {
     double distance = 0;

     //for (int i=0; i <= (((KauthamDEEnvironment*) env_.get())->getNumLinksFirstRobot()-1); i++)
     //for (int i=0; i <= (env_->getNumLinksFirstRobot()-1); i++)
     //{
         const double *p1 = s1->as<oc::OpenDEStateSpace::StateType>()->getBodyPosition(0);
         const double *p2 = s2->as<oc::OpenDEStateSpace::StateType>()->getBodyPosition(0);
         double dx = fabs(p1[0]-p2[0]);
         double dy = fabs(p1[1]-p2[1]);
     //  double dz = fabs(p1[2]-p2[2]);
         //distance = distance + fabs(p1[0]-p2[0])+fabs(p1[1]-p2[1])+fabs(p1[2]-p2[2]);
     //}
         distance =sqrt(dx*dx + dy*dy);// + dz*dz);
     return distance;

 }
 void KauthamDEStateSpace::registerProjections(void)
 {
     registerDefaultProjection(ob::ProjectionEvaluatorPtr(new KauthamDEStateProjectionEvaluator(this)));
 }

 /*KauthamDEObjectControlSampler::KauthamDEObjectControlSampler(const oc::ControlSpace *cm) : oc::RealVectorControlUniformSampler(cm)
    {
    }

    void KauthamDEObjectControlSampler :: sampleNext(oc::Control *control, const oc::Control *previous)
    {
    space_->copyControl(control, previous);
    const ob::RealVectorBounds &b = space_->as<oc::OpenDEControlSpace>()->getBounds();
    if (rng_.uniform01() > 0.3)
    {
        double &v = control->as<oc::OpenDEControlSpace::ControlType>()->values[0];
        static const double DT0 = 0.05;
        v += (rng_.uniformBool() ? 1 : -1) * DT0;
        if (v > b.high[0])
        v = b.high[0] - DT0;
        if (v < b.low[0])
        v = b.low[0] + DT0;
    }
    if (rng_.uniform01() > 0.3)
    {
        double &v = control->as<oc::OpenDEControlSpace::ControlType>()->values[1];
        static const double DT1 = 0.05;
        v += (rng_.uniformBool() ? 1 : -1) * DT1;
        if (v > b.high[1])
        v = b.high[1] - DT1;
        if (v < b.low[1])
        v = b.low[1] + DT1;
    }
    }

   void KauthamDEObjectControlSampler :: sampleNext(oc::Control *control, const oc::Control *previous, const ob::State* /*state*///)
  //{
 //   sampleNext(control, previous);
 // }

 }

}



#endif//KAUTHAM_USE_ODE
#endif// KAUTHAM_USE_OMPL
