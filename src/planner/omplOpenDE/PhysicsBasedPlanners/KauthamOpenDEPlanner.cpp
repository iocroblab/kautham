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

/* Author: Joan Fontanals Martinez, Muhayyuddin */

#if defined(KAUTHAM_USE_OMPL)
#if defined(KAUTHAM_USE_ODE)

#include"KauthamOpenDEPlanner.h"
#include "planner/omplg/omplplanner.h"
#include "planner/omplc/omplcplanner.h"
#include <Inventor/nodes/SoTransform.h>
#include <Inventor/nodes/SoCoordinate3.h>
#include <Inventor/nodes/SoCube.h>
#include <Inventor/nodes/SoPointSet.h>
#include <Inventor/nodes/SoLineSet.h>
#include <Inventor/nodes/SoDrawStyle.h>
#include <Inventor/nodes/SoMaterial.h>

#include <ompl/tools/benchmark/Benchmark.h>
#include <ompl/control/planners/est/EST.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/syclop/SyclopRRT.h>
#include <ompl/control/planners/syclop/SyclopEST.h>

#include <ompl/control/planners/kpiece/KPIECE1.h>
#include <ompl/control/planners/syclop/GridDecomposition.h>
#include <ctime>
class KauthamDEGoal;

namespace Kautham {

namespace omplcplanner{
//Igual que omplcplanner es la base per a tots els planners de control com omplcrrtplanner , etc.. KauthamOpenDEPlanner té com a objectiu servir de base per a tots els planners que derivessin i que usessin la simulació dinàmica) Sobretot ha de reimplementar el mètode trysolve basantse en la demo d'OMPL OpenDERigidBodyPlanning.
// Like omplcplanner is the basis for all planners as omplcrrtplanner control, etc. .. KauthamOpenDEPlanner aims to serve as a basis for deriving all the planners and would use dynamic simulation) Especially should reimplement the method trysolve On the basis of the demo fill OpenDERigidBodyPlanning.
/*! Constructor define all the necessary parameters for planning in dynamic enviroment.
 *  It define the pointer to the dynamic enviroment and pointer to the start space.
 */
KauthamDEPlanner::KauthamDEPlanner(SPACETYPE stype, Sample *init, Sample *goal, SampleSet *samples, WorkSpace *ws) : Planner(stype, init, goal, samples, ws)
{
    ws->moveRobotsTo(init);
    //set intial values from parent class data
    _speedFactor = 0.5;
    _solved = false;
    _maxspeed = 5;
    _onlyend = false;
    _planningTime=60;
    _propagationStepSize=0.07;
    _maxContacts=3;
    _minControlSteps=10;
    _maxControlSteps=50;
    _controlDimensions=2;
    _erp=0.5;
    _cfm=0.3;
    addParameter("PropagationStepSize", _propagationStepSize);
    addParameter("Max Planning Time", _planningTime);
    addParameter("Speed Factor", _speedFactor);
    addParameter("Max Speed", _maxspeed);
    addParameter("only final link position?", _onlyend);
    addParameter("Min Control Steps", _minControlSteps);
    addParameter("Max Control Steps", _maxControlSteps);
    addParameter("Max Contacts", _maxContacts);
    addParameter("Control Dimensions", _controlDimensions);
    addParameter("Error Reduction Parameter", _erp);
    addParameter("Constraint Force Mixing", _cfm);

    //PROBTYPE="LTL";
    //PROBTYPE="MULTIQUERY";
    PROBTYPE="SINGLEQUERY";

}

//! void destructor
KauthamDEPlanner::~KauthamDEPlanner(){ }

/*! this is the main function that compute the path in dynamic enviroment.
   * where the control will be applied by the ODE.If it found the siolution it returns
   * true otherwise false.
   */
bool KauthamDEPlanner::trySolve(void)
{
    dInitODE2(0);
    std::vector<query> Qu;

    query qq;
    qq.action="pull";
    qq.pose.resize(2);
    qq.pose[0]=195.5;
    qq.pose[1]=55.8;
    qq.f.resize(3);
    qq.f[0]=0.0;
    qq.f[1]=5.0;
    qq.f[2]=0.0;
    qq.targetbody=6;
    Qu.push_back(qq);

    query qq1;
    qq1.action="move";
    qq1.pose.resize(2);
    qq1.pose[0]=171.5;
    qq1.pose[1]=-75.8;
    qq1.f.resize(3);
    qq1.targetbody=0;
    Qu.push_back(qq1);

    query qqq;
    qqq.action="pull";
    qqq.pose.resize(2);
    qqq.pose[0]=247.9;
    qqq.pose[1]=-75.8;
    qqq.f.resize(3);
    qqq.f[0]=7.0;
    qqq.f[1]=0.0;
    qqq.f[2]=0.0;
    qqq.targetbody=7;

    Qu.push_back(qqq);

    ob::RealVectorBounds vb(3);
    ob::RealVectorBounds bounds(3);
    vb.low[0] = _wkSpace->getRobot(0)->getLimits(0)[0];
    vb.low[1] = _wkSpace->getRobot(0)->getLimits(1)[0];
    vb.low[2] = _wkSpace->getRobot(0)->getLimits(2)[0];
    vb.high[0] = _wkSpace->getRobot(0)->getLimits(0)[1];
    vb.high[1] = _wkSpace->getRobot(0)->getLimits(1)[1];
    vb.high[2] = _wkSpace->getRobot(0)->getLimits(2)[1];
    ss->setVolumeBounds(vb);
    bounds.setLow(-20);
    bounds.setHigh(20);
    ss->setLinearVelocityBounds(bounds);
    bounds.setLow(-20);
    bounds.setHigh(20);
    ss->setAngularVelocityBounds(bounds);
    ob::ScopedState<oc::OpenDEStateSpace> final(ss->getSpaceInformation());

    if(PROBTYPE=="SINGLEQUERY")
    {
        Sample* aux=goalSamp();
        ss->setStartState(ss->getCurrentState());
        ob::ScopedState<oc::OpenDEStateSpace> starts(ss->getSpaceInformation());
        starts = ss->getCurrentState();
        ss->setGoal(ob::GoalPtr(new KauthamDEGoalSamplableRegion(ss->getSpaceInformation(),_wkSpace,_onlyend,aux)));//goalPose[0].pose[0],goalPose[0].pose[1])));
        ss->setup();
        ss->print();
        bool solution1=false;
        _solved=solution1 = ss->solve(_planningTime);
        final = ss->getSolutionPath().getStates().back();

        if(solution1)
        {

            std::cout<<"1st Query is solved"<<std::endl;
            solutionStates sstat;
            std::vector<ob::State*> &States0 =ss->getSolutionPath().getStates();
            std::vector<oc::Control*> &Control0= ss->getSolutionPath().getControls();
            std::vector<double> &Duration0= ss->getSolutionPath().getControlDurations();

            sstat.substates =States0;
            sstat.control =Control0;
            sstat.duration =Duration0;
            sStates.push_back(sstat);
        }
    }
    if(PROBTYPE=="MULTIQUERY")
    {
        ss->setStartState(ss->getCurrentState());
        ob::ScopedState<oc::OpenDEStateSpace> last(ss->getSpaceInformation());
        //last = ss1->getSolutionPath().getStates().back();

        for(unsigned int numQ=0; numQ<Qu.size();numQ++)
        {
            oc::OpenDESimpleSetup *ss1= new oc::OpenDESimpleSetup(stateSpacePtr);
            ss1->setVolumeBounds(vb);
            ss1->setAngularVelocityBounds(bounds);
            ss1->setLinearVelocityBounds(bounds);
            ss1->setStartState(last);
            ss1->setGoal(ob::GoalPtr(new KauthamDEGoalRegion(ss->getSpaceInformation(),_wkSpace,_onlyend,Qu[numQ].pose[0],Qu[numQ].pose[1])));
            ((KauthamDEEnvironment*)envPtr.get())->manipulationQuery->setActionType(Qu[numQ].action);

            ((KauthamDEEnvironment*)envPtr.get())->manipulationQuery->setforce(Qu[numQ].f);
            dJointID joint1;
            if(Qu[numQ].action=="pull" || Qu[numQ].action=="Pull")
            {
                joint1=dJointCreateHinge( ((KauthamDEEnvironment*)envPtr.get())->world_,0);
                dJointAttach(joint1 ,((KauthamDEEnvironment*)envPtr.get())->stateBodies_[0],((KauthamDEEnvironment*)envPtr.get())->stateBodies_[Qu[numQ].targetbody]);
            }
            ss1->setup();
            ss1->print();
            bool sol;
            sol=ss1->solve(_planningTime);
            if(Qu[numQ].action=="pull" || Qu[numQ].action=="Pull")
                dJointDestroy(joint1);
            if(sol)
            {
                std::cout<<"2nd Query is solved"<<std::endl;
                solutionStates sstat1;
                std::vector<ob::State*> &States =ss1->getSolutionPath().getStates();
                std::vector<oc::Control*> &Control= ss1->getSolutionPath().getControls();
                std::vector<double> &Duration= ss1->getSolutionPath().getControlDurations();

                sstat1.substates =States;
                sstat1.control =Control;
                sstat1.duration =Duration;
                sStates.push_back(sstat1);

                last = ss1->getSolutionPath().getStates().back();
            }
            else
            {
                break;
            }
        }
    }

    if (_solved)
    {
        _path.clear();
        clearSimulationPath();
        Sample *Robsmp;
        Sample *Obssmp;
        for(unsigned int l=0;l<sStates.size();l++)
        {

            std::vector<ob::State*> &states = sStates[l].substates;
            std::vector<oc::Control*> &control= sStates[l].control;
            std::vector<double> &duration=sStates[l].duration;
            std::vector<float> ang;
            int max=_wkSpace->getRobot(0)->getNumLinks()-1;
            if(_wkSpace->getRobot(0)->getNumJoints()>1)
            {
                for(unsigned int i=0;i<states.size()-1;i++)
                {
                    int k=0;
                    ang.clear();
                    std::cout<<"Angles are :";
                    for(int j=0;j<max;j++)
                    {

                        const double *posRobB1 = states[i]->as<oc::OpenDEStateSpace::StateType>()->getBodyPosition(k);
                        const ob::SO3StateSpace::StateType &oriRobB1 = states[i]->as<oc::OpenDEStateSpace::StateType>()->getBodyRotation(k);

                        const double *posRobB2 = states[i]->as<oc::OpenDEStateSpace::StateType>()->getBodyPosition(k+1);
                        const ob::SO3StateSpace::StateType &oriRobB2 = states[i]->as<oc::OpenDEStateSpace::StateType>()->getBodyRotation(k+1);


                        const double *posRobB11 = states[i+1]->as<oc::OpenDEStateSpace::StateType>()->getBodyPosition(k);
                        const ob::SO3StateSpace::StateType &oriRobB11 = states[i+1]->as<oc::OpenDEStateSpace::StateType>()->getBodyRotation(k);

                        const double *posRobB22 = states[i+1]->as<oc::OpenDEStateSpace::StateType>()->getBodyPosition(k+1);
                        const ob::SO3StateSpace::StateType &oriRobB22 = states[i+1]->as<oc::OpenDEStateSpace::StateType>()->getBodyRotation(k+1);

                        mt::Transform TwB1,TwB2;
                        mt::Transform TwB11,TwB22;

                        mt::Transform Twl1l2_i;
                        mt::Transform Twl1l2_i1;

                        TwB1.setTranslation(mt::Point3(posRobB1[0],posRobB1[1],posRobB1[2]));
                        TwB1.setRotation(mt::Rotation(oriRobB1.x,oriRobB1.y,oriRobB1.z,oriRobB1.w));

                        TwB2.setTranslation(mt::Point3(posRobB2[0],posRobB2[1],posRobB2[2]));
                        TwB2.setRotation(mt::Rotation(oriRobB2.x,oriRobB2.y,oriRobB2.z,oriRobB2.w));


                        Twl1l2_i=TwB1.inverse()*TwB2;

                        TwB11.setTranslation(mt::Point3(posRobB11[0],posRobB11[1],posRobB11[2]));
                        TwB11.setRotation(mt::Rotation(oriRobB11.x,oriRobB11.y,oriRobB11.z,oriRobB11.w));

                        TwB22.setTranslation(mt::Point3(posRobB22[0],posRobB22[1],posRobB22[2]));
                        TwB22.setRotation(mt::Rotation(oriRobB22.x,oriRobB22.y,oriRobB22.z,oriRobB22.w));

                        Twl1l2_i1=TwB11.inverse()*TwB22;

                        mt::Transform T_angle;

                        T_angle=Twl1l2_i.inverse()*Twl1l2_i1;

                        mt::Unit3 axis;
                        Scalar angle;
                        T_angle.getRotation().getAxisAngle(axis,angle);
                        ang.push_back(angle);
                        k++;
                        std::cout<< angle<<" , ";

                    }
                    std::cout<<std::endl;
                    JointAngle.push_back(ang);

                }

            }

            //std::cout<<"Control Size is "<< control.size()<<std::endl;
            //std::cout<<"state Size is "<< states.size()<<std::endl;
            //std::cout<<"Duration Size is "<< duration.size()<<std::endl;
            //int size= states.size();
            std::vector<double> Joint_Angle;
            Joint_Angle.resize(_wkSpace->getRobot(0)->getNumJoints());
            //int j=0;
            ComputeAction(states,control,duration);
            ComputeJerkIndex(states,duration);
            ComputePowerConsumed(states,control,duration);

            std::cout<<"===============   Query Numer  "<<(l+1)<<"  =============== "<<std::endl;
            std::cout<<"Actions is:  "<<Action<<std::endl;
            std::cout<<"Smoothness is:  "<<Smoothness<<std::endl;
            std::cout<<"Power Consumed is:  "<<PowerConsumed<<std::endl;
            std::cout<<"Generated Solution states are  :  "<<states.size()<<std::endl;
            std::cout<<"Generated Solution controls are:  "<<control.size()<<std::endl;
            std::cout<<"Solution control durations  are:  "<<duration.size()<<std::endl;
            std::cout<<"last rob state is " <<final->getBodyPosition(0)[0] << " " << final->getBodyPosition(0)[1] << std::endl;

            std::vector<float> jangle;
            jangle.resize(7);
            for(unsigned int i=0;i<states.size()-1;i++)
            {
                //std::cout<<"Duration is "<< duration[i]<<std::endl;
                State tmpstate;
                Robsmp=new Sample(_wkSpace->getNumRobControls());
                //Robsmp->setMappedConf(_wkSpace->getRobot(0));
                if(_wkSpace->getRobot(0)->getNumJoints()>1)
                {
                    for(int j=0;j<7;j++)
                        jangle[j]=jangle[j]+JointAngle[i][j];
                }
                KauthamOpenDEState2Robsmp(states[i], Robsmp,control[0],duration[i],&Joint_Angle,jangle);

                tmpstate.setRob(*Robsmp);
                if(_wkSpace->getNumObstacles()>0)
                {
                    Obssmp=new Sample(_wkSpace->getNumObsControls());
                    KauthamOpenDEState2Obssmp(states[i],Obssmp,control[0],duration[0]);
                    tmpstate.setObs(*Obssmp);
                }
                worldState.push_back(tmpstate);
                _path.push_back(Robsmp);
                _samples->add(Robsmp);

                //std::cout<<std::endl;
            }
        }
        //drawCspace(0);

        return _solved;
    }

    dCloseODE();
    return 0;

}
//    if(PROBTYPE=="LTL")
//    {
//        clock_t startTime = clock();
//        _solved = ltlplanner->as<ob::Planner>()->solve(200.0);
//        clock_t endTime = clock();
//        clock_t clockTicksTaken = endTime - startTime;
//        double timeInSeconds = clockTicksTaken / (double) CLOCKS_PER_SEC;
//        std::cout<<"Planning Time is : "<<timeInSeconds<<std::endl;
//        // pDefp->getLowerSolutionPath()->print(std::cout);

//        const og::PathGeometric  &gPath = dynamic_cast<ompl::control::PathControl*>(pDefp->getLowerSolutionPath().get())->asGeometric();

//        if (_solved)
//        {
//            _path.clear();
//            clearSimulationPath();
//            Sample *Robsmp;
//            Sample *Obssmp;
//            std::vector<double> Joint_Angle;
//            Joint_Angle.resize(_wkSpace->getRobot(0)->getNumJoints());
//            double control[2]={2.0,0.0};
//            oc::Control *c = ss->getSpaceInformation()->allocControl();
//            memcpy(c->as<oc::OpenDEControlSpace::ControlType>()->values, control, sizeof(double) * ss->getControlSpace()->getDimension());
//            std::vector<float> jangle;
//            jangle.resize(7);
//            for(unsigned int i=0;i<gPath.getStateCount()-2;i=i+2)
//            {
//                State tmpstate;
//                Robsmp=new Sample(_wkSpace->getNumRobControls());
//                //Robsmp->setMappedConf(_init->getMappedConf());
//                KauthamOpenDEState2Robsmp(gPath.getState(i), Robsmp,c,2,&Joint_Angle,jangle);

//                tmpstate.setRob(*Robsmp);
//                if(_wkSpace->getNumObstacles()>0)
//                {
//                    Obssmp=new Sample(_wkSpace->getNumObsControls());
//                    KauthamOpenDEState2Obssmp(gPath.getState(i),Obssmp,c,2);
//                    tmpstate.setObs(*Obssmp);
//                }
//                worldState.push_back(tmpstate);
//                _path.push_back(Robsmp);
//                _samples->add(Robsmp);
//            }
//        }
//        // drawCspace(0);
//        return _solved;
//    }

//dCloseODE();
//return 0;

//}

//! This member function converts an OpenDE State to a Kautham robot sample
void KauthamDEPlanner::KauthamOpenDEState2Robsmp(const ob::State *state, Sample* smp, const oc::Control *control,const double duration,std::vector<double> *angle,std::vector<float> Ang)
{
    int k=0;
    vector<RobConf> rc;
    double low;
    double high;
    for(unsigned int i=0; i<_wkSpace->getNumRobots(); i++)
    {
        RobConf *rcj = new RobConf;
        if(_wkSpace->getRobot(0)->getNumLinks()<2)
        {
            for(unsigned int j=0;j<_wkSpace->getRobot(i)->getNumLinks();j++)
            {
                //const double *pos = states[i]->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1)->values;
                const double *posRob = state->as<oc::OpenDEStateSpace::StateType>()->getBodyPosition(k);
                const ob::SO3StateSpace::StateType &oriRob = state->as<oc::OpenDEStateSpace::StateType>()->getBodyRotation(k);
                // make a pointer to openDERObot Enviroment and get the value of joint

                //RobConf to store the robots configurations read form the ompl state
                RobConf *rcj = new RobConf;
                //convert to a vector of 7 components
                vector<KthReal> se3coords;
                se3coords.resize(7);
                se3coords[0] = posRob[0];
                se3coords[1] = posRob[1];
                se3coords[2] = posRob[2];
                se3coords[3] = oriRob.x;
                se3coords[4] = oriRob.y;
                se3coords[5] = oriRob.z;
                se3coords[6] = oriRob.w;

                //create the sample
                SE3Conf se3;
                se3.setCoordinates(se3coords);
                rcj->setSE3(se3);
                rc.push_back(*rcj);
                k++;
                //std::cout<<"Position of Robot "<< i <<" Link "<<j<<" = ["<<se3coords[0]<<" , "<<se3coords[1]<<" , "<<se3coords[2]<< " ]  ["<<se3coords[3]<<" , "<<se3coords[4]<<" , "<<se3coords[5]<< " , "<<se3coords[6]<<" ] "<<std::endl;
            }
        }

        if(_wkSpace->getRobot(0)->getNumJoints()>1)
        {

            //=================================================================================================


            const double *posRob = state->as<oc::OpenDEStateSpace::StateType>()->getBodyPosition(_wkSpace->getRobot(0)->getNumJoints()-1);

            std::cout<<"Position of TCP:  "<<posRob[0]<<" , "<<posRob[1]<<" , "<<posRob[2]<<std::endl;
            //=================================================================================================
            vector<KthReal> coords;
            std::vector<double> ang;
            ang = *angle;
            //std::cout<<"duration:  "<<duration<<std::endl;
            const double* vv= control->as<oc::OpenDEControlSpace::ControlType>()->values;
            //std::cout<<"Controls are:  "<<vv[0]<<" , "<<vv[1]<<" , "<<vv[2]<<" , "<<vv[3]<<" , "<<vv[4]<<" , "<<vv[5]<<" , "<<vv[6]<<std::endl;
            //std::cout<<"Normalize Value:  ";
            for(unsigned int j=0;j<_wkSpace->getRobot(0)->getNumJoints();j++)
            {
                ang[j]=ang[j]+(vv[j]*duration);
                low = *_wkSpace->getRobot(i)->getLink(j+1)->getLimits(true);//lower limit of joint
                high = *_wkSpace->getRobot(i)->getLink(j+1)->getLimits(false);//upper limit of joint

                double normalize = (Ang[j]-low)/(high-low);
                coords.push_back(normalize);
                // std::cout<<" "<<normalize<<" , ";
                //std::cout<<"Limit for joint one:  "<<low<<", "<<high<<std::endl;
            }
            std::cout<<std::endl;
            rcj->setRn(coords);
            rc.push_back(*rcj);
            *angle = ang;
            // std::cout<<"New Values of Angles are:  "<<ang[0]<<", "<<ang[1]<<", "<<ang[2]<<", "<<ang[3]<<", "<<ang[4]<<", "<<ang[5]<<", "<<ang[6]<<std::endl;
        }
    }
    smp->setMappedConf(rc);
}

//! This member function converts an OpenDE State to a Kautham obstacle sample
void KauthamDEPlanner::KauthamOpenDEState2Obssmp(const ob::State *state, Sample* smp,const oc::Control *control,const double duration)
{
    int k=0;
    for(unsigned int i=0; i<_wkSpace->getNumRobots(); i++)
    {
        k = k + _wkSpace->getRobot(i)->getNumLinks();
    }
    vector<RobConf> rc;
    for(unsigned int i=0; i<_wkSpace->getNumObstacles(); i++)
    {
        //const double *pos = states[i]->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1)->values;
        const double *posObs = state->as<oc::OpenDEStateSpace::StateType>()->getBodyPosition(k);
        const ob::SO3StateSpace::StateType &oriObs = state->as<oc::OpenDEStateSpace::StateType>()->getBodyRotation(k);
        RobConf *rcj = new RobConf;
        //convert to a vector of 7 components
        vector<KthReal> se3coords;
        se3coords.resize(7);
        se3coords[0] = posObs[0];
        se3coords[1] = posObs[1];
        se3coords[2] = posObs[2];
        se3coords[3] = oriObs.x;
        se3coords[4] = oriObs.y;
        se3coords[5] = oriObs.z;
        se3coords[6] = oriObs.w;
        //create the sample
        SE3Conf se3;
        se3.setCoordinates(se3coords);
        rcj->setSE3(se3);
        rc.push_back(*rcj);
        k++;
        //std::cout<<"Position of Obstacle "<< i <<" = "<<posObs[0]<<" , "<<posObs[1]<<" , "<<posObs[2]<<std::endl;

    }
    smp->setMappedConf(rc);
}
//computePath fuunction will take the goal position and planning time
// as arguments and return the result of ompl solve function abter solving the qury
bool KauthamDEPlanner::computePath(oc::OpenDESimpleSetup *ssetup, ob::RealVectorBounds vb,ob::RealVectorBounds bounds, double x, double y,double planningTime)
{
    ob::ScopedState<oc::OpenDEStateSpace> startState(ssetup->getSpaceInformation());
    startState = ssetup->getSolutionPath().getStates().back();
    //ssetup->clear();
    ssetup->setVolumeBounds(vb);
    ssetup->setAngularVelocityBounds(bounds);
    ssetup->setLinearVelocityBounds(bounds);
    ssetup->setStartState(startState);
    ssetup->setGoal(ob::GoalPtr(new KauthamDEGoalRegion(ssetup->getSpaceInformation(),_wkSpace,_onlyend,x,y)));

    ssetup->setup();
    ssetup->print();

    bool solve;
    solve = ssetup->solve(planningTime);

    if(solve)
    {
        solutionStates state;
        std::vector<ob::State*> &States =ssetup->getSolutionPath().getStates();
        std::vector<oc::Control*> &Control= ssetup->getSolutionPath().getControls();
        std::vector<double> &Duration= ssetup->getSolutionPath().getControlDurations();

        state.substates =States;
        state.control =Control;
        state.duration =Duration;

        sStates.push_back(state);
    }
    return solve;
}

//This function compute the ACTION that is a dynamical attribute of physical system it takes trajectory or path as argument and
//return a number, it has units Newton-meter-second. computed as sum(force*distance*time)
void KauthamDEPlanner::ComputeAction(const std::vector<ob::State*> &states, const std::vector<oc::Control*> &control, const std::vector<double> duration)
{
    int robIndex=0;
    Action = 0;
    for(unsigned int i=0;i<states.size()-1;i++)
    {
        const double* pos1 = states[i]->as<oc::OpenDEStateSpace::StateType>()->getBodyPosition(robIndex);
        const double* pos2 = states[i+1]->as<oc::OpenDEStateSpace::StateType>()->getBodyPosition(robIndex);
        const double* vv= control[i]->as<oc::OpenDEControlSpace::ControlType>()->values;
        double distance =sqrt(pow((pos2[0]-pos1[0]),2) + pow((pos2[1]-pos1[1]),2));// +(pos2[2]-pos1[2])^2);
        double force= sqrt((vv[0]*vv[0])+(vv[1]*vv[1]));
        //in Kautham distance is represented in mm, the final distance of the trajectory
        //will be converted in meters to get the correct units of action (newton*meter*second)
        distance = distance/1000;
        Action = Action + (force * distance * duration[i]);
    }
}
//This function compute the JerkIndex that is basically the derivative of aceleration (third derivative of position)
//it describe the degree of smoothness of a path its unit is meter/second^3.
void KauthamDEPlanner::ComputeJerkIndex(const std::vector<ob::State*> &states, const std::vector<double> duration)
{
    int robIndex=0;
    const double* pos;
    const double* pos2;
    const double* pos1;
    const double* pos3;

    double  rn;
    double  rn1;
    double  rn2;
    double  rn3;
    JerkIndex.clear();
    Smoothness=0;
    if(states.size()>4)
    {
        for(unsigned int n=0;n<states.size()-4;n++)
        {
            pos  = states[n]->as<oc::OpenDEStateSpace::StateType>()->getBodyPosition(robIndex);
            pos1 = states[n+1]->as<oc::OpenDEStateSpace::StateType>()->getBodyPosition(robIndex);
            pos2 = states[n+2]->as<oc::OpenDEStateSpace::StateType>()->getBodyPosition(robIndex);
            pos3 = states[n+3]->as<oc::OpenDEStateSpace::StateType>()->getBodyPosition(robIndex);

            rn  = sqrt(pos[0]*pos[0] + pos[1]*pos[1])/1000;
            rn1 = sqrt(pos1[0]*pos1[0] + pos1[1]*pos1[1])/1000;
            rn2 = sqrt(pos2[0]*pos2[0] + pos2[1]*pos2[1])/1000;
            rn3 = sqrt(pos3[0]*pos3[0] + pos3[1]*pos3[1])/1000;
            //4 points forward difference method to compute the 3rd derivative of position i.e. jerk
            JerkIndex.push_back((-rn + 3*rn1 - 3*rn2 + rn3)/(duration[n]*duration[n+1]*duration[n+2]));
        }
        for(unsigned int i=0;i<duration.size();i++)
        {
            //std::cout<<"Jerk Index: "<<JerkIndex[i]<<"  Smoothness: "<<Smoothness<<std::endl;
            Smoothness = Smoothness+(duration[i]*(pow(JerkIndex[i+1],2)+pow(JerkIndex[i],2)));
        }
        Smoothness = Smoothness/2;
    }
}

void KauthamDEPlanner::ComputePowerConsumed(const std::vector<ob::State*> &states,const std::vector<oc::Control*> &control, const std::vector<double> duration)
{
    int robIndex=0;
    PowerConsumed = 0;
    double time = duration[states.size()-1];
    for(unsigned int i=0;i<states.size()-1;i++)
    {
        const double* pos1 = states[i]->as<oc::OpenDEStateSpace::StateType>()->getBodyPosition(robIndex);
        const double* pos2 = states[i+1]->as<oc::OpenDEStateSpace::StateType>()->getBodyPosition(robIndex);
        const double* vv= control[i]->as<oc::OpenDEControlSpace::ControlType>()->values;
        double distance =sqrt(pow((pos2[0]-pos1[0]),2) + pow((pos2[1]-pos1[1]),2));// +(pos2[2]-pos1[2])^2);
        double force= sqrt((vv[0]*vv[0])+(vv[1]*vv[1]));
        time=time+duration[i];
        //in Kautham distance is represented in mm, the final distance of the trajectory
        //will be converted in meters to get the correct units of action (newton*meter*second)
        // std::cout<<"DISTANCE IN MM =  "<<distance<<std::endl;
        distance = distance;//1000;
        //        std::cout<<"DISTANCE IN M  =  "<<distance<<std::endl;
        //        std::cout<<"Force is       =  "<<force<<std::endl;
        //        std::cout<<"duration is    =  "<<duration[i]<<std::endl;
        //        std::cout<<"Power is       =  "<<PowerConsumed<<std::endl;
        //            std::cout<<"State: [ "<<pos1[0]<<" , "<<pos1[1];
        //            std::cout<<" ]  Control [ "<<vv[0]<<" , "<<vv[1];
        //            std::cout<<" ]  distance = "<<distance;
        //            std::cout<<" duration = "<<duration[i];
        //            std::cout<<" force = "<<force<<std::endl;

        PowerConsumed = PowerConsumed + (force * distance);
    }
    PowerConsumed = PowerConsumed/time;
    std::cout<<"total power: "<<PowerConsumed<<std::endl;
}

//! setParameter function set the planning parameters for the planners
bool KauthamDEPlanner::setParameters()
{
    try{
        HASH_S_K::iterator it = _parameters.find("Speed Factor");
        if(it != _parameters.end())
            _speedFactor = it->second;
        else
            return false;

        it = _parameters.find("Max Planning Time");
        if(it != _parameters.end())
            _planningTime = it->second;
        else
            return false;

        it = _parameters.find("PropagationStepSize");
        if(it != _parameters.end()){
            _propagationStepSize = it->second;
            ss->getSpaceInformation()->setPropagationStepSize(_propagationStepSize);
        }
        else
            return false;

        it = _parameters.find("Max Speed motors");
        if(it != _parameters.end())
            _maxspeed = it->second;
        else
            return false;

        it = _parameters.find("only final link position?");
        if(it != _parameters.end())
            _onlyend = it->second;
        else
            return false;
        it = _parameters.find("Max Contacts");
        if(it != _parameters.end())
            _maxContacts = it->second;
        else
            return false;
        it = _parameters.find("Min Control Steps");
        if(it != _parameters.end())
            _minControlSteps = it->second;
        else
            return false;
        it = _parameters.find("Max Control Steps");
        if(it != _parameters.end())
            _maxControlSteps = it->second;
        else
            return false;
        it = _parameters.find("Error Reduction Parameter");
        if(it != _parameters.end())
            _erp = it->second;
        else
            return false;
        it = _parameters.find("Constraint Force Mixing");
        if(it != _parameters.end())
            _cfm = it->second;
        else
            return false;

    }catch(...){
        return false;
    }
    return true;

}
vector<KauthamDEPlanner::KauthamDEobject> KauthamDEPlanner::smp2KauthamOpenDEState(WorkSpace *wkSpace, Sample *goal)
{
    vector<KauthamDEobject> kauthamob;
    for(unsigned int i=0; i<(unsigned int)wkSpace->getNumRobots(); i++)
    {
        //wkSpace->getRobot(0)->Kinematics(goal->getMappedConf()[0]);
        wkSpace->getRobot(i)->Kinematics(goal->getMappedConf().at(0).getSE3());
        for(unsigned int j=0; j<wkSpace->getRobot(i)->getNumLinks();j++)
        {

            KauthamDEobject odeob;
            odeob.objectposition[0] = wkSpace->getRobot(i)->getLink(j)->getElement()->getPosition()[0];
            odeob.objectposition[1] = wkSpace->getRobot(i)->getLink(j)->getElement()->getPosition()[1];
            odeob.objectposition[2] = wkSpace->getRobot(i)->getLink(j)->getElement()->getPosition()[2];
            //Kauthamodebodies.insert(pair<int,KauthamODEobject>(i,*(wkSpace->getRobot(i)->getLink(j)->getElement()->getPosition()));
            odeob.objectorientation[0] = wkSpace->getRobot(i)->getLink(j)->getElement()->getOrientation()[0];
            odeob.objectorientation[1] = wkSpace->getRobot(i)->getLink(j)->getElement()->getOrientation()[1];
            odeob.objectorientation[2] = wkSpace->getRobot(i)->getLink(j)->getElement()->getOrientation()[2];
            odeob.objectorientation[3] = wkSpace->getRobot(i)->getLink(j)->getElement()->getOrientation()[3];
            kauthamob.push_back(odeob);
        }

    }
    return kauthamob;

}

void KauthamDEPlanner::moveAlongPath(unsigned int step){
    if(_solved){

        if(_simulationPath.size() == 0 ){ // Then calculate the simulation path based on stepsize
            //unsigned int d =  _path[0]->getDim() ;
            for(unsigned int i=0; i<_path.size();i++)
            {
                Sample *s=new Sample(*_path.at(i));
                _simulationPath.push_back(s);

            }
        }
        if( _simulationPath.size() >= 2 ){
            step = step % _simulationPath.size();
            _wkSpace->moveRobotsTo(worldState[step].getRob());
            if(_wkSpace->getNumObstacles()>0)
            {
                _wkSpace->moveObstaclesTo(worldState[step].getObs());
            }

        }else
            std::cout << "The problem is wrong solved. The solution path has less than two elements." << std::endl;
    }
}

oc::PathControl* KauthamDEPlanner::RectMotion()
{

    oc::PathControl *p = new oc::PathControl(ss->getSpaceInformation());
    double control[2]={2.0,0.0};
    oc::Control *c = ss->getSpaceInformation()->allocControl();
    memcpy(c->as<oc::OpenDEControlSpace::ControlType>()->values, control, sizeof(double) * ss->getControlSpace()->getDimension());
    //       for(int i=0;i<10;i++)
    //       {
    //       ob::State *s0 = ss->getSpaceInformation()->allocState();
    //       ss->getStateSpace()->as<oc::OpenDEStateSpace>()->writeState(s0);
    //       if(i==0)
    //       p->getStates().push_back(s0);

    //       ob::State *s1 = ss->getSpaceInformation()->allocState();
    //       ss->getSpaceInformation()->propagate(s0, c, 2, s1);
    //       p->getStates().push_back(s1);
    //       p->getControls().push_back(ss->getSpaceInformation()->cloneControl(c));
    //       p->getControlDurations().push_back(0.5);
    //       }
    //       const double* vv= c->as<oc::OpenDEControlSpace::ControlType>()->values;
    //       std::cout<<"Control 1 is "<<vv[0]<<std::endl;

    return p;

}

SoSeparator *KauthamDEPlanner::getIvCspaceScene()
{
    _sceneCspace = new SoSeparator();
    _sceneCspace->ref();
    return Planner::getIvCspaceScene();
}
//! This routine allows to draw the 2D projection of a roadmap or tree. The one corresponding to robot number numrob is drawn.
void KauthamDEPlanner::drawCspace(int numrob)
{
    if(_wkSpace->getRobot(0)->getNumLinks()>1)
    {
        if(_sceneCspace==NULL) return;

        //first delete whatever is already drawn
        while (_sceneCspace->getNumChildren() > 0)
        {
            _sceneCspace->removeChild(0);
        }

        //to draw points
        SoSeparator *psep = new SoSeparator();
        SoCoordinate3 *points  = new SoCoordinate3();
        SoPointSet *pset  = new SoPointSet();

        //get the first subspace
        //        ob::StateSpacePtr ssRoboti = ((ob::StateSpacePtr) space->as<ob::CompoundStateSpace>()->getSubspace(numrob));
        //        ob::StateSpacePtr ssRobotifirst =  ((ob::StateSpacePtr) ssRoboti->as<ob::CompoundStateSpace>()->getSubspace(0));

        //space bounds
        int k;
        KthReal xmin;
        KthReal xmax;
        KthReal ymin;
        KthReal ymax;
        KthReal zmin;
        KthReal zmax;

        //if(_wkSpace->getRobot(numrob)->isSE3Enabled())
        {
            xmin = _wkSpace->getRobot(0)->getLimits(0)[0];
            ymin = _wkSpace->getRobot(0)->getLimits(1)[0];
            zmin = _wkSpace->getRobot(0)->getLimits(2)[0];
            xmax = _wkSpace->getRobot(0)->getLimits(0)[1];
            ymax = _wkSpace->getRobot(0)->getLimits(1)[1];
            zmax = _wkSpace->getRobot(0)->getLimits(2)[1];
            //k=stateSpace->getEnvironment()->getControlDimension();
            k = stateSpace->getDimension();

        }
        //        else
        //        {
        //            k = ssRobotifirst->as<ob::RealVectorStateSpace>()->getDimension();
        //            if(k<=2)
        //            {
        //                xmin=ssRobotifirst->as<ob::RealVectorStateSpace>()->getBounds().low[0];
        //                xmax=ssRobotifirst->as<ob::RealVectorStateSpace>()->getBounds().high[0];
        //                ymin=ssRobotifirst->as<ob::RealVectorStateSpace>()->getBounds().low[1];
        //                ymax=ssRobotifirst->as<ob::RealVectorStateSpace>()->getBounds().high[1];
        //            }
        //            else
        //            {
        //                xmin=ssRobotifirst->as<ob::RealVectorStateSpace>()->getBounds().low[0];
        //                xmax=ssRobotifirst->as<ob::RealVectorStateSpace>()->getBounds().high[0];
        //                ymin=ssRobotifirst->as<ob::RealVectorStateSpace>()->getBounds().low[1];
        //                ymax=ssRobotifirst->as<ob::RealVectorStateSpace>()->getBounds().high[1];
        //                zmin=ssRobotifirst->as<ob::RealVectorStateSpace>()->getBounds().low[2];
        //                zmax=ssRobotifirst->as<ob::RealVectorStateSpace>()->getBounds().high[2];
        //            }
        //        }


        KthReal x,y,z;
        //load the planner data to be drawn
        ob::PlannerDataPtr pdata;
        pdata = ((ob::PlannerDataPtr) new ob::PlannerData(ss->getSpaceInformation()));
        ss->getPlannerData(*pdata);

        if(ss->getProblemDefinition()->hasOptimizationObjective())
            pdata->computeEdgeWeights( *ss->getProblemDefinition()->getOptimizationObjective() );
        else
            pdata->computeEdgeWeights();


        //Use the projection associated to the subspace of the robot index passed as a parameter.
        string projname = "drawprojection"; //
        string robotnumber = static_cast<ostringstream*>( &(ostringstream() << numrob) )->str();//the string correspoding to number numrob
        projname.append(robotnumber); //the name of the projection: "drawprojection0", "drawprojection1",...
        //ob::ProjectionEvaluatorPtr projToUse = stateSpace->getProjection(projname.c_str());
        ob::ProjectionEvaluatorPtr projToUse = stateSpace->getDefaultProjection();

        //draw path:
        if(_solved)
        {
            //separator for the solution path
            SoSeparator *pathsep = new SoSeparator();
            //get the states of the solution path
            //                                      ******TODO*******
            //check the solution as geometric as well ss->getSolutionPath().asGeometric().getStates();
            std::vector< ob::State * > & pathstates = ss->getSolutionPath().getStates();

            //loop for all the states of the solution path
            for(unsigned int i=0; i<pathstates.size()-1; i++)
            {
                //initial edgepoint
                SoCoordinate3 *edgepoints  = new SoCoordinate3();
                //if(_wkSpace->getRobot(numrob)->isSE3Enabled())
                {
                    ob::EuclideanProjection projection(3);
                    //space->getProjection("drawprojection")->project(pathstates[i], projection);
                    projToUse->project(pathstates[i], projection);
                    x=projection[0];
                    y=projection[1];
                    z=projection[2];
                    //z=0;

                    edgepoints->point.set1Value(0,x,y,z);

                    //final edgepoint
                    //space->getProjection("drawprojection")->project(pathstates[i+1], projection);
                    projToUse->project(pathstates[i+1], projection);
                    x=projection[0];
                    y=projection[1];
                    z=projection[2];
                    //z=0;

                    edgepoints->point.set1Value(1,x,y,z);
                }
                //                else
                //                {
                //                    k = ssRobotifirst->as<ob::RealVectorStateSpace>()->getDimension();
                //                    if(k<=2)
                //                    {
                //                        ob::EuclideanProjection projection(k);
                //                        //space->getProjection("drawprojection")->project(pathstates[i], projection);
                //                        projToUse->project(pathstates[i], projection);
                //                        x=projection[0];
                //                        y=projection[1];
                //                        z=0.0;
                //                        edgepoints->point.set1Value(0,x,y,z);
                //                        //space->getProjection("drawprojection")->project(pathstates[i+1], projection);
                //                        projToUse->project(pathstates[i+1], projection);
                //                        x=projection[0];
                //                        y=projection[1];
                //                        edgepoints->point.set1Value(1,x,y,z);
                //                    }
                //                    else
                //                    {
                //                        ob::EuclideanProjection projection(k);
                //                        //space->getProjection("drawprojection")->project(pathstates[i], projection);
                //                        projToUse->project(pathstates[i], projection);
                //                        x=projection[0];
                //                        y=projection[1];
                //                        z=projection[2];
                //                        edgepoints->point.set1Value(0,x,y,z);
                //                        //space->getProjection("drawprojection")->project(pathstates[i+1], projection);
                //                        projToUse->project(pathstates[i+1], projection);
                //                        x=projection[0];
                //                        y=projection[1];
                //                        z=projection[2];
                //                        edgepoints->point.set1Value(1,x,y,z);
                //                    }
                //                }

                //edge of the path
                pathsep->addChild(edgepoints);
                SoLineSet *ls = new SoLineSet;
                ls->numVertices.set1Value(0,2);//two values
                SoDrawStyle *lstyle = new SoDrawStyle;
                lstyle->lineWidth=3;
                SoMaterial *path_color = new SoMaterial;
                path_color->diffuseColor.setValue(1.0,0.0,0.0);
                pathsep->addChild(path_color);
                pathsep->addChild(lstyle);
                pathsep->addChild(ls);
            }
            _sceneCspace->addChild(pathsep);
        }


        //loop for all vertices of the roadmap or tree and create the coin3D points
        for(unsigned int i=0;i<pdata->numVertices();i++)
        {
            //if(_wkSpace->getRobot(numrob)->isSE3Enabled())
            {
                ob::EuclideanProjection projection(k);
                //&(projection) = new ob::EuclideanProjection;
                //try
                //{
                //space->getProjection("drawprojection")->project(pdata->getVertex(i).getState(), projection);
                projToUse->project(pdata->getVertex(i).getState(), projection);
                // }
                //catch(ompl::Exception e){
                //  e.what();
                //}
                x = projection[0];
                y = projection[1];
                z = projection[2];

                //z=0;

                points->point.set1Value(i,x,y,z);
                /* DEBUG
    if(x>xpmax) xpmax=x;
    else if(x<xpmin) xpmin=x;
    if(y>ypmax) ypmax=y;
    else if(y<ypmin) ypmin=y;
    */

            }
            //            else
            //            {
            //                k = ssRobotifirst->as<ob::RealVectorStateSpace>()->getDimension();
            //                if(k<=2)
            //                {
            //                    ob::EuclideanProjection projection(k);
            //                    //space->getProjection("drawprojection")->project(pdata->getVertex(i).getState(), projection);
            //                    projToUse->project(pdata->getVertex(i).getState(), projection);
            //                    x = projection[0];
            //                    y = projection[1];
            //                    points->point.set1Value(i,x,y,0);
            //                }
            //                else
            //                {
            //                    ob::EuclideanProjection projection(k);
            //                    //space->getProjection("drawprojection")->project(pdata->getVertex(i).getState(), projection);
            //                    projToUse->project(pdata->getVertex(i).getState(), projection);
            //                    x = projection[0];
            //                    y = projection[1];
            //                    z = projection[2];
            //                    points->point.set1Value(i,x,y,z);
            //                }
            //            }
        }
        SoDrawStyle *pstyle = new SoDrawStyle;
        pstyle->pointSize = 1;
        SoMaterial *color = new SoMaterial;
        color->diffuseColor.setValue(0.2,0.8,0.2);

        //draw the points
        psep->addChild(color);
        psep->addChild(points);
        psep->addChild(pstyle);
        psep->addChild(pset);
        _sceneCspace->addChild(psep);

        //draw edges:
        SoSeparator *lsep = new SoSeparator();
        int numOutgoingEdges;
        std::vector< unsigned int > outgoingVertices;
        ob::Cost edgeweight;

        //loop for all nodes
        for(unsigned int i=0;i<pdata->numVertices();i++)
        {
            numOutgoingEdges = pdata->getEdges (i, outgoingVertices);

            //for each node loop for all the outgoing edges
            for (unsigned int j=0; j<(unsigned int) numOutgoingEdges; j++ )
            {
                SoCoordinate3 *edgepoints  = new SoCoordinate3();

                //initial edgepoint
                float x1,y1,x2,y2,z1,z2;
                //if(_wkSpace->getRobot(numrob)->isSE3Enabled())
                {
                    ob::EuclideanProjection projection(k);
                    //space->getProjection("drawprojection")->project(pdata->getVertex(i).getState(), projection);
                    projToUse->project(pdata->getVertex(i).getState(), projection);
                    x1=projection[0];
                    y1=projection[1];
                    z1=projection[2];

                    edgepoints->point.set1Value(0,x1,y1,z1);

                    //final edgepoint
                    //space->getProjection("drawprojection")->project(pdata->getVertex(outgoingVertices.at(j)).getState(), projection);
                    projToUse->project(pdata->getVertex(outgoingVertices.at(j)).getState(), projection);
                    x2=projection[0];
                    y2=projection[1];
                    z2=projection[2];

                    edgepoints->point.set1Value(1,x2,y2,z2);
                }
                //                else
                //                {
                //                    k = ssRobotifirst->as<ob::RealVectorStateSpace>()->getDimension();
                //                    if(k<=2)
                //                    {
                //                        ob::EuclideanProjection projection(k);
                //                        //space->getProjection("drawprojection")->project(pdata->getVertex(i).getState(), projection);
                //                        projToUse->project(pdata->getVertex(i).getState(), projection);
                //                        x1=projection[0];
                //                        y1=projection[1];
                //                        z=0.0;
                //                        edgepoints->point.set1Value(0,x1,y1,z);
                //                        //space->getProjection("drawprojection")->project(pdata->getVertex(outgoingVertices.at(j)).getState(), projection);
                //                        projToUse->project(pdata->getVertex(outgoingVertices.at(j)).getState(), projection);
                //                        x2=projection[0];
                //                        y2=projection[1];
                //                        edgepoints->point.set1Value(1,x2,y2,z);
                //                    }
                //                    else
                //                    {
                //                        ob::EuclideanProjection projection(k);
                //                        //space->getProjection("drawprojection")->project(pdata->getVertex(i).getState(), projection);
                //                        projToUse->project(pdata->getVertex(i).getState(), projection);
                //                        x1=projection[0];
                //                        y1=projection[1];
                //                        z1=projection[2];
                //                        edgepoints->point.set1Value(0,x1,y1,z1);
                //                        //space->getProjection("drawprojection")->project(pdata->getVertex(outgoingVertices.at(j)).getState(), projection);
                //                        projToUse->project(pdata->getVertex(outgoingVertices.at(j)).getState(), projection);
                //                        x2=projection[0];
                //                        y2=projection[1];
                //                        z2=projection[2];
                //                        edgepoints->point.set1Value(1,x2,y2,z2);
                //                    }
                //                }
                //the edge
                pdata->getEdgeWeight(i, outgoingVertices.at(j), &edgeweight);
                SoMaterial *edge_color = new SoMaterial;
                edge_color->diffuseColor.setValue(1.0,1.0,1.0);

                //BE CAREFUL! a magic number!
                /*
                    if(edgeweight.v>0.1) edge_color->diffuseColor.setValue(1.0,0.8,0.8);
                    else edge_color->diffuseColor.setValue(1.0,1.0,1.0);
                    */
                lsep->addChild(edge_color);
                lsep->addChild(edgepoints);
                SoLineSet *ls = new SoLineSet;
                ls->numVertices.set1Value(0,2);//two values
                lsep->addChild(ls);
            }
        }
        _sceneCspace->addChild(lsep);



        SoSeparator *floorsep = new SoSeparator();
        SoCube *cs = new SoCube();
        SoTransform *cub_transf = new SoTransform;
        SbVec3f centre;
        SoMaterial *cub_color = new SoMaterial;
        //draw floor
        //if(_wkSpace->getRobot(numrob)->isSE3Enabled())
        {
            cs->width = xmax-xmin;
            cs->depth = (zmax-zmin);
            cs->height = ymax-ymin;

            centre.setValue(xmin+(xmax-xmin)/2,ymin+(ymax-ymin)/2,zmin+(zmax-zmin)/2);
            cub_transf->translation.setValue(centre);
            cub_transf->recenter(centre);

            //SoMaterial *cub_color = new SoMaterial;
            cub_color->diffuseColor.setValue(0.2,0.2,0.2);
            cub_color->transparency.setValue(0.5);

            floorsep->addChild(cub_color);
            floorsep->addChild(cub_transf);
            floorsep->addChild(cs);
            _sceneCspace->addChild(floorsep);
        }
        //        else
        //        {
        //            k = ssRobotifirst->as<ob::RealVectorStateSpace>()->getDimension();
        //            if(k<=2)
        //            {
        //                cs->width = xmax-xmin;
        //                cs->depth = (xmax-xmin)/50.0;
        //                cs->height = ymax-ymin;
        //                centre.setValue(xmin+(xmax-xmin)/2,ymin+(ymax-ymin)/2,-cs->depth.getValue());
        //                cub_transf->translation.setValue(centre);
        //                cub_transf->recenter(centre);
        //                cub_color->diffuseColor.setValue(0.2,0.2,0.2);
        //                //cub_color->transparency.setValue(0.98);
        //                floorsep->addChild(cub_color);
        //                floorsep->addChild(cub_transf);
        //                floorsep->addChild(cs);
        //                _sceneCspace->addChild(floorsep);
        //            }
        //            else
        //            {
        //                cs->width = xmax-xmin;
        //                cs->depth = (zmax-zmin);
        //                cs->height = ymax-ymin;
        //                centre.setValue(xmin+(xmax-xmin)/2,ymin+(ymax-ymin)/2,zmin+(zmax-zmin)/2);
        //                cub_transf->translation.setValue(centre);
        //                cub_transf->recenter(centre);
        //                cub_color->diffuseColor.setValue(0.2,0.2,0.2);
        //                cub_color->transparency.setValue(0.3);//0.98
        //                floorsep->addChild(cub_color);
        //                floorsep->addChild(cub_transf);
        //                floorsep->addChild(cs);
        //                _sceneCspace->addChild(floorsep);
        //            }
        //         }

        /* DEBUG
            cout<<"xpmax="<<xpmax;
            cout<<" xpmin="<<xpmin;
            cout<<" ypmax="<<ypmax;
            cout<<" ypmin="<<ypmin<<endl;
            cout<<"xsmax="<<xM;
            cout<<" xsmin="<<xm;
            cout<<" ysmax="<<yM;
            cout<<" ysmin="<<ym<<endl;
    */


        ///////////////////////////////////////////////////////////////////////////////////////////
    }
    else
    {
        if(_sceneCspace==NULL) return;

        //first delete whatever is already drawn
        while (_sceneCspace->getNumChildren() > 0)
        {
            _sceneCspace->removeChild(0);
        }

        //to draw points
        SoSeparator *psep = new SoSeparator();
        SoCoordinate3 *points  = new SoCoordinate3();
        SoPointSet *pset  = new SoPointSet();

        //get the first subspace
        //        ob::StateSpacePtr ssRoboti = ((ob::StateSpacePtr) space->as<ob::CompoundStateSpace>()->getSubspace(numrob));
        //        ob::StateSpacePtr ssRobotifirst =  ((ob::StateSpacePtr) ssRoboti->as<ob::CompoundStateSpace>()->getSubspace(0));

        //space bounds
        int k;
        KthReal xmin;
        KthReal xmax;
        KthReal ymin;
        KthReal ymax;
        KthReal zmin;
        KthReal zmax;

        //if(_wkSpace->getRobot(numrob)->isSE3Enabled())
        {
            xmin = _wkSpace->getRobot(0)->getLimits(0)[0];
            ymin = _wkSpace->getRobot(0)->getLimits(1)[0];
            zmin = _wkSpace->getRobot(0)->getLimits(2)[0];
            xmax = _wkSpace->getRobot(0)->getLimits(0)[1];
            ymax = _wkSpace->getRobot(0)->getLimits(1)[1];
            zmax = _wkSpace->getRobot(0)->getLimits(2)[1];
            k = stateSpace->getDimension();

        }
        //        else
        //        {
        //            k = ssRobotifirst->as<ob::RealVectorStateSpace>()->getDimension();
        //            if(k<=2)
        //            {
        //                xmin=ssRobotifirst->as<ob::RealVectorStateSpace>()->getBounds().low[0];
        //                xmax=ssRobotifirst->as<ob::RealVectorStateSpace>()->getBounds().high[0];
        //                ymin=ssRobotifirst->as<ob::RealVectorStateSpace>()->getBounds().low[1];
        //                ymax=ssRobotifirst->as<ob::RealVectorStateSpace>()->getBounds().high[1];
        //            }
        //            else
        //            {
        //                xmin=ssRobotifirst->as<ob::RealVectorStateSpace>()->getBounds().low[0];
        //                xmax=ssRobotifirst->as<ob::RealVectorStateSpace>()->getBounds().high[0];
        //                ymin=ssRobotifirst->as<ob::RealVectorStateSpace>()->getBounds().low[1];
        //                ymax=ssRobotifirst->as<ob::RealVectorStateSpace>()->getBounds().high[1];
        //                zmin=ssRobotifirst->as<ob::RealVectorStateSpace>()->getBounds().low[2];
        //                zmax=ssRobotifirst->as<ob::RealVectorStateSpace>()->getBounds().high[2];
        //            }
        //        }


        KthReal x,y,z;
        //load the planner data to be drawn
        ob::PlannerDataPtr pdata;
        pdata = ((ob::PlannerDataPtr) new ob::PlannerData(ss->getSpaceInformation()));
        ss->getPlannerData(*pdata);

        if(ss->getProblemDefinition()->hasOptimizationObjective())
            pdata->computeEdgeWeights( *ss->getProblemDefinition()->getOptimizationObjective() );
        else
            pdata->computeEdgeWeights();


        //Use the projection associated to the subspace of the robot index passed as a parameter.
        string projname = "drawprojection"; //
        string robotnumber = static_cast<ostringstream*>( &(ostringstream() << numrob) )->str();//the string correspoding to number numrob
        projname.append(robotnumber); //the name of the projection: "drawprojection0", "drawprojection1",...
        //ob::ProjectionEvaluatorPtr projToUse = stateSpace->getProjection(projname.c_str());
        ob::ProjectionEvaluatorPtr projToUse = stateSpace->getDefaultProjection();

        //draw path:
        if(_solved)
        {
            //separator for the solution path
            SoSeparator *pathsep = new SoSeparator();
            //get the states of the solution path
            //                                      ******TODO*******
            //check the solution as geometric as well ss->getSolutionPath().asGeometric().getStates();
            std::vector< ob::State * > & pathstates = ss->getSolutionPath().getStates();

            //loop for all the states of the solution path
            for(unsigned int i=0; i<pathstates.size()-1; i++)
            {
                //initial edgepoint
                SoCoordinate3 *edgepoints  = new SoCoordinate3();
                //if(_wkSpace->getRobot(numrob)->isSE3Enabled())
                {
                    ob::EuclideanProjection projection(2);
                    //space->getProjection("drawprojection")->project(pathstates[i], projection);
                    projToUse->project(pathstates[i], projection);
                    x=projection[0];
                    y=projection[1];
                    //z=projection[2];
                    z=0;

                    edgepoints->point.set1Value(0,x,y,z);

                    //final edgepoint
                    //space->getProjection("drawprojection")->project(pathstates[i+1], projection);
                    projToUse->project(pathstates[i+1], projection);
                    x=projection[0];
                    y=projection[1];
                    //z=projection[2];
                    //z=0;

                    edgepoints->point.set1Value(1,x,y,z);
                }
                //                else
                //                {
                //                    k = ssRobotifirst->as<ob::RealVectorStateSpace>()->getDimension();
                //                    if(k<=2)
                //                    {
                //                        ob::EuclideanProjection projection(k);
                //                        //space->getProjection("drawprojection")->project(pathstates[i], projection);
                //                        projToUse->project(pathstates[i], projection);
                //                        x=projection[0];
                //                        y=projection[1];
                //                        z=0.0;
                //                        edgepoints->point.set1Value(0,x,y,z);
                //                        //space->getProjection("drawprojection")->project(pathstates[i+1], projection);
                //                        projToUse->project(pathstates[i+1], projection);
                //                        x=projection[0];
                //                        y=projection[1];
                //                        edgepoints->point.set1Value(1,x,y,z);
                //                    }
                //                    else
                //                    {
                //                        ob::EuclideanProjection projection(k);
                //                        //space->getProjection("drawprojection")->project(pathstates[i], projection);
                //                        projToUse->project(pathstates[i], projection);
                //                        x=projection[0];
                //                        y=projection[1];
                //                        z=projection[2];
                //                        edgepoints->point.set1Value(0,x,y,z);
                //                        //space->getProjection("drawprojection")->project(pathstates[i+1], projection);
                //                        projToUse->project(pathstates[i+1], projection);
                //                        x=projection[0];
                //                        y=projection[1];
                //                        z=projection[2];
                //                        edgepoints->point.set1Value(1,x,y,z);
                //                    }
                //                }

                //edge of the path
                pathsep->addChild(edgepoints);
                SoLineSet *ls = new SoLineSet;
                ls->numVertices.set1Value(0,2);//two values
                SoDrawStyle *lstyle = new SoDrawStyle;
                lstyle->lineWidth=50000;//3;
                SoMaterial *path_color = new SoMaterial;
                path_color->diffuseColor.setValue(1.0,0.0,0.0);
                pathsep->addChild(path_color);
                pathsep->addChild(lstyle);
                pathsep->addChild(ls);
            }
            _sceneCspace->addChild(pathsep);
        }

        /* DEBUG
double xpmax=-1.0;
double xpmin=100.0;
double ypmax=-1.0;
double ypmin=100.0;
*/
        //JAN - DEBUG
        // ///////////////////////////////////////////
        //for debug. draw streering samples for rrt

        //                        SoSeparator *psep_steerpoints = new SoSeparator();
        //                        SoCoordinate3 *steerpoints  = new SoCoordinate3();
        //                        SoPointSet *pset_steerpoints  = new SoPointSet();

        //                        SoDrawStyle *pstyle_steerpoints = new SoDrawStyle;
        //                        SoMaterial *color_steerpoints = new SoMaterial;
        //                        pstyle_steerpoints->pointSize = 3;
        //                        color_steerpoints->diffuseColor.setValue(0.5,0.5,1.0);

        //                        for(int i=0;i<steersamples.size();i++)
        //                        {
        //                            x = steersamples[i][0];
        //                            y = steersamples[i][1];
        //                            z = steersamples[i][2];
        //                            steerpoints->point.set1Value(i,x,y,z);
        //                        }
        //                        //draw the points

        //                        psep_steerpoints->addChild(color_steerpoints);
        //                        psep_steerpoints->addChild(steerpoints);
        //                        psep_steerpoints->addChild(pstyle_steerpoints);
        //                        psep_steerpoints->addChild(pset_steerpoints);

        //                        _sceneCspace->addChild(psep_steerpoints);

        //END: for debug. draw streering samples for rrt
        //JAN - DEBUG
        // ///////////////////////////////////////////

        //loop for all vertices of the roadmap or tree and create the coin3D points
        for(unsigned int i=0;i<pdata->numVertices();i++)
        {
            //if(_wkSpace->getRobot(numrob)->isSE3Enabled())
            {
                ob::EuclideanProjection projection(k);
                //&(projection) = new ob::EuclideanProjection;
                //try
                //{
                //space->getProjection("drawprojection")->project(pdata->getVertex(i).getState(), projection);
                projToUse->project(pdata->getVertex(i).getState(), projection);
                // }
                //catch(ompl::Exception e){
                //  e.what();
                //}
                x = projection[0];
                y = projection[1];
                //z = projection[2];

                //z=0;

                points->point.set1Value(i,x,y,z);
                /* DEBUG
if(x>xpmax) xpmax=x;
else if(x<xpmin) xpmin=x;
if(y>ypmax) ypmax=y;
else if(y<ypmin) ypmin=y;
*/

            }
            //            else
            //            {
            //                k = ssRobotifirst->as<ob::RealVectorStateSpace>()->getDimension();
            //                if(k<=2)
            //                {
            //                    ob::EuclideanProjection projection(k);
            //                    //space->getProjection("drawprojection")->project(pdata->getVertex(i).getState(), projection);
            //                    projToUse->project(pdata->getVertex(i).getState(), projection);
            //                    x = projection[0];
            //                    y = projection[1];
            //                    points->point.set1Value(i,x,y,0);
            //                }
            //                else
            //                {
            //                    ob::EuclideanProjection projection(k);
            //                    //space->getProjection("drawprojection")->project(pdata->getVertex(i).getState(), projection);
            //                    projToUse->project(pdata->getVertex(i).getState(), projection);
            //                    x = projection[0];
            //                    y = projection[1];
            //                    z = projection[2];
            //                    points->point.set1Value(i,x,y,z);
            //                }
            //            }
        }
        SoDrawStyle *pstyle = new SoDrawStyle;
        pstyle->pointSize = 1;
        SoMaterial *color = new SoMaterial;
        color->diffuseColor.setValue(0.2,0.8,0.2);

        //draw the points
        psep->addChild(color);
        psep->addChild(points);
        psep->addChild(pstyle);
        psep->addChild(pset);
        _sceneCspace->addChild(psep);

        //draw edges:
        SoSeparator *lsep = new SoSeparator();
        int numOutgoingEdges;
        std::vector< unsigned int > outgoingVertices;
        ob::Cost edgeweight;

        //loop for all nodes
        for(unsigned int i=0;i<pdata->numVertices();i++)
        {
            numOutgoingEdges = pdata->getEdges (i, outgoingVertices);

            //for each node loop for all the outgoing edges
            for ( int j=0; j<numOutgoingEdges; j++ )
            {
                SoCoordinate3 *edgepoints  = new SoCoordinate3();

                //initial edgepoint
                float x1,y1,x2,y2,z1,z2;
                //if(_wkSpace->getRobot(numrob)->isSE3Enabled())
                {
                    ob::EuclideanProjection projection(k);
                    //space->getProjection("drawprojection")->project(pdata->getVertex(i).getState(), projection);
                    projToUse->project(pdata->getVertex(i).getState(), projection);
                    x1=projection[0];
                    y1=projection[1];
                    //z1=projection[2];
z1=0;
                    edgepoints->point.set1Value(0,x1,y1,z1);

                    //final edgepoint
                    //space->getProjection("drawprojection")->project(pdata->getVertex(outgoingVertices.at(j)).getState(), projection);
                    projToUse->project(pdata->getVertex(outgoingVertices.at(j)).getState(), projection);
                    x2=projection[0];
                    y2=projection[1];
                    //z2=projection[2];

                    edgepoints->point.set1Value(1,x2,y2,z2);
                }
                //                else
                //                {
                //                    k = ssRobotifirst->as<ob::RealVectorStateSpace>()->getDimension();
                //                    if(k<=2)
                //                    {
                //                        ob::EuclideanProjection projection(k);
                //                        //space->getProjection("drawprojection")->project(pdata->getVertex(i).getState(), projection);
                //                        projToUse->project(pdata->getVertex(i).getState(), projection);
                //                        x1=projection[0];
                //                        y1=projection[1];
                //                        z=0.0;
                //                        edgepoints->point.set1Value(0,x1,y1,z);
                //                        //space->getProjection("drawprojection")->project(pdata->getVertex(outgoingVertices.at(j)).getState(), projection);
                //                        projToUse->project(pdata->getVertex(outgoingVertices.at(j)).getState(), projection);
                //                        x2=projection[0];
                //                        y2=projection[1];
                //                        edgepoints->point.set1Value(1,x2,y2,z);
                //                    }
                //                    else
                //                    {
                //                        ob::EuclideanProjection projection(k);
                //                        //space->getProjection("drawprojection")->project(pdata->getVertex(i).getState(), projection);
                //                        projToUse->project(pdata->getVertex(i).getState(), projection);
                //                        x1=projection[0];
                //                        y1=projection[1];
                //                        z1=projection[2];
                //                        edgepoints->point.set1Value(0,x1,y1,z1);
                //                        //space->getProjection("drawprojection")->project(pdata->getVertex(outgoingVertices.at(j)).getState(), projection);
                //                        projToUse->project(pdata->getVertex(outgoingVertices.at(j)).getState(), projection);
                //                        x2=projection[0];
                //                        y2=projection[1];
                //                        z2=projection[2];
                //                        edgepoints->point.set1Value(1,x2,y2,z2);
                //                    }
                //                }
                //the edge
                pdata->getEdgeWeight(i, outgoingVertices.at(j), &edgeweight);
                SoMaterial *edge_color = new SoMaterial;
                edge_color->diffuseColor.setValue(1.0,1.0,1.0);

                //BE CAREFUL! a magic number!
                /*
                if(edgeweight.v>0.1) edge_color->diffuseColor.setValue(1.0,0.8,0.8);
                else edge_color->diffuseColor.setValue(1.0,1.0,1.0);
                */
                lsep->addChild(edge_color);
                lsep->addChild(edgepoints);
                SoLineSet *ls = new SoLineSet;
                ls->numVertices.set1Value(0,2);//two values
                lsep->addChild(ls);
            }
        }
        _sceneCspace->addChild(lsep);



        SoSeparator *floorsep = new SoSeparator();
        SoCube *cs = new SoCube();
        SoTransform *cub_transf = new SoTransform;
        SbVec3f centre;
        SoMaterial *cub_color = new SoMaterial;
        //draw floor
        //if(_wkSpace->getRobot(numrob)->isSE3Enabled())
        {
            cs->width = xmax-xmin;
            cs->depth = (zmax-zmin);
            cs->height = ymax-ymin;

            centre.setValue(xmin+(xmax-xmin)/2,ymin+(ymax-ymin)/2,zmin+(zmax-zmin)/2);
            cub_transf->translation.setValue(centre);
            cub_transf->recenter(centre);

            //SoMaterial *cub_color = new SoMaterial;
            cub_color->diffuseColor.setValue(0.2,0.2,0.2);
            cub_color->transparency.setValue(0.5);

            floorsep->addChild(cub_color);
            floorsep->addChild(cub_transf);
            floorsep->addChild(cs);
            _sceneCspace->addChild(floorsep);
        }
        //        else
        //        {
        //            k = ssRobotifirst->as<ob::RealVectorStateSpace>()->getDimension();
        //            if(k<=2)
        //            {
        //                cs->width = xmax-xmin;
        //                cs->depth = (xmax-xmin)/50.0;
        //                cs->height = ymax-ymin;
        //                centre.setValue(xmin+(xmax-xmin)/2,ymin+(ymax-ymin)/2,-cs->depth.getValue());
        //                cub_transf->translation.setValue(centre);
        //                cub_transf->recenter(centre);
        //                cub_color->diffuseColor.setValue(0.2,0.2,0.2);
        //                //cub_color->transparency.setValue(0.98);
        //                floorsep->addChild(cub_color);
        //                floorsep->addChild(cub_transf);
        //                floorsep->addChild(cs);
        //                _sceneCspace->addChild(floorsep);
        //            }
        //            else
        //            {
        //                cs->width = xmax-xmin;
        //                cs->depth = (zmax-zmin);
        //                cs->height = ymax-ymin;
        //                centre.setValue(xmin+(xmax-xmin)/2,ymin+(ymax-ymin)/2,zmin+(zmax-zmin)/2);
        //                cub_transf->translation.setValue(centre);
        //                cub_transf->recenter(centre);
        //                cub_color->diffuseColor.setValue(0.2,0.2,0.2);
        //                cub_color->transparency.setValue(0.3);//0.98
        //                floorsep->addChild(cub_color);
        //                floorsep->addChild(cub_transf);
        //                floorsep->addChild(cs);
        //                _sceneCspace->addChild(floorsep);
        //            }
        //         }

        /* DEBUG
        cout<<"xpmax="<<xpmax;
        cout<<" xpmin="<<xpmin;
        cout<<" ypmax="<<ypmax;
        cout<<" ypmin="<<ypmin<<endl;
        cout<<"xsmax="<<xM;
        cout<<" xsmin="<<xm;
        cout<<" ysmax="<<yM;
        cout<<" ysmin="<<ym<<endl;
*/


    }
}

}
}


/////////////////////////////////////////////////////////////////////
//    oc::PathControl *p = RectMotion();
//    solutionStates pstat;
//    std::vector<ob::State*> &pstates =p->getStates();
//      std::vector<oc::Control*> &pControl= p->getControls();
//                       std::vector<double> &pduration= p->getControlDurations();
//                      pstat.substates =pstates;
//                       pstat.control =pControl;
//                       pstat.duration =pduration;
//                       sStates.push_back(pstat);
///////////////////////////////////////////////////////////////////////





//    //*******************TODO******************
//    //Solution paths of multiple quires can be appended in geometric path
//    //to generate the single path of multiple quires.
//    ////////////////////
//    oc::OpenDESimpleSetup *ss1= new oc::OpenDESimpleSetup(stateSpacePtr);

//    ss1->setVolumeBounds(vb);
//    ss1->setAngularVelocityBounds(bounds);
//    ss1->setLinearVelocityBounds(bounds);
//    ob::ScopedState<oc::OpenDEStateSpace> last(ss->getSpaceInformation());
//    last = ss->getSolutionPath().getStates().back();
//    ss1->setStartState(last);
//    //ss1->setGoal(ob::GoalPtr(new KauthamDEGoal(ss->getSpaceInformation(),_wkSpace,_onlyend,-111,53)));
//    ss1->setGoal(ob::GoalPtr(new KauthamDEGoal(ss->getSpaceInformation(),_wkSpace,_onlyend,124,-50)));

//    ss1->setup();
//    ss1->print();
//    bool solution2,solution3;
//    solution2=ss1->solve(100);
//    if(solution2)
//    {
//        std::cout<<"2nd Query is solved"<<std::endl;

//        solutionStates sstat1;
//        std::vector<ob::State*> &States =ss1->getSolutionPath().getStates();
//        std::vector<oc::Control*> &Control= ss1->getSolutionPath().getControls();
//        std::vector<double> &Duration= ss1->getSolutionPath().getControlDurations();

//        sstat1.substates =States;
//        sstat1.control =Control;
//        sstat1.duration =Duration;

//        sStates.push_back(sstat1);

//        oc::OpenDESimpleSetup *ss3= new oc::OpenDESimpleSetup(stateSpacePtr);

//        ss3->setVolumeBounds(vb);
//        ss3->setAngularVelocityBounds(bounds);
//        ss3->setLinearVelocityBounds(bounds);

//        ob::ScopedState<oc::OpenDEStateSpace> last1(ss1->getSpaceInformation());
//        last1 = ss1->getSolutionPath().getStates().back();
//        ss3->setStartState(last1);
//        //ss3->setGoal(ob::GoalPtr(new KauthamDEGoal(ss1->getSpaceInformation(),_wkSpace,_onlyend,-124,39)));
//        ss3->setGoal(ob::GoalPtr(new KauthamDEGoal(ss1->getSpaceInformation(),_wkSpace,_onlyend,aux)));

//        ss3->setup();
//        ss3->print();
//        solution3 = ss3->solve(100);
//        if(solution3)
//        {
//            std::cout<<"3rd Query is solved"<<std::endl;

//            solutionStates sstat2;
//            std::vector<ob::State*> &States =ss3->getSolutionPath().getStates();
//            std::vector<oc::Control*> &Control= ss3->getSolutionPath().getControls();
//            std::vector<double> &Duration= ss3->getSolutionPath().getControlDurations();
//            sstat2.substates =States;
//            sstat2.control =Control;
//            sstat2.duration =Duration;

//            sStates.push_back(sstat2);

//////            oc::OpenDESimpleSetup *ss4= new oc::OpenDESimpleSetup(stateSpacePtr);

//////            ss4->setVolumeBounds(vb);
//////            ss4->setAngularVelocityBounds(bounds);
//////            ss4->setLinearVelocityBounds(bounds);

//////            ob::ScopedState<oc::OpenDEStateSpace> last2(ss3->getSpaceInformation());
//////            last2 = ss3->getSolutionPath().getStates().back();
//////            ss4->setStartState(last2);
//////            ss4->setGoal(ob::GoalPtr(new KauthamDEGoal(ss3->getSpaceInformation(),_wkSpace,_onlyend,-123,-44)));
//////            ss4->setup();
//////            ss4->print();
//////            bool solution4 = ss4->solve(60);
//////            if(solution4)
//////            {
//////                std::cout<<"4th Query is solved"<<std::endl;

//////                solutionStates sstat3;
//////                std::vector<ob::State*> &States =ss4->getSolutionPath().getStates();
//////                std::vector<oc::Control*> &Control= ss4->getSolutionPath().getControls();
//////                std::vector<double> &Duration= ss4->getSolutionPath().getControlDurations();
//////                sstat3.substates =States;
//////                sstat3.control =Control;
//////                sstat3.duration =Duration;

//////                sStates.push_back(sstat3);

//////                oc::OpenDESimpleSetup *ss5= new oc::OpenDESimpleSetup(stateSpacePtr);

//////                ss5->setVolumeBounds(vb);
//////                ss5->setAngularVelocityBounds(bounds);
//////                ss5->setLinearVelocityBounds(bounds);

//////                ob::ScopedState<oc::OpenDEStateSpace> last3(ss4->getSpaceInformation());
//////                last3 = ss4->getSolutionPath().getStates().back();
//////                ss5->setStartState(last3);
//////                ss5->setGoal(ob::GoalPtr(new KauthamDEGoal(ss4->getSpaceInformation(),_wkSpace,_onlyend,aux)));
//////                ss5->setup();
//////                ss5->print();
//////                bool solution5 = ss5->solve(60);
//////                if(solution5)
//////                {
//////                    std::cout<<"5th Query is solved"<<std::endl;

//////                    solutionStates sstat4;
//////                    std::vector<ob::State*> &States =ss5->getSolutionPath().getStates();
//////                    std::vector<oc::Control*> &Control= ss5->getSolutionPath().getControls();
//////                    std::vector<double> &Duration= ss5->getSolutionPath().getControlDurations();
//////                    sstat4.substates =States;
//////                    sstat4.control =Control;
//////                    sstat4.duration =Duration;

//////                    sStates.push_back(sstat4);
//////                }
//////            }
//        }
//    }

//////        std::cout<<"Result of Qury 1 is:  "<<solution1<<std::endl;
//////        std::cout<<"Result of Qury 2 is:  "<<solution2<<std::endl;
//////        std::cout<<"Result of Qury 3 is:  "<<solution3<<std::endl;
//        _solved=solution1 || solution2 ;//|| solution3;
#endif //KAUTHAM_USE_ODE
#endif //KAUTHAM_USE_OMPL
