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

#include <kautham/planner/omplOpenDE/PhysicsBasedPlanners/KauthamOpenDEPlanner.h>
#include <kautham/planner/omplg/omplplanner.h>
#include <kautham/planner/omplc/omplcplanner.h>
#include <Inventor/nodes/SoTransform.h>
#include <Inventor/nodes/SoCoordinate3.h>
#include <Inventor/nodes/SoCube.h>
#include <Inventor/nodes/SoPointSet.h>
#include <Inventor/nodes/SoLineSet.h>
#include <Inventor/nodes/SoDrawStyle.h>
#include <Inventor/nodes/SoMaterial.h>
#include <ctime>

#include <boost/thread.hpp>

class KauthamDEGoal;

namespace Kautham {

#ifdef dDOUBLE
#define dsDrawBox dsDrawBoxD
#define dsDrawSphere dsDrawSphereD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCapsule dsDrawCapsuleD
#endif
namespace omplcplanner{
static DisplayOpenDESpaces                         DISP;
static std::vector< std::pair<double, double> > POINTS;
static bool                                     drawTree = true;
static void start()
{
    dAllocateODEDataForThread(dAllocateMaskAll);
    static float xyz[3] = { 3.8548f,9.0843f,7.5900f} ;
    static float hpr[3] = { -145.5f,-22.5f,0.25f } ;

    dsSetViewpoint (xyz,hpr);
}

static void command (int cmd)
{
    if ((char)cmd == 't')
        drawTree = !drawTree;
}

static void simLoop (int /*pause*/)
{
    if(DISP.tmd.size()>0)
        DISP.displaySpaces(&(DISP.tmd));
    else
        DISP.displaySpaces(NULL);


    if (drawTree)
    {
        glPointSize(3.0);
        glColor3f(1.0f, 0.0f, 0.0f);
        glBegin(GL_POINTS);
        for (unsigned int i = 0 ; i < POINTS.size() ; ++i)
            glVertex3d(POINTS[i].first, POINTS[i].second, 0.05);
        glEnd();
    }
    boost::this_thread::sleep(boost::posix_time::milliseconds(1));

}

static void playPath(oc::OpenDESimpleSetup *ss, std::string robot)
{
    while (1)
    {
        if(robot=="SimpleCar")
        ss->playSolutionPath(0.1);
        else
            ss->playSolutionPath(1);
        boost::this_thread::sleep(boost::posix_time::milliseconds(1));

    }
}

/*! Constructor define all the necessary parameters for planning in dynamic enviroment.
 *  It define the pointer to the dynamic enviroment and pointer to the start space.
 */
KauthamDEPlanner::KauthamDEPlanner(SPACETYPE stype, Sample *init, Sample *goal, SampleSet *samples, WorkSpace *ws) : Planner(stype, init, goal, samples, ws)
{
    _family =ODEPLANNER;
    ws->moveRobotsTo(init);
    //set intial values from parent class data
    _speedFactor = 3;
    _solved = false;
    _maxspeed = 5;
    _onlyend = false;
    _planningTime=60;
    _maxContacts=5;
    if(_wkSpace->getRobot(0)->getNumJoints()>1 && _wkSpace->getRobot(0)->getName()!="SimpleCar")
    {
        _propagationStepSize=0.02;
        _minControlSteps=1;
        _maxControlSteps=30;
    }
    else{
        _propagationStepSize=0.05;
        _minControlSteps=2;
        _maxControlSteps=50;
    }
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

        ss->setGoal(ob::GoalPtr(new KauthamDEGoalRegion (ss->getSpaceInformation(),_wkSpace,_onlyend,aux)));//goalPose[0].pose[0],goalPose[0].pose[1])));
        string action="move";
        ((KauthamDEEnvironment*)envPtr.get())->manipulationQuery->setActionType(action);
        std::vector<double> f(3);
        f[0]=4.0;
        f[1]=0.0;
        f[2]=0.0;
        ((KauthamDEEnvironment*)envPtr.get())->manipulationQuery->setforce(f);
        dJointID joint1;
        if(action=="pull" || action=="Pull")
        {
            joint1=dJointCreateHinge( ((KauthamDEEnvironment*)envPtr.get())->world_,0);
            dJointAttach(joint1 ,((KauthamDEEnvironment*)envPtr.get())->stateBodies_[0],((KauthamDEEnvironment*)envPtr.get())->stateBodies_[2]);
        }
        ss->setup();
        ss->print();
        bool solution1=false;
        _solved=solution1 = ss->solve(_planningTime);

        ((KauthamDEEnvironment*)envPtr.get())->manipulationQuery->setPlanningPhase(false);
        if(action=="pull" || action=="Pull")
            dJointDestroy(joint1);
        std::cout<<"Query Solved"<<std::endl;
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

    if (_solved)
    {
        callDrawStuffViewer();

        _path.clear();
        clearSimulationPath();
        Sample *Robsmp;
        Sample *Obssmp;
        for(unsigned int l=0;l<sStates.size();l++)
        {
            std::vector<ob::State*> &states = sStates[l].substates;
            std::vector<oc::Control*> &control= sStates[l].control;
            std::vector<double> &duration=sStates[l].duration;
            std::cout<<"Propagation Step is :"<<ss->getSpaceInformation().get()->getPropagationStepSize()<<std::endl;
            std::cout<<"control count is :"<<control.size()<<" duration is : "<<duration.size()<<std::endl;

            std::vector<double> Joint_Angle;
            Joint_Angle.resize(_wkSpace->getRobot(0)->getNumJoints());

            final=ss->getSolutionPath().getStates().back();
            std::cout<<"Generated Solution states are  :  "<<states.size()<<std::endl;
            std::cout<<"Generated Solution controls are:  "<<control.size()<<std::endl;
            std::cout<<"Solution control durations  are:  "<<duration.size()<<std::endl;
            //std::cout<<"last rob state is " <<final->getBodyPosition(0)[0] << " " << final->getBodyPosition(0)[1] << std::endl;

            lastState.push_back(final->getBodyPosition(0)[0]);
            lastState.push_back(final->getBodyPosition(0)[1]);
            std::ofstream pathD ("Path.txt", std::ios::out | std::ios::app);
            for(unsigned int i=0;i<states.size()-1;i++)
            {
                //==============================================
                //Temperarly changes
                const double *ps = states[i]->as<oc::OpenDEStateSpace::StateType>()->getBodyPosition(0);

                   pathD <<ps[0] << " ";
                   pathD <<ps[1] << " ";
                   pathD <<duration[i]<<std::endl;


                //==============================================
                State tmpstate;
                Robsmp=new Sample(_wkSpace->getNumRobControls());

                    KauthamOpenDEState2Robsmp(states[i],Robsmp);

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

            }
            pathD.close();

        }
       drawCspace(0);

        return _solved;
    }

    dCloseODE();
    return 0;

}
//! This member function converts an OpenDE State to a Kautham robot sample
void KauthamDEPlanner::KauthamOpenDEState2Robsmp(const ob::State *state, Sample* smp)
{
    int k=0;
       vector<RobConf> rc;
       SE3Conf basepose;
       const double *basep = state->as<oc::OpenDEStateSpace::StateType>()->getBodyPosition(0);
       const ob::SO3StateSpace::StateType &baseori = state->as<oc::OpenDEStateSpace::StateType>()->getBodyRotation(0);
       std::vector<float> tmp1(3);
       std::vector<float> tmp2(4);
       tmp1[0] = basep[0];
       tmp1[1] = basep[1];
       tmp1[2] = basep[2];
       tmp2[0] = baseori.x;
       tmp2[1] = baseori.y;
       tmp2[2] = baseori.z;
       tmp2[3] = baseori.w;
       basepose.setPos(tmp1);
       basepose.setOrient(tmp2);
       for(unsigned int i=0; i<_wkSpace->getNumRobots(); i++)
       {
           RobConf *rcj = new RobConf;
           if(!_onlyend)
           {
               for(unsigned int j=0;j<_wkSpace->getRobot(i)->getNumLinks();j++)
               {
                   //const double *pos = states[i]->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1)->values;
                   const double *posRob = state->as<oc::OpenDEStateSpace::StateType>()->getBodyPosition(k);
                   const ob::SO3StateSpace::StateType &oriRob = state->as<oc::OpenDEStateSpace::StateType>()->getBodyRotation(k);
                   //RobConf to store the robots configurations read form the ompl state
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
           else

           {
               rcj->setSE3(basepose);

                   std::vector<KthReal> coords=ComputeRn(state);
                   rcj->setRn(coords);
                   rc.push_back(*rcj);
           }
       }
       smp->setMappedConf(rc);
   }

//! This member function converts an OpenDE State to a Kautham obstacle sample
void KauthamDEPlanner::KauthamOpenDEState2Obssmp(const ob::State *state, Sample* smp,const oc::Control *control,const double duration)
{
    (void) control;//unused
    (void) duration;//unused

    int k=0;
    for(unsigned int i=0; i<_wkSpace->getNumRobots(); i++)
    {
        k = k + _wkSpace->getRobot(i)->getNumLinks();
    }
    vector<RobConf> rc;
    for(unsigned int i=0; i<_wkSpace->getNumObstacles(); i++)
    {
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

//Compute Rn function computes the ODE state to kautham Rn sample
std::vector<float> KauthamDEPlanner::ComputeRn(const ob::State *state)
{
    unsigned int k=0;
    vector<float> coords;
    //std::cout<<"Joint Size :  "<<((KauthamDEEnvironment*)envPtr.get())->_Joint.size()<<std::endl;
    for(unsigned int j=0;j<((KauthamDEEnvironment*)envPtr.get())->_Joint.size();j++)
    {
        const double *posRob1 = state->as<oc::OpenDEStateSpace::StateType>()->getBodyPosition(k);
        const ob::SO3StateSpace::StateType &oriRob1 = state->as<oc::OpenDEStateSpace::StateType>()->getBodyRotation(k);
        const double *posRob2 = state->as<oc::OpenDEStateSpace::StateType>()->getBodyPosition(k+1);
        const ob::SO3StateSpace::StateType &oriRob2 = state->as<oc::OpenDEStateSpace::StateType>()->getBodyRotation(k+1);

        dBodySetPosition(envPtr.get()->stateBodies_[k],posRob1[0],posRob1[1],posRob1[2]);
        dQuaternion qt1={oriRob1.w,oriRob1.x,oriRob1.y,oriRob1.z};
        dBodySetQuaternion(envPtr.get()->stateBodies_[k],qt1);
        dBodySetPosition(envPtr.get()->stateBodies_[k+1],posRob2[0],posRob2[1],posRob2[2]);
        dQuaternion qt2={oriRob2.w,oriRob2.x,oriRob2.y,oriRob2.z};
        dBodySetQuaternion(envPtr.get()->stateBodies_[k+1],qt2);
        dJointType Jtype =dJointGetType(((KauthamDEEnvironment*)envPtr.get())->_Joint[j]);
        if(Jtype==dJointTypeHinge)
        {
            dReal q=dJointGetHingeAngle(((KauthamDEEnvironment*)envPtr.get())->_Joint[j]);
            coords.push_back(-q);
             //std::cout<<q<<" ";

        }
        else if(Jtype==dJointTypeSlider)
        {

            dReal q=dJointGetSliderPosition(((KauthamDEEnvironment*)envPtr.get())->_Joint[j]);
            coords.push_back(q);

        }
        k++;
    }
coords[0]=coords[0];
coords[1]=coords[3];
coords[2]=0;coords[3]=0;coords[4]=0;
    return coords;
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
        distance = distance;
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

            rn  = sqrt(pos[0]*pos[0] + pos[1]*pos[1]);
            rn1 = sqrt(pos1[0]*pos1[0] + pos1[1]*pos1[1]);
            rn2 = sqrt(pos2[0]*pos2[0] + pos2[1]*pos2[1]);
            rn3 = sqrt(pos3[0]*pos3[0] + pos3[1]*pos3[1]);
            //4 points forward difference method to compute the 3rd derivative of position i.e. jerk
            JerkIndex.push_back((-rn + 3*rn1 - 3*rn2 + rn3)/(duration[n]*duration[n+1]*duration[n+2]));
        }
        for(unsigned int i=0;i<duration.size();i++)
        {
            //std::cout<<"Jerk Index: "<<JerkIndex[i]<<"  Smoothness: "<<Smoothness<<std::endl;
            Smoothness = fabs(Smoothness+(duration[i]*(JerkIndex[i+1]+JerkIndex[i])));
            Smoothness = Smoothness/2;
        }

    }
}

void KauthamDEPlanner::ComputePowerConsumed(const std::vector<ob::State*> &states,const std::vector<oc::Control*> &control, const std::vector<double> duration)
{

    std::ofstream Power ("PowerConsumed.txt", std::ios::out | std::ios::app);
    std::ofstream Time ("Time.txt", std::ios::out | std::ios::app);

    int robIndex=0;
    PowerConsumed = 0;
    double time = 0.0;
    for(unsigned int i=0;i<states.size()-1;i++)
    {
        const double* pos1 = states[i]->as<oc::OpenDEStateSpace::StateType>()->getBodyPosition(robIndex);
        const double* pos2 = states[i+1]->as<oc::OpenDEStateSpace::StateType>()->getBodyPosition(robIndex);
        const double* vv= control[i]->as<oc::OpenDEControlSpace::ControlType>()->values;
        double distance =sqrt(pow((pos2[0]-pos1[0]),2) + pow((pos2[1]-pos1[1]),2));// +(pos2[2]-pos1[2])^2);
        double force= sqrt((vv[0]*vv[0])+(vv[1]*vv[1]));
        time=time+duration[i];
        distance = distance;
        PowerConsumed = PowerConsumed + (force * distance)/duration[i];
        if (Power.is_open() && Time.is_open())
        {
           Power <<PowerConsumed<<" ";
           Time <<time<<" ";

        }
    }
    Power.close();
    Time.close();
    //PowerConsumed = PowerConsumed/time;
    std::cout<<"total power: "<<PowerConsumed<<std::endl;
}
void KauthamDEPlanner::ComputePowerConsumed(const std::vector<oc::Control*> &control,const std::vector<double> duration)
{

    std::ofstream Power ("PowerConsumed.txt", std::ios::out | std::ios::app);
    std::ofstream Time ("Time.txt", std::ios::out | std::ios::app);
    PowerConsumed = 0;
    double time = 0.0;
    if(_wkSpace->getRobot(0)->getName()=="SimpleCar")
    {
        for(unsigned int i=0;i<control.size();i++)
        {
              const double* vv= control[i]->as<oc::OpenDEControlSpace::ControlType>()->values;
              PowerConsumed = PowerConsumed + (fabs(vv[1] * vv[2]) + fabs(vv[1]*vv[3])+ fabs(vv[1]*vv[4])+ fabs(vv[1]*vv[5]));
              time=time+duration[i];

            if (Power.is_open() && Time.is_open())
            {
               Power <<PowerConsumed<<" ";
               Time <<time<<" ";

            }
        }
    }
    else
    {
    for(unsigned int i=0;i<control.size();i++)
    {
          const double* vv= control[i]->as<oc::OpenDEControlSpace::ControlType>()->values;
          PowerConsumed = PowerConsumed + (fabs(vv[0] * vv[2]) + fabs(vv[1]*vv[3]));
          time=time+duration[i];

        if (Power.is_open() && Time.is_open())
        {
           Power <<PowerConsumed<<" ";
           Time <<time<<" ";

        }
    }
    }
    Power <<"\n===================================================="<<"\n";
    Power <<"===================================================="<<"\n";
    Time <<"\n===================================================="<<"\n";
    Time <<"===================================================="<<"\n";
    Power.close();
    Time.close();
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
        it = _parameters.find("kinematic chain");
        if(it != _parameters.end())
            _isKchain = it->second;
        else
            return false;
        it = _parameters.find("Max Speed");
        if(it != _parameters.end())
            _maxspeed = it->second;
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
        wkSpace->getRobot(i)->Kinematics(goal->getMappedConf().at(0).getSE3());
        for(unsigned int j=0; j<wkSpace->getRobot(i)->getNumLinks();j++)
        {

            KauthamDEobject odeob;
            odeob.objectposition[0] = wkSpace->getRobot(i)->getLink(j)->getElement()->getPosition()[0];
            odeob.objectposition[1] = wkSpace->getRobot(i)->getLink(j)->getElement()->getPosition()[1];
            odeob.objectposition[2] = wkSpace->getRobot(i)->getLink(j)->getElement()->getPosition()[2];
            odeob.objectorientation[0] = wkSpace->getRobot(i)->getLink(j)->getElement()->getOrientation()[0];
            odeob.objectorientation[1] = wkSpace->getRobot(i)->getLink(j)->getElement()->getOrientation()[1];
            odeob.objectorientation[2] = wkSpace->getRobot(i)->getLink(j)->getElement()->getOrientation()[2];
            odeob.objectorientation[3] = wkSpace->getRobot(i)->getLink(j)->getElement()->getOrientation()[3];
            kauthamob.push_back(odeob);
        }

    }
    return kauthamob;

}
void KauthamDEPlanner::callDrawStuffViewer(void)
{
    dsFunctions fn;
    fn.version = DS_VERSION;
    fn.start = &start;
    fn.step = &simLoop;
    fn.command = &command;
    fn.stop = 0;
    fn.path_to_textures = "/home/muhayyuddin/catkin_ws/src/kautham/textures";

    DISP.addSpace( envPtr.get()->collisionSpaces_[0], 0.9, 0.9, 0.5);
    DISP.addGeoms(((KauthamDEEnvironment*)envPtr.get())->GeomID);

    if(((KauthamDEEnvironment*)envPtr.get())->meshID.size()>1)
    {
        Tmesh tmesh;
        for(unsigned int i=0;i<((KauthamDEEnvironment*)envPtr.get())->meshID.size();i++)
        {

            tmesh.indexSize=((KauthamDEEnvironment*)envPtr.get())->meshID[i].indexSize;
            tmesh.indices=((KauthamDEEnvironment*)envPtr.get())->meshID[i].indices;
            tmesh.vertices=((KauthamDEEnvironment*)envPtr.get())->meshID[i].vertices;
            tmesh.meshD=((KauthamDEEnvironment*)envPtr.get())->meshID[i].meshD;

            DISP.tmd.push_back(tmesh);

        }
        //DISP.tmd.push_back(tmesh);

    }

    //geom color temporary code
    //todo: automatically read the color of SOMaterial and assign to the ODE Geom.
    if(_wkSpace->getRobot(0)->getNumJoints()>1 && _wkSpace->getRobot(0)->getName()!="SimpleCar")
    {
        for( int i=0;i<dSpaceGetNumGeoms(envPtr.get()->collisionSpaces_[0]);i++)

        {
            if(i<6)
                DISP.setGeomColor(((KauthamDEEnvironment*)envPtr.get())->GeomID[i], 0.0, 0.9, 0.9);
            else if(i==6)
                DISP.setGeomColor(((KauthamDEEnvironment*)envPtr.get())->GeomID[i], 0.9, 0.9, 0.1);
            else if(i>6 && i<9)
                DISP.setGeomColor(((KauthamDEEnvironment*)envPtr.get())->GeomID[i], 0.0, 0.0, 0.9);
            else if(i==9)
                DISP.setGeomColor(((KauthamDEEnvironment*)envPtr.get())->GeomID[i], 0.1, 0.9, 0.1);
            else
                DISP.setGeomColor(((KauthamDEEnvironment*)envPtr.get())->GeomID[i], 0.9, 0.1, 0.1);
        }
    }
    if( _wkSpace->getRobot(0)->getName()=="SimpleCar")
    {
        for( int i=0;i<dSpaceGetNumGeoms(envPtr.get()->collisionSpaces_[0]);i++)
        {
            if(i<5)
                DISP.setGeomColor(((KauthamDEEnvironment*)envPtr.get())->GeomID[i], 0.0, 0.9, 0.9);

            else if(i>4 && i<8)
                DISP.setGeomColor(((KauthamDEEnvironment*)envPtr.get())->GeomID[i], 0.0, 0.0, 0.9);
            else
                DISP.setGeomColor(((KauthamDEEnvironment*)envPtr.get())->GeomID[i], 0.9, 0.1, 0.1);
        }

    }
    if( _wkSpace->getRobot(0)->getNumJoints()<2)
    {
        DISP.setGeomColor(((KauthamDEEnvironment*)envPtr.get())->GeomID[0], 0.0, 0.9, 0.9);
        for( int i=1;i<dSpaceGetNumGeoms(envPtr.get()->collisionSpaces_[0]);i++)

        {
            if(i<3)
                DISP.setGeomColor(((KauthamDEEnvironment*)envPtr.get())->GeomID[i], 0.0, 0.0, 0.9);

            else if(i>2 && i<5)
                DISP.setGeomColor(((KauthamDEEnvironment*)envPtr.get())->GeomID[i], 0.9, 0.0, 0.9);
            else
                DISP.setGeomColor(((KauthamDEEnvironment*)envPtr.get())->GeomID[i], 0.9, 0.1, 0.1);
        }
    }

    if(_solved)
    {
        POINTS.clear();
        ob::PlannerData pd(ss->getSpaceInformation());
        ss->getPlannerData(pd);
        std::vector<ob::State*> &stat = ss->getSolutionPath().getStates();
        //for (unsigned int i = 0 ; i < pd.numVertices() ; ++i)
        for(unsigned int i=0; i<stat.size();i++)
        {
            const double *pos = stat[i]->as<oc::OpenDEStateSpace::StateType>()->getBodyPosition(3);
            //const double *pos = pd.getVertex(i).getState()->as<oc::OpenDEStateSpace::StateType>()->getBodyPosition(0);
            POINTS.push_back(std::make_pair(pos[0], pos[1]));
        }
        std::cout<<"States in Solution: "<<ss->getSolutionPath().getStates().size();
        string name=_wkSpace->getRobot(0)->getName();
        boost::thread *th = NULL;
        th = new boost::thread(boost::bind(&playPath, ss,name));
        dsSimulationLoop(0,NULL, 800, 600, &fn);
        if (th)
        {
            th->interrupt();
            th->join();
        }

    }
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
            _wkSpace->moveRobotsTo(worldState[step].first);//.getRob());
            if(_wkSpace->getNumObstacles()>0)
            {
                _wkSpace->moveObstaclesTo(worldState[step].second);//.getObs());
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
            k = stateSpacePtr->getDimension();

        }

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
        ob::ProjectionEvaluatorPtr projToUse = stateSpacePtr->getDefaultProjection();

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
                    Kautham::Vector projection(3);
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
                Kautham::Vector projection(k);
                //&(projection) = new Kautham::Vector;
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


            }

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
                    Kautham::Vector projection(k);
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
                    Kautham::Vector projection(2);
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

        //loop for all vertices of the roadmap or tree and create the coin3D points
        for(unsigned int i=0;i<pdata->numVertices();i++)
        {
            //if(_wkSpace->getRobot(numrob)->isSE3Enabled())
            {
                Kautham::Vector projection(k);
                //&(projection) = new Kautham::Vector;
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


            }

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
                    Kautham::Vector projection(k);
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
z2=0;

                    edgepoints->point.set1Value(1,x2,y2,z2);
                }

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

    }
}

}
}


#endif //KAUTHAM_USE_ODE
#endif //KAUTHAM_USE_OMPL
