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

#include <boost/bind/mem_fn.hpp>

#include <kautham/planner/omplg/omplplanner.h>


#include <Inventor/nodes/SoTranslation.h>
#include <Inventor/nodes/SoCoordinate3.h>
#include <Inventor/nodes/SoCube.h>
#include <Inventor/nodes/SoPointSet.h>
#include <Inventor/nodes/SoLineSet.h>
#include <Inventor/nodes/SoDrawStyle.h>
#include <Inventor/nodes/SoMaterial.h>
#include <Inventor/nodes/SoIndexedLineSet.h>

#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/goals/GoalStates.h>
#include <ompl/base/goals/GoalState.h>

namespace Kautham {
    //! Namespace omplplanner contains the planners based on the OMPL::geometric library
    namespace omplplanner{
        //declaration of class
        class weightedRealVectorStateSpace;


        /////////////////////////////////////////////////////////////////////////////////////////////////
        // weigthedRealVectorStateSpace functions
        /////////////////////////////////////////////////////////////////////////////////////////////////
        //! The constructor initializes all the weights to 1
        weigthedRealVectorStateSpace::weigthedRealVectorStateSpace(unsigned int dim) : RealVectorStateSpace(dim)
        {
            //by default set all the weights to 1
            for (unsigned i=0; i<dim; i++)
            {
                weights.push_back(1.0);
            }
        }

        //! The destructor
        weigthedRealVectorStateSpace::~weigthedRealVectorStateSpace(void){}


        //! This function sets the values of the weights. The values passed as a parameter are scaled in order not to change the maximim extend of the space
        void weigthedRealVectorStateSpace::setWeights(vector<KthReal> w)
        {
            double fitFactor;

            //compute the maximum weigthed distance
            double maxweightdist=0.0;
            for (unsigned i=0; i<dimension_; i++)
            {
                double diff = getBounds().getDifference()[i]*w[i];
                maxweightdist += diff * diff;
            }
            maxweightdist = sqrt(maxweightdist);
            //compute the scale factor
            fitFactor = getMaximumExtent()/maxweightdist;
            //set the weights
            for (unsigned i=0; i<dimension_; i++)
            {
                weights[i] = w[i]*fitFactor;
            }
        }

        //! This function computes the weighted distance between states
        double weigthedRealVectorStateSpace::distance(const ob::State *state1, const ob::State *state2) const
        {
            double dist = 0.0;
            const double *s1 = static_cast<const StateType*>(state1)->values;
            const double *s2 = static_cast<const StateType*>(state2)->values;

            for (unsigned int i = 0 ; i < dimension_ ; ++i)
            {
                double diff = ((*s1++) - (*s2++))*weights[i];
                dist += diff * diff;
            }
            return sqrt(dist);
        }



        /////////////////////////////////////////////////////////////////////////////////////////////////
        // KauthamStateSampler functions
        /////////////////////////////////////////////////////////////////////////////////////////////////
        KauthamStateSampler::KauthamStateSampler(const ob::StateSpace *sspace, Planner *p) : ob::CompoundStateSampler(sspace)
        {
            kauthamPlanner_ = p;
            centersmp = NULL;
            _samplerRandom = new RandomSampler(kauthamPlanner_->wkSpace()->getNumRobControls());
            // _samplerHalton = new HaltonSampler(kauthamPlanner_->wkSpace()->getNumRobControls());
        }

        KauthamStateSampler::~KauthamStateSampler() {
            delete _samplerRandom;
            delete centersmp;
        }

        void KauthamStateSampler::setCenterSample(ob::State *state, double th)
        {
            if(state!=NULL)
            {
                //create sample
                int d = kauthamPlanner_->wkSpace()->getNumRobControls();
                centersmp = new Sample(d);
                //copy the conf of the init smp. Needed to capture the home positions.
                centersmp->setMappedConf(kauthamPlanner_->initSamp()->getMappedConf());
                //load the RobConf of smp form the values of the ompl::state
                ((omplPlanner*)kauthamPlanner_)->omplState2smp(state,centersmp);
            }
            else
                centersmp = NULL;

            //initialize threshold
            threshold = th;
        }

        void KauthamStateSampler::sampleUniform(ob::State *state)
        {

            //Sample around centersmp
            //this does the same as sampleUniformNear, but the Near configuration is set beforehand as "centersmp" configuration.
            //this has been added to modify the behavior of the randombounce walk of the PRM. It used the sampleUniform and
            //we wanted to use the sampleUniformNear

            ob::ScopedState<ob::CompoundStateSpace> sstate(  ((omplPlanner*)kauthamPlanner_)->getSpace() );

            if(centersmp != NULL && threshold > 0.0)
            {
                int trials = 0;
                int maxtrials=100;
                bool found = false;
                int d = kauthamPlanner_->wkSpace()->getNumRobControls();
                Sample *smp = new Sample(d);
                Sample *smp2 = new Sample(d);
                double dist;

                bool withinbounds=false;
                vector<KthReal> deltacoords(d);
                vector<KthReal> coords(d);
                double fraction;
                do{
                    //sample the kautham control space. Controls are defined in the input xml files. Eeach control value lies in the [0,1] interval
                    for (int i=0;i<d;i++)
                        coords[i] = rng_.uniformReal(0,1.0);
                    //those controls that are disabled for sampling are now restored to 0.5
                    for (unsigned j=0; j < ((omplPlanner*)kauthamPlanner_)->getDisabledControls()->size(); j++)
                        coords[ ((omplPlanner*)kauthamPlanner_)->getDisabledControls()->at(j) ] = 0.5;
                    //load the obtained coords to a sample, and compute the mapped configurations (i.e.se3+Rn values) by calling MoveRobotsto function.
                    smp2->setCoords(coords);
                    kauthamPlanner_->wkSpace()->moveRobotsTo(smp2);
                    //interpolate from the centersample towards smp a fraction determined by the threshold

                    dist = kauthamPlanner_->wkSpace()->distanceBetweenSamples(*smp2,*centersmp,CONFIGSPACE);
                    if(trials==0){
                        fraction = rng_.uniformReal(0,1.0)*threshold/dist;
                        if(fraction>1.0) fraction=1.0;
                    }
                    smp = centersmp->interpolate(smp2,fraction);
                    kauthamPlanner_->wkSpace()->moveRobotsTo(smp);
                    //check
                    //double dist1 = kauthamPlanner_->wkSpace()->distanceBetweenSamples(*smp,*centersmp,CONFIGSPACE);

                    withinbounds = smp->getwithinbounds();
                    if(withinbounds==false) fraction = fraction-fraction/maxtrials;
                    else found = true;
                    trials ++;
                }while(found==false && trials <maxtrials);


                if(trials==maxtrials)
                {
                    //not found within the limits. return the centersmp
                    ((omplPlanner*)kauthamPlanner_)->smp2omplScopedState(centersmp, &sstate);
                }
                else
                {
                    //convert the sample found to scoped state
                    ((omplPlanner*)kauthamPlanner_)->smp2omplScopedState(smp, &sstate);
                }

                //return in parameter state
                ((omplPlanner*)kauthamPlanner_)->getSpace()->copyState(state, sstate.get());
            }
            //sample the whole workspace
            else
            {


                bool withinbounds=false;
                int trials=0;
                Sample* smp = NULL;
                do{
                    //sample the kautham control space. Controls are defined in the input xml files. Eeach control value lies in the [0,1] interval
                    smp = _samplerRandom->nextSample();
                    //smp = _samplerHalton->nextSample();

                    //those controls that are disabled for sampling are now restored to 0.5
                    for (unsigned j=0; j<((omplPlanner*)kauthamPlanner_)->getDisabledControls()->size(); j++)
                        smp->getCoords()[ ((omplPlanner*)kauthamPlanner_)->getDisabledControls()->at(j) ] = 0.5;

                    //compute the mapped configurations (i.e.se3+Rn values) by calling MoveRobotsto function.
                    kauthamPlanner_->wkSpace()->moveRobotsTo(smp);
                    withinbounds = smp->getwithinbounds();
                    trials++;
                }while(withinbounds==false && trials<100);


                //If trials==100 is because we have not been able to find a sample within limits
                //In this case the config is set to the border in the moveRobotsTo function.
                //The smp is finally converted to state and returned

                //convert from sample to scoped state
                ((omplPlanner*)kauthamPlanner_)->smp2omplScopedState(smp, &sstate);
                //return in parameter state
                ((omplPlanner*)kauthamPlanner_)->getSpace()->copyState(state, sstate.get());

            }
            //JAN DEBUG
            //temporary store for visualizaytion purposes
            int dim = kauthamPlanner_->wkSpace()->getNumRobControls();
            Sample* smp2 = new Sample(dim);
            //convert back to a sample to make sure that the state has been well computed
            smp2->setMappedConf(((omplPlanner*)kauthamPlanner_)->initSamp()->getMappedConf());//NEEDED
            ((omplPlanner*)kauthamPlanner_)->omplScopedState2smp(sstate, smp2);

            vector<double> point(3);

            if(kauthamPlanner_->wkSpace()->getRobot(0)->isSE3Enabled())
            {
                point[0] = smp2->getMappedConf()[0].getSE3().getPos()[0];
                point[1] = smp2->getMappedConf()[0].getSE3().getPos()[1];
                point[2] = smp2->getMappedConf()[0].getSE3().getPos()[2];
            }
            else{
                point[0] = smp2->getMappedConf()[0].getRn().getCoordinate(0);
                point[1] = smp2->getMappedConf()[0].getRn().getCoordinate(1);
                if(dim>=3) point[2] = smp2->getMappedConf()[0].getRn().getCoordinate(2);
                else point[2]=0.0;
            }
        }


        void KauthamStateSampler::sampleUniformNear(ob::State *state, const ob::State *near, const double distance)
        {
            int trials = 0;
            int maxtrials=100;
            bool found = false;
            Sample *smp;
            do{
                bool withinbounds=false;
                int trialsbounds=0;
                do{
                    //sample the kautham control space. Controls are defined in the input xml files. Eeach control value lies in the [0,1] interval
                    smp = _samplerRandom->nextSample();
                    //load the obtained coords to a sample, and compute the mapped configurations (i.e.se3+Rn values) by calling MoveRobotsto function.
                    kauthamPlanner_->wkSpace()->moveRobotsTo(smp);
                    withinbounds = smp->getwithinbounds();
                    trialsbounds++;
                }while(withinbounds==false && trialsbounds<100);
                //convert from sample to scoped state
                ob::ScopedState<ob::CompoundStateSpace> sstate(  ((omplPlanner*)kauthamPlanner_)->getSpace() );
                ((omplPlanner*)kauthamPlanner_)->smp2omplScopedState(smp, &sstate);
                //return the stae in the parameter state and a bool telling if the smp is in collision or not
                ((omplPlanner*)kauthamPlanner_)->getSpace()->copyState(state, sstate.get());
                if (((omplPlanner*)kauthamPlanner_)->getSpace()->distance(state,near)> distance)
                    found = false;
                else
                    found=true;
                trials ++;
            }while(found==false && trials <maxtrials);

            if (!found){
                ((omplPlanner*)kauthamPlanner_)->getSpace()->copyState(state, near);
            }

        }


        /////////////////////////////////////////////////////////////////////////////////////////////////
        // KauthamValidStateSampler functions
        /////////////////////////////////////////////////////////////////////////////////////////////////
        //! Creator. The parameter samplername is defaulted to "Random" and the value to "0.0
        KauthamValidStateSampler::KauthamValidStateSampler(const ob::SpaceInformation *si, Planner *p) : ob::ValidStateSampler(si)
        {
            name_ = "kautham sampler";
            kauthamPlanner_ = p;
            si_ = si;
            //be careful these values should be set somehow!
            int level = 3;
            KthReal sigma = 0.1;

            _samplerRandom = new RandomSampler(kauthamPlanner_->wkSpace()->getNumRobControls());
            _samplerHalton = new HaltonSampler(kauthamPlanner_->wkSpace()->getNumRobControls());
            _samplerSDK = new SDKSampler(kauthamPlanner_->wkSpace()->getNumRobControls(), level);
            _samplerGaussian = new GaussianSampler(kauthamPlanner_->wkSpace()->getNumRobControls(), sigma, kauthamPlanner_->wkSpace());

            _samplerVector.push_back(_samplerRandom);
            _samplerVector.push_back(_samplerHalton);
            _samplerVector.push_back(_samplerSDK);
            _samplerVector.push_back(_samplerGaussian);
        }


        //!Gets a sample. The samplername parameter is defaulted to Random.
        //bool KauthamValidStateSampler::sample(ob::State *state, string samplername)
        bool KauthamValidStateSampler::sample(ob::State *state)
        {
            //gets a new sample using the sampler specified by the planner
            Sample* smp = NULL;
            unsigned numSampler = ((omplPlanner*)kauthamPlanner_)->getSamplerUsed();
            if(numSampler>= _samplerVector.size()) numSampler = 0;//set default Random sampler if out of bounds value
            smp = _samplerVector[numSampler]->nextSample();

            //those controls that are disabled for sampling are now restored to 0.5
            for (unsigned j=0; j<((omplPlanner*)kauthamPlanner_)->getDisabledControls()->size(); j++)
                smp->getCoords()[ ((omplPlanner*)kauthamPlanner_)->getDisabledControls()->at(j) ] = 0.5;

            //computes the mapped configurations (i.e.se3+Rn values) by calling MoveRobotsto function.
            kauthamPlanner_->wkSpace()->moveRobotsTo(smp);

            //convert from sample to scoped state
            ob::ScopedState<ob::CompoundStateSpace> sstate(  ((omplPlanner*)kauthamPlanner_)->getSpace() );
            ((omplPlanner*)kauthamPlanner_)->smp2omplScopedState(smp, &sstate);

            //return the stae in the parameter state and a bool telling if the smp is in collision or not
            ((omplPlanner*)kauthamPlanner_)->getSpace()->copyState(state, sstate.get());

            if(  (si_->satisfiesBounds(state)==false) || (kauthamPlanner_->wkSpace()->collisionCheck(smp)) )
                return false;
            return true;
        }

        //!Gets a sample near a given state, after several trials (retruns false if not found)
        bool KauthamValidStateSampler::sampleNear(ob::State *state, const ob::State *near, const double distance)
        {
            int trials = 0;
            int maxtrials=100;
            bool found = false;
            do{
                //get a random sample, and compute the mapped configurations (i.e.se3+Rn values) by calling MoveRobotsto function.
                Sample* smp = NULL;
                int numSampler = 0; //Random sampler
                smp = _samplerVector[numSampler]->nextSample();

                //those controls that are disabled for sampling are now restored to 0.5
                for (unsigned j=0; j<((omplPlanner*)kauthamPlanner_)->getDisabledControls()->size(); j++)
                    smp->getCoords()[ ((omplPlanner*)kauthamPlanner_)->getDisabledControls()->at(j) ] = 0.5;


                kauthamPlanner_->wkSpace()->moveRobotsTo(smp);
                //convert from sample to scoped state
                ob::ScopedState<ob::CompoundStateSpace> sstate(  ((omplPlanner*)kauthamPlanner_)->getSpace() );
                ((omplPlanner*)kauthamPlanner_)->smp2omplScopedState(smp, &sstate);
                //return the stae in the parameter state and a bool telling if the smp is in collision or not
                ((omplPlanner*)kauthamPlanner_)->getSpace()->copyState(state, sstate.get());
                if( kauthamPlanner_->wkSpace()->collisionCheck(smp) | (((omplPlanner*)kauthamPlanner_)->getSpace()->distance(state,near)> distance) | !(si_->satisfiesBounds(state)))
                    found = false;
                else
                    found=true;
                trials ++;
            }while(found==false && trials <maxtrials);
            return found;
            //throw ompl::Exception("KauthamValidStateSampler::sampleNear", "not implemented");
            //return false;
        }


        /////////////////////////////////////////////////////////////////////////////////////////////////
        // AUXILIAR functions
        /////////////////////////////////////////////////////////////////////////////////////////////////

        /////////////////////////////////////////////////////////////////////////////////////////////////
        //! This function is used to allocate a state sampler
        ob::StateSamplerPtr allocStateSampler(const ob::StateSpace *mysspace, Planner *p)
        {
            return ob::StateSamplerPtr(new KauthamStateSampler(mysspace, p));
        }

        /////////////////////////////////////////////////////////////////////////////////////////////////
        //! This function is used to allocate a valid state sampler
        ob::ValidStateSamplerPtr allocValidStateSampler(const ob::SpaceInformation *si, Planner *p)
        {
            return ob::ValidStateSamplerPtr(new KauthamValidStateSampler(si, p));
        }

        /////////////////////////////////////////////////////////////////////////////////////////////////
        // omplPlanner functions
        /////////////////////////////////////////////////////////////////////////////////////////////////
        //! Constructor
        omplPlanner::omplPlanner(SPACETYPE stype, Sample *init, Sample *goal, SampleSet *samples, WorkSpace *ws, og::SimpleSetup *ssptr):
            Planner(stype, init, goal, samples, ws)
        {
            _family = OMPLPLANNER;
            //set intial values from parent class data
            _speedFactor = 1;
            _solved = false;
            _guiName = "ompl Planner";
            _idName = "ompl Planner";

            _samplerUsed = 0;
            _validSegmentCount = 5;//number of samples per range step

            //set own intial values
            _planningTime = 10;
            _simplify = 2;//by default shorten and smooth
            _interpolate = true;//by default interpolate the computed path
            _incremental = 0;//by default makes a clear before any new call to solve in function trysolve().
            _drawnrobot = 0; //by default we draw the cspace of robot 0.
            _drawnPath = true; //by default we show the path into the workspace

            //add planner parameters
            addParameter("Incremental (0/1)",_incremental);
            addParameter("Max Planning Time",_planningTime);
            addParameter("Speed Factor",_speedFactor);
            addParameter("Simplify Solution",_simplify);
            addParameter("Cspace Drawn",_drawnrobot);
            addParameter("Path Drawn (0/1)",_drawnPath);


            if (ssptr == NULL) {
                //Construct the state space we are planning in. It is a compound state space composed of a compound state space for each robot
                //Each robot has a compound state space composed of a (optional) SE3 state space and a (optional) Rn state space
                vector<ob::StateSpacePtr> spaceRn;
                vector<ob::StateSpacePtr> spaceSE3;
                vector<ob::StateSpacePtr> spaceRob;
                vector< double > weights;

                spaceRn.resize(_wkSpace->getNumRobots());
                spaceSE3.resize(_wkSpace->getNumRobots());
                spaceRob.resize(_wkSpace->getNumRobots());
                weights.resize(_wkSpace->getNumRobots());

                //loop for all robots
                for (unsigned i=0; i<_wkSpace->getNumRobots(); i++)
                {
                    vector<ob::StateSpacePtr> compoundspaceRob;
                    vector< double > weightsRob;
                    std::stringstream sstm;

                    //create state space SE3 for the mobile base, if necessary
                    if(_wkSpace->getRobot(i)->isSE3Enabled())
                    {
                        //create the SE3 state space
                        spaceSE3[i] = ((ob::StateSpacePtr) new ob::SE3StateSpace());
                        sstm << "ssRobot" << i<<"_SE3";
                        spaceSE3[i]->setName(sstm.str());

                        //set the bounds. If the bounds are equal or its difference is below a given epsilon value (0.001) then
                        //set the higher bound to the lower bound plus this eplsion
                        ob::RealVectorBounds bounds(3);

                        //x-direction
                        double low = _wkSpace->getRobot(i)->getLimits(0)[0];
                        double high = _wkSpace->getRobot(i)->getLimits(0)[1];
                        filterBounds(low, high, 0.001);
                        bounds.setLow(0, low);
                        bounds.setHigh(0, high);

                        //y-direction
                        low = _wkSpace->getRobot(i)->getLimits(1)[0];
                        high = _wkSpace->getRobot(i)->getLimits(1)[1];
                        filterBounds(low, high, 0.001);
                        bounds.setLow(1, low);
                        bounds.setHigh(1, high);

                        //z-direction
                        low = _wkSpace->getRobot(i)->getLimits(2)[0];
                        high = _wkSpace->getRobot(i)->getLimits(2)[1];
                        filterBounds(low, high, 0.001);
                        bounds.setLow(2, low);
                        bounds.setHigh(2, high);

                        spaceSE3[i]->as<ob::SE3StateSpace>()->setBounds(bounds);

                        //create projections evaluator for this spaces -
                        //The default projections (needed for some planners) and
                        //the projections called "drawprojections"for the drawcspace function.
                        //(the use of defaultProjections for drawspace did not succeed because the ss->setup() calls registerProjections() function that
                        //for the case of RealVectorStateSpace sets the projections as random for dim>2 and identity otherwise, thus
                        //resetting what the user could have tried to do.
                        ob::ProjectionEvaluatorPtr peR3; //projection for R3
                        peR3 = (ob::ProjectionEvaluatorPtr) new ob::RealVectorIdentityProjectionEvaluator(spaceSE3[i]->as<ob::SE3StateSpace>()->getSubspace(0));
                        peR3->setup();//??
                        spaceSE3[i]->as<ob::SE3StateSpace>()->getSubspace(0)->registerProjection("drawprojection",peR3);
                        spaceSE3[i]->as<ob::SE3StateSpace>()->getSubspace(0)->registerDefaultProjection(peR3);
                        ob::ProjectionEvaluatorPtr peSE3; //projection for SE3
                        ob::ProjectionEvaluatorPtr projToUse = spaceSE3[i]->as<ob::CompoundStateSpace>()->getSubspace(0)->getProjection("drawprojection");
                        peSE3 = (ob::ProjectionEvaluatorPtr) new ob::SubspaceProjectionEvaluator(&*spaceSE3[i],0,projToUse);
                        peSE3->setup(); //necessary to set projToUse as theprojection
                        spaceSE3[i]->registerProjection("drawprojection",peSE3);
                        spaceSE3[i]->registerDefaultProjection(peSE3);

                        //sets the weights between translation and rotation
                        spaceSE3[i]->as<ob::SE3StateSpace>()->setSubspaceWeight(0,_wkSpace->getRobot(i)->getWeightSE3()[0]);//translational weight
                        spaceSE3[i]->as<ob::SE3StateSpace>()->setSubspaceWeight(1,_wkSpace->getRobot(i)->getWeightSE3()[1]);//rotational weight

                        //load to the compound state space of robot i
                        compoundspaceRob.push_back(spaceSE3[i]);
                        weightsRob.push_back(1);
                    }

                    //create the Rn state space for the kinematic chain, if necessary
                    int nj = _wkSpace->getRobot(i)->getNumJoints();
                    if(nj>0)
                    {
                        //create the Rn state space
                        spaceRn[i] = ((ob::StateSpacePtr) new weigthedRealVectorStateSpace(nj));
                        sstm << "ssRobot" << i<<"_Rn";
                        spaceRn[i]->setName(sstm.str());

                        //create projections evaluator for this spaces
                        ob::ProjectionEvaluatorPtr peRn;
                        peRn = ((ob::ProjectionEvaluatorPtr) new ob::RealVectorIdentityProjectionEvaluator(spaceRn[i]));
                        peRn->setup();
                        spaceRn[i]->registerProjection("drawprojection",peRn);
                        spaceRn[i]->registerDefaultProjection(peRn);

                        // set the bounds and the weights
                        vector<KthReal> jointweights;
                        ob::RealVectorBounds bounds(nj);
                        double low, high;
                        for (int j=0; j<nj;j++)
                        {
                            //the limits of joint j between link j and link (j+1) are stroed in the data structure of link (j+1)
                            low = *_wkSpace->getRobot(i)->getLink(j+1)->getLimits(true);
                            high = *_wkSpace->getRobot(i)->getLink(j+1)->getLimits(false);
                            filterBounds(low, high, 0.001);
                            bounds.setLow(j, low);
                            bounds.setHigh(j, high);
                            //the weights
                            jointweights.push_back(_wkSpace->getRobot(i)->getLink(j+1)->getWeight());
                        }
                        spaceRn[i]->as<weigthedRealVectorStateSpace>()->setBounds(bounds);
                        spaceRn[i]->as<weigthedRealVectorStateSpace>()->setWeights(jointweights);

                        //load to the compound state space of robot i
                        compoundspaceRob.push_back(spaceRn[i]);
                        weightsRob.push_back(1);
                    }
                    //the compound state space for robot i is (SE3xRn), and either SE3 or Rn may be missing
                    spaceRob[i] = ((ob::StateSpacePtr) new ob::CompoundStateSpace(compoundspaceRob,weightsRob));
                    weights[i] = 1;
                    sstm.str("");
                    sstm << "ssRobot" << i;
                    spaceRob[i]->setName(sstm.str());

                    ob::ProjectionEvaluatorPtr peRob;
                    ob::ProjectionEvaluatorPtr projToUse = spaceRob[i]->as<ob::CompoundStateSpace>()->getSubspace(0)->getProjection("drawprojection");
                    peRob = (ob::ProjectionEvaluatorPtr) new ob::SubspaceProjectionEvaluator(&*spaceRob[i],0,projToUse);
                    peRob->setup();
                    spaceRob[i]->registerProjection("drawprojection",peRob);
                    spaceRob[i]->registerDefaultProjection(peRob);
                }
                //the state space for the set of robots. All the robots have the same weight.
                space = ((ob::StateSpacePtr) new ob::CompoundStateSpace(spaceRob,weights));

                vector<ob::ProjectionEvaluatorPtr> peSpace;
                for (unsigned i=0; i<_wkSpace->getNumRobots();i++)
                {
                    ob::ProjectionEvaluatorPtr projToUse = space->as<ob::CompoundStateSpace>()->getSubspace(i)->getProjection("drawprojection");
                    peSpace.push_back( (ob::ProjectionEvaluatorPtr) new ob::SubspaceProjectionEvaluator(&*space,i,projToUse) );
                    peSpace[i]->setup();
                    string projname = "drawprojection"; //
                    string robotnumber = static_cast<ostringstream*>( &(ostringstream() << i) )->str();//the string correspoding to number i
                    projname.append(robotnumber); //the name of the projection: "drawprojection0", "drawprojection1",...
                    space->registerProjection(projname.c_str(),peSpace[i]);
                }
                space->registerDefaultProjection(peSpace[0]);//the one corresponding to the first robot is set as default

                //create simple setup
                ss = ((og::SimpleSetupPtr) new og::SimpleSetup(space));
                si=ss->getSpaceInformation();
                //set validity checker
                si->setStateValidityChecker(ob::StateValidityCheckerPtr(new ValidityChecker(si,  (Planner*)this)));
                //ss->setStateValidityChecker(std::bind(&omplplanner::isStateValid, si.get(),std::placeholders::_1, (Planner*)this));

                //Add start states
                ss->clearStartStates();
                for (std::vector<Sample*>::const_iterator start(_init.begin());
                     start != _init.end(); ++start) {
                    //Start state: convert from smp to scoped state
                    ob::ScopedState<ob::CompoundStateSpace> startompl(space);
                    smp2omplScopedState(*start,&startompl);
                    cout << "startompl:" << endl;
                    startompl.print();
                    ss->addStartState(startompl);
                }

                //Add goal states
                ob::GoalStates *goalStates(new ob::GoalStates(si));
                for (std::vector<Sample*>::const_iterator goal(_goal.begin());
                     goal != _goal.end(); ++goal) {
                    //Goal state: convert from smp to scoped state
                    ob::ScopedState<ob::CompoundStateSpace> goalompl(space);
                    smp2omplScopedState(*goal,&goalompl);
                    cout << "goalompl:" << endl;
                    goalompl.print();
                    goalStates->addState(goalompl);
                }
                ss->setGoal(ob::GoalPtr(goalStates));

                //alloc valid state sampler
                si->setValidStateSamplerAllocator(std::bind(&allocValidStateSampler, std::placeholders::_1, (Planner*)this));
                //alloc state sampler
                space->setStateSamplerAllocator(std::bind(&allocStateSampler, std::placeholders::_1, (Planner*)this));
            } else {
                ss = (og::SimpleSetupPtr)ssptr;
                si = ss->getSpaceInformation();
                space = ss->getStateSpace();
            }
            space->setup();
        }


        //! void destructor
        omplPlanner::~omplPlanner(){

        }


        /*!
     * disablePMDControlsFromSampling disables from sampling those controls that have the PMD in its name.
     * \param enableall if its true the function is used to enable all, It is defaulted to false.
     */
        void omplPlanner::disablePMDControlsFromSampling(bool enableall)
        {
            _disabledcontrols.clear();
            //enable all
            if(enableall)
            {
                return;
            }
            //else diable those that are called PMD


            string listcontrolsname = wkSpace()->getRobControlsName();
            vector<string*> controlname;
            string *newcontrol = new string;
            for (unsigned i=0; i<listcontrolsname.length();i++)
            {
                if(listcontrolsname[i]=='|')
                {
                    controlname.push_back(newcontrol);
                    newcontrol = new string;
                }
                else
                    newcontrol->push_back(listcontrolsname[i]);
            }
            //add last control (since listcontrolsname does not end with a |)
            controlname.push_back(newcontrol);

            for (unsigned i=0;i<controlname.size();i++)
            {
                if(controlname[i]->find("PMD") != string::npos)
                {
                    //Load to the diable vector for disabling sampling. We do not want to sample coupled controls.
                    //JAN DEBUG: commented next line
                    _disabledcontrols.push_back(i);
                }
            }
        }


        //! This function setParameters sets the parameters of the planner
        bool omplPlanner::setParameters(){
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


                it = _parameters.find("Cspace Drawn");
                if(it != _parameters.end()){
                    if(it->second < 0 || it->second >= _wkSpace->getNumRobots()) {
                        _drawnrobot = 0;
                        setParameter("Cspace Drawn",0);
                    } else {
                        _drawnrobot = it->second;
                    }
                }
                else
                    return false;

                it = _parameters.find("Path Drawn (0/1)");
                if (it != _parameters.end()) {
                    if (it->second == 0.0) {
                        _drawnPath = false;
                    } else if (it->second == 1.0) {
                            _drawnPath = true;
                    } else {
                        setParameter("Path Drawn (0/1)", _drawnPath);
                    }
                }

                it = _parameters.find("Simplify Solution");
                if(it != _parameters.end())
                {
                    if(it->second==0) _simplify=0;
                    else if(it->second==1) _simplify=1;
                    else _simplify=2;
                }
                else
                    return false;

                it = _parameters.find("Incremental (0/1)");
                if (it != _parameters.end()) {
                    _incremental = (it->second == 1);
                }
                else
                    return false;

            }catch(...){
                return false;
            }
            return true;
        }


        //! This function is used to verify that the low bound is below the high bound
        void omplPlanner::filterBounds(double &l, double &h, double epsilon)
        {
            if((h - l) < epsilon) h = l + epsilon;
        }


        //! This function creates the separator for the ivscene to show the path.
        SoSeparator *omplPlanner::getIvPathScene()
        {
            _scenePath = new SoSeparator();
            _scenePath->ref();
            return Planner::getIvPathScene();
        }


        //! This function creates the separator for the ivscene to show the configuration space.
        SoSeparator *omplPlanner::getIvCspaceScene()
        {
            _sceneCspace = new SoSeparator();
            _sceneCspace->ref();
            return Planner::getIvCspaceScene();
        }


        //! This function draws the projection of the Configuration Space into the Workspace using the the position of the trunk link
        void omplPlanner::drawPath(bool show) {
            if (!_scenePath) return;

            //Delete whatever is already drawn
            _scenePath->removeAllChildren();

            if (!show) return;

            //Load the planner data to be drawn
            ob::PlannerDataPtr pdata(new ob::PlannerData(ss->getSpaceInformation()));
            ss->getPlanner()->getPlannerData(*pdata);

            if (ss->getPlanner()->getProblemDefinition()->hasOptimizationObjective()) {
                //Compute the weight for all edges given the OptimizationObjective
                pdata->computeEdgeWeights(*ss->getPlanner()->getProblemDefinition()->
                                          getOptimizationObjective());
            } else {
                //Compute all edge weights using state space distance
                pdata->computeEdgeWeights();
            }

            //Draw tree
            Sample *smp(new Sample(_wkSpace->getNumRobControls()));
            smp->setMappedConf(_init.at(0)->getMappedConf());
            ob::GoalStates *goalStates(dynamic_cast<ob::GoalStates*>
                                       (ss->getProblemDefinition()->getGoal().get()));

            //Nothing to draw
            if (pdata->numVertices() < (pdata->numStartVertices()+pdata->numGoalVertices())) {
                std::cout<<"omplPlanner::drawPath - nothing to draw"<<std::endl;
                return;
            }


            SbVec3f *goalVertices = new SbVec3f[goalStates->getStateCount()*_wkSpace->getNumRobots()];
            for (unsigned int i(0); i < goalStates->getStateCount(); ++i) {
                omplState2smp(goalStates->getState(i)->as<ob::CompoundStateSpace::StateType>(),smp);
                _wkSpace->moveRobotsTo(smp);
                for (unsigned int j(0); j < _wkSpace->getNumRobots(); ++j) {
                    mt::Point3 point(_wkSpace->getRobot(j)->
                                     getLink(_wkSpace->getRobot(j)->getTrunk()-1)->
                                     getTransformation()->getTranslation());

                    goalVertices[j*goalStates->getStateCount()+i][0] = point[0];
                    goalVertices[j*goalStates->getStateCount()+i][1] = point[1];
                    goalVertices[j*goalStates->getStateCount()+i][2] = point[2];
                }
            }
            SbVec3f *vertices = new SbVec3f[pdata->numVertices()*_wkSpace->getNumRobots()];
            SbVec3f *startVertices = new SbVec3f[pdata->numStartVertices()*_wkSpace->getNumRobots()];
            SbVec3f *otherVertices = new SbVec3f[(pdata->numVertices()-pdata->numStartVertices()-
                                                  pdata->numGoalVertices())*_wkSpace->getNumRobots()];

            int32_t *coordIndices = new int32_t[3*pdata->numEdges()*_wkSpace->getNumRobots()];
            int32_t *edgeColorIndices = new int32_t[pdata->numEdges()*_wkSpace->getNumRobots()];


            unsigned int le(0), ls(0), lo(0);
            for (unsigned int i = 0; i < pdata->numVertices(); ++i) {
                omplState2smp(pdata->getVertex(i).getState()->as<ob::CompoundStateSpace::StateType>(),smp);
                _wkSpace->moveRobotsTo(smp);
                for (unsigned int j(0); j < _wkSpace->getNumRobots(); ++j) {
                    mt::Point3 point(_wkSpace->getRobot(j)->
                                     getLink(_wkSpace->getRobot(j)->getTrunk()-1)->
                                     getTransformation()->getTranslation());

                    vertices[j*pdata->numVertices()+i][0] = point[0];
                    vertices[j*pdata->numVertices()+i][1] = point[1];
                    vertices[j*pdata->numVertices()+i][2] = point[2];

                    if (pdata->isStartVertex(i)) {
                        startVertices[ls][0] = point[0];
                        startVertices[ls][1] = point[1];
                        startVertices[ls][2] = point[2];
                        ls++;
                    } else if (pdata->isGoalVertex(i)) {

                    } else {
                        otherVertices[lo][0] = point[0];
                        otherVertices[lo][1] = point[1];
                        otherVertices[lo][2] = point[2];
                        lo++;
                    }
                }

                std::vector<unsigned int> outgoingVertices;
                pdata->getEdges(i,outgoingVertices);
                for (std::vector<unsigned int>::const_iterator it(outgoingVertices.begin());
                     it != outgoingVertices.end(); ++it) {
                    for (unsigned int j(0); j < _wkSpace->getNumRobots(); ++j) {
                        coordIndices[3*le+0] = j*pdata->numVertices()+i;
                        coordIndices[3*le+1] = j*pdata->numVertices()+*it;
                        coordIndices[3*le+2] = SO_END_LINE_INDEX;

                        ob::Cost edgeWeight;
                        pdata->getEdgeWeight(i,*it,&edgeWeight);

                        edgeColorIndices[le] = pdata->getVertex(i).getTag();

                        le++;
                    }
                }
            }

            SoDrawStyle *drawStyle(new SoDrawStyle);
            drawStyle->lineWidth = 1;
            drawStyle->pointSize = 3;

            SoVertexProperty *vertexProperty(new SoVertexProperty);
            vertexProperty->vertex.setValues(0,pdata->numVertices()*_wkSpace->getNumRobots(),vertices);
            uint32_t edgeColors[3] = {SbColor(0,1,0.8).getPackedValue(),
                                      SbColor(0,1,0.2).getPackedValue(),
                                      SbColor(0,0.6,1).getPackedValue()};
            vertexProperty->orderedRGBA.setValues(0,3,edgeColors);
            vertexProperty->materialBinding.setValue(SoVertexProperty::PER_FACE_INDEXED);

            SoIndexedLineSet *lineSet(new SoIndexedLineSet);
            lineSet->coordIndex.setValues(0,3*pdata->numEdges()*_wkSpace->getNumRobots(),coordIndices);
            lineSet->vertexProperty.setValue(vertexProperty);
            lineSet->materialIndex.setValues(0,pdata->numEdges()*_wkSpace->getNumRobots(),edgeColorIndices);

            SoSeparator *lines(new SoSeparator);
            lines->setName("Edges");
            lines->addChild(drawStyle);
            lines->addChild(lineSet);

            _scenePath->addChild(lines);

            vertexProperty = new SoVertexProperty;
            vertexProperty->vertex.setValues(0,(pdata->numVertices()-pdata->numStartVertices()-
                                             pdata->numGoalVertices())*_wkSpace->getNumRobots(),otherVertices);
            vertexProperty->orderedRGBA.setValue(SbColor(1,1,1).getPackedValue());

            SoPointSet *pointSet(new SoPointSet);
            pointSet->vertexProperty.setValue(vertexProperty);

            SoSeparator *points(new SoSeparator);
            points->setName("NormalVertices");
            points->addChild(drawStyle);
            points->addChild(pointSet);

            _scenePath->addChild(points);

            //Draw path
            if (_solved) {
                SoDrawStyle *drawStyle(new SoDrawStyle);
                drawStyle->lineWidth = 4.;

                SbVec3f *vertices = new SbVec3f[_path.size()*_wkSpace->getNumRobots()];
                for (unsigned int i(0); i < _path.size(); ++i) {
                    _wkSpace->moveRobotsTo(_path.at(i));
                    for (unsigned int j(0); j < _wkSpace->getNumRobots(); ++j) {
                        mt::Point3 point(_wkSpace->getRobot(j)->
                                         getLink(_wkSpace->getRobot(j)->getTrunk()-1)->
                                         getTransformation()->getTranslation());

                        vertices[j*_path.size()+i][0] = point[0];
                        vertices[j*_path.size()+i][1] = point[1];
                        vertices[j*_path.size()+i][2] = point[2];
                    }
                }
                int32_t *numVertices = new int32_t[_wkSpace->getNumRobots()];
                std::fill(numVertices,numVertices+_wkSpace->getNumRobots(),_path.size());

                SoVertexProperty *vertexProperty(new SoVertexProperty);
                vertexProperty->vertex.setValues(0,_path.size()*_wkSpace->getNumRobots(),vertices);
                vertexProperty->orderedRGBA.setValue(SbColor(1,0,0.2).getPackedValue());

                SoLineSet *lineSet(new SoLineSet);
                lineSet->vertexProperty.setValue(vertexProperty);
                lineSet->numVertices.setValues(0,_wkSpace->getNumRobots(),numVertices);

                SoSeparator *path(new SoSeparator);
                path->setName("Path");
                path->addChild(drawStyle);
                path->addChild(lineSet);

                _scenePath->addChild(path);
            }

            points = new SoSeparator;
            points->setName("SpecialVertices");

            drawStyle = new SoDrawStyle;
            drawStyle->pointSize = 5;
            points->addChild(drawStyle);

            vertexProperty = new SoVertexProperty;
            vertexProperty->vertex.setValues(0,pdata->numStartVertices()*_wkSpace->getNumRobots(),startVertices);
            vertexProperty->orderedRGBA.setValue(SbColor(0.6,0,1).getPackedValue());

            pointSet = new SoPointSet;
            pointSet->vertexProperty.setValue(vertexProperty);
            points->addChild(pointSet);

            vertexProperty = new SoVertexProperty;
            vertexProperty->vertex.setValues(0,goalStates->getStateCount()*_wkSpace->getNumRobots(),goalVertices);
            vertexProperty->orderedRGBA.setValue(SbColor(1,1,0).getPackedValue());

            pointSet = new SoPointSet;
            pointSet->vertexProperty.setValue(vertexProperty);
            points->addChild(pointSet);

            _scenePath->addChild(points);
        }


        //! This routine allows to draw the 2D projection of a roadmap or tree.
        //! The one corresponding to robot number numrob is drawn.
        void omplPlanner::drawCspace(unsigned int robot, unsigned int link) {
            if (!_sceneCspace) return;

            //Delete whatever is already drawn
            _sceneCspace->removeAllChildren();

            //Get the subspace
            ob::StateSpacePtr stateSpace(space->as<ob::CompoundStateSpace>()->getSubspace(robot)->
                                         as<ob::CompoundStateSpace>()->getSubspace(link));

            //Set space bounds
            unsigned int k;
            KthReal xmin, xmax, ymin, ymax, zmin, zmax;
            if (_wkSpace->getRobot(robot)->isSE3Enabled()) {
                k = stateSpace->as<ob::SE3StateSpace>()->getDimension();

                xmin = stateSpace->as<ob::SE3StateSpace>()->getBounds().low[0];
                xmax = stateSpace->as<ob::SE3StateSpace>()->getBounds().high[0];
                ymin = stateSpace->as<ob::SE3StateSpace>()->getBounds().low[1];
                ymax = stateSpace->as<ob::SE3StateSpace>()->getBounds().high[1];
                zmin = stateSpace->as<ob::SE3StateSpace>()->getBounds().low[2];
                zmax = stateSpace->as<ob::SE3StateSpace>()->getBounds().high[2];
            } else {
                k = stateSpace->as<ob::RealVectorStateSpace>()->getDimension();

                xmin = stateSpace->as<ob::RealVectorStateSpace>()->getBounds().low[0];
                xmax = stateSpace->as<ob::RealVectorStateSpace>()->getBounds().high[0];
                ymin = stateSpace->as<ob::RealVectorStateSpace>()->getBounds().low[1];
                ymax = stateSpace->as<ob::RealVectorStateSpace>()->getBounds().high[1];
                if (k > 2) {
                    zmin = stateSpace->as<ob::RealVectorStateSpace>()->getBounds().low[2];
                    zmax = stateSpace->as<ob::RealVectorStateSpace>()->getBounds().high[2];
                } else {
                    zmin = -FLT_MIN;
                    xmax =  FLT_MIN;
                }
            }

            //Use the projection associated to the subspace of the robot passed as a parameter.
            string projectionName("drawprojection" + static_cast<ostringstream*>
                                  (&(ostringstream() << robot))->str());
            ob::ProjectionEvaluatorPtr projection(space->getProjection(projectionName));
            Kautham::Vector state(k);

            //Load the planner data to be drawn
            ob::PlannerDataPtr pdata(new ob::PlannerData(ss->getSpaceInformation()));
            ss->getPlanner()->getPlannerData(*pdata);

            if (ss->getPlanner()->getProblemDefinition()->hasOptimizationObjective()) {
                //Compute the weight for all edges given the OptimizationObjective
                pdata->computeEdgeWeights(*ss->getPlanner()->getProblemDefinition()->
                                          getOptimizationObjective());
            } else {
                //Compute all edge weights using state space distance
                pdata->computeEdgeWeights();
            }

            //Draw tree
            ob::GoalStates *goalStates(dynamic_cast<ob::GoalStates*>
                                       (ss->getProblemDefinition()->getGoal().get()));
            //Nothing to draw
            if (pdata->numVertices() < (pdata->numStartVertices()+pdata->numGoalVertices())) {
                std::cout<<"omplPlanner::drawCspace - no tree to draw"<<std::endl;
                return;
            }

            SbVec3f *goalVertices = new SbVec3f[goalStates->getStateCount()];
            for (unsigned int i(0); i < goalStates->getStateCount(); ++i) {
                projection->project(goalStates->getState(i),state);

                goalVertices[i][0] = state[0];
                goalVertices[i][1] = state[1];
                goalVertices[i][2] = ((k > 2)? state[2] : 0.0);
            }
            SbVec3f *vertices = new SbVec3f[pdata->numVertices()];
            SbVec3f *startVertices = new SbVec3f[pdata->numStartVertices()];
            SbVec3f *otherVertices = new SbVec3f[pdata->numVertices()-
                    pdata->numStartVertices()-pdata->numGoalVertices()];
            int32_t *coordIndices = new int32_t[3*pdata->numEdges()];
            int32_t *edgeColorIndices = new int32_t[pdata->numEdges()];
            unsigned int j(0), ls(0), lo(0);
            for (unsigned int i = 0; i < pdata->numVertices(); ++i) {
                projection->project(pdata->getVertex(i).getState(),state);

                vertices[i][0] = state[0];
                vertices[i][1] = state[1];
                vertices[i][2] = ((k > 2)? state[2] : 0.0);

                if (pdata->isStartVertex(i)) {
                    startVertices[ls][0] = vertices[i][0];
                    startVertices[ls][1] = vertices[i][1];
                    startVertices[ls][2] = vertices[i][2];
                    ls++;
                } else if (pdata->isGoalVertex(i)) {

                } else {
                    otherVertices[lo][0] = vertices[i][0];
                    otherVertices[lo][1] = vertices[i][1];
                    otherVertices[lo][2] = vertices[i][2];
                    lo++;
                }

                std::vector<unsigned int> outgoingVertices;
                pdata->getEdges(i,outgoingVertices);
                for (std::vector<unsigned int>::const_iterator it(outgoingVertices.begin());
                     it != outgoingVertices.end(); ++it) {
                    coordIndices[3*j+0] = i;
                    coordIndices[3*j+1] = *it;
                    coordIndices[3*j+2] = SO_END_LINE_INDEX;

                    ob::Cost edgeWeight;
                    pdata->getEdgeWeight(i,*it,&edgeWeight);

                    edgeColorIndices[j] = pdata->getVertex(i).getTag();

                    j++;
                }
            }

            SoDrawStyle *drawStyle(new SoDrawStyle);
            drawStyle->lineWidth = 1;
            drawStyle->pointSize = 3;

            SoVertexProperty *vertexProperty(new SoVertexProperty);
            vertexProperty->vertex.setValues(0,pdata->numVertices(),vertices);
            uint32_t edgeColors[3] = {SbColor(0,1,0.8).getPackedValue(),
                                      SbColor(0,1,0.2).getPackedValue(),
                                      SbColor(0,0.6,1).getPackedValue()};
            vertexProperty->orderedRGBA.setValues(0,3,edgeColors);
            vertexProperty->materialBinding.setValue(SoVertexProperty::PER_FACE_INDEXED);

            SoIndexedLineSet *lineSet(new SoIndexedLineSet);
            lineSet->coordIndex.setValues(0,3*pdata->numEdges(),coordIndices);
            lineSet->vertexProperty.setValue(vertexProperty);
            lineSet->materialIndex.setValues(0,pdata->numEdges(),edgeColorIndices);

            SoSeparator *lines(new SoSeparator);
            lines->setName("Edges");
            lines->addChild(drawStyle);
            lines->addChild(lineSet);

            _sceneCspace->addChild(lines);

            vertexProperty = new SoVertexProperty;
            vertexProperty->vertex.setValues(0,pdata->numVertices()-pdata->numStartVertices()-
                                             pdata->numGoalVertices(),otherVertices);
            vertexProperty->orderedRGBA.setValue(SbColor(1,1,1).getPackedValue());

            SoPointSet *pointSet(new SoPointSet);
            pointSet->vertexProperty.setValue(vertexProperty);

            SoSeparator *points(new SoSeparator);
            points->setName("NormalVertices");
            points->addChild(drawStyle);
            points->addChild(pointSet);

            _sceneCspace->addChild(points);

            //Draw path
            if (_solved) {
                SoDrawStyle *drawStyle(new SoDrawStyle);
                drawStyle->lineWidth = 4.;

                std::vector<ob::State*> &states(ss->getSolutionPath().getStates());
                SbVec3f *vertices = new SbVec3f[states.size()];
                for (unsigned int i = 0; i < states.size(); ++i) {
                    projection->project(states.at(i),state);

                    vertices[i][0] = state[0];
                    vertices[i][1] = state[1];
                    vertices[i][2] = ((k > 2)? state[2] : 0.0);
                }

                SoVertexProperty *vertexProperty(new SoVertexProperty);
                vertexProperty->vertex.setValues(0,states.size(),vertices);
                vertexProperty->orderedRGBA.setValue(SbColor(1,0,0.2).getPackedValue());

                SoLineSet *lineSet(new SoLineSet);
                lineSet->vertexProperty.setValue(vertexProperty);

                SoSeparator *path(new SoSeparator);
                path->setName("Path");
                path->addChild(drawStyle);
                path->addChild(lineSet);

                _sceneCspace->addChild(path);
            }

            points = new SoSeparator;
            points->setName("SpecialVertices");

            drawStyle = new SoDrawStyle;
            drawStyle->pointSize = 5;
            points->addChild(drawStyle);

            vertexProperty = new SoVertexProperty;
            vertexProperty->vertex.setValues(0,pdata->numStartVertices(),startVertices);
            vertexProperty->orderedRGBA.setValue(SbColor(0.6,0,1).getPackedValue());

            pointSet = new SoPointSet;
            pointSet->vertexProperty.setValue(vertexProperty);
            points->addChild(pointSet);

            vertexProperty = new SoVertexProperty;
            vertexProperty->vertex.setValues(0,goalStates->getStateCount(),goalVertices);
            vertexProperty->orderedRGBA.setValue(SbColor(1,1,0).getPackedValue());

            pointSet = new SoPointSet;
            pointSet->vertexProperty.setValue(vertexProperty);
            points->addChild(pointSet);

            _sceneCspace->addChild(points);

            //Draw bounds
            SoMaterial *material(new SoMaterial);
            material->transparency.setValue(0.8);

            SoTranslation *translation(new SoTranslation);
            translation->translation.setValue((xmax+xmin)/2.,(ymax+ymin)/2.,(zmax+zmin)/2.);

            SoCube *cube(new SoCube);
            cube->width = xmax-xmin;
            cube->height = ymax-ymin;
            cube->depth = zmax-zmin;

            SoSeparator *bounds(new SoSeparator);
            bounds->setName("Bounds");
            bounds->addChild(material);
            bounds->addChild(translation);
            bounds->addChild(cube);

            _sceneCspace->addChild(bounds);
        }

        //! This function converts a Kautham sample to an ompl scoped state.
        void omplPlanner::smp2omplScopedState(Sample* smp, ob::ScopedState<ob::CompoundStateSpace> *sstate)
        {
            //Extract the mapped configuration of the sample. It is a vector with as many components as robots.
            //each component has the RobConf of the robot (the SE3 and the Rn configurations)
            if(smp->getMappedConf().size()==0)
            {
                _wkSpace->moveRobotsTo(smp); // to set the mapped configuration
            }
            std::vector<RobConf>& smpRobotsConf = smp->getMappedConf();


            //loop for all the robots
            for (unsigned i=0; i<_wkSpace->getNumRobots(); i++)
            {
                int k=0; //counter of subspaces contained in subspace of robot i

                //get the subspace of robot i
                ob::StateSpacePtr ssRoboti = ((ob::StateSpacePtr) space->as<ob::CompoundStateSpace>()->getSubspace(i));
                string ssRobotiname = ssRoboti->getName();

                //if it has se3 part
                if(_wkSpace->getRobot(i)->isSE3Enabled())
                {
                    //get the kautham SE3 configuration
                    SE3Conf c = smpRobotsConf.at(i).getSE3();
                    vector<KthReal>& pp = c.getPos();
                    vector<KthReal>& aa = c.getAxisAngle();

                    //set the ompl SE3 configuration
                    ob::StateSpacePtr ssRobotiSE3 =  ((ob::StateSpacePtr) ssRoboti->as<ob::CompoundStateSpace>()->getSubspace(k));
                    string ssRobotiSE3name = ssRobotiSE3->getName();

                    ob::ScopedState<ob::SE3StateSpace> cstart(ssRobotiSE3);
                    cstart->setX(pp[0]);
                    cstart->setY(pp[1]);
                    cstart->setZ(pp[2]);
                    cstart->rotation().setAxisAngle(aa[0],aa[1],aa[2],aa[3]);

                    //load the global scoped state with the info of the se3 data of robot i
                    (*sstate)<<cstart;
                    k++;
                }

                //has Rn part
                if(_wkSpace->getRobot(i)->getNumJoints()>0)
                {
                    //get the kautham Rn configuration
                    RnConf r = smpRobotsConf.at(i).getRn();

                    //set the ompl Rn configuration
                    ob::StateSpacePtr ssRobotiRn =  ((ob::StateSpacePtr) ssRoboti->as<ob::CompoundStateSpace>()->getSubspace(k));
                    ob::ScopedState<weigthedRealVectorStateSpace> rstart(ssRobotiRn);

                    for (unsigned j=0; j<_wkSpace->getRobot(i)->getNumJoints();j++)
                        rstart->values[j] = r.getCoordinate(j);

                    //cout<<"sstate[0]="<<rstart->values[0]<<"sstate[1]="<<rstart->values[1]<<endl;


                    //load the global scoped state with the info of the Rn data of robot i
                    (*sstate) << rstart;
                    k++;//dummy
                }
            }
        }

        //! This member function converts an ompl State to a Kautham sample
        void omplPlanner::omplState2smp(const ob::State *state, Sample* smp)
        {
            ob::ScopedState<ob::CompoundStateSpace> sstate(space);
            sstate = *state;
            omplScopedState2smp( sstate, smp);
        }

        //! This member function converts an ompl ScopedState to a Kautham sample
        void omplPlanner::omplScopedState2smp(ob::ScopedState<ob::CompoundStateSpace> sstate, Sample* smp)
        {
            vector<RobConf> rc;

            //Loop for all the robots
            for (unsigned int i = 0; i < _wkSpace->getNumRobots(); ++i) {
                //RobConf to store the robots configurations read from the ompl state
                RobConf *rcj = new RobConf;

                //Get the subspace corresponding to robot i
                ob::StateSpacePtr ssRoboti = ((ob::StateSpacePtr)space->as<ob::CompoundStateSpace>()->getSubspace(i));

                //Get the SE3 subspace of robot i, if it exists, extract the SE3 configuration
                unsigned int k = 0; //counter of subspaces of robot i
                if (_wkSpace->getRobot(i)->isSE3Enabled()) {
                    //Get the SE3 subspace of robot i
                    ob::StateSpacePtr ssRobotiSE3 = ((ob::StateSpacePtr)ssRoboti->as<ob::CompoundStateSpace>()->getSubspace(k));

                    //Create a SE3 scoped state and load it with the data extracted from the global scoped state
                    ob::ScopedState<ob::SE3StateSpace> pathscopedstatese3(ssRobotiSE3);
                    sstate >> pathscopedstatese3;

                    //Convert it to a vector of 7 components
                    vector<KthReal> se3coords;
                    se3coords.resize(7);
                    se3coords[0] = pathscopedstatese3->getX();
                    se3coords[1] = pathscopedstatese3->getY();
                    se3coords[2] = pathscopedstatese3->getZ();
                    se3coords[3] = pathscopedstatese3->rotation().x;
                    se3coords[4] = pathscopedstatese3->rotation().y;
                    se3coords[5] = pathscopedstatese3->rotation().z;
                    se3coords[6] = pathscopedstatese3->rotation().w;

                    //Create the sample
                    SE3Conf se3;
                    se3.setCoordinates(se3coords);
                    rcj->setSE3(se3);

                    k++;
                } else {
                    //If the robot does not have mobile SE3 dofs then the SE3 configuration of the sample is maintained
                    if (smp->getMappedConf().size() == 0) {
                        throw ompl::Exception("omplPlanner::omplScopedState2smp", "parameter smp must be a sample with the MappedConf");
                    } else {
                        rcj->setSE3(smp->getMappedConf()[i].getSE3());
                    }
                }

                //Get the Rn subspace of robot i, if it exisits, and extract the Rn configuration
                if (_wkSpace->getRobot(i)->getNumJoints() > 0) {
                    //Get the Rn subspace of robot i
                    ob::StateSpacePtr ssRobotiRn = ((ob::StateSpacePtr)ssRoboti->as<ob::CompoundStateSpace>()->getSubspace(k));

                    //Create a Rn scoped state and load it with the data extracted from the global scoped state
                    ob::ScopedState<weigthedRealVectorStateSpace> pathscopedstateRn(ssRobotiRn);
                    sstate >> pathscopedstateRn;

                    //Convert it to a vector of n components
                    vector<KthReal> coords;
                    for (unsigned int j = 0; j < _wkSpace->getRobot(i)->getNumJoints(); ++j){
                        coords.push_back(pathscopedstateRn->values[j]);
                    }
                    rcj->setRn(coords);

                    k++;//dummy
                } else {
                    //If the robot does not have mobile Rn dofs then the Rn configuration of the sample is maintained
                    if (smp->getMappedConf().size() == 0) {
                        throw ompl::Exception("omplPlanner::omplScopedState2smp", "parameter smp must be a sample with the MappedConf");
                    } else {
                        rcj->setRn(smp->getMappedConf()[i].getRn());
                    }
                }

                //Load the RobConf with the data of robot i
                rc.push_back(*rcj);
            }
            //create the sample with the RobConf
            //the coords (controls) of the sample are kept void
            smp->setMappedConf(rc);
        }


        //! function to find a solution path
        bool omplPlanner::trySolve() {
            //Add start states
            ss->clearStartStates();
            for (std::vector<Sample*>::const_iterator start(_init.begin());
                 start != _init.end(); ++start) {
                //Start state: convert from smp to scoped state
                ob::ScopedState<ob::CompoundStateSpace> startompl(space);
                smp2omplScopedState(*start,&startompl);
                cout << "startompl:" << endl;
                startompl.print();
                ss->addStartState(startompl);
            }

            //Add goal states
            ob::GoalStates *goalStates(new ob::GoalStates(si));
            for (std::vector<Sample*>::const_iterator goal(_goal.begin());
                 goal != _goal.end(); ++goal) {
                //Goal state: convert from smp to scoped state
                ob::ScopedState<ob::CompoundStateSpace> goalompl(space);
                smp2omplScopedState(*goal,&goalompl);
                cout << "goalompl:" << endl;
                goalompl.print();
                goalStates->addState(goalompl);
            }
            ss->setGoal(ob::GoalPtr(goalStates));

            //Remove previous solutions, if any
            if (_incremental) {
                ss->getProblemDefinition()->clearSolutionPaths();
            } else {
                ss->clear();
                ss->getPlanner()->clear();
            }

            //Attempt to solve the problem within _planningTime seconds of planning time
            ss->setup();

            ob::PlannerStatus::StatusType solved(ss->solve(_planningTime));
            switch (solved) {
                case ob::PlannerStatus::EXACT_SOLUTION:
                    std::cout << std::endl << "EXACT_SOLUTION found" << std::endl;

                    switch (_simplify) {
                        case 1://smooth
                            ss->getPathSimplifier()->smoothBSpline(ss->getSolutionPath(),5);
                        break;
                        case 2://shorten and smoot
                            ss->simplifySolution();
                        break;
                    }

                    if (_interpolate)
                        ss->getSolutionPath().interpolate();

                    _path.clear();
                    clearSimulationPath();
                    Sample *smp;
                    //load the kautham _path variable from the ompl solution
                    for (std::size_t j(0); j < ss->getSolutionPath().getStateCount(); ++j) {
                        //create a smp and load the RobConf of the init configuration (to have the same if the state does not changi it)
                        smp = new Sample(_wkSpace->getNumRobControls());
                        smp->setMappedConf(_init.at(0)->getMappedConf());

                        //convert form state to smp
                        omplState2smp(ss->getSolutionPath().getState(j)->as<ob::CompoundStateSpace::StateType>(),smp);

                        _path.push_back(smp);
                        _samples->add(smp);
                    }
                    _solved = true;
                break;
                case ob::PlannerStatus::APPROXIMATE_SOLUTION:
                    std::cout << "APPROXIMATE_SOLUTION - No exact solution found" << std::endl;
                    std::cout << "Difference = " <<ss->getProblemDefinition()->getSolutions().at(0).difference_ << std::endl;
                    _solved = false;
                break;
                case ob::PlannerStatus::TIMEOUT:
                    std::cout << "TIMEOUT - No solution found" << std::endl;
                    _solved = false;
                break;
                default:
                    std::cout << "solve returned " << solved << "- No exact solution found" << std::endl;
                    _solved = false;
                break;
            }

            drawPath(_drawnPath);
            drawCspace(_drawnrobot);

            return _solved;
        }
    }
}

#endif // KAUTHAM_USE_OMPL
