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

#include <ompl/base/OptimizationObjective.h>

#include <ompl/geometric/planners/prm/ConnectionStrategy.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/datastructures/PDF.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/tools/config/SelfConfig.h>

#include <ompl/base/StateSampler.h>

#include <boost/bind/mem_fn.hpp>
#include <boost/lambda/bind.hpp>
#include <boost/graph/astar_search.hpp>
#include <boost/graph/incremental_components.hpp>
#include <boost/property_map/vector_property_map.hpp>
#include <boost/foreach.hpp>
#include <boost/thread.hpp>

#define foreach_fwd BOOST_FOREACH
#define foreach_reverse BOOST_REVERSE_FOREACH

#include <kautham/planner/omplg/omplPRMplanner.h>
#include <kautham/planner/omplg/omplValidityChecker.h>
//#include <kautham/planner/omplg/omplplanner.h>


namespace Kautham {
  namespace omplplanner{

  class KauthamStateSampler;

  ////////////////////////////////////////////////////////////////////////////////////////////////////////
      //The following magic values are the same as the ones in ompl::PRM.cpp file. I did not know how to use them from here so I duplicated them,
      namespace mymagic
      {
          /** \brief Maximum number of sampling attempts to find a valid state,
              without checking whether the allowed time elapsed. This value
              should not really be changed. */
          static const unsigned int FIND_VALID_STATE_ATTEMPTS_WITHOUT_TIME_CHECK = 2;

          /** \brief The number of steps to take for a random bounce
              motion generated as part of the expansion step of PRM. */
          static const unsigned int MAX_RANDOM_BOUNCE_STEPS   = 5;

          /** \brief The number of nearest neighbors to consider by
              default in the construction of the PRM roadmap */
          static const unsigned int DEFAULT_NEAREST_NEIGHBORS = 10;

          /** \brief The time in seconds for a single roadmap building operation (dt)*/
          static const double ROADMAP_BUILD_TIME = 0.2;

          /** \brief The distance threshold is DISTANCE_THRESHOLD_FACTOR times the step size*/
          static const double DISTANCE_THRESHOLD_FACTOR = 100.0;
      }

  ////////////////////////////////////////////////////////////////////////////////////////////////////////
  class myPRM; //class definition

  ////////////////////////////////////////////////////////////////////////////////////////////////////////
  //the same typedefs used in ompl::PRM class
  struct vertex_state_t {
      typedef boost::vertex_property_tag kind;
  };

  struct vertex_total_connection_attempts_t {
      typedef boost::vertex_property_tag kind;
  };

  struct vertex_successful_connection_attempts_t {
      typedef boost::vertex_property_tag kind;
  };

  struct vertex_flags_t {
      typedef boost::vertex_property_tag kind;
  };

  struct edge_flags_t {
      typedef boost::edge_property_tag kind;
  };


                  typedef boost::adjacency_list <
                      boost::vecS, boost::vecS, boost::undirectedS,
                      boost::property < vertex_state_t, ob::State*,
                      boost::property < vertex_total_connection_attempts_t, unsigned int,
                      boost::property < vertex_successful_connection_attempts_t, unsigned int,
                      boost::property < vertex_flags_t, unsigned int,
                      boost::property < boost::vertex_predecessor_t, unsigned long int,
                      boost::property < boost::vertex_rank_t, unsigned long int > > > > > >,
                      boost::property < boost::edge_weight_t, ob::Cost,
                      boost::property < boost::edge_index_t, unsigned int,
                      boost::property < edge_flags_t, unsigned int > > >
                  > Graph;

                  typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;
                  typedef boost::graph_traits<Graph>::edge_descriptor   Edge;




  ////////////////////////////////////////////////////////////////////////////////////////////////////////
  //! This is a class derived form the class ompl::PRM. Its purpose is to slightly change its behavior, by
  //!      1) Making the ratio fo the steps grow and expand variable. It is done in the reimplementation of the solve function
  //!      2) The function expandRoadmap is changed by myexpandRoadmap. For now they are equal, but possible changes include
  //!          the distance threshold in the edges of the bounce motions and changing the number of bounce steps
  class myPRM:public og::PRM
  {
    public:
      double mingrowtime;
      double minexpandtime;
      double maxdistancebouncemotions;
      bool useKauthamSampler;
      int bouncesteps;

      //! myPRM creator. The variables mingrowtime and minexpandtime are set at the values used in the ompl::PRM.
      myPRM(const ob::SpaceInformationPtr &si, bool starStrategy=false) :  og::PRM(si,starStrategy)
      {
          mingrowtime = 2.0*mymagic::ROADMAP_BUILD_TIME;
          minexpandtime = mymagic::ROADMAP_BUILD_TIME;
          maxdistancebouncemotions = -1;//not set
          useKauthamSampler = false;
          bouncesteps = mymagic::MAX_RANDOM_BOUNCE_STEPS;

          //By default PRM uses minimization of distance as optimization goal
          //this can be changed by modifying the problem definition (pdef). If pdef has optimization then it uses it (see PRM::setup)
          //Also PRM::setconnectionstrategy can be used to change the behavior of the PRM- To be explored!! e. using PCA!
      }


      //! destructor
      ~myPRM(void)
      {
          freeMemory();
      }

      bool checkSameComponent(Vertex v1, Vertex v2)
      {
           return sameComponent(v1,v2);
      }

      //!  Sets the mingrowtime value
      void setMinGrowTime(double gt)
      {
          mingrowtime = gt;
          if(mingrowtime < mymagic::ROADMAP_BUILD_TIME) mingrowtime = mymagic::ROADMAP_BUILD_TIME;
      }

      //!  Sets the minexpandtime value
      void setMinExpandTime(double et)
      {
          minexpandtime = et;
      }

      //! sets the flag saying that the kautham sampler is being used. It is necessary to limit the distance in the expand step
      //! during bounce motions
      void usingKauthamSampler(bool f)
      {
          useKauthamSampler = f;
      }


      //!  Sets the mnumber of bounce steps of the expand phase
      void setBounceSteps(double n)
      {
        bouncesteps = n;
      }

      //!  Sets the maxdistance for the bounce motions in the expand step
      void setMaxDistanceBounceMotions(double m)
      {
          maxdistancebouncemotions = m;
      }

      //! Computes the distance between two milestones (this is simply the distance between the states of the milestones)
      double distance(const Vertex a, const Vertex b) const
      {
          return distanceFunction(a,b);
      }


      //! Reimplemented in order that data stores too the isolated milestones
      void getPlannerData(ob::PlannerData &data) const
      {
          Planner::getPlannerData(data);

          // Explicitly add start and goal states:
          for (size_t i = 0; i < startM_.size(); ++i)
              data.addStartVertex(ob::PlannerDataVertex(stateProperty_[startM_[i]], const_cast<myPRM*>(this)->disjointSets_.find_set(startM_[i])));

          for (size_t i = 0; i < goalM_.size(); ++i)
              data.addGoalVertex(ob::PlannerDataVertex(stateProperty_[goalM_[i]], const_cast<myPRM*>(this)->disjointSets_.find_set(goalM_[i])));

          // Adding edges
          foreach_fwd(const Edge e, boost::edges(g_))
          {
              const Vertex v1 = boost::source(e, g_);
              const Vertex v2 = boost::target(e, g_);
              data.addEdge(ob::PlannerDataVertex(stateProperty_[v1]),
                           ob::PlannerDataVertex(stateProperty_[v2]));

              // Add the reverse edge, since we're constructing an undirected roadmap
              data.addEdge(ob::PlannerDataVertex(stateProperty_[v2]),
                           ob::PlannerDataVertex(stateProperty_[v1]));

              // Add tags for the newly added vertices
              data.tagState(stateProperty_[v1], const_cast<myPRM*>(this)->disjointSets_.find_set(v1));
              data.tagState(stateProperty_[v2], const_cast<myPRM*>(this)->disjointSets_.find_set(v2));
          }

          // Adding vertices
          foreach_fwd(const Vertex v1, boost::vertices(g_))
          {
              data.addVertex(ob::PlannerDataVertex(stateProperty_[v1]));

              // Add tags for the newly added vertices
              data.tagState(stateProperty_[v1], const_cast<myPRM*>(this)->disjointSets_.find_set(v1));
          }
      }


      //! This functions is identical to the PRM::constructRoadmap function except that the ratio between grow and expand steps is not
      //! fixed to 2:1 but is configurable by the mingrowtime and mingrowtime class variables
      void constructRoadmap(const ob::PlannerTerminationCondition &ptc)
      {
          if (!isSetup())
              setup();
          if (!sampler_)
              sampler_ = si_->allocValidStateSampler();
          if (!simpleSampler_)
              simpleSampler_ = si_->allocStateSampler();

          std::vector<ob::State*> xstates(mymagic::MAX_RANDOM_BOUNCE_STEPS);
          si_->allocStates(xstates);
          bool grow = true;

          while (ptc() == false)
          {
              // maintain a 2:1 ratio for growing/expansion of roadmap
              // call growRoadmap() twice as long for every call of expandRoadmap()
              //This has changed for myPRM: the time of the expand step is minexpandtime and the time of the grow step is mingrowtime
              if (grow)
                  //growRoadmap(base::plannerOrTerminationCondition(ptc, base::timedPlannerTerminationCondition(2.0 * magic::ROADMAP_BUILD_TIME)), xstates[0]);
                  growRoadmap(ob::plannerOrTerminationCondition(ptc, ob::timedPlannerTerminationCondition(mingrowtime)), xstates[0]);
              else
                  //expandRoadmap(base::plannerOrTerminationCondition(ptc, base::timedPlannerTerminationCondition(magic::ROADMAP_BUILD_TIME)), xstates);
                  myexpandRoadmap(ob::plannerOrTerminationCondition(ptc, ob::timedPlannerTerminationCondition(minexpandtime)), xstates);
              grow = !grow;
          }

          si_->freeStates(xstates);
      }


      //! This function is reimplemented to call to  myexpandRoadmap(ptc, states) instead of calling to expandRoadmap(ptc, states)
      void expandRoadmap(const ob::PlannerTerminationCondition &ptc)
      {
          if (!simpleSampler_)
              simpleSampler_ = si_->allocStateSampler();

          std::vector<ob::State*> states(bouncesteps);
          si_->allocStates(states);
          myexpandRoadmap(ptc, states);
          si_->freeStates(states);
      }

      //! This function is, by now, equal to ompl::PRM:expandRoadmap. It can be changed to include a different number
      //! of bounce steps or to filter the length of the bounce edges.
      void myexpandRoadmap(const ob::PlannerTerminationCondition &ptc,
                                               std::vector<ob::State*> &workStates)
      {
          // construct a probability distribution over the vertices in the roadmap
          // as indicated in
          //  "Probabilistic Roadmaps for Path Planning in High-Dimensional Configuration Spaces"
          //        Lydia E. Kavraki, Petr Svestka, Jean-Claude Latombe, and Mark H. Overmars

          while (ptc == true)
               return;

          ompl::PDF<Vertex> pdf;
          foreach_fwd (Vertex v, boost::vertices(g_))
          {
              const unsigned int t = totalConnectionAttemptsProperty_[v];
              pdf.add(v, (double)(t - successfulConnectionAttemptsProperty_[v]) / (double)t);
          }

          if (pdf.empty())
              return;

          while (ptc == false)
          {
              Vertex v = pdf.sample(rng_.uniform01());

              //verify if the sampler is a KauthamStateSampler, in order to be sure that it has the setCenterSample function
              if(useKauthamSampler)
              {
                  try
                  {
                    ((KauthamStateSampler*) &(*simpleSampler_))->setCenterSample(stateProperty_[v],maxdistancebouncemotions);
                  }
                  catch(...){
                      OMPL_ERROR("myPRM: You should have allocated the kautham sampler...");
                  }
              }
              //Call the random bounce walk
              unsigned int s = si_->randomBounceMotion(simpleSampler_, stateProperty_[v], workStates.size(), workStates, false);
              //reset the sample near option of the kautham samper
              if(useKauthamSampler)
              {
                  try
                  {
                      //call the setCenterSample with NULL argument
                    ((KauthamStateSampler*) &(*simpleSampler_))->setCenterSample(NULL,maxdistancebouncemotions);
                  }
                  catch(...){
                      OMPL_ERROR("myPRM: You should have allocated the kautham sampler...");
                  }
              }
              //create the edges
              if (s > 0)
              {
                  s--;
                  Vertex last = addMilestone(si_->cloneState(workStates[s]));

                  graphMutex_.lock();
                  for (unsigned int i = 0 ; i < s ; ++i)
                  {
                      // add the vertex along the bouncing motion
                      Vertex m = boost::add_vertex(g_);
                      stateProperty_[m] = si_->cloneState(workStates[i]);
                      totalConnectionAttemptsProperty_[m] = 1;
                      successfulConnectionAttemptsProperty_[m] = 0;
                      disjointSets_.make_set(m);

                      // add the edge to the parent vertex
                      const ob::Cost weight = opt_->motionCost(stateProperty_[v], stateProperty_[m]);

                      //cout<<"d= "<<weight<<endl;
                      const Graph::edge_property_type properties(weight);
                      boost::add_edge(v, m, properties, g_);
                      uniteComponents(v, m);

                      // add the vertex to the nearest neighbors data structure
                      nn_->add(m);
                      v = m;
                  }

                  // if there are intermediary states or the milestone has not been connected to the initially sampled vertex,
                  // we add an edge
                  if (s > 0 || !boost::same_component(v, last, disjointSets_))
                  {
                      // add the edge to the parent vertex
                      const ob::Cost weight = opt_->motionCost(stateProperty_[v], stateProperty_[last]);

                      //cout<<"d2= "<<weight<<endl;
                      const Graph::edge_property_type properties(weight);
                      boost::add_edge(v, last, properties, g_);
                      uniteComponents(v, last);
                  }
                  graphMutex_.unlock();
              }
          }
      }

ob::PlannerStatus solve(const ob::PlannerTerminationCondition &ptc)
            {
                checkValidity();
                ob::GoalSampleableRegion *goal = dynamic_cast<ob::GoalSampleableRegion*>(pdef_->getGoal().get());

                if (!goal)
                {
                    OMPL_ERROR("%s: Unknown type of goal", getName().c_str());
                    return ob::PlannerStatus::UNRECOGNIZED_GOAL_TYPE;
                }

                // Add the valid start states as milestones
                while (const ob::State *st = pis_.nextStart())
                    startM_.push_back(addMilestone(si_->cloneState(st)));

                if (startM_.size() == 0)
                {
                    OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
                    return ob::PlannerStatus::INVALID_START;
                }

                if (!goal->couldSample())
                {
                    OMPL_ERROR("%s: Insufficient states in sampleable goal region", getName().c_str());
                    return ob::PlannerStatus::INVALID_GOAL;
                }

                // Ensure there is at least one valid goal state
                if (goal->maxSampleCount() > goalM_.size() || goalM_.empty())
                {
                    const ob::State *st = goalM_.empty() ? pis_.nextGoal(ptc) : pis_.nextGoal();
                    if (st)
                        goalM_.push_back(addMilestone(si_->cloneState(st)));

                    if (goalM_.empty())
                    {
                        OMPL_ERROR("%s: Unable to find any valid goal states", getName().c_str());
                        return ob::PlannerStatus::INVALID_GOAL;
                    }
                }

                unsigned long int nrStartStates = boost::num_vertices(g_);
                OMPL_INFORM("%s: Starting planning with %lu states already in datastructure", getName().c_str(), nrStartStates);

                // Reset addedNewSolution_ member and create solution checking thread
                addedNewSolution_ = false;
                ob::PathPtr sol;


                boost::thread slnThread(std::bind(&myPRM::checkForSolution, this, ptc, boost::ref(sol)));

                // construct new planner termination condition that fires when the given ptc is true, or a solution is found
                ob::PlannerTerminationCondition ptcOrSolutionFound =
                        ob::plannerOrTerminationCondition(ptc, ob::PlannerTerminationCondition(std::bind(&myPRM::addedNewSolution, this)));

                constructRoadmap(ptcOrSolutionFound);

                // Ensure slnThread is ceased before exiting solve
                slnThread.join();

                OMPL_INFORM("%s: Created %u states", getName().c_str(), boost::num_vertices(g_) - nrStartStates);

                if (sol)
                {
                    ob::PlannerSolution psol(sol);
                    psol.setPlannerName(getName());
                    // if the solution was optimized, we mark it as such
                    psol.setOptimized(opt_, bestCost_, addedNewSolution());
                    pdef_->addSolutionPath(psol);
                }

                return sol ? ob::PlannerStatus::EXACT_SOLUTION : ob::PlannerStatus::TIMEOUT;
            }


  };


  ////////////////////////////////////////////////////////////////////////////
  //! This function is a distance filter used as a connectionFilter_ in the PRM
  bool connectionDistanceFilter(const Vertex& v1, const Vertex& v2, double d, ob::PlannerPtr pl)
  {
      if(pl->as<myPRM>()->checkSameComponent(v1,v2) == false)
      {
        if(pl->as<myPRM>()->distance(v1,v2) < d) return true;
      }
      return false;
  }

  ////////////////////////////////////////////////////////////////////////////
  //! Constructor
  omplPRMPlanner::omplPRMPlanner(SPACETYPE stype, Sample *init, Sample *goal, SampleSet *samples, WorkSpace *ws, og::SimpleSetup *ssptr):
              omplPlanner(stype, init, goal, samples, ws, ssptr)
  {
        _guiName = "ompl PRM Planner";
        _idName = "omplPRM";

        //create planner
        ob::PlannerPtr planner(new myPRM(si));


        //set grow and expand time
        _MinGrowTime = 2.0*mymagic::ROADMAP_BUILD_TIME;
        _MinExpandTime = mymagic::ROADMAP_BUILD_TIME;
        planner->as<myPRM>()->setMinGrowTime(_MinGrowTime);
        planner->as<myPRM>()->setMinExpandTime(_MinExpandTime);
        addParameter("MinGrowTime", _MinGrowTime);
        addParameter("MinExpandTime", _MinExpandTime);

        addParameter("Sampler 0(r) 1(h) 2(sdk) 3(g) 4(gl)", _samplerUsed);//defaulted to 0 (Random)

        //set the connectionFilter_
        double _distanceThreshold = 0.1 * mymagic::DISTANCE_THRESHOLD_FACTOR;//default value
        addParameter("DistanceThreshold", _distanceThreshold);
        planner->as<myPRM>()->setConnectionFilter(std::bind(&omplplanner::connectionDistanceFilter, std::placeholders::_1,std::placeholders::_2, _distanceThreshold, planner));

        //set the distance threshold for expand motions. Can be set if kauthamsampler is used
        _BounceDistanceThreshold = _distanceThreshold;
        addParameter("BounceDistanceThreshold", _BounceDistanceThreshold);
        planner->as<myPRM>()->usingKauthamSampler(true);
        planner->as<myPRM>()->setMaxDistanceBounceMotions(_BounceDistanceThreshold);

        //set the number of bounce steps in the expand phase
        _BounceSteps = mymagic::MAX_RANDOM_BOUNCE_STEPS;
        addParameter("BounceSteps", _BounceSteps);
        planner->as<myPRM>()->setBounceSteps(_BounceSteps);

        //set max neighbors
        _MaxNearestNeighbors = mymagic::DEFAULT_NEAREST_NEIGHBORS;
        addParameter("MaxNearestNeighbors", _MaxNearestNeighbors);
        planner->as<myPRM>()->setMaxNearestNeighbors(_MaxNearestNeighbors);




        //set the planner
        ss->setPlanner(planner);
        //disablePMDControlsFromSampling();
    }

	//! void destructor
    omplPRMPlanner::~omplPRMPlanner(){
			
	}
	
	//! setParameters sets the parameters of the planner
    bool omplPRMPlanner::setParameters(){

      omplPlanner::setParameters();
      try{
        HASH_S_K::iterator it = _parameters.find("MaxNearestNeighbors");

         if(it != _parameters.end()){
          _MaxNearestNeighbors = it->second;
          ss->getPlanner()->as<myPRM>()->setMaxNearestNeighbors(_MaxNearestNeighbors);
         }
        else
          return false;


        it = _parameters.find("MinExpandTime");
        if(it != _parameters.end()){
            _MinExpandTime = it->second;
            ss->getPlanner()->as<myPRM>()->setMinExpandTime(_MinExpandTime);
        }
        else
          return false;

        it = _parameters.find("MinGrowTime");
        if(it != _parameters.end()){
            _MinGrowTime = it->second;
            ss->getPlanner()->as<myPRM>()->setMinGrowTime(_MinGrowTime);
        }
        else
          return false;

        it = _parameters.find("BounceSteps");
        if(it != _parameters.end()){
            _BounceSteps = it->second;
            ss->getPlanner()->as<myPRM>()->setBounceSteps(_BounceSteps);
        }
        else
          return false;


        it = _parameters.find("BounceDistanceThreshold");
        if(it != _parameters.end()){
            _BounceDistanceThreshold = it->second;
            ss->getPlanner()->as<myPRM>()->setMaxDistanceBounceMotions(_BounceDistanceThreshold);
        }
        else
          return false;


        it = _parameters.find("DistanceThreshold");
        if(it != _parameters.end()){
            _distanceThreshold = it->second;
            ss->getPlanner()->as<myPRM>()->setConnectionFilter(std::bind(&omplplanner::connectionDistanceFilter, std::placeholders::_1,std::placeholders::_2, _distanceThreshold, ss->getPlanner()));
         }
        else
          return false;

        it = _parameters.find("Sampler 0(r) 1(h) 2(sdk) 3(g) 4(gl)");
        if(it != _parameters.end()){
            setSamplerUsed(it->second);
         }
        else
          return false;


      }catch(...){
        return false;
      }
      return true;
    }
  }
}

#endif // KAUTHAM_USE_OMPL

