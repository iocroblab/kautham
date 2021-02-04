
#ifndef TSRRT_H
#define TSRRT_H

#include "ompl/base/ProjectionEvaluator.h"
#include "ompl/geometric/planners/PlannerIncludes.h"
#include "ompl/datastructures/NearestNeighbors.h"

namespace ompl
 {
     namespace geometric
     {
         OMPL_CLASS_FORWARD(TaskSpaceConfig);

         class TaskSpaceConfig
         {
         public:
             virtual ~TaskSpaceConfig()
             {
             }
             // Returns the dimension of the task space.
             virtual int getDimension() const = 0;
             // Project the c-space state into the task-space.
             virtual void project(const base::State *state, Eigen::Ref<Eigen::VectorXd> ts_proj) const = 0;

             // Sample point uniformly in task-space.
             virtual void sample(Eigen::Ref<Eigen::VectorXd> ts_proj) const = 0;

             // Given a point in task-space, generate a configuraton space state
             // that projects to this point.  seed is the nearest task-space neighbor.
             virtual bool lift(const Eigen::Ref<Eigen::VectorXd> &ts_proj, const base::State *seed,
                               base::State *state) const = 0;
         };

         class TSRRT : public base::Planner
         {
         public:
             TSRRT(const base::SpaceInformationPtr &si, const TaskSpaceConfigPtr &task_space);

             virtual ~TSRRT();

             virtual void getPlannerData(base::PlannerData &data) const;

             virtual base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc);

             virtual void clear();

             void setGoalBias(double goalBias)
             {
                 goalBias_ = goalBias;
             }

             double getGoalBias() const
             {
                 return goalBias_;
             }

             void setRange(double distance)
             {
                 maxDistance_ = distance;
             }

             double getRange() const
             {
                 return maxDistance_;
             }

             template <template <typename T> class NN>
             void setNearestNeighbors()
             {
                 nn_.reset(new NN<Motion *>());
             }

             virtual void setup();

         protected:
             class Motion
             {
             public:
                 Motion(const base::SpaceInformationPtr &si) : state(si->allocState()), parent(nullptr)
                 {
                 }

                 ~Motion() = default;

                 base::State *state{nullptr};

                 Motion *parent{nullptr};

                 // Projection of the state into the task space.
                 Eigen::VectorXd proj;
             };

             void freeMemory();

             double distanceFunction(const Motion *a, const Motion *b) const
             {
                 double sqr_dist = 0.0;
                 for (int ix = 0; ix < a->proj.size(); ++ix)
                 {
                     double sqr_val = (*b).proj[ix] - (*a).proj[ix];
                     sqr_val *= sqr_val;

                     sqr_dist += sqr_val;
                 }
                 return sqr_dist;
             }

             std::shared_ptr<NearestNeighbors<Motion *>> nn_;

             double goalBias_{0.05};

             double maxDistance_{0.};

             RNG rng_;

             Motion *lastGoalMotion_{nullptr};

             // Mapping to/from task space.
             TaskSpaceConfigPtr task_space_;
         };

     }  // namespace geometric
 }  // namespace ompl

 #endif
