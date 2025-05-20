#include <kautham/planner/omplconstr/omplconstrplanner.hpp>

#if defined(KAUTHAM_USE_OMPL)

#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/ConstrainedSpaceInformation.h>
#include <kautham/planner/omplconstr/omplconstrValidityChecker.hpp>
#include <kautham/planner/omplconstr/constraint_factory.hpp>
#include <kautham/planner/omplconstr/constraints/abstract_ompl_constraint.hpp>
#include <kautham/planner/omplconstr/constraints/kautham_ompl_constraint_intersection.hpp>

// Planners:
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>

#include <ompl/base/Path.h>

#include <ompl/base/goals/GoalStates.h>
#include <ompl/base/goals/GoalState.h>

namespace Kautham {
    //! Namespace omplconstrplanner contains the planners based on the OMPL::constraint library
    namespace omplconstrplanner{

        /////////////////////////////////////////////////////////////////////////////////////////////////
        // KauthamConstrStateSampler functions
        /////////////////////////////////////////////////////////////////////////////////////////////////
        KauthamConstrStateSampler::KauthamConstrStateSampler(const ob::StateSpace *sspace, Planner *p) : ob::CompoundStateSampler(sspace)
        {
            kauthamPlanner_ = p;
            _samplerRandom = new RandomSampler(kauthamPlanner_->wkSpace()->getNumRobControls());
        }

        KauthamConstrStateSampler::~KauthamConstrStateSampler() {
            delete _samplerRandom;
        }

        void KauthamConstrStateSampler::sampleUniform(ob::State *state)
        {

            ob::ScopedState<ob::CompoundStateSpace> sstate(  ((omplConstraintPlanner*)kauthamPlanner_)->getSpace() );

            bool withinbounds = false;
            int trials = 0;
            Sample* smp = NULL;

            do {
                /*
                Sample the Kautham control space.
                Controls are defined in the input xml files.
                Each control value lies in the [0,1] interval
                */
                smp = _samplerRandom->nextSample();

                // Those controls that are disabled for sampling are now restored to 0.5
                for (unsigned j=0; j<((omplConstraintPlanner*)kauthamPlanner_)->getDisabledControls()->size(); j++) {
                    smp->getCoords()[ ((omplConstraintPlanner*)kauthamPlanner_)->getDisabledControls()->at(j) ] = 0.5;
                }

                // Compute the mapped configurations (i.e.se3+Rn values) by calling MoveRobotsTo function.
                kauthamPlanner_->wkSpace()->moveRobotsTo(smp);
                withinbounds = smp->getwithinbounds();
                trials++;
            } while (!withinbounds && trials < 100);


            // If trials==100 is because we have not been able to find a sample within limits
            // In this case the config is set to the border in the moveRobotsTo function.
            // The smp is finally converted to state and returned

            // Convert from sample to scoped state
            ((omplConstraintPlanner*)kauthamPlanner_)->smp2omplScopedState(smp, &sstate);

            // Print for debugging
            // std::cout << "Generated Sample State:\n";
            // sstate.print(std::cout);

            // Return in parameter state:
            ((omplConstraintPlanner*)kauthamPlanner_)->getSpace()->copyState(state, sstate.get());

            // POLs TESTING:
            // const auto& full_space = ((omplConstraintPlanner*)kauthamPlanner_)->getSpace()->as<ob::CompoundStateSpace>();
            // const auto* full_state = state->as<ob::CompoundStateSpace::StateType>();

            // const std::vector<ob::StateSpacePtr>& robot_spaces = full_space->getSubspaces();

            // for (size_t r = 0; r < robot_spaces.size(); ++r) {
            //     if (auto robot_space = std::dynamic_pointer_cast<ob::CompoundStateSpace>(robot_spaces[r])) {
            //         std::cout << "Subspace " << r << " (" << robot_space->getName() << "):" << std::endl;
            //         const auto* robot_state = full_state->as<ob::CompoundStateSpace::StateType>(r);

            //         const std::vector<ob::StateSpacePtr>& robot_sub_spaces = robot_space->getSubspaces();
            //         for (size_t s = 0; s < robot_sub_spaces.size(); ++s) {
            //             std::string robot_sub_space_name = robot_sub_spaces[s]->getName();
                        
            //             if (robot_sub_space_name.find("_RnConstr_") != std::string::npos) {
            //                 auto rnConstr_space = robot_sub_spaces[s]->as<ob::ProjectedStateSpace>();
            //                 if (rnConstr_space) {
            //                     std::cout << "ProjectedStateSpace for " << robot_sub_space_name << std::endl;
            //                     const auto* constrained_state = robot_state->as<ob::ProjectedStateSpace::StateType>(s);
            //                     const auto* constrainedRealVector_state = constrained_state->getState()->as<ob::RealVectorStateSpace::StateType>();

            //                     for (unsigned int i = 0; i < rnConstr_space->getDimension(); ++i) {
            //                         std::cout << "  Dimension " << i << ": " << constrainedRealVector_state->values[i] << std::endl;
            //                     }
            //                     auto constraint = std::dynamic_pointer_cast<AbstractOMPLConstraint>(rnConstr_space->getConstraint());
            //                     // ompl::base::State* pol = const_cast<ompl::base::State*>(constrained_state->getState());
            //                     // constraint->project(std::move(pol));
            //                     std::cout << "Constraint name: " << constraint->name_ << std::endl;

            //                     /*###########*/
            //                     // try {
            //                     //     auto constrained_state0 = robot_state->as<ob::ProjectedStateSpace::StateType>(s);
            //                     //     ompl::base::State pol = dynamic_cast<ompl::base::State>(constrained_state0->getState());
            //                     //     auto constrained_state2 = pol.as<ompl::base::ProjectedStateSpace::StateType>();
    
            //                     //     if (constrained_state2) {
            //                     //         std::cout << "Projected State:" << std::endl;
                                        
            //                     //         // Get the underlying state
            //                     //         auto constrainedRealVector_state2 = constrained_state2.getState()->as<ompl::base::RealVectorStateSpace::StateType>();
                                        
            //                     //         if (constrainedRealVector_state2) {
            //                     //             std::cout << "Num DOF2 = " << constraint->getAmbientDimension() << std::endl;
            //                     //             for (unsigned int i = 0; i < constraint->getAmbientDimension(); ++i) {
            //                     //                 std::cout << "  Dimension " << i << ": " << &(constrainedRealVector_state2.values[i]) << std::endl;
            //                     //                 std::cout << "  Dimension " << i << ": " << (constrainedRealVector_state2.values[i]) << std::endl;
            //                     //             }
            //                     //         } else {
            //                     //             std::cout << "Error: Unable to cast state to RealVectorStateSpace::StateType" << std::endl;
            //                     //         }

            //                     //     } else {
            //                     //         std::cout << "Error: Unable to cast state to ProjectedStateSpace::StateType" << std::endl;
            //                     //     }
            //                     // } catch (const std::exception& e) {
            //                     //     std::cerr << "Exception caught: " << e.what() << std::endl;
            //                     // }
                                
                                
            //                     /*###########*/

            //                 }
            //             }
            //             // This "else if" is needed for "_Rn", to avoid collisions with "_RnConstr_". Also, it's is important to keep the order, first "_RnConstr_" and then "_Rn"
            //             else if (robot_sub_space_name.find("_Rn") != std::string::npos) {
            //                 auto rn_space = robot_sub_spaces[s]->as<ob::RealVectorStateSpace>();
            //                 if (rn_space) {
            //                     std::cout << "RealVectorStateSpace for " << robot_sub_space_name << std::endl;
            //                     const auto* real_vector_state = robot_state->as<ob::RealVectorStateSpace::StateType>(s);

            //                     for (unsigned int i = 0; i < rn_space->getDimension(); ++i) {
            //                         std::cout << "  Dimension " << i << ": " << real_vector_state->values[i] << std::endl;
            //                     }
            //                 }
            //             }
            //         }
            //     }
            // }
        }


        /////////////////////////////////////////////////////////////////////////////////////////////////
        // AUXILIAR functions
        /////////////////////////////////////////////////////////////////////////////////////////////////
        //! This function is used to allocate a state sampler
        ob::StateSamplerPtr allocStateSampler(const ob::StateSpace *mysspace, Planner *p)
        {
            return ob::StateSamplerPtr(new KauthamConstrStateSampler(mysspace, p));
        }


        // Constructor implementation
        omplConstraintPlanner::omplConstraintPlanner(SPACETYPE stype, Sample *init, Sample *goal, SampleSet *samples, WorkSpace *ws, og::SimpleSetup *ssptr)
            : Planner(stype, init, goal, samples, ws)
        {
            _family = OMPLCONSTRPLANNER;

            // Set intial values from parent class data:
            _speedFactor = 1;
            _solved = false;
            _guiName = "ompl Constr Planner";
            _idName = "ompl Constr Planner";

            // Set own intial values:
            _planningTime = 10;
            _range = 0.1;
            _simplify = 0;
            _incremental = 0;

            // Add planner parameters:
            addParameter("_Max Planning Time",_planningTime);
            addParameter("_Speed Factor",_speedFactor);
            addParameter("_Range",_range);
            addParameter("_Simplify Solution",_simplify);
            addParameter("_Incremental (0/1)",_incremental);

            if (ssptr == NULL) {

                // Init constraint factory to create the constraints based on the name:
                std::shared_ptr<ConstraintFactory> constraints_factory = std::make_shared<ConstraintFactory>();
                constraints_factory->registerConstraints();
                // constraints_factory->printRegisteredConstraintNames();

                //Construct the state space we are planning in. It is a compound state space composed of a compound state space for each robot
                //Each robot has a compound state space composed of a (optional) SE3 state space and a (optional) Rn state space

                this->space_ = std::make_shared<ompl::base::CompoundStateSpace>();
                this->space_->setName("full_space");

                // Create SE3 state space
                std::shared_ptr<ob::StateSpace> spaceSE3;

                // Create Rn state space
                std::shared_ptr<ob::StateSpace> spaceRn;

                // Create a constrained space for Rn space
                std::shared_ptr<ob::StateSpace> spaceRnConstr;

                // Create a robot compound state space [SE3 + Rn]
                std::shared_ptr<ob::StateSpace> spaceRob;

                std::string space_name;

                size_t rob_subspace_index = 0;
                
                // Loop for all robots:
                for (unsigned rob = 0; rob < _wkSpace->getNumRobots(); rob++) {
                    auto current_rob = _wkSpace->getRobot(rob);
                    rob_subspace_index = 0;
                    spaceRob = std::make_shared<ob::CompoundStateSpace>();
                    
                    // Create state space SE3 for the mobile base, if necessary:
                    if (current_rob->isSE3Enabled()) {
                        spaceSE3 = std::make_shared<ob::SE3StateSpace>();
                        space_name = "ssRobot" + std::to_string(rob) + "_SE3";
                        std::cout << "isSE3Enabled: " << space_name << std::endl;
                        spaceSE3->setName(space_name);

                        //set the bounds. If the bounds are equal or its difference is below a given epsilon value (0.001) then
                        //set the higher bound to the lower bound plus this eplsion
                        ob::RealVectorBounds bounds(3);

                        //x-direction
                        double low = current_rob->getLimits(0)[0];
                        double high = current_rob->getLimits(0)[1];
                        filterBounds(low, high, 0.001);
                        bounds.setLow(0, low);
                        bounds.setHigh(0, high);

                        //y-direction
                        low = current_rob->getLimits(1)[0];
                        high = current_rob->getLimits(1)[1];
                        filterBounds(low, high, 0.001);
                        bounds.setLow(1, low);
                        bounds.setHigh(1, high);

                        //z-direction
                        low = current_rob->getLimits(2)[0];
                        high = current_rob->getLimits(2)[1];
                        filterBounds(low, high, 0.001);
                        bounds.setLow(2, low);
                        bounds.setHigh(2, high);

                        spaceSE3->as<ob::SE3StateSpace>()->setBounds(bounds);

                        spaceRob->as<ob::CompoundStateSpace>()->addSubspace(spaceSE3, 1.0);
                        rob_subspace_index++;
                    }

                    // Create the Rn state space for the kinematic chain, if necessary:
                    // Also create the constrained space if requested by the robot.
                    const unsigned int num_dof = current_rob->getNumJoints();
                    if (num_dof > 0) {
                        have_constr_space = false;
                        have_unconstr_space = false;

                        // 1. First assume that all joint are unconstrained:
                        std::vector<std::string> unconstrained_joint_names;
                        std::vector<std::string> all_joint_names;
                        for (unsigned int n = 0; n < num_dof; n++) {    // n = 0 is the base
                            unconstrained_joint_names.push_back(current_rob->getLink(n+1)->getName());
                            all_joint_names.push_back(current_rob->getLink(n+1)->getName());
                        }
                        
                        // 2. Get all constraints for this robot and group them by intersection group:
                        std::vector<std::shared_ptr<RobotProblemConstraint>> prob_constraints = current_rob->getConstraints();
                        std::map<std::string, std::vector<std::shared_ptr<RobotProblemConstraint>>> intersection_groups;
                        for (const auto& prob_constr : prob_constraints) {
                            std::string group = prob_constr->getIntersectionGroup();
                            if (group.empty()) {
                                // Fallback: treat as unique group if no group assigned.
                                group = prob_constr->getConstraintId();
                            }
                            intersection_groups[group].push_back(prob_constr);
                        }

                        // 3. For each intersection group, create a single constrained state space:
                        for (const auto& [group_name, group_constraints] : intersection_groups) {
                            have_constr_space = true;

                            // Gather all unique joint names and indices in this group:
                            std::vector<std::pair<std::string, uint>> group_constr_joints;
                            std::set<std::string> joint_names_set;
                            for (const auto& constr : group_constraints) {
                                for (const auto& pair : constr->getConstrainedJoints()) {
                                    if (joint_names_set.insert(pair.first).second) {
                                        group_constr_joints.push_back(pair);
                                    }
                                }
                            }

                            // Build the OMPL RealVectorStateSpace for this group:
                            spaceRn = std::make_shared<ob::RealVectorStateSpace>(group_constr_joints.size());
                            ob::RealVectorBounds bounds(group_constr_joints.size());
                            for (unsigned int j = 0; j < group_constr_joints.size(); j++) {
                                double low = *current_rob->getLink(group_constr_joints[j].first)->getLimits(true);
                                double high = *current_rob->getLink(group_constr_joints[j].first)->getLimits(false);
                                bounds.setLow(j, low);
                                bounds.setHigh(j, high);
                            }
                            spaceRn->as<ob::RealVectorStateSpace>()->setBounds(bounds);

                            // Create OMPL constraints for each constraint in the group:
                            std::vector<std::shared_ptr<ompl::base::Constraint>> ompl_constraints;
                            for (const auto& constr : group_constraints) {
                                auto constraint = constraints_factory->createConstraint(constr->getConstraintType(), constr, group_constr_joints.size());
                                constraint->associateRobot(current_rob);
                                constraint->assignReferencedEntity(_wkSpace->getObstacle(constr->getReferenceFrameEntity()));
                                constraint->initConstraintStuff();
                                this->constraint_map_[constr->getConstraintId()] = constraint;
                                ompl_constraints.push_back(constraint);
                            }

                            auto intersection_constraint = std::make_shared<KauthamOMPLConstraintIntersection>(group_constr_joints.size(), ompl_constraints);
                            spaceRnConstr = std::make_shared<ob::ProjectedStateSpace>(spaceRn, intersection_constraint);
                            space_name = "ssRobot" + std::to_string(rob) + "_RnConstr_" + group_name;
                            spaceRob->as<ob::CompoundStateSpace>()->addSubspace(spaceRnConstr, 1.0);
                            rob_subspace_index++;
                            spaceRob->as<ob::CompoundStateSpace>()->getSubspace(rob_subspace_index-1)->setName(space_name); // Si no es fa aquí, no s'aplica bé el nom en el cas de les constraints...!

                            // Create SpaceInformation specifically for the ProjectedStateSpace:
                            auto spaceRnConstr_si = std::make_shared<ompl::base::SpaceInformation>(spaceRnConstr);
                            // Associate the SpaceInformation object with the ConstrainedStateSpace:
                            spaceRnConstr->as<ob::ProjectedStateSpace>()->setSpaceInformation(spaceRnConstr_si.get());

                            // Get all the constraint names of this constraint (or group of constraints):
                            std::vector<std::string> constr_names;
                            for (const auto& pair : group_constr_joints) {
                                constr_names.push_back(pair.first);
                            }

                            // Convert constr_joints to an unordered_set for efficient lookups:
                            std::unordered_set<std::string> to_remove(constr_names.begin(), constr_names.end());

                            // Remove elements in unconstrained_joint_names that are present in the to_remove set:
                            unconstrained_joint_names.erase(std::remove_if(unconstrained_joint_names.begin(), unconstrained_joint_names.end(), 
                                [&to_remove](const std::string& element) {
                                    return to_remove.count(element) > 0;
                                }), unconstrained_joint_names.end());
                            
                        }

                        // 4. Create the Rn space with the remaining unconstrained joints:
                        if (unconstrained_joint_names.size() > 0) {                            
                            have_unconstr_space = true;
                            current_rob->setUnconstrainedJointNames(unconstrained_joint_names);
                            auto katxopo = current_rob->getUnconstrainedJointNames();

                            // Iterate over the remaining data and find the corresponding original indices
                            for (const auto& item : unconstrained_joint_names) {
                                // Find the original index of the remaining item
                                auto it = std::find(all_joint_names.begin(), all_joint_names.end(), item);
                                
                                if (it != all_joint_names.end()) {
                                    // Calculate the original index and store it
                                    size_t original_index = std::distance(all_joint_names.begin(), it);
                                    index_mapping_.push_back(original_index);
                                }
                            }

                            spaceRn = std::make_shared<ob::RealVectorStateSpace>(unconstrained_joint_names.size());
                            space_name = "ssRobot" + std::to_string(rob) + "_Rn";
                            spaceRn->setName(space_name);

                            // Set the bounds:
                            ob::RealVectorBounds bounds(unconstrained_joint_names.size());
                            double low, high;
                            for (unsigned int j = 0; j < unconstrained_joint_names.size(); j++) {
                                low = *current_rob->getLink(unconstrained_joint_names[j])->getLimits(true);      // True = getLowLimit
                                high = *current_rob->getLink(unconstrained_joint_names[j])->getLimits(false);    // Flase = getHighLimit
                                // std::cout << "Constr name: " << unconstrained_joint_names[j] << " Low: " << low << std::endl;
                                // std::cout << "Constr name: " << unconstrained_joint_names[j] << " High: " << high << std::endl;
                                bounds.setLow(j, low);
                                bounds.setHigh(j, high);
                            }
                            spaceRn->as<ob::RealVectorStateSpace>()->setBounds(bounds);

                            spaceRob->as<ob::CompoundStateSpace>()->addSubspace(spaceRn, 1.0);
                            rob_subspace_index++;
                        }
                    }

                    space_name = "ssRobot" + std::to_string(rob);
                    spaceRob->setName(space_name);
                    this->space_->as<ob::CompoundStateSpace>()->addSubspace(spaceRob, 1.0);

                }

                this->space_->diagram(std::cout);


                // ############################################
                // Define problem stuff
                // ############################################

                // Set the custom state sampler:
                this->space_->setStateSamplerAllocator(std::bind(&allocStateSampler, std::placeholders::_1, (Planner*)this));

                // Create SpaceInformation for the compound space
                this->si_ = std::make_shared<ompl::base::SpaceInformation>(this->space_);

                // Set validity checker:
                auto my_ValidityChecker = std::make_shared<Kautham::omplconstrplanner::ValidityChecker>(this->si_,  (Planner*)this);
                this->si_->setStateValidityChecker(ob::StateValidityCheckerPtr(my_ValidityChecker));

                this->si_->setup(); // Finalize the SpaceInformation setup

                // Create a problem definition
                this->pdef_ = std::make_shared<ompl::base::ProblemDefinition>(this->si_);

                // ############################################
                // Define start and goal states using subspaces
                // ############################################
                omplConstraintPlanner::setStartAndGoalFromKthmSample();

                // ############################################
                // Set the planner:
                // ############################################

                // this->ompl_planner_ = std::make_shared<ompl::geometric::RRT>(this->si_);
                this->ompl_planner_ = std::make_shared<ompl::geometric::RRTConnect>(this->si_);
                // this->ompl_planner_ = std::make_shared<ompl::geometric::RRTstar>(this->si_);
                
                // this->ompl_planner_->as<ompl::geometric::RRT>()->setRange(_range);
                this->ompl_planner_->as<ompl::geometric::RRTConnect>()->setRange(_range);
                // this->ompl_planner_->as<ompl::geometric::RRTstar>()->setRange(_range);
                
                this->ompl_planner_->setProblemDefinition(this->pdef_);
                
                this->ompl_planner_->setup();

            } else {
                std::cout << "ACTUALITZAR ELSE!" << std::endl;

                // simple_setup_ = (og::SimpleSetupPtr)ssptr;
                // this->si_ = simple_setup_->getSpaceInformation();
                // this->space_ = simple_setup_->getStateSpace();
            }
        }

        omplConstraintPlanner::~omplConstraintPlanner(){

        }

        void omplConstraintPlanner::setStartAndGoalFromKthmSample() {

            ompl::base::ScopedState<> start(this->space_);
            ompl::base::ScopedState<> goal(this->space_);

            // Add start states:
            this->pdef_->clearStartStates();
            for (std::vector<Sample*>::const_iterator start(_init.begin()); start != _init.end(); ++start) {
                //Start state: convert from smp to scoped state
                ob::ScopedState<ob::CompoundStateSpace> startompl(this->space_);
                omplConstraintPlanner::smp2omplScopedState(*start, &startompl);
                std::cout << "OMPL Constraint: Start State:" << std::endl;
                startompl.print();
                this->pdef_->addStartState(startompl);
            }

            // Add goal states:
            this->pdef_->clearGoal();
            ob::GoalStates *goalStates(new ob::GoalStates(si_));
            for (std::vector<Sample*>::const_iterator goal(_goal.begin()); goal != _goal.end(); ++goal) {
                //Goal state: convert from smp to scoped state
                ob::ScopedState<ob::CompoundStateSpace> goalompl(this->space_);
                omplConstraintPlanner::smp2omplScopedState(*goal, &goalompl);
                std::cout << "OMPL Constraint: Goal State:" << std::endl;
                goalompl.print();
                goalStates->addState(goalompl);
                // Assign the goal values to each constraint.
                // Aiming of known the goal pose and to compute error between the current and goal sample pose:
                omplConstraintPlanner::assignConstrGoalPoseFromState(goalompl);
            }
            this->pdef_->setGoal(ob::GoalPtr(goalStates));
        }

        bool omplConstraintPlanner::setParameters() {
            try {
                HASH_S_K::iterator it;

                it = _parameters.find("_Speed Factor");
                if(it != _parameters.end()) {
                    if (_speedFactor != it->second) {
                        std::cout << "Speed Factor has been modified from " << _speedFactor;
                        _speedFactor = it->second;
                        std::cout << " to " << _speedFactor << std::endl;
                    }
                } else {
                    return false;
                }

                it = _parameters.find("_Max Planning Time");
                if(it != _parameters.end()) {
                    if (_planningTime != it->second) {
                        std::cout << "Max Planning Time has been modified from " << _planningTime;
                        _planningTime = it->second;
                        std::cout << " to " << _planningTime << std::endl;
                    }
                } else {
                    return false;
                }

                it = _parameters.find("_Range");
                if(it != _parameters.end()) {
                    if (_range != it->second) {
                        std::cout << "Range has been modified from " << _range;
                        _range = it->second;
                        std::cout << " to " << _range << std::endl;
                    }
                } else {
                    return false;
                }

                it = _parameters.find("_Simplify Solution");
                if(it != _parameters.end()) {
                    if (_simplify != it->second) {
                        std::cout << "Simplify Solution has been modified from " << _simplify;
                        if (it->second == 0 || it->second == 1 || it->second == 2) {
                            _simplify = it->second;
                        } else {
                            _simplify = 0;
                            it->second = 0;
                        }
                        std::cout << " to " << _simplify << std::endl;
                    }
                } else {
                    return false;
                }

                it = _parameters.find("_Incremental (0/1)");
                if(it != _parameters.end()) {
                    if (_incremental != it->second) {
                        std::cout << "Incremental has been modified from " << _incremental;
                        if (it->second == 0 || it->second == 1) {
                            _incremental = it->second;
                        } else {
                            _incremental = 0;
                            it->second = 0;
                        }
                        std::cout << " to " << _incremental << std::endl;
                    }
                } else {
                    return false;
                }

            } catch(...) {
                return false;
            }
            return true;
        }

        bool omplConstraintPlanner::trySolve() {
            // Implement the logic that attempts to solve the planning problem.
            // Return true if successful, false otherwise.

            // Re-define start and goal states (could be different when the space has been constructed):
            omplConstraintPlanner::setStartAndGoalFromKthmSample();

            // Remove previous solutions path, if any:
            if (_incremental) {
                this->pdef_->clearSolutionPaths();
            } else {
                // this->pdef_->clear();   // OMPL v1.6.+
                // Forget the solution paths (thread safe). Memory is freed. 
                this->pdef_->clearSolutionPaths();
                // Clear all internal datastructures. Planner settings are not affected. Subsequent calls to solve() will ignore all previous work. 
                this->ompl_planner_->clear();
            }
            
            // Attempt to solve the problem within _planningTime seconds:
            ompl::base::PlannerTerminationCondition ptc = ompl::base::timedPlannerTerminationCondition(_planningTime);
            ompl::base::PlannerStatus solved = this->ompl_planner_->solve(ptc);

            if (solved) {

                std::shared_ptr<ompl::base::Path> solution_path = this->pdef_->getSolutionPath();

                // solution_path->print(std::cout);

                og::PathGeometric& path = static_cast<og::PathGeometric&>(*solution_path);

                if (_simplify != 0) {
                    og::PathSimplifier simplifier(this->si_);
                    if (_simplify == 1) {   // Smooth
                        simplifier.smoothBSpline(path);
                    } else if (_simplify == 2) {    // Shorten
                        simplifier.simplifyMax(path);
                    }
                }

                _path.clear();
                clearSimulationPath();
                Sample *smp;

                // Load the simplified path into Kautham _path variable
                for (std::size_t j = 0; j < path.getStateCount(); ++j) {
                    smp = new Sample(_wkSpace->getNumRobControls());
                    smp->setMappedConf(_init.at(0)->getMappedConf());

                    omplConstraintPlanner::omplState2smp(path.getState(j)->as<ob::CompoundStateSpace::StateType>(), smp);

                    _path.push_back(smp);
                    _samples->add(smp);
                }

                _solved = true;

                // std::cout << "Joint path ready to be used in robot_arm as CSV file input:" << std::endl;
                // printJointPath(joint_path);

            } else {
                std::cout << "No solution found." << std::endl;
                _solved = false;
            }
            return _solved;
        }

        void omplConstraintPlanner::assignConstrGoalPoseFromState(const ob::ScopedState<ob::CompoundStateSpace>& _ompl_state) {
            // For each robot, assign the goal to each constraint group using the state vector.
        
            // 1. Flatten the state vector, skipping SE3 if present:
            std::vector<double> ompl_state_reals;
            size_t state_offset = 0;
            // ToDo/Note: Not check with multiple robots...
            for (unsigned rob = 0; rob < _wkSpace->getNumRobots(); rob++) {
                auto current_rob = _wkSpace->getRobot(rob);
                if (current_rob->isSE3Enabled()) {
                    // Each SE3 is 7 values (x, y, z, qx, qy, qz, qw)
                    state_offset += 7;
                }
            }
            const auto& reals = _ompl_state.reals();
            if (state_offset < reals.size()) {
                // Extract the whole vector except the first 7 values of the SE3 (position + orientation):
                ompl_state_reals.assign(reals.begin() + state_offset, reals.end());
            } else {
                ompl_state_reals.clear();   // Only assign when SE3 is not enabled, because is full Rn.
            }
        
            // 2. For each robot, walk through its constraint groups as in initialization:
            size_t state_cursor = 0;
            for (unsigned rob = 0; rob < _wkSpace->getNumRobots(); rob++) {
                auto current_rob = _wkSpace->getRobot(rob);
        
                // --- Gather constraint groups as in initialization ---

                std::vector<std::shared_ptr<RobotProblemConstraint>> prob_constraints = current_rob->getConstraints();
                std::map<std::string, std::vector<std::shared_ptr<RobotProblemConstraint>>> intersection_groups;
                for (const auto& prob_constr : prob_constraints) {
                    std::string group = prob_constr->getIntersectionGroup();
                    if (group.empty()) {
                        group = prob_constr->getConstraintId();
                    }
                    intersection_groups[group].push_back(prob_constr);
                }
        
                // For each constraint group, assign the corresponding segment of the state vector:
                for (const auto& [group_name, group_constraints] : intersection_groups) {
                    // Gather all unique joint names in this group (order must match initialization)
                    std::vector<std::pair<std::string, uint>> group_constr_joints;
                    std::set<std::string> joint_names_set;
                    for (const auto& constr : group_constraints) {
                        for (const auto& pair : constr->getConstrainedJoints()) {
                            if (joint_names_set.insert(pair.first).second) {
                                group_constr_joints.push_back(pair);
                            }
                        }
                    }
                    size_t group_size = group_constr_joints.size();
        
                    // Extract the relevant values for this group from the state vector:
                    if (state_cursor + group_size > ompl_state_reals.size()) {
                        std::cerr << "assignConstrGoalPoseFromState: State vector too small for constraint group " << group_name << std::endl;
                        return;
                    }
                    std::vector<double> group_joint_values(
                        ompl_state_reals.begin() + state_cursor,
                        ompl_state_reals.begin() + state_cursor + group_size
                    );
                    state_cursor += group_size;

                    // std::cout << "Auto constraint (" << group_name << ") config: [ ";
                    // for (const auto& value : group_joint_values) {
                    //     std::cout << value << " ";
                    // }
                    // std::cout << "]" << std::endl;

        
                    // Assign this configuration as the target for all OMPL constraints in the group
                    for (const auto& constr : group_constraints) {
                        std::string constr_id_name = constr->getConstraintId();
                        auto it = this->constraint_map_.find(constr_id_name);
                        if (it != this->constraint_map_.end()) {
                            it->second->setGoalSampleConfiguration(group_joint_values);
                        } else {
                            std::cerr << "assignConstrGoalPoseFromState: Constraint ID " << constr_id_name << " not found in constraint_map_!" << std::endl;
                        }
                    }
                }
        
                // Handle unconstrained joints: skip their values in the state vector
                std::vector<std::string> unconstrained_joint_names = current_rob->getUnconstrainedJointNames();
                size_t unconstr_size = unconstrained_joint_names.size();
                state_cursor += unconstr_size; // skip these values, not used for constraints
            }
        }        


        //! This function converts a Kautham sample to an ompl scoped state.
        void omplConstraintPlanner::smp2omplScopedState(Sample* smp, ob::ScopedState<ob::CompoundStateSpace> *sstate)
        {
            // Extract the mapped configuration of the sample. It is a vector with as many components as robots.
            // Each component has the RobConf of the robot (the SE3 and the Rn configurations including constraints)
            if (smp->getMappedConf().size() == 0) {
                _wkSpace->moveRobotsTo(smp); // To set the mapped configuration
            }
            std::vector<RobConf>& smpRobotsConf = smp->getMappedConf();

            // Loop for all the robots:
            for (unsigned rob = 0; rob < _wkSpace->getNumRobots(); rob++)
            {
                auto current_rob = _wkSpace->getRobot(rob);
                size_t rob_subspace_index = 0;

                // Get the subspace of robot:
                // ob::StateSpacePtr ssRoboti = ((ob::StateSpacePtr) this->space_->as<ob::CompoundStateSpace>()->getSubspace(rob));
                ob::StateSpacePtr ssRoboti = this->space_->as<ob::CompoundStateSpace>()->getSubspace(rob);

                // Create the Scoped State of the current robot:
                ompl::base::ScopedState<> ScopedStateRobot(ssRoboti);

                // If it has se3 part:
                if (current_rob->isSE3Enabled()) {
                    // Get the kautham SE3 configuration:
                    SE3Conf c = smpRobotsConf.at(rob).getSE3();
                    vector<double>& pp = c.getPos();
                    vector<double>& aa = c.getAxisAngle();

                    // Set the OMPL SE3 configuration:
                    // ob::StateSpacePtr ssRobotiSE3 =  ((ob::StateSpacePtr) ssRoboti->as<ob::CompoundStateSpace>()->getSubspace(rob_subspace_index));
                    ob::StateSpacePtr ssRobotiSE3 = ssRoboti->as<ob::CompoundStateSpace>()->getSubspace(rob_subspace_index);

                    ob::ScopedState<ob::SE3StateSpace> scopedStateSE3(ssRobotiSE3);
                    scopedStateSE3->setX(pp[0]);
                    scopedStateSE3->setY(pp[1]);
                    scopedStateSE3->setZ(pp[2]);
                    scopedStateSE3->rotation().setAxisAngle(aa[0],aa[1],aa[2],aa[3]);

                    // Add the SE3 ScopedState to the robot state:
                    ScopedStateRobot << scopedStateSE3;
                    rob_subspace_index++;
                }

                // Has Rn part:
                const unsigned int num_dof = current_rob->getNumJoints();
                if (num_dof > 0) {

                    // Extract the joint configuration (Rn) from the sample:
                    const RnConf& r = smpRobotsConf.at(rob).getRn();
                    // RnConf r = smpRobotsConf.at(rob).getRn();

                    // --- Handle constraint groups as in initialization ---

                    // Get the constraint groups (intersection groups)
                    std::vector<std::shared_ptr<RobotProblemConstraint>> prob_constraints = current_rob->getConstraints();
                    std::map<std::string, std::vector<std::shared_ptr<RobotProblemConstraint>>> intersection_groups;
                    for (const auto& prob_constr : prob_constraints) {
                        std::string group = prob_constr->getIntersectionGroup();
                        if (group.empty()) {
                            group = prob_constr->getConstraintId();
                        }
                        intersection_groups[group].push_back(prob_constr);
                    }

                    // For each constraint group, fill the corresponding subspace:
                    for (const auto& [group_name, group_constraints] : intersection_groups) {
                        // Gather all unique joint names in this group (order must match initialization)
                        std::vector<std::pair<std::string, uint>> group_constr_joints;
                        std::set<std::string> joint_names_set;
                        for (const auto& constr : group_constraints) {
                            for (const auto& pair : constr->getConstrainedJoints()) {
                                if (joint_names_set.insert(pair.first).second) {
                                    group_constr_joints.push_back(pair);
                                }
                            }
                        }
                        // Get the subspace for this constraint group:
                        ob::StateSpacePtr ssRobotiRnConstr = ssRoboti->as<ob::CompoundStateSpace>()->getSubspace(rob_subspace_index);
                        ob::ScopedState<ob::ProjectedStateSpace> scopedStateRnConstr(ssRobotiRnConstr);

                        // Fill the constrained joints in the correct order:
                        size_t q = 0;
                        for (const auto& [joint_name, _] : group_constr_joints) {
                            // Find the joint index in the robot
                            for (unsigned int n = 0; n < num_dof; n++) {
                                if (current_rob->getLink(n+1)->getName() == joint_name) {   // n=0 is the base
                                    // std::cout << "Match with constr " << group_name << " of " << joint_name << std::endl;
                                    scopedStateRnConstr[q] = r.getCoordinate(n);
                                    // std::cout << "Joint[" << q << "] = " << r.getCoordinate(n) << std::endl;
                                    q++;
                                    break;
                                }
                            }
                        }
                        // Add the RnConstr ScopedState to the robot state:
                        ScopedStateRobot << scopedStateRnConstr;
                        rob_subspace_index++;
                    }

                    // Handle unconstrained joints (if any):
                    std::vector<std::string> unconstrained_joint_names = current_rob->getUnconstrainedJointNames();
                    if (!unconstrained_joint_names.empty()) {
                        ob::StateSpacePtr ssRobotiRn = ssRoboti->as<ob::CompoundStateSpace>()->getSubspace(rob_subspace_index);
                        ob::ScopedState<ob::RealVectorStateSpace> scopedStateRn(ssRobotiRn);

                        size_t q = 0;
                        for (unsigned int n = 0; n < num_dof; n++) {
                            std::string joint_name = current_rob->getLink(n+1)->getName();
                            if (std::find(unconstrained_joint_names.begin(), unconstrained_joint_names.end(), joint_name) != unconstrained_joint_names.end()) {
                                // std::cout << "Match with Rn of " << joint_name << std::endl;
                                scopedStateRn[q] = r.getCoordinate(n);
                                // std::cout << "Joint[" << q << "] = " << r.getCoordinate(n) << std::endl;
                                q++;
                            }
                        }
                        // Add the Rn ScopedState to the robot state:
                        ScopedStateRobot << scopedStateRn;
                        rob_subspace_index++;
                    }

                }   // END: IF (num_dof > 0)

                // Add the robot's ScopedState to the full state:
                (*sstate) << ScopedStateRobot;
            }
        }

        //! This member function converts an ompl State to a Kautham sample
        void omplConstraintPlanner::omplState2smp(const ob::State *state, Sample* smp)
        {
            ob::ScopedState<ob::CompoundStateSpace> sstate(space_);
            sstate = *state;
            omplConstraintPlanner::omplScopedState2smp(sstate, smp);
        }

        //! This member function converts an ompl ScopedState to a Kautham sample
        void omplConstraintPlanner::omplScopedState2smp(ob::ScopedState<ob::CompoundStateSpace> sstate, Sample* smp)
        {
            std::vector<RobConf> rc;

            // Loop for all the robots:
            for (unsigned int rob = 0; rob < _wkSpace->getNumRobots(); ++rob) {

                auto current_rob = _wkSpace->getRobot(rob);

                // RobConf to store the robots configurations read from the ompl state:
                RobConf rcj;

                // Get the subspace corresponding to robot:
                ob::StateSpacePtr ssRoboti = space_->as<ob::CompoundStateSpace>()->getSubspace(rob);

                // Get the SE3 subspace of the current robot, if it exists, extract the SE3 configuration:
                unsigned int rob_subspace_index = 0; // Counter of subspaces of the current robot.
                if (current_rob->isSE3Enabled()) {
                    // Get the SE3 subspace of the curent robot:
                    ob::StateSpacePtr ssRobotiSE3 = ssRoboti->as<ob::CompoundStateSpace>()->getSubspace(rob_subspace_index);

                    // Create a SE3 scoped state and load it with the data extracted from the global scoped state:
                    ob::ScopedState<ob::SE3StateSpace> pathscopedstatese3(ssRobotiSE3);
                    sstate >> pathscopedstatese3;

                    // Convert it to a vector of 7 components:
                    std::vector<double> se3coords(7);
                    se3coords[0] = pathscopedstatese3->getX();
                    se3coords[1] = pathscopedstatese3->getY();
                    se3coords[2] = pathscopedstatese3->getZ();
                    se3coords[3] = pathscopedstatese3->rotation().x;
                    se3coords[4] = pathscopedstatese3->rotation().y;
                    se3coords[5] = pathscopedstatese3->rotation().z;
                    se3coords[6] = pathscopedstatese3->rotation().w;

                    // Create the sample:
                    SE3Conf se3;
                    se3.setCoordinates(se3coords);
                    rcj.setSE3(se3);

                    rob_subspace_index++;
                } else {
                    //If the robot does not have mobile SE3 dofs then the SE3 configuration of the sample is maintained
                    if (smp->getMappedConf().size() == 0) {
                        throw ompl::Exception("omplConstraintPlanner::omplScopedState2smp", "parameter smp must be a sample with the MappedConf");
                    } else {
                        rcj.setSE3(smp->getMappedConf()[rob].getSE3());
                    }
                }

                // Handle Rn (constrained + unconstrained) subspaces
                const unsigned int num_dof = current_rob->getNumJoints();
                if (num_dof > 0) {
                    std::vector<double> coords(num_dof, 0.0);
                    // Create all the possible index (fill with values 0 to num_dof - 1):
                    std::vector<uint> unconstr_indexes(num_dof);
                    std::iota(unconstr_indexes.begin(), unconstr_indexes.end(), 0);

                    // --- Handle constraint groups as in initialization ---
                    
                    // Build intersection groups
                    std::vector<std::shared_ptr<RobotProblemConstraint>> prob_constraints = current_rob->getConstraints();
                    std::map<std::string, std::vector<std::shared_ptr<RobotProblemConstraint>>> intersection_groups;
                    for (const auto& prob_constr : prob_constraints) {
                        std::string group = prob_constr->getIntersectionGroup();
                        if (group.empty()) {
                            group = prob_constr->getConstraintId();
                        }
                        intersection_groups[group].push_back(prob_constr);
                    }

                    // For each constraint group, extract the corresponding subspace:
                    for (const auto& [group_name, group_constraints] : intersection_groups) {
                        ob::StateSpacePtr ssRobotiRnConstr = ssRoboti->as<ob::CompoundStateSpace>()->getSubspace(rob_subspace_index);
                        ob::ScopedState<ob::ProjectedStateSpace> pathscopedstateRnConstr(ssRobotiRnConstr);
                        sstate >> pathscopedstateRnConstr;
                        // pathscopedstateRnConstr.print(std::cout);

                        // Gather all unique joint names and indices in this group:
                        std::vector<std::pair<std::string, uint>> group_constr_joints;
                        std::set<std::string> joint_names_set;
                        for (const auto& constr : group_constraints) {
                            for (const auto& pair : constr->getConstrainedJoints()) {
                                if (joint_names_set.insert(pair.first).second) {
                                    group_constr_joints.push_back(pair);
                                }
                            }
                        }

                        // Fill the coords vector for the constrained joints:
                        size_t q = 0;
                        for (const auto& [joint_name, joint_idx] : group_constr_joints) {
                            coords[joint_idx] = pathscopedstateRnConstr[q];
                            // std::cout << joint_name << "(" << joint_idx << ") = " << pathscopedstateRnConstr[q] << std::endl;
                            // Remove this index from unconstr_indexes:
                            unconstr_indexes.erase(
                                std::remove(unconstr_indexes.begin(), unconstr_indexes.end(), joint_idx),
                                unconstr_indexes.end()
                            );
                            q++;
                        }

                        rob_subspace_index++;
                    }

                    // Handle unconstrained joints:
                    std::vector<std::string> unconstrained_joint_names = current_rob->getUnconstrainedJointNames();
                    if (!unconstrained_joint_names.empty()) {
                        ob::StateSpacePtr ssRobotiRn = ssRoboti->as<ob::CompoundStateSpace>()->getSubspace(rob_subspace_index);
                        ob::ScopedState<ob::RealVectorStateSpace> pathscopedstateRn(ssRobotiRn);
                        sstate >> pathscopedstateRn;
                        // pathscopedstateRn.print(std::cout);

                        for (size_t u = 0; u < unconstrained_joint_names.size(); ++u) {
                            // Find the original index of this joint
                            const std::string& joint_name = unconstrained_joint_names[u];
                            for (unsigned int n = 0; n < num_dof; n++) {
                                if (current_rob->getLink(n+1)->getName() == joint_name) {
                                    coords[n] = pathscopedstateRn[u];
                                    // std::cout << joint_name << "(" << n << ") = " << pathscopedstateRn[u] << std::endl;
                                    break;
                                }
                            }
                        }
                        rob_subspace_index++;
                    }

                    rcj.setRn(coords);

                } else {
                    //If the robot does not have mobile Rn dofs then the Rn configuration of the sample is maintained
                    if (smp->getMappedConf().size() == 0) {
                        throw ompl::Exception("omplConstraintPlanner::omplScopedState2smp", "parameter smp must be a sample with the MappedConf");
                    } else {
                        rcj.setRn(smp->getMappedConf()[rob].getRn());
                    }
                }

                //Load the RobConf with the data of robot i
                rc.push_back(rcj);
            }
            //create the sample with the RobConf
            //the coords (controls) of the sample are kept void
            smp->setMappedConf(rc);
        }

    }
}

#endif // KAUTHAM_USE_OMPL