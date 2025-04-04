#include <kautham/planner/omplconstr/omplconstrplanner.hpp>

#if defined(KAUTHAM_USE_OMPL)

#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/ConstrainedSpaceInformation.h>
#include <kautham/planner/omplconstr/omplconstrValidityChecker.hpp>
#include <kautham/planner/omplconstr/constraint_factory.hpp>
#include <kautham/planner/omplconstr/constraints/abstract_ompl_constraint.hpp>

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

            // Add planner parameters:
            addParameter("_Max Planning Time",_planningTime);
            addParameter("_Speed Factor",_speedFactor);
            addParameter("_Range",_range);

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
                        // First assume that all joint are unconstrained:
                        std::vector<std::string> unconstrained_joint_names;
                        std::vector<std::string> all_joint_names;
                        for (unsigned int n = 0; n < num_dof; n++) {    // n = 0 is the base
                            unconstrained_joint_names.push_back(current_rob->getLink(n+1)->getName());
                            all_joint_names.push_back(current_rob->getLink(n+1)->getName());
                        }
                        
                        std::vector<std::shared_ptr<RobotProblemConstraint>> prob_constraints = current_rob->getConstraints();
                        
                        std::vector<std::pair<std::string, uint>> constr_joints;
                        for (auto& prob_constr : prob_constraints) {
                            have_constr_space = true;
                            std::cout << "----- Robot Problem Constraint: " << prob_constr->getConstraintId() << " -----" << std::endl;
                            // Get the constraint data:
                            constr_joints = prob_constr->getConstrainedJoints();
                            std::vector<std::string> constr_names;
                            for (const auto& [name, index] : constr_joints) {
                                constr_names.push_back(name);
                                index_mapping_.push_back(index);
                            }
                                                
                            spaceRn = std::make_shared<ob::RealVectorStateSpace>(constr_joints.size());
                            // Set the bounds:
                            ob::RealVectorBounds bounds(constr_joints.size());
                            double low, high;
                            for (unsigned int j = 0; j < constr_joints.size(); j++) {
                                low = *current_rob->getLink(constr_joints[j].first)->getLimits(true);      // True = getLowLimit
                                high = *current_rob->getLink(constr_joints[j].first)->getLimits(false);    // Flase = getHighLimit
                                // std::cout << "Constr name: " << constr_joints[j].first << " Low: " << low << std::endl;
                                // std::cout << "Constr name: " << constr_joints[j].first << " High: " << high << std::endl;
                                bounds.setLow(j, low);
                                bounds.setHigh(j, high);
                            }
                            spaceRn->as<ob::RealVectorStateSpace>()->setBounds(bounds);

                            // Get the creator of the constraint by string and stores in the map:
                            auto constraint = constraints_factory->createConstraint(prob_constr->getConstraintType(), prob_constr, constr_joints.size(),3,0.1);
                            constraint->associateRobot(current_rob);
                            constraint->assignReferencedEntity(_wkSpace->getObstacle(prob_constr->getReferenceFrameEntity()));
                            this->constraint_map_[prob_constr->getConstraintId()] = constraint;

                            spaceRnConstr = std::make_shared<ob::ProjectedStateSpace>(spaceRn, constraint);
                            space_name = "ssRobot" + std::to_string(rob) + "_RnConstr_" + prob_constr->getConstraintId();
                            spaceRob->as<ob::CompoundStateSpace>()->addSubspace(spaceRnConstr, 1.0);
                            rob_subspace_index++;
                            spaceRob->as<ob::CompoundStateSpace>()->getSubspace(rob_subspace_index-1)->setName(space_name);   // Si no es fa aquí, no s'aplica bé el nom en el cas de les constraints...!

                            // Create SpaceInformation specifically for the ProjectedStateSpace
                            auto spaceRnConstr_si = std::make_shared<ompl::base::SpaceInformation>(spaceRnConstr);
                            // Associate the SpaceInformation object with the ConstrainedStateSpace
                            spaceRnConstr->as<ob::ProjectedStateSpace>()->setSpaceInformation(spaceRnConstr_si.get());

                            // Convert constr_joints to an unordered_set for efficient lookups
                            std::unordered_set<std::string> to_remove(constr_names.begin(), constr_names.end());

                            // Remove elements in unconstrained_joint_names that are present in the to_remove set
                            unconstrained_joint_names.erase(std::remove_if(unconstrained_joint_names.begin(), unconstrained_joint_names.end(), 
                                                            [&to_remove](const std::string& element) {
                                                                return to_remove.count(element) > 0;
                                                            }), unconstrained_joint_names.end());
                        }

                        // Create the Rn space with the unconstrained joints:
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
                            std::cout << "spaceRn name = " << spaceRn->getName() << std::endl;

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

                ompl::base::ScopedState<> start(this->space_);
                ompl::base::ScopedState<> goal(this->space_);

                auto &subspaces_robots = start.getSpace()->as<ompl::base::CompoundStateSpace>()->getSubspaces();
                std::cout << "Num robots: " << subspaces_robots.size() << std::endl;

                // Add start states:
                this->pdef_->clearStartStates();
                for (std::vector<Sample*>::const_iterator start(_init.begin()); start != _init.end(); ++start) {
                    //Start state: convert from smp to scoped state
                    ob::ScopedState<ob::CompoundStateSpace> startompl(this->space_);
                    omplConstraintPlanner::smp2omplScopedState(*start, &startompl);
                    std::cout << "startomplconstr:" << std::endl;
                    startompl.print();
                    this->pdef_->addStartState(startompl);
                    
                    // Assign the start values to each constraint as a target automatically:
                    // omplConstraintPlanner::assignConstrTargetFromState(startompl);
                }

                // Add goal states:
                this->pdef_->clearGoal();
                ob::GoalStates *goalStates(new ob::GoalStates(si_));
                for (std::vector<Sample*>::const_iterator goal(_goal.begin()); goal != _goal.end(); ++goal) {
                    //Goal state: convert from smp to scoped state
                    ob::ScopedState<ob::CompoundStateSpace> goalompl(this->space_);
                    omplConstraintPlanner::smp2omplScopedState(*goal, &goalompl);
                    std::cout << "goalomplconstr:" << std::endl;
                    goalompl.print();
                    goalStates->addState(goalompl);
                    // DEBUG: Assign the goal values to each constraint as a target automatically:
                    // omplConstraintPlanner::assignConstrTargetFromState(goalompl);
                }
                this->pdef_->setGoal(ob::GoalPtr(goalStates));


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

            } catch(...) {
                return false;
            }
            return true;
        }

        bool omplConstraintPlanner::trySolve() {
            // Implement the logic that attempts to solve the planning problem.
            // Return true if successful, false otherwise.
            // Attempt to solve the problem within _planningTime seconds:
            ompl::base::PlannerTerminationCondition ptc = ompl::base::timedPlannerTerminationCondition(_planningTime);
            ompl::base::PlannerStatus solved = this->ompl_planner_->solve(ptc);

            if (solved) {

                std::shared_ptr<ompl::base::Path> solution_path = this->pdef_->getSolutionPath();

                // solution_path->print(std::cout);

                og::PathGeometric& path = static_cast<og::PathGeometric&>(*solution_path);
                og::PathSimplifier simplifier(this->si_);
                // simplifier.simplifyMax(path);
                // simplifier.smoothBSpline(path);

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

        void omplConstraintPlanner::assignConstrTargetFromState(const ob::ScopedState<ob::CompoundStateSpace> _ompl_state) {
            // Assign the constraint target with the state values (e.g.: Orientation target):
            /*
            The key is that from the RealVectorStates in the Space, first are listed the contraints and then the unconstrained.
            The contraint order is the same as the construnction of the space, that is the same of the problem XML file.
                - ToDo: Adapt the code to handle when SE3 is used with more that a single robot!
                - ToDo: Adapt the code to handle different Constraints Types!
            */

            std::vector<double> ompl_state_reals;
            auto current_rob_SE3 = _wkSpace->getRobot(0);
            if (current_rob_SE3->isSE3Enabled()) {
                // Extract the whole vector except the first 7 values of the SE3 (position + orientation):
                const auto& reals = _ompl_state.reals();
                ompl_state_reals.assign(reals.begin() + 7, reals.end());
            } else {
                ompl_state_reals = _ompl_state.reals(); // Only assign when SE3 is not enabled, because is full Rn.
            }

            for (unsigned rob = 0; rob < _wkSpace->getNumRobots(); rob++) {
                auto current_rob = _wkSpace->getRobot(rob);
                auto constraints = current_rob->getConstraints();
                std::vector<std::shared_ptr<RobotProblemConstraint>> prob_constraints = current_rob->getConstraints();

                for (const auto& constraint : prob_constraints) {
                    std::string constr_id_name = constraint->getConstraintId();
                    if (this->constraint_map_.find(constr_id_name) != this->constraint_map_.end()) {
                        size_t constr_joints_size = constraint->getConstrainedJoints().size();
                        std::vector<double> constr_joints(ompl_state_reals.begin(), ompl_state_reals.begin() + constr_joints_size);
                        ompl_state_reals.erase(ompl_state_reals.begin(), ompl_state_reals.begin() + constr_joints_size);

                        std::cout << "Auto constraint (" << constr_id_name << ") config: [ ";
                        for (const auto& value : constr_joints) {
                            std::cout << value << " ";
                        }
                        std::cout << "]" << std::endl;

                        // my_constraint is the final instance, not the abstract:
                        auto my_constraint = this->constraint_map_[constr_id_name];
                        my_constraint->useJointConfig2SetConstraintTarget(constr_joints);
                        my_constraint->printConstraintTarget();

                    }
                }
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
                ob::StateSpacePtr ssRoboti = ((ob::StateSpacePtr) this->space_->as<ob::CompoundStateSpace>()->getSubspace(rob));

                // Create the Scoped State of the current robot:
                ompl::base::ScopedState<> ScopedStateRobot(ssRoboti);

                // If it has se3 part:
                if (current_rob->isSE3Enabled()) {
                    // Get the kautham SE3 configuration:
                    SE3Conf c = smpRobotsConf.at(rob).getSE3();
                    vector<double>& pp = c.getPos();
                    vector<double>& aa = c.getAxisAngle();

                    // Set the OMPL SE3 configuration:
                    ob::StateSpacePtr ssRobotiSE3 =  ((ob::StateSpacePtr) ssRoboti->as<ob::CompoundStateSpace>()->getSubspace(rob_subspace_index));

                    ob::ScopedState<ob::SE3StateSpace> scopedStateSE3(ssRobotiSE3);
                    scopedStateSE3->setX(pp[0]);
                    scopedStateSE3->setY(pp[1]);
                    scopedStateSE3->setZ(pp[2]);
                    scopedStateSE3->rotation().setAxisAngle(aa[0],aa[1],aa[2],aa[3]);

                    // Load the global scoped state with the info of the SE3 data of robot:
                    ScopedStateRobot << scopedStateSE3;
                    rob_subspace_index++;
                }

                // Has Rn part:
                const unsigned int num_dof = current_rob->getNumJoints();
                if (num_dof > 0) {

                    // Get the kautham Rn configuration:
                    RnConf r = smpRobotsConf.at(rob).getRn();

                    // Set the OMPL Rn configuration(s):
                    std::shared_ptr<ompl::base::StateSpace> ssRobotiRn;

                    // First the constraints:
                    std::vector<std::pair<std::string,uint>> constr_joints;
                    std::string joint_name;
                    auto constraints = current_rob->getConstraints();
                    std::vector<std::shared_ptr<RobotProblemConstraint>> prob_constraints = current_rob->getConstraints();

                    for (const auto& constr : prob_constraints) {
                        ssRobotiRn = ssRoboti->as<ob::CompoundStateSpace>()->getSubspace(rob_subspace_index);
                        // std::cout << "Constrained Rn space name: " << ssRobotiRn->getName() << std::endl;
                        rob_subspace_index++;

                        ob::ScopedState<ob::ProjectedStateSpace> scopedStateRnConstr(ssRobotiRn);

                        constr_joints = constr->getConstrainedJoints();
                        // Extract only the names:
                        std::vector<std::string> constr_names;
                        for (const auto& [name, _] : constr_joints) {
                            constr_names.push_back(name);
                        }

                        size_t q = 0;
                        for (unsigned int n = 0; n < num_dof; n++) {
                            joint_name = current_rob->getLink(n+1)->getName();  // n=0 is the base
                            if (std::find(constr_names.begin(), constr_names.end(), joint_name) != constr_names.end()) {
                                // std::cout << "Match with constr " << constr.getConstraintId() << " of " << joint_name << std::endl;
                                scopedStateRnConstr[q] = r.getCoordinate(n);
                                // std::cout << "Joint[" << q << "] = " << r.getCoordinate(n) << std::endl;
                                q++;
                            }
                        }
                        ScopedStateRobot << scopedStateRnConstr;
                    }
                   
                    std::vector<std::string> unconstrained_joint_names = current_rob->getUnconstrainedJointNames();
                    if (unconstrained_joint_names.size() > 0) {
                        ssRobotiRn = ssRoboti->as<ob::CompoundStateSpace>()->getSubspace(rob_subspace_index);
                        // std::cout << "Unconstrained Rn space name: " << ssRobotiRn->getName() << std::endl;
                        rob_subspace_index++;

                        ob::ScopedState<ob::RealVectorStateSpace> scopedStateRn(ssRobotiRn);

                        size_t q = 0;
                        for (unsigned int n = 0; n < num_dof; n++) {
                            joint_name = current_rob->getLink(n+1)->getName();  // n=0 is the base
                            if (std::find(unconstrained_joint_names.begin(), unconstrained_joint_names.end(), joint_name) != unconstrained_joint_names.end()) {
                                // std::cout << "Match with Rn of " << joint_name << std::endl;
                                scopedStateRn[q] = r.getCoordinate(n);
                                // std::cout << "Joint[" << q << "] = " << r.getCoordinate(n) << std::endl;
                                q++;
                            }
                        }
                        ScopedStateRobot << scopedStateRn;
                    }

                    // Load the global scoped state with the info of the Rn data of the current robot:
                    (*sstate) << ScopedStateRobot;
                }
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
            vector<RobConf> rc;

            // Loop for all the robots:
            for (unsigned int rob = 0; rob < _wkSpace->getNumRobots(); ++rob) {

                auto current_rob = _wkSpace->getRobot(rob);

                // RobConf to store the robots configurations read from the ompl state:
                RobConf *rcj = new RobConf;

                // Get the subspace corresponding to robot:
                ob::StateSpacePtr ssRoboti = ((ob::StateSpacePtr)space_->as<ob::CompoundStateSpace>()->getSubspace(rob));

                // Get the SE3 subspace of the current robot, if it exists, extract the SE3 configuration:
                unsigned int rob_subspace_index = 0; // Counter of subspaces of the current robot.
                if (current_rob->isSE3Enabled()) {
                    // Get the SE3 subspace of the curent robot:
                    ob::StateSpacePtr ssRobotiSE3 = ((ob::StateSpacePtr)ssRoboti->as<ob::CompoundStateSpace>()->getSubspace(rob_subspace_index));

                    // Create a SE3 scoped state and load it with the data extracted from the global scoped state:
                    ob::ScopedState<ob::SE3StateSpace> pathscopedstatese3(ssRobotiSE3);
                    sstate >> pathscopedstatese3;

                    // Convert it to a vector of 7 components:
                    std::vector<double> se3coords;
                    se3coords.resize(7);
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
                    rcj->setSE3(se3);

                    rob_subspace_index++;
                } else {
                    //If the robot does not have mobile SE3 dofs then the SE3 configuration of the sample is maintained
                    if (smp->getMappedConf().size() == 0) {
                        throw ompl::Exception("omplConstraintPlanner::omplScopedState2smp", "parameter smp must be a sample with the MappedConf");
                    } else {
                        rcj->setSE3(smp->getMappedConf()[rob].getSE3());
                    }
                }

                // Get the Rn subspaces (unconstrained + constrained) of robot, if it exisits, and extract the Rn configuration:
                const unsigned int num_dof = current_rob->getNumJoints();
                if (num_dof > 0) {
                    vector<double> coords;
                    coords.resize(num_dof);

                    // Create all the possible index (fill with values 0 to num_dof - 1):
                    std::vector<uint> unconstr_indexes(num_dof);
                    std::iota(unconstr_indexes.begin(), unconstr_indexes.end(), 0);

                    // Get the OMPL Rn configuration(s) of the robot:
                    std::shared_ptr<ompl::base::StateSpace> ssRobotiRn;

                    // First the constraints:
                    std::vector<std::pair<std::string,uint>> constr_joints;
                    std::vector<std::shared_ptr<RobotProblemConstraint>> prob_constraints = current_rob->getConstraints();

                    for (const auto& constr : prob_constraints) {
                        ssRobotiRn = ssRoboti->as<ob::CompoundStateSpace>()->getSubspace(rob_subspace_index);
                        // std::cout << "Constrained Rn space name: " << ssRobotiRn->getName() << std::endl;
                        rob_subspace_index++;

                        // Create a RnConstr scoped state and load it with the data extracted from the global scoped state
                        ob::ScopedState<ob::ProjectedStateSpace> pathscopedstateRnConstr(ssRobotiRn);
                        sstate >> pathscopedstateRnConstr;

                        // pathscopedstateRnConstr.print(std::cout);

                        // Convert it to a vector of n-constr components:
                        constr_joints = constr->getConstrainedJoints();
                        size_t q = 0;
                        for (unsigned int n = 0; n < num_dof; n++) {
                            for (const auto& [_, index] : constr_joints) {
                                if (n == index) {
                                    coords[n] = pathscopedstateRnConstr[q];
                                    unconstr_indexes.erase(
                                        std::remove(unconstr_indexes.begin(), unconstr_indexes.end(), index),
                                        unconstr_indexes.end()
                                    );
                                    // std::cout << "n = index = " << n << " = " << pathscopedstateRnConstr[q] << std::endl;
                                    q++;
                                }
                            }
                        }
                    }

                    // Then the remaining unconstrained Rn :
                    if (unconstr_indexes.size() > 0) {
                        ssRobotiRn = ssRoboti->as<ob::CompoundStateSpace>()->getSubspace(rob_subspace_index);
                        // std::cout << "Unconstrained Rn space name: " << ssRobotiRn->getName() << std::endl;
                        rob_subspace_index++;

                        ob::ScopedState<ob::RealVectorStateSpace> pathscopedstateRn(ssRobotiRn);
                        sstate >> pathscopedstateRn;

                        // pathscopedstateRn.print(std::cout);

                        // Convert it to a vector of n-unconstr components:
                        for (unsigned int n = 0; n < num_dof; n++) {
                            for (unsigned int u = 0; u < unconstr_indexes.size(); u++) {
                                if (n == unconstr_indexes[u]) {
                                    coords[n] = pathscopedstateRn[u];
                                    // std::cout << "n = u = " << unconstr_indexes[u] << " = " << pathscopedstateRn[u] << std::endl;
                                }
                            }
                        }
                    }

                    rcj->setRn(coords);

                } else {
                    //If the robot does not have mobile Rn dofs then the Rn configuration of the sample is maintained
                    if (smp->getMappedConf().size() == 0) {
                        throw ompl::Exception("omplConstraintPlanner::omplScopedState2smp", "parameter smp must be a sample with the MappedConf");
                    } else {
                        rcj->setRn(smp->getMappedConf()[rob].getRn());
                    }
                }

                //Load the RobConf with the data of robot i
                rc.push_back(*rcj);
            }
            //create the sample with the RobConf
            //the coords (controls) of the sample are kept void
            smp->setMappedConf(rc);
        }

    }
}

#endif // KAUTHAM_USE_OMPL