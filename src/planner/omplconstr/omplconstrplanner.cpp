#include <kautham/planner/omplconstr/omplconstrplanner.hpp>

#if defined(KAUTHAM_USE_OMPL)

#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/ConstrainedSpaceInformation.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <kautham/planner/omplconstr/omplconstrValidityChecker.hpp>

#include <ompl/base/Path.h>

#include <ompl/base/goals/GoalStates.h>
#include <ompl/base/goals/GoalState.h>

// Include all the constraints:
#include <kautham/planner/omplconstr/constraints/orientation_constr.hpp>


namespace Kautham {
    //! Namespace omplconstrplanner contains the planners based on the OMPL::constraint library
    namespace omplconstrplanner{

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

            // Add planner parameters:
            addParameter("_Max Planning Time",_planningTime);
            addParameter("_Speed Factor",_speedFactor);

            if (ssptr == NULL) {

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
                        // filterBounds(low, high, 0.001);
                        bounds.setLow(0, low);
                        bounds.setHigh(0, high);

                        //y-direction
                        low = current_rob->getLimits(1)[0];
                        high = current_rob->getLimits(1)[1];
                        // filterBounds(low, high, 0.001);
                        bounds.setLow(1, low);
                        bounds.setHigh(1, high);

                        //z-direction
                        low = current_rob->getLimits(2)[0];
                        high = current_rob->getLimits(2)[1];
                        // filterBounds(low, high, 0.001);
                        bounds.setLow(2, low);
                        bounds.setHigh(2, high);

                        spaceSE3->as<ob::SE3StateSpace>()->setBounds(bounds);



                        // spaceRob->as<ob::CompoundStateSpace>()->addSubspace(spaceSE3, 1.0);
                        // rob_subspace_index++;
                    }

                    // Create the Rn state space for the kinematic chain, if necessary:
                    // Also create the constrained space if requested by the robot.
                    const unsigned int num_dof = current_rob->getNumJoints();
                    if (num_dof > 0) {
                        // First assume that all joint are unconstrained:
                        std::vector<std::string> unconstrained_joint_names;
                        for (unsigned int n = 0; n < num_dof; n++) {    // n = 0 is the base
                            unconstrained_joint_names.push_back(current_rob->getLink(n+1)->getName());
                        }
                        
                        auto constraints = current_rob->getConstraints();

                        std::vector<std::pair<std::string, uint>> constr_joints;
                        for (const auto& constr : constraints) {
                            std::cout << "----- Constraint: " << std::get<0>(constr) << " -----" << std::endl;
                            // Get the constraint data:
                            constr_joints = std::get<2>(constr);
                            // Extract only the names:
                            std::vector<std::string> constr_names;
                            for (const auto& [name, _] : constr_joints) {
                                constr_names.push_back(name);
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

                            auto constraint = std::make_shared<OrientationConstraint>(constr_joints.size(),3,0.1);
                            this->constraint_map_[std::get<0>(constr)] = constraint;

                            spaceRnConstr = std::make_shared<ob::ProjectedStateSpace>(spaceRn, constraint);
                            space_name = "ssRobot" + std::to_string(rob) + "_RnConstr_" + std::get<0>(constr);
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
                            current_rob->setUnconstrainedJointNames(unconstrained_joint_names);
                            auto katxopo = current_rob->getUnconstrainedJointNames();

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
                for (std::vector<Sample*>::const_iterator start(_init.begin()); start != _init.end(); ++start) {
                    //Start state: convert from smp to scoped state
                    ob::ScopedState<ob::CompoundStateSpace> startompl(this->space_);
                    omplConstraintPlanner::smp2omplScopedState(*start, &startompl);
                    std::cout << "startomplconstr:" << std::endl;
                    startompl.print();
                    this->pdef_->addStartState(startompl);
                    // S'HA DE FER GENERIC:
                    std::string constr_id_name = "arm_ur5";
                    if (this->constraint_map_.find(constr_id_name) != this->constraint_map_.end()) {
                        auto my_constraint = std::dynamic_pointer_cast<OrientationConstraint>(this->constraint_map_[constr_id_name]);
                        my_constraint->OrientationConstraint::setJointConfigAsTargetOrientation(startompl.reals());
                        my_constraint->OrientationConstraint::printTargetOrientation();
                    }
                }

                // Add goal states:
                ob::GoalStates *goalStates(new ob::GoalStates(si_));
                for (std::vector<Sample*>::const_iterator goal(_goal.begin()); goal != _goal.end(); ++goal) {
                    //Goal state: convert from smp to scoped state
                    ob::ScopedState<ob::CompoundStateSpace> goalompl(this->space_);
                    omplConstraintPlanner::smp2omplScopedState(*goal, &goalompl);
                    std::cout << "goalomplconstr:" << std::endl;
                    goalompl.print();
                    goalStates->addState(goalompl);
                    // auto debug = std::make_shared<OrientationConstraint>(6,3,0.1);
                    // debug->OrientationConstraint::setJointConfigAsTargetOrientation(goalompl.reals());
                    // debug->OrientationConstraint::printTargetOrientation();
                }
                this->pdef_->setGoal(ob::GoalPtr(goalStates));

                // Set the planner (for example, RRT)
                this->ompl_planner_ = std::make_shared<ompl::geometric::RRT>(this->si_);
                this->ompl_planner_->setProblemDefinition(this->pdef_);
                this->ompl_planner_->as<ompl::geometric::RRT>()->setRange(0.1);
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

                std::vector<ompl::base::State*> states = solution_path->as<og::PathGeometric>()->getStates();

                _path.clear();
                clearSimulationPath();
                Sample *smp;
                
                // Load the kautham _path variable from the ompl solution
                for (std::size_t j(0); j < solution_path->as<og::PathGeometric>()->getStateCount(); ++j) {
                    // Create a smp and load the RobConf of the init configuration (to have the same if the state does not changi it)
                    smp = new Sample(_wkSpace->getNumRobControls());
                    smp->setMappedConf(_init.at(0)->getMappedConf());

                    omplConstraintPlanner::omplState2smp(solution_path->as<og::PathGeometric>()->getState(j)->as<ob::CompoundStateSpace::StateType>(),smp);

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

        //! This function converts a Kautham sample to an ompl scoped state.
        void omplConstraintPlanner::smp2omplScopedState(Sample* smp, ob::ScopedState<ob::CompoundStateSpace> *sstate)
        {
            // Extract the mapped configuration of the sample. It is a vector with as many components as robots.
            // Each component has the RobConf of the robot (the SE3 and the Rn configurations including constraints)
            if(smp->getMappedConf().size()==0)
            {
                _wkSpace->moveRobotsTo(smp); // to set the mapped configuration
            }
            std::vector<RobConf>& smpRobotsConf = smp->getMappedConf();

            // Loop for all the robots:
            for (unsigned rob = 0; rob < _wkSpace->getNumRobots(); rob++)
            {
                auto current_rob = _wkSpace->getRobot(rob);
                size_t rob_subspace_index = 0;

                // Get the subspace of robot:
                ob::StateSpacePtr ssRoboti = ((ob::StateSpacePtr) this->space_->as<ob::CompoundStateSpace>()->getSubspace(rob));
                std::string ssRobotiname = ssRoboti->getName();

                // Create the Scoped State of the current robot:
                ompl::base::ScopedState<> ScopedStateRobot(ssRoboti);

                size_t num_subspaces_rn = ssRoboti->as<ompl::base::CompoundStateSpace>()->getSubspaces().size();
                std::cout << "Number of subspaces in Rn: " << num_subspaces_rn << std::endl;

                // If it has se3 part:
                // if (current_rob->isSE3Enabled()) {
                //     //get the kautham SE3 configuration
                //     SE3Conf c = smpRobotsConf.at(rob).getSE3();
                //     vector<KthReal>& pp = c.getPos();
                //     vector<KthReal>& aa = c.getAxisAngle();

                //     //set the ompl SE3 configuration
                //     ob::StateSpacePtr ssRobotiSE3 =  ((ob::StateSpacePtr) ssRoboti->as<ob::CompoundStateSpace>()->getSubspace(rob_subspace_index));
                //     string ssRobotiSE3name = ssRobotiSE3->getName();

                //     ob::ScopedState<ob::SE3StateSpace> cstart(ssRobotiSE3);
                //     cstart->setX(pp[0]);
                //     cstart->setY(pp[1]);
                //     cstart->setZ(pp[2]);
                //     cstart->rotation().setAxisAngle(aa[0],aa[1],aa[2],aa[3]);

                //     //load the global scoped state with the info of the se3 data of robot
                //     (*sstate)<<cstart;
                //     rob_subspace_index++;
                //     num_subspaces_rn--;  // If SE3 is enabled discount a subspace.
                // }

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

                    for (const auto& constr : constraints) {
                        ssRobotiRn = ssRoboti->as<ob::CompoundStateSpace>()->getSubspace(rob_subspace_index);
                        // std::cout << "Constrained Rn space name: " << ssRobotiRn->getName() << std::endl;
                        rob_subspace_index++;

                        ob::ScopedState<ob::ProjectedStateSpace> scopedStateRnConstr(ssRobotiRn);

                        constr_joints = std::get<2>(constr);
                        // Extract only the names:
                        std::vector<std::string> constr_names;
                        for (const auto& [name, _] : constr_joints) {
                            constr_names.push_back(name);
                        }

                        size_t q = 0;
                        for (unsigned int n = 0; n < num_dof; n++) {
                            joint_name = current_rob->getLink(n+1)->getName();  // n=0 is the base
                            if (std::find(constr_names.begin(), constr_names.end(), joint_name) != constr_names.end()) {
                                // std::cout << "Match with constr " << std::get<0>(constr) << " of " << joint_name << std::endl;
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

                // Get the SE3 subspace of robot i, if it exists, extract the SE3 configuration
                unsigned int rob_subspace_index = 0; //counter of subspaces of robot i
                // if (current_rob->isSE3Enabled()) {
                if (0) {
                    //Get the SE3 subspace of robot i
                    ob::StateSpacePtr ssRobotiSE3 = ((ob::StateSpacePtr)ssRoboti->as<ob::CompoundStateSpace>()->getSubspace(rob_subspace_index));

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

                    rob_subspace_index++;
                } else {
                    //If the robot does not have mobile SE3 dofs then the SE3 configuration of the sample is maintained
                    if (smp->getMappedConf().size() == 0) {
                        throw ompl::Exception("omplPlanner::omplScopedState2smp", "parameter smp must be a sample with the MappedConf");
                    } else {
                        rcj->setSE3(smp->getMappedConf()[rob].getSE3());
                    }
                }

                // Get the Rn subspaces (unconstrained + constrained) of robot, if it exisits, and extract the Rn configuration:
                const unsigned int num_dof = current_rob->getNumJoints();
                if (num_dof > 0) {
                    vector<KthReal> coords;
                    coords.resize(num_dof);

                    // Create all the possible index (fill with values 0 to num_dof - 1):
                    std::vector<uint> unconstr_indexes(num_dof);
                    std::iota(unconstr_indexes.begin(), unconstr_indexes.end(), 0);

                    // Get the OMPL Rn configuration(s) of the robot:
                    std::shared_ptr<ompl::base::StateSpace> ssRobotiRn;

                    // First the constraints:
                    std::vector<std::pair<std::string,uint>> constr_joints;
                    auto constraints = current_rob->getConstraints();

                    for (const auto& constr : constraints) {
                        ssRobotiRn = ssRoboti->as<ob::CompoundStateSpace>()->getSubspace(rob_subspace_index);
                        // std::cout << "Constrained Rn space name: " << ssRobotiRn->getName() << std::endl;
                        rob_subspace_index++;

                        // Create a RnConstr scoped state and load it with the data extracted from the global scoped state
                        ob::ScopedState<ob::ProjectedStateSpace> pathscopedstateRnConstr(ssRobotiRn);
                        sstate >> pathscopedstateRnConstr;

                        // pathscopedstateRnConstr.print(std::cout);

                        // Convert it to a vector of n-constr components:
                        constr_joints = std::get<2>(constr);
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
                        throw ompl::Exception("omplPlanner::omplScopedState2smp", "parameter smp must be a sample with the MappedConf");
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


        void omplConstraintPlanner::removeDuplicateStates(ompl::geometric::PathGeometric& path) {
            std::vector<ompl::base::State*> states = path.getStates();
            std::vector<ompl::base::State*> uniqueStates;
            
            for (size_t i = 0; i < states.size(); ++i) {
                if (i == 0 || !path.getSpaceInformation()->equalStates(states[i], states[i-1])) {
                    uniqueStates.push_back(states[i]);
                }
            }
            
            path.getStates().swap(uniqueStates);
        }

        void omplConstraintPlanner::printJointPath(const std::vector<std::vector<double>>& _joint_path) {
            std::cout << std::fixed << std::setprecision(5);
            for (auto joint_waypoint : _joint_path) {
                for (size_t i = 0; i < joint_waypoint.size(); ++i) {
                    std::cout << joint_waypoint[i];
                    if (i < joint_waypoint.size() - 1) std::cout << ", ";
                }
                std::cout << std::endl;
            }
            std::cout << std::endl;
        }

    }
}

#endif // KAUTHAM_USE_OMPL