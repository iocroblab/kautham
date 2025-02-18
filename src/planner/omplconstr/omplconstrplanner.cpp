#include <kautham/planner/omplconstr/omplconstrplanner.hpp>

#if defined(KAUTHAM_USE_OMPL)

#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/ConstrainedSpaceInformation.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <kautham/planner/omplconstr/omplconstrValidityChecker.hpp>

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
                
/*                this->space_ = std::make_shared<ompl::base::CompoundStateSpace>();

                // Create SE3 state space
                std::shared_ptr<ob::StateSpace> spaceSE3;

                // Create Rn state space
                std::shared_ptr<ob::StateSpace> spaceRn;

                // Create a constrained space for Rn space
                std::shared_ptr<ob::StateSpace> spaceRnConstr;

                // Create a robot compound state space [SE3 + Rn]
                std::shared_ptr<ob::StateSpace> spaceRob;

                std::string space_name;
                
                // Loop for all robots:
                for (unsigned rob = 0; rob < _wkSpace->getNumRobots(); rob++) {
                    spaceRob = std::make_shared<ob::CompoundStateSpace>();
                    
                    // Create state space SE3 for the mobile base, if necessary:
                    if (_wkSpace->getRobot(rob)->isSE3Enabled()) {
                        spaceSE3 = std::make_shared<ob::SE3StateSpace>();
                        space_name = "ssRobot" + std::to_string(rob) + "_SE3";
                        spaceSE3->setName(space_name);


                        spaceRob->as<ob::CompoundStateSpace>()->addSubspace(spaceSE3, 1.0);
                    }

                    // Create the Rn state space for the kinematic chain, if necessary:
                    const unsigned int num_dof = _wkSpace->getRobot(rob)->getNumJoints();
                    if (num_dof > 0) {
                        spaceRn = std::make_shared<ob::RealVectorStateSpace>(num_dof);
                        space_name = "ssRobot" + std::to_string(rob) + "_Rn";
                        spaceRn->setName(space_name);

                        // Set the bounds:
                        ob::RealVectorBounds bounds(num_dof);
                        double low, high;
                        for (unsigned int j = 0; j < num_dof; j++) {
                            // The limits of joint j between link j and link (j+1) are stroed in the data structure of link (j+1)
                            low = *_wkSpace->getRobot(rob)->getLink(j+1)->getLimits(true);      // True = getLowLimit
                            high = *_wkSpace->getRobot(rob)->getLink(j+1)->getLimits(false);    // Flase = getHighLimit
                            // filterBounds(low, high, 0.001);
                            bounds.setLow(j, low);
                            bounds.setHigh(j, high);
                        }
                        spaceRn->as<ob::RealVectorStateSpace>()->setBounds(bounds);


                        auto constraint = std::make_shared<OrientationConstraint>(num_dof,3,0.1);

                        spaceRnConstr = std::make_shared<ob::ProjectedStateSpace>(spaceRn, constraint);
                        space_name = "ssRobot" + std::to_string(rob) + "_RnConstr";
                        spaceRnConstr->setName(space_name);

                        spaceRob->as<ob::CompoundStateSpace>()->addSubspace(spaceRnConstr, 1.0);
                    }

                    space_name = "ssRobot" + std::to_string(rob);
                    spaceRob->setName(space_name);
                    this->space_->as<ob::CompoundStateSpace>()->addSubspace(spaceRob, 1.0);

                }

                // Initialize the SimpleSetup with the main state space:
                simple_setup_ = std::make_shared<ompl::geometric::SimpleSetup>(this->space_);

                // Create constrained space information:
                si_ = simple_setup_->getSpaceInformation();

                // Set validity checker:
                auto my_ValidityChecker = std::make_shared<Kautham::omplconstrplanner::ValidityChecker>(si_,  (Planner*)this);

                si_->setStateValidityChecker(ob::StateValidityCheckerPtr(my_ValidityChecker));

                //Add start states
                simple_setup_->clearStartStates();
                for (std::vector<Sample*>::const_iterator start(_init.begin()); start != _init.end(); ++start) {
                    //Start state: convert from smp to scoped state
                    ob::ScopedState<ob::CompoundStateSpace> startompl(this->space_);
                    omplConstraintPlanner::smp2omplScopedState(*start, &startompl);
                    cout << "startomplconstr:" << endl;
                    startompl.print();
                    simple_setup_->addStartState(startompl);
                    // my_constraint->OrientationConstraint::setJointConfigAsTargetOrientation(startompl.reals());
                    // my_constraint->OrientationConstraint::printTargetOrientation();
                }

                //Add goal states
                ob::GoalStates *goalStates(new ob::GoalStates(si_));
                for (std::vector<Sample*>::const_iterator goal(_goal.begin()); goal != _goal.end(); ++goal) {
                    //Goal state: convert from smp to scoped state
                    ob::ScopedState<ob::CompoundStateSpace> goalompl(this->space_);
                    omplConstraintPlanner::smp2omplScopedState(*goal, &goalompl);
                    cout << "goalomplconstr:" << endl;
                    goalompl.print();
                    goalStates->addState(goalompl);
                }
                simple_setup_->setGoal(ob::GoalPtr(goalStates));
*/

                std::cout << "Number of robots: " << _wkSpace->getNumRobots() << std::endl;

                const unsigned int num_dof = 6;

                // Initialize state_space: (ONLY Rn for constrainted planning)
                state_space_ = std::make_shared<ompl::base::RealVectorStateSpace>(num_dof);

                // Set bounds for the state space:
                ompl::base::RealVectorBounds bounds(num_dof);
                bounds.setLow(-2*M_PI);
                bounds.setHigh(2*M_PI);
                state_space_->setBounds(bounds);
                
                // Create an instance of the custom constraint (UR5 hardcoded case):
                auto my_constraint = std::make_shared<OrientationConstraint>(num_dof,3,0.1);
                
                // Initialize the constrained state space:
                constrained_state_space_ = std::make_shared<ompl::base::ProjectedStateSpace>(state_space_, my_constraint);

                // Create constrained space information:
                auto si = std::make_shared<ompl::base::ConstrainedSpaceInformation>(constrained_state_space_);

                auto my_ValidityChecker = std::make_shared<Kautham::omplconstrplanner::ValidityChecker>(si,  (Planner*)this);

                si->setStateValidityChecker(ob::StateValidityCheckerPtr(my_ValidityChecker));

                // Initialize the SimpleSetup with the constrained state space:
                simple_setup_ = std::make_shared<ompl::geometric::SimpleSetup>(si);

                //Add start states
                simple_setup_->clearStartStates();
                for (std::vector<Sample*>::const_iterator start(_init.begin()); start != _init.end(); ++start) {
                    //Start state: convert from smp to scoped state
                    ob::ScopedState<ob::ProjectedStateSpace> startompl(constrained_state_space_);
                    omplConstraintPlanner::smp2omplScopedState(*start, &startompl);
                    cout << "startomplconstr:" << endl;
                    startompl.print();
                    simple_setup_->addStartState(startompl);
                    my_constraint->OrientationConstraint::setJointConfigAsTargetOrientation(startompl.reals());
                    my_constraint->OrientationConstraint::printTargetOrientation();
                }

                //Add goal states
                ob::GoalStates *goalStates(new ob::GoalStates(si));
                for (std::vector<Sample*>::const_iterator goal(_goal.begin()); goal != _goal.end(); ++goal) {
                    //Goal state: convert from smp to scoped state
                    ob::ScopedState<ob::ProjectedStateSpace> goalompl(constrained_state_space_);
                    omplConstraintPlanner::smp2omplScopedState(*goal, &goalompl);
                    cout << "goalomplconstr:" << endl;
                    goalompl.print();
                    goalStates->addState(goalompl);
                }
                simple_setup_->setGoal(ob::GoalPtr(goalStates));



                // Set the planner (for example, RRT)
                auto planner = std::make_shared<ompl::geometric::RRT>(simple_setup_->getSpaceInformation());
                planner->setRange(0.1);
                simple_setup_->setPlanner(planner);


            } else {
                simple_setup_ = (og::SimpleSetupPtr)ssptr;
                si_ = simple_setup_->getSpaceInformation();
                space_ = simple_setup_->getStateSpace();
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
            ompl::base::PlannerStatus solved = simple_setup_->solve(_planningTime);

            if (solved) {
                ompl::geometric::PathGeometric& solution_path = simple_setup_->getSolutionPath();
                
                removeDuplicateStates(solution_path);

                std::vector<ompl::base::State*> states = solution_path.getStates();
                // std::vector<std::vector<double>> joint_path;

                _path.clear();
                clearSimulationPath();
                Sample *smp;
                
                // Load the kautham _path variable from the ompl solution
                // for (auto state : states) {
                for (std::size_t j(0); j < solution_path.getStateCount(); ++j) {
                    // std::vector<double> reals;
                    // constrained_state_space_->copyToReals(reals, states[j]);
                    // joint_path.push_back(reals);
                    // Create a smp and load the RobConf of the init configuration (to have the same if the state does not changi it)
                    smp = new Sample(_wkSpace->getNumRobControls());
                    smp->setMappedConf(_init.at(0)->getMappedConf());

                    //convert form state to smp
                    // omplConstraintPlanner::omplState2smp(solution_path.getState(j)->as<ob::CompoundStateSpace::StateType>(),smp);

                    omplConstraintPlanner::omplState22smp(states[j], smp);

                    _path.push_back(smp);
                    _samples->add(smp);
                }
                _solved = true;

                // std::cout << "Joint path ready to be used in robot_arm as CSV file input:" << std::endl;
                // printJointPath(joint_path);

                return _solved;
            }

            std::cout << "No solution found." << std::endl;

            return false;
        }

        //! This function converts a Kautham sample to an ompl scoped state.
        void omplConstraintPlanner::smp2omplScopedState(Sample* smp, ob::ScopedState<ob::ProjectedStateSpace> *sstate)
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

                //has Rn part
                if(_wkSpace->getRobot(i)->getNumJoints()>0)
                {
                    //get the kautham Rn configuration
                    RnConf r = smpRobotsConf.at(i).getRn();

                    //set the ompl Rn configuration
                    ob::ScopedState<ob::ProjectedStateSpace> rstart(constrained_state_space_);

                    for (unsigned j=0; j<_wkSpace->getRobot(i)->getNumJoints();j++) {
                        rstart[j] = r.getCoordinate(j);
                        // std::cout << "j = " << j << " : " << r.getCoordinate(j) << std::endl;
                    }

                    //load the global scoped state with the info of the Rn data of robot i
                    (*sstate) << rstart;
                }
            }
        }

        //! This function converts a Kautham sample to an ompl scoped state.
        void omplConstraintPlanner::smp2omplScopedState(Sample* smp, ob::ScopedState<ob::CompoundStateSpace> *sstate)
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
                ob::StateSpacePtr ssRoboti = ((ob::StateSpacePtr) space_->as<ob::CompoundStateSpace>()->getSubspace(i));
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
                    ob::ScopedState<ob::ProjectedStateSpace> rstart(ssRobotiRn);

                    std::cout << "TYPE: " << ssRobotiRn->getType() << std::endl;

                    for (unsigned j=0; j<_wkSpace->getRobot(i)->getNumJoints();j++)
                        rstart[j] = r.getCoordinate(j);


                    //load the global scoped state with the info of the Rn data of robot i
                    (*sstate) << rstart;
                    k++;//dummy
                }
            }
        }

        //! This member function converts an ompl::base::State to a Kautham::Sample for the constrainted_state_space
        void omplConstraintPlanner::omplState22smp(const ob::State *state, Sample* smp) {
            vector<RobConf> rc;
            // Loop for all the robots:
            for (unsigned int r = 0; r < _wkSpace->getNumRobots(); ++r) {
                // RobConf to store the robots configurations read from the ompl state:
                RobConf *rcj = new RobConf;

                //Get the SE3 subspace of robot r, if it exists, extract the SE3 configuration
                if (_wkSpace->getRobot(r)->isSE3Enabled()) {


                } else {
                    //If the robot does not have mobile SE3 dofs then the SE3 configuration of the sample is maintained
                    if (smp->getMappedConf().size() == 0) {
                        throw ompl::Exception("omplPlanner::omplScopedState2smp", "parameter smp must be a sample with the MappedConf");
                    } else {
                        rcj->setSE3(smp->getMappedConf()[r].getSE3());
                    }
                }

                //Get the Rn subspace of robot r, if it exisits, and extract the Rn configuration
                if (_wkSpace->getRobot(r)->getNumJoints() > 0) {
                // Convert it to a vector of n components
                    std::vector<double> doubleCoords;  // Create a vector to hold double values
                    constrained_state_space_->copyToReals(doubleCoords, state);

                    vector<KthReal> coords;
                    for (unsigned int j = 0; j < _wkSpace->getRobot(r)->getNumJoints(); ++j){
                        // coords.push_back(pathscopedstateRn->values[j]);
                        coords.push_back(static_cast<KthReal>(doubleCoords[j]));
                    }
                    
                    rcj->setRn(coords);
                } else {
                    //If the robot does not have mobile Rn dofs then the Rn configuration of the sample is maintained
                    if (smp->getMappedConf().size() == 0) {
                        throw ompl::Exception("omplPlanner::omplScopedState2smp", "parameter smp must be a sample with the MappedConf");
                    } else {
                        rcj->setRn(smp->getMappedConf()[r].getRn());
                    }
                }

                //Load the RobConf with the data of robot "r"
                rc.push_back(*rcj);
            }
            // Create the sample with the RobConf
            // The coords (controls) of the sample are kept void
            smp->setMappedConf(rc);
        }

        //! This member function converts an ompl State to a Kautham sample
        void omplConstraintPlanner::omplState2smp(const ob::State *state, Sample* smp)
        {
            ob::ScopedState<ob::CompoundStateSpace> sstate(space_);
            sstate = *state;
            omplConstraintPlanner::omplScopedState2smp( sstate, smp);
        }

        //! This member function converts an ompl ScopedState to a Kautham sample
        void omplConstraintPlanner::omplScopedState2smp(ob::ScopedState<ob::CompoundStateSpace> sstate, Sample* smp)
        {
            vector<RobConf> rc;

            //Loop for all the robots
            for (unsigned int i = 0; i < _wkSpace->getNumRobots(); ++i) {
                //RobConf to store the robots configurations read from the ompl state
                RobConf *rcj = new RobConf;

                //Get the subspace corresponding to robot i
                ob::StateSpacePtr ssRoboti = ((ob::StateSpacePtr)space_->as<ob::CompoundStateSpace>()->getSubspace(i));

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
                    ob::ScopedState<ob::ProjectedStateSpace> pathscopedstateRn(ssRobotiRn);
                    sstate >> pathscopedstateRn;

                    //Convert it to a vector of n components
                    vector<KthReal> coords;
                    for (unsigned int j = 0; j < _wkSpace->getRobot(i)->getNumJoints(); ++j){
                        coords.push_back(pathscopedstateRn[j]);
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