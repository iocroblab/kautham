#include <pybind11/pybind11.h>
#include <pybind11/stl.h>  // For std::vector, std::string, and other STL containers
#include <pybind11/stl_bind.h>  // For complex STL bindings
#include <pybind11/functional.h> // For binding lambdas and callbacks
#include <sstream>  // For std::istream and std::ostream
#include "kauthamshell_bindings.h"

namespace py = pybind11;

void register_kauthamshell(pybind11::module &m){
     // class kauthamshell
     py::class_<Kautham::kauthamshell>(m, "Kauthamshell")
        .def(py::init<>())
        .def("problemOpened", &Kautham::kauthamshell::problemOpened)
        .def("closeProblem", &Kautham::kauthamshell::closeProblem)
        .def("openProblem", py::overload_cast<std::string, std::vector<std::string>>(&Kautham::kauthamshell::openProblem))

        // Example of handling a method with complex arguments and return types
        .def("checkCollision", [](Kautham::kauthamshell& self, const std::vector<double>& smpcoords) {
            bool collisionFree = false;
            std::string msg;
            std::pair<std::pair<int, std::string>, std::pair<int, int>> colliding_elements;
            self.checkCollision(smpcoords, &collisionFree, &msg, &colliding_elements);
            return py::make_tuple(collisionFree, msg, colliding_elements);
        })
        // --- Robot-related methods ---
        .def("setRobotsConfig", 
             py::overload_cast<std::vector<double>>(&Kautham::kauthamshell::setRobotsConfig),
             "Set the robot configurations using a vector")
        .def("setRobControls", 
             py::overload_cast<std::istream*, std::vector<double>, std::vector<double>>(&Kautham::kauthamshell::setRobControls),
             "Set robot controls using an input stream")
        .def("setRobControls", 
             py::overload_cast<std::string, std::vector<double>, std::vector<double>>(&Kautham::kauthamshell::setRobControls),
             "Set robot controls using a file")
        .def("setRobControlsNoQuery", &Kautham::kauthamshell::setRobControlsNoQuery,
             "Set robot controls without a query")
        .def("setDefaultRobControls", &Kautham::kauthamshell::setDefaultRobControls,
             "Set default robot controls")
        .def("addRobot", &Kautham::kauthamshell::addRobot,
             "Add a robot to the environment")
        .def("removeRobot", &Kautham::kauthamshell::removeRobot,
             "Remove a robot by index")
        .def("setRobPos", &Kautham::kauthamshell::setRobPos,
             "Set a robot's position by index")
        .def("getRobPos", &Kautham::kauthamshell::getRobPos,
             "Get a robot's position by index")
        .def("getRobHomePos", &Kautham::kauthamshell::getRobHomePos,
             "Get the home position of a robot by index")
        .def("getRobotFileNames", &Kautham::kauthamshell::getRobotFileNames,
             "Get the filenames of the robots")
        .def("getRobotJointNames", &Kautham::kauthamshell::getRobotJointNames,
             "Get the joint names of a robot")
        .def("getRobotIsSE3enabled", &Kautham::kauthamshell::getRobotIsSE3enabled,
             "Check if a robot has SE(3) enabled")

        // --- Obstacle-related methods ---
        .def("setObstaclesConfig", &Kautham::kauthamshell::setObstaclesConfig,
             "Set obstacle configurations")
        .def("setInitObs", &Kautham::kauthamshell::setInitObs,
             "Set the initial obstacle configuration")
        .def("addObstacle", &Kautham::kauthamshell::addObstacle,
             "Add an obstacle to the environment")
        .def("removeObstacle", &Kautham::kauthamshell::removeObstacle,
             "Remove an obstacle by name")
        .def("setObstaclePos", &Kautham::kauthamshell::setObstaclePos,
             "Set the position of an obstacle by name")
        .def("getObstaclePos", &Kautham::kauthamshell::getObstaclePos,
             "Get the position of an obstacle by name")
        .def("attachObstacle2RobotLink", &Kautham::kauthamshell::attachObstacle2RobotLink,
             "Attach an obstacle to a robot's link")
        .def("detachObstacle", &Kautham::kauthamshell::detachObstacle,
             "Detach an obstacle from a robot link")
        .def("getObstaclesMap", &Kautham::kauthamshell::getObstaclesMap,
             "Get the map of obstacles")
        .def("getObstaclesNames", &Kautham::kauthamshell::getObstaclesNames,
             "Get the names of the obstacles")

        // --- Planner-related methods ---
        .def("setPlannerByName", &Kautham::kauthamshell::setPlannerByName,
             "Set the planner by name")
        .def("setPlanner", 
             py::overload_cast<std::istream*>(&Kautham::kauthamshell::setPlanner),
             "Set the planner using an input stream")
        .def("setPlanner", 
             py::overload_cast<std::string>(&Kautham::kauthamshell::setPlanner),
             "Set the planner using a problem file")
        .def("setPlannerParameter", &Kautham::kauthamshell::setPlannerParameter,
             "Set a parameter for the planner")
        .def("solve", 
             py::overload_cast<>(&Kautham::kauthamshell::solve),
             "Solve the planning problem")
        .def("solve", 
             py::overload_cast<std::ostream&>(&Kautham::kauthamshell::solve),
             "Solve the planning problem and write data to a file")
        .def("getLastPlanComputationTime", &Kautham::kauthamshell::getLastPlanComputationTime,
             "Get the time for the last plan computation")
        .def("getNumEdges", &Kautham::kauthamshell::getNumEdges,
             "Get the number of edges in the graph")
        .def("getNumVertices", &Kautham::kauthamshell::getNumVertices,
             "Get the number of vertices in the graph")

        // --- Path and trajectory-related methods ---
        .def("getPath", 
             py::overload_cast<std::ostream&>(&Kautham::kauthamshell::getPath),
             "Get the path and write it to an output stream")
        .def("getPath", 
             py::overload_cast<std::vector<std::vector<double>>&, bool>(&Kautham::kauthamshell::getPath),
             "Get the path as a vector of vectors")
        .def("computeTrajecotry", &Kautham::kauthamshell::computeTrajecotry,
             "Compute a trajectory for the robot")
        .def("getTrajecotry", &Kautham::kauthamshell::getTrajecotry,
             "Get the trajectory for the robot")
        .def("setInterpolatePath", &Kautham::kauthamshell::setInterpolatePath,
             "Enable or disable path interpolation")
        .def("getInterpolatePath", &Kautham::kauthamshell::getInterpolatePath,
             "Check if path interpolation is enabled")

        // --- General utility methods ---
        .def("clearSampleSet", &Kautham::kauthamshell::clearSampleSet,
             "Clear the set of samples")
        .def("connect", &Kautham::kauthamshell::connect,
             "Connect two configurations")
        .def("motionPlanner", &Kautham::kauthamshell::motionPlanner,
             "Plan a motion from init to goal")
        .def("cumDistCheck", &Kautham::kauthamshell::cumDistCheck,
             "Check cumulative distance for a configuration")
        .def("getNumRobots", &Kautham::kauthamshell::getNumRobots,
             "Get the number of robots")
        .def("getNumObstacles", &Kautham::kauthamshell::getNumObstacles,
             "Get the number of obstacles");
}

