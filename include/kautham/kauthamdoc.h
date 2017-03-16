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

/* Author: Jan Rosell */


  
 /*	This file contains the main Doxygen documentation, related to pages and modules
*/


/**
* \mainpage
*
* \section Abstract
*
* The [Kautham Project] (http://sir.upc.edu/kautham) is a software tool developped at the [Service and Industrial Robotics]
* (http://robotics.upc.edu) (SIR) group of the [Institute of Industrial and
* Control Engineering](http://ioc.upc.edu) (IOC) of the [Universitat Politècnica de Catalunya] (
* http://www.upc.edu (UPC), for teaching and research in robot motion planning.
* The tool allows to cope with problems with one or more robots, being a generic
* robot defined as a kinematic tree with a mobile base, i.e. the tool can plan
* and simulate from simple two degrees of freedom free-flying robots to multi-robot
* scenarios with mobile manipulators equipped with anthropomorphic hands.
* The main core of planners is provided by the Open Motion Planning Library ([OMPL](http://ompl.kavrakilab.org)).
* Different basic planners can be flexibly used and parameterized, allowing
* students to gain insight into the different planning algorithms. Among the
* advanced features the tool allows to easily define the coupling between degrees of
* freedom, the dynamic simulation and the integration with task planers. It is
* principally being used in the research of motion planning strategies for dexterous dual arm
* robotic systems.
*
* \section section1 Main Features
* -# Uses [OMPL](http://ompl.kavrakilab.org) suite of planners (geometric and control based)
* -# Uses [PQP](http://gamma.cs.unc.edu/SSV/) or [FCL](http://gamma.cs.unc.edu/FCL/fcl_docs/webpage/generated/index.html) for collision detection
* -# Uses [Coin3D] (http://www.coin3d.org/) for visualization
* -# Uses [ODE](http://www.ode.org/) physics engine
* -# Robot models are defined using [urdf](http://wiki.ros.org/urdf/XML/model) (or [DH](https://en.wikipedia.org/wiki/Denavit%E2%80%93Hartenberg_parameters)
*     parameters coded in an XML file, deprecated)
* -# Describes robots as kinematic trees with a mobile base (SE3xRn configuration space)
* -# Geometry is described in [VRML](https://en.wikipedia.org/wiki/VRML) files (if the [ASSIMP](http://www.assimp.org/) package
*    is available, many other formats like [DAE](https://www.khronos.org/collada/) or [STL](https://en.wikipedia.org/wiki/STL_%28file_format%29) are also supported).
* -# Allows multi-robot systems
* -# Allows the coupling of degrees of freedom
* -# Can be encapsuled as a [ROS](http://www.ros.org/) node

* \section section2 Credits
* Service and Industrial Robotics ([SIR](http://robotics.upc.edu))\n
* Institute of Industrial and Control Engineering ([IOC](http://ioc.upc.edu))\n
* Universitat Polirecnica de Catalunya ([UPC](http://www.upc.edu))\n
* [Barcelona](http://meet.barcelona.cat/en/), Spain\n
*
* Version 1.0 was developed in collaboration with  the [Escuela Colombiana
* de Ingenieria "Julio Garavito"](http://www.escuelaing.edu.co/es/) placed in Bogota D.C.
* Colombia
*
* <b>Contact:</b> Prof. [Jan Rosell](http://ioc.upc.edu/ca/personal/jan.rosell/) (email: <mailto:jan.rosell@upc.edu>)
*
* \section section3 External Documentation
*
* <b>Webpage:</b> <A HREF="http://sir.upc.edu/kautham"> sir.upc.edu/kautham</A>
*
* <b>Paper:</b> [The Kautham Project: A teaching and research tool for robot motion planning (pdf)](https://ioc.upc.edu/ca/personal/jan.rosell/publications/papers/the-kautham-project-a-teaching-and-research-tool-for-robot-motion-planning/@@download/file/PID3287499.pdf)
*
* \section Examples
*       \image html simple_examples.png "Simple examples: 2D free-flying robots"
*       \image html simple_examples2.png "Simple examples: 3D free-flying robot / 2-link manipulator"
*       \image html complex_examples.png "Complex examples: Multiple robots / dual-arm robots / hand-arm systems / virtual bronchoscopes"
*
* \page page1 Basic Features
*  \tableofcontents
*  \section sec11 Problem description
*       Problems are described with XML files that contain four basic main sections:
*       \image html problem_description.png
*
*  \section sec12 Modelling robots and obstacles
*       The robots that can be modelled are kinematic trees with a mobile base (i.e. with  a \f$SE(3)\times R^n\f$ configuration space).
*       They are described using the [urdf](http://wiki.ros.org/urdf/XML/model) format. Obstacles are also modelled as robots (although not actuated, i.e. static).
*
*       As an example, the first lines of the file describing the Allegro mechanical hand are shown:
*       \image html urdf.png
*       The native format to describe the geometry of the rendering models and (optionally) the collision models is [VRML](https://en.wikipedia.org/wiki/VRML).
*       If the [ASSIMP](http://www.assimp.org/) package is available, many others like
*       [DAE](https://www.khronos.org/collada/) or [STL](https://en.wikipedia.org/wiki/STL_%28file_format%29) are also supported.
*
*       More than one robot can be considered in the scene.
*
*  \section sec13 Controls
*       By default one control per each joint defined as moveable is automaticaly set, plus six for the mobile base
*       (provided that the problem definition file includes the translational limits of motion thus indicating that the base is mobile).
*
*       Optionally, the controls are set with an XML file with extension <b>.ctrl</b> defining how the degrees of freedom of the robot are actuated.
*       As an example, a free-flying robot with only two translational degrees of freedom has the following control file:
*       \image html XYZcontrols.png
*       And the control file describing the controls of a TX90 Staubli robot with a mobile base with two degrees of freedom is:
*       \image html TX90-mobilebase.png
*
*       More complex controls can be defined to move simultaneously several d.o.f. in a coupled way, as detailed here: \ref sec21.
*
*  \section sec14 Planners
*       Type of planners:
*           - Potential field-based planners: NF1, HF.
*           - Sampling-based planners (OMPL): PRM, RRT, RRTconnect, RRT*, SBL, EST, KPIECE.
*
*        Any other OMPL planner can be easily incorporated, just by creating a class that wraps the planner and defines the parameters to be tuned by the user
*        (see for instance omplRRTplanner.h and \ref omplRRTplanner.cpp). The GUI is easily adaptable to the new planners from the code:
*       \image html addParameters.png "Setting planner parameters from code for the PRM planner"
*       \image html parametersGUI.png "GUI: Planner parameters for the RRTConnect planner"
*
*  \section sec15 Visualization
*       The workspace can be visualized in two different tabs. The WSpace tab shows the
*       rendering models and the CollisionWSpace tab shows the collision models (oriented
*       bounding boxes can be optionally computed if no collision models are provided).
*
*       The translational trajectories of the end-effectors can also be shown in the WSpace tab.
*       \image html visualizationWorkspace.png "Workspace visualization: visual and collision models"
*       The configuration space can be visualized in the CSpace tab.
*
*       For 2- or 3-dimensional problems it shows the roadmaps, the tree structures or the potential landscapes:
*       \image html visualizationCspace.png "Visualization of 2D Cspaces"
*
*       For higher dimensional problems it shows the projections of the configuration space onto the first two or three degrees of freedom of each robot:
*       \image html visualizationCspace2.png "Visualization of 2D projections of high dimensional Cspaces"
*
* \page page2 Advanced features
*  \tableofcontents
*  \section sec21 Coupling between degrees of freedom
* The configuration space corresponding to <i>m</i> robots is \f${\cal C}   = {\cal C}_1 \times \cdots \times {\cal C}_i \times \cdots \times {\cal C}_m \f$,
* where in the general case \f${\cal C}_i=SE3\times {R}^{n_i}\f$.
* Configurations in \f$\cal C\f$ are represented by vectors of dimension \f$d=\sum_{i=1}^m (6+n_i)\f$,
* with each component normalized in the range \f$[0,1]\f$, i.e.:
*    \f[
*       \hat{q}  = (\hat{q}_{1}, \dots, \hat{q}_{d})^T \;\textrm{with}\;\hat{q}_{i} = \frac{q_{i}-q^{\min}_{i}}{q^{\max}_{i}-q^{\min}_{i}}
*    \f]
* The \f$d\f$ degrees of freedom of the robotic system can be actuated  either separately or in a coupled way.
* To model this, the following expression is used to determine a configuration \f$\hat{q}\f$ using a vector of \f$p\f$ controls, \f$(c_{1}, \dots, c_{p})^T\f$,
* a \f$d\times p\f$ mapping matrix, \f$K\f$, and a  vector of offsets, \f$(o_{1}, \dots, o_{d} )^T\f$:
* \f[
*   \left[ {\begin{array}{c}
*         \hat{q}_{1}\\
*         \vdots\\
*         \hat{q}_{d}
*    \end{array} } \right]  = K
*    \left[ {\begin{array}{c}
*         c_{1}-0.5\\
*         \vdots\\
*         c_{p}-0.5
*    \end{array} } \right]  +
*    \left[ {\begin{array}{c}
*         o_{1}\\
*         \vdots\\
*         o_{d}
*     \end{array} } \right]
* \f]
* Controls and offsets take values in the range \f$[0,1]\f$, and the values \f$\hat{q}_{i}\f$ are forced to lie also in this range.
*
* The mapping matrix defines how the degrees of freedom are actuated:
*   - An identity mapping matrix indicates that  all the degrees of freedom are independently actuated.
*   - A mapping matrix with some zero rows indicates that some degrees of freedom are not actuated, i.e. fixed.
*   - A mapping matrix with columns with several non-zero elements indicates that some degrees of freedom (that may be from the same robot or from different robots) are coupled.
* For instance, the joints of a mechanical hand can be coupled to mimic the synergies that there exist in the human hand motions:
*       \image html PMD1controls.png "Control file describing one of the coupled motions of the SAH hand"
*       \image html PMD1handmotion.png  "Hand motion along the corresponding coupled motion"
*
*  \section sec22 Constrained motion planning
*      For constrained motion planning, the Kautham Project defines a class for each planner and type of robot that includes the kinematic constrained model and the bounds
*      of the valid controls. All this classes derive from a parent class that contains the state space of the robotic system, instantiated from the description of the problem,
*      and procedures to compute the changes of the system state when controls are applied.
*
*      This procedures are encapsulatedin a class derived from the OMPL StatePropagator class and contain an instance of a template class that performs an Euler integration
*      using a generic kinematic model of the robot that is later particularized. Any of the planers included in the OMPL control namespace can be used within The Kautham Project.
*      As an example, the next figure shows a path planning problem of a non-holonomic  robot moving through a narrow passage solved with an control-based RRT planner.
*       \image html constrainedMotionPlanning.png "Motion planning for a car-like robot"
*
*  \section sec23 Dynamic simulation
*       In order to consider dynamic simulation within Kautham, the dynamic information of robots/obstacles must be introduced using the URDF descriptions and a planner from the
*       OMPL control-based suite of planners must be defined in the problem input file. Then, the dynamic environment is created by defining an ODE body for each robot/obstacle link and then
*       generating the OMPL OpenDE state space corresponding to the set of resulting bodies (each body state has 12 dimensions, 3 for position, 3 for orientation, 3 for linear velocity and 3
*       for angular velocity). Planning parameters like world step size are defined in the planer section of the XML input file. The controls computed by the OMPL control-based planners are
*       applied as forces/torques to the bodies. The goal is defined as a goal region and a distance to this goal is defined and used by the planners.
*           \image html urdf_dynamic_info.png
*           \image html ode_planning.png
*  \section sec24 Integration with task planning
*       The ROS middleware is used: Kautham is encapsulated as a motion planning ROS service and a Prolog-based task planner as a ROS client.
*       \image html taskmotionplanning.png "Architecture for simultaneous task and motion planning."
*
*  \section sec25 Benchmarking
*    Kautham provides benchmarking capabilities based on the benchmark utility of OMPL (http://ompl.kavrakilab.org/benchmark.html).
*
*    The benckmarking information is introduced in a XML file and consists in a list of paths to
*       the problem descriptions files and the parameters to configure the benchmarking:
*       \image html benchmarking.png
*   Robots, controls, obstacles and the query to be solved must be the same for all the problems defined in the benchmarking file.
*   Some of the benchmarking parameters that can be set up are the maximum amount of time and memory a planner is allowed to spend in planning, the number of times to run
*   each planner, whether progress is to be displayed or not, a name for the experiment and a file path where results will be saved. Nevertheless, code provides default values
*   for all theparameters.
*
*    The Kautham Console application can be used to benchmark planners:
*       \image html benchmarking_launch.png
*
*   Once the benchmark has been launched, a log file is generated. This files contain information about the settings of the planners, the parameters of the problem tested
*   on, etc. To visualize this information, the user can make use of OMPL scripts to parse the log files and generate a series of plots showing the results for each planner:
*      \image html benchmarking_results.png
*   Or it can be uploaded to [plannerarena.org] (http://plannerarena.org).
*
*   \page page3 The Graphical User Interface
*      \image html getting_started.png
*      \image html getting_started_2.png
*      \image html getting_started_3.png
*
*/

/** \defgroup GridPlanners  Grid Planners
 *  \brief Contains classes that implement potential field planners based on grids.
 *
 *   The Grid Planners module contains classes to partition the configuration space into a grid and to compute
 *   different types of potential functions. The current available planners are:
 *      - %Planner based on the navigation function NF1 [(Latombe, 1991)](http://link.springer.com/book/10.1007%2F978-1-4615-4022-9).
 *      - %Planner based on harmonic functions [(Connolly and Groupen, 1993)] (https://www.researchgate.net/publication/227763642_The_application_of_harmonic_functions_to_robotics).
 *
 */

/** \defgroup ControlPlanners Control-based Planners
 *  \brief Contains classes that implement sampling-based planners based on controls
 *
 *   The  Control-based Planners module contains planners for systems subject to differential constraints, and that
 *  rely on state propagation rather than simple interpolation to generate motions. They are based on the
 *  [control-based planners provided by the OMPL library] (http://ompl.kavrakilab.org/planners.html#control_planners).
 *
 *   \todo list of control planners provided
 */

/** \defgroup GeometricPlanners Geometric Planners
 *  \brief Contains classes that implement sampling-based geometric planners.
 *
 *   The  Geometric Planners module contains planners that only accounts for the geometric and kinematic constraints of the system.
 *   and are based on the [geometric planners provided by the OMPL library] (http://ompl.kavrakilab.org/planners.html#geometric_planners).
 *
 *   Some classes are simple wrappers of the OMPL class planners. Any planner provided by OMPL can be incorporated by writting
 *   the wrapping class that calls it and that sets the parameters to be tuned by the user (see the template class omplNEWplanner).
 *
 *   Other classes are minor variants of the OMPL class planners.
 *
 *   Some other classes implement more advanced planners based on synergies (coupled motions) with the aim to plan motions for
 *   for dual-arm robotic systems with human-like appearance.
 *
 *   The following planners from OMPL are currently available:
 *      - RTT
 *      - RRTConnect
 *      - SBL
 *      - EST
 *
 *   The following variants of OMPL have been incorporated by deriving from the corresponding ompl classes:
 *      - PRM: it includes the possibility tochange the ratio between explore and expand steps of the construction phase of the roadmap.
 *      - RRT*: it allows to optimize for different functions, and to filter nodes and to bias towards the first solution in order to improve the efficient in high dimensions.
 *      - KPIECE: it allows motoions composed of severla steps limits the growing towards the goal to a fixed value; it also disallows moving backwards.
 *      - TRRT:
 *      - lazyTRRT:
 *
 *   The following planners are also included:
 *      - FOSRRT
 *      - FOSKPIECE
 *      - FOSBKPIECE
 *      - FOSLBKPIECE
 *
 *
 *   \todo Add detailed description of FOS planners
 */

/** \defgroup group4 Physics-based Planners
 *  \brief Contains classes that implement sampling-based planners based on dynamic simulation
 *
 *   \todo Add detailed description of Physics-based Planners module
 *
 */

/** \defgroup Problem  Problem  Definition
 *  \brief Contains classes to define the problem setup: robots and obstacles.
 *
 *   \todo Add detailed description of Problem Definition module
 *
 */

/** \defgroup Sampling Sampling module
 *  \brief Contains classes to sample the configuration space
 *
 *   \todo Add detailed description of Sampling module
 *
 */

/** \defgroup  IK Inverse Kinematics
 *  \brief Contains classes to solve the inverse kinematics of some robots
 *
 *   \todo Add detailed description of Kinematics module
 *
 */
