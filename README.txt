
/***************************************************************************
*                                                                          *
*           Institute of Industrial and Control Engineering                *
*                 Technical University of Catalunya                        *
*                        Barcelona, Spain                                  *
*                                                                          *
*                Project Name:       Kautham Planner                       *
*                                                                          *
*     Copyright (C) 2007 - 2010 by Alexander Pérez and Jan Rosell          *
*            alexander.perez@upc.edu and jan.rosell@upc.edu                *
*                                                                          *
*             This is a motion planning tool to be used into               *
*             academic environment and it's provided without               *
*                     any warranty by the authors.                         *
*                                                                          *
*          Alexander Pérez is also with the Escuela Colombiana             *
*          de Ingeniería "Julio Garavito" placed in Bogotá D.C.            *
*             Colombia.  alexander.perez@escuelaing.edu.co                 *
*                                                                          *
***************************************************************************/
/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/
 

The Kautham Planner is an initiative that attempts to be a programming tool 
for the development of robot motion planners for both research and teaching
purposes. It is conceived as a middle point between the need of programming
everything from scratch and the use of the very abstract component-based 
middlewares available in the web (both extremes being very tough for the
newbie programming level that students may have). 

This project has been developed following the guidelines outlined in
the article "A Roadmap to Robot Motion Planning Software Development": 
ability to run on different platforms, code accessability and software
modularity. The first two led to the use of cross-platform and open-source 
tools such as Qt for the user interface, Coin3D for the 3D rendering, PQP 
for the collision detection, and Boost Graph for the graph management. 
Regarding the software modularity, the project is conceived to be library-
based, thus, different libraries have been developed such as a Geometric 
library for the treatment of the bodies and their kinematic relation, a 
Sampling library with different sampling strategies, a Planning library 
essentially composed of sampling-based planners, a Device library for the 
communication with different devices such as sensorized gloves, robot hands 
and arms and haptic devices, and, finally the GUI library that implements 
the user interface and library management.

A doxygen configuration file is provided in the root folder and can be used 
to generate the API documentation. This documentation has been packaged into 
the Kautham2.chm file (a compressed  html help file).  All potential users 
can use these documents to know more closely the system capabilities.

Problem set-up: The Kautham uses two kinds of input XML files, a more 
general one used to describe the problem to be solved (examples/*.xml) that 
includes the robot, the obstacles and (optionally) the planner to be used
and the query to be solved, and another file used to provide the description 
of the robot (examples/robots/*.rob) that includes its mechanical structure
and controls. These files contains relative paths about the obstacles and 
robots used and the user should take care to organize correctly the 
problem's files and the files that they are pointing to. See the 
"examples/IOC_Remote_Cell.xml" and the "examples/robot/example.rob" files 
for a more deep explanation of each type of XML file.

For impatient users, you can start by opening one of the xml files 
included in the "examples" folder: 

a) "table_rooms_R2.xml": example of a free-flying robot in R2.
b) "ele_two_columns_R3.xml": example of a free-flying robot in R3.
c) "ele_two_columns_SE3.xml": example of a free-flying robot in SE3.
c) "Staubli_R3_two_columns.xml": example of an industrial robot with three dof.
d) "Staubli_R6_two_columns.xml": example of an industrial robot with six dof.
e) "Staubli_R6_mobileBase_two_columns.xml": example of an industrial robot with eight dof.
f) "IOC_Remote_Cell.xml": example of two industrial robot with six and seven dof.

Once the file is opened, you can:

1) Move the robot to a desired configuration with the sliders provided 
in the "UsrCtrl-xxx" tab.

2) Add the current configuration of the robot as a sample into the sample 
collection  with the "add current configuration as a Sample" button 
placed in the "Samples" tab, or add any number of samples to the collection
using the samplers provided.

3) Change the planner using the planner toolbar (a basic PRM planner is 
provided and also a void planner called MyPlanner to be used as a template to
implement other planners), or change the local planner. When the problem 
file does not contain a predefined planner, the planner toolbar to chose
one only appears after clicking the blue semi-sphere icon in the main toolbar.

4) Solve a query using the planner tab labeled with the name of the planner, 
PRM in the present case. Since the provided files contain query information,
the init and goal configurations appear already loaded in the planner tab.
You may change them by selecting any of the samples that you may have 
included (see 2). To invoke the planner the  "getPath" button" must be clicked.
If a solution is found you can animate it using the "Start Move" button.


Please let us know what do you do with this robotic planning framework and in 
the case you find it interesting or if you want to cite our work, please use 
the following publication:

@ARTICLE{PerezR:09,
  author = {A. P\'erez and J. Rosell},
  title = {A Roadmap to Robot Motion Planning Software Development},
  journal = {Computer Applications in Engineering Education},
  year = {2009},
  doi = {10.1002/cae.20269}
}


 
