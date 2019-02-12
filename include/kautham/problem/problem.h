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


#if !defined(_PROBLEM_H)
#define _PROBLEM_H


#include <string>
#include <fstream>
#include <iostream>

#include <boost/algorithm/string.hpp>

//solving convertions problems
#include <locale.h>

#include <pugixml.hpp>

#include <ompl/geometric/SimpleSetup.h>

#include <kautham/planner/planner.h>
#include <kautham/sampling/sampling.h>
#include <kautham/problem/robot.h>
#include <kautham/problem/ivworkspace.h>
#include <kautham/problem/workspace.h>
#include <kautham/sampling/state.h>

#if defined(KAUTHAM_USE_IOC)
#include <kautham/planner/ioc/mygridplanner.h>
#include <kautham/planner/ioc/NF1planner.h>
#include <kautham/planner/ioc/HFplanner.h>
#endif

#if defined(KAUTHAM_USE_OMPL)
#include <kautham/planner/omplg/omplPRMplanner.h>
#include <kautham/planner/omplg/omplRRTplanner.h>
#include <kautham/planner/omplg/omplLazyRRTplanner.h>
#include <kautham/planner/omplg/omplpRRTplanner.h>
#include <kautham/planner/omplg/omplRRTConnectplanner.h>
#include <kautham/planner/omplg/omplRRTStarplanner.h>
#include <kautham/planner/omplg/omplTRRTplanner.h>
#include <kautham/planner/omplg/omplTRRTConnectplanner.h>
#include <kautham/planner/omplg/omplLazyTRRTplanner.h>
#include <kautham/planner/omplg/omplRRTConnectplannerEUROC.h>
#include <kautham/planner/omplg/omplFOSVFRRTplanner.h>
#include <kautham/planner/omplg/omplMyFOSVFRRTplanner.h>
#include <kautham/planner/omplg/omplFOSKPIECE1planner.h>
#include <kautham/planner/omplg/omplFOSBKPIECE1planner.h>
#include <kautham/planner/omplg/omplFOSLBKPIECE1planner.h>
#include <kautham/planner/omplg/omplFOSRRTStarplanner.h>
#include <kautham/planner/omplg/omplFOSRRTplanner.h>
#include <kautham/planner/omplg/omplFOSRRTConnectplanner.h>
#include <kautham/planner/omplg/omplFOSTRRTplanner.h>
#include <kautham/planner/omplg/omplFOSTRRTConnectplanner.h>
#include <kautham/planner/omplg/omplProjESTplanner.h>
#include <kautham/planner/omplg/omplSBLplanner.h>
#include <kautham/planner/omplg/omplKPIECEplanner.h>
#include <kautham/planner/omplc/omplcRRTcarplanner.h>
#include <kautham/planner/omplc/omplcRRTf16planner.h>
#include <kautham/planner/omplc/omplcRRTdualdriveplanner.h>
#include <kautham/planner/omplc/omplcSSTcarplanner.h>
#include <kautham/planner/omplc/omplcSSTf16planner.h>
#include <kautham/planner/omplc/omplcSSTdualdriveplanner.h>
#endif

#if defined(KAUTHAM_USE_ODE)
#include <kautham/planner/omplOpenDE/PhysicsBasedPlanners/KPIECE2DPlanner.h>
#include <kautham/planner/omplOpenDE/PhysicsBasedPlanners/KPIECECarPlanner.h>
#include <kautham/planner/omplOpenDE/PhysicsBasedPlanners/KPIECEChainPlanner.h>
#include <kautham/planner/omplOpenDE/PhysicsBasedPlanners/RRT2DPlanner.h>
#include <kautham/planner/omplOpenDE/PhysicsBasedPlanners/Syclop2DPlanner.h>
#include <kautham/planner/omplOpenDE/PhysicsBasedPlanners/KnowledgeOrientedKPIECE2DPlanner.h>
#include <kautham/planner/omplOpenDE/PhysicsBasedPlanners/KnowledgeOrientedRRT2DPlanner.h>
#include <kautham/planner/omplOpenDE/PhysicsBasedPlanners/KnowledgeOrientedSyclop2DPlanner.h>
//#include <planner/omplOpenDE/PhysicsBasedPlanners/KauthamOpenDELTLPlanner.h>
#endif

#if !defined(M_PI)
#define M_PI 3.1415926535897932384626433832795
#endif


using namespace std;
using namespace pugi;


namespace Kautham {
    /** \addtogroup Problem
     *  @{
    */
    class Problem {
    public:
        Problem();
        ~Problem();

        //!	This member function create the work phisycal space.
        /*!	With this fuction you can to create the work phisycal space that represent
         * the problem. One or more Robot and one or more obstacle  compose it.
         * These Robots could be both the freefly type or cinematic chain type, it
         * mean that if a problem contain many robots all of them should be the
         * same class.
         * \sa WSpace Robot ChainRobot Obstacle
         */
        bool 	               createWSpaceFromFile(pugi::xml_document *doc, bool useBBOX, progress_struct *progress = NULL);

        //! This method is deprecated. Please take care with the problem XML file.
        //bool			              createWSpace(ProbStruc *reader);

        bool                    createPlanner(string name, string synergyTreeFilename = "");
        bool                    createPlannerFromFile(istream *xml_inputfile);
        bool                    createPlannerFromFile(string problemFile);
        bool                    createPlannerFromFile(pugi::xml_document *doc);

        bool                    createPlanner(string name, ompl::geometric::SimpleSetup *ssptr, string synergyTreeFilename = "");
        bool                    createPlannerFromFile(istream *xml_inputfile, ompl::geometric::SimpleSetup *ssptr);
        bool                    createPlannerFromFile(string problemFile, ompl::geometric::SimpleSetup *ssptr);
        bool                    createPlannerFromFile(pugi::xml_document *doc, ompl::geometric::SimpleSetup *ssptr);

        bool                    createPlanner(string name, ompl::control::SimpleSetup *ssptr);
        bool                    createPlannerFromFile(istream *xml_inputfile, ompl::control::SimpleSetup *ssptr);
        bool                    createPlannerFromFile(string problemFile, ompl::control::SimpleSetup *ssptr);
        bool                    createPlannerFromFile(pugi::xml_document *doc, ompl::control::SimpleSetup *ssptr);

        bool                    createCSpace();
        bool                    createCSpaceFromFile(pugi::xml_document *doc);
        bool                    tryToSolve();
        bool                    setCurrentRobControls(vector<KthReal> &val);
        bool                    setCurrentObsControls(vector<KthReal> &val);
        //! Returns WSpace
        WorkSpace*		        wSpace();
        //! Returns CSpace
        SampleSet*              cSpace();
        //! Sets WSpace
        inline void             setWSpace(WorkSpace* WSpace) {
            _wspace = WSpace;
        }

        //! Sets CSpace
        inline void             setCSpace(SampleSet* CSpace) {
            _cspace = CSpace;
        }

        void                    setHomeConf(Robot* rob, HASH_S_K* param);
        void                    resetPlanner(){delete(_planner);_planner=NULL;}
        void                    setPlanner(Planner* plan){if(_planner==NULL)_planner = plan;}
        inline Planner*         getPlanner(){return _planner;}
        inline SampleSet*       getSampleSet(){return _cspace;}
        inline Sampler*         getSampler(){return _sampler;}
        inline void             setSampler(Sampler* smp){_sampler = smp;}
        inline int              getDimension(){return _wspace->getNumRobControls();}
        inline vector<KthReal>& getCurrentRobControls(){return _currentRobControls;}
        inline vector<KthReal>& getCurrentObsControls(){return _currentObsControls;}
        inline string           getFilePath(){return _filePath;}
        bool                    inheritSolution();
        bool                    setupFromFile(string xml_doc, vector <string> def_path = vector <string>(), bool useBBOX = false);
        bool                    setupFromFile(istream *xml_inputfile, vector <string> def_path = vector <string>(), bool useBBOX = false);
        bool                    setupFromFile(pugi::xml_document *doc, bool useBBOX, progress_struct *progress = NULL);
        bool addRobot2WSpace(string robFile, KthReal scale, vector<KthReal> home,vector< vector<KthReal> > limits);
        bool addObstacle2WSpace(string robFile, KthReal scale, vector<KthReal> home);


        //! This method saves the information of the problem's planner .
        //! This method checks if the file_path file exists or not. In
        //! case the file doesn't exist, the method copies the current
        //! problem file and adds the planner's attributes.
        bool                    saveToFile(string file_path = "");

        /*!
     * \brief loads the robot controls file,
     creates the controls, adds them to the workspace and creates the mapMatrix and
     offMatrix of every robot. All the robots must have already been loaded.
     * \param inputfile stream where controls are defined
     * \return true if controls could be loaded to the workspace
     */
        bool setRobotControls(istream *inputfile);

        /*!
     * \brief loads the robot controls file,
     creates the controls, adds them to the workspace and creates the mapMatrix and
     offMatrix of every robot. All the robots must have already been loaded.
     * \param cntrFile file where controls are defined
     * \return true if controls could be loaded to the workspace
     */
        bool setRobotControls(string cntrFile);

        /*!
     * \brief loads the robot controls file,
     creates the controls, adds them to the workspace and creates the mapMatrix and
     offMatrix of every robot. All the robots must have already been loaded.
     * \param doc document where controls are defined
     * \return true if controls could be loaded to the workspace
     */
        bool setRobotControls(xml_document *doc);

        /*!
     * \brief creates the default controls, adds them to the workspace and creates the mapMatrix and
     offMatrix of every robot. All the robots must have already been loaded.
     * \return true if controls could be loaded to the workspace
     */
        bool setDefaultRobotControls();

        /*!
     * \brief loads the obstacle controls file of the problem file,
     creates the controls, adds them to the workspace and creates the mapMatrix and
     offMatrix of every osbtacle. All the obstacles must have already been loaded.
     * \param inputfile stream where controls are defined
     * \return true if controls could be loaded to the workspace
     */
        bool setObstacleControls(istream *inputfile);

        /*!
     * \brief loads the obstacle controls file of the problem file,
     creates the controls, adds them to the workspace and creates the mapMatrix and
     offMatrix of every osbtacle. All the obstacles must have already been loaded.
     * \param cntrFile file where controls are defined
     * \return true if controls could be loaded to the workspace
     */
        bool setObstacleControls(string cntrFile);

        /*!
     * \brief loads the obstacle controls file of the problem file,
     creates the controls, adds them to the workspace and creates the mapMatrix and
     offMatrix of every osbtacle. All the obstacles must have already been loaded.
     * \param doc document where controls are defined
     * \return true if controls could be loaded to the workspace
     */
        bool setObstacleControls(xml_document *doc);

        /*!
     * \brief creates fixed controls, adds them to the workspace and creates the mapMatrix and
     offMatrix of every osbtacle. All the obstacles must have already been loaded.
     * \return true if controls could be loaded to the workspace
     */
        bool setFixedObstacleControls();


        //! Parses a problem file
        xml_document *parseProblemFile(string filename, vector <string> def_path, int *links2Load);

    private:
        WorkSpace*              _wspace;
        SampleSet*              _cspace;
        vector<State>           _sspace;
        CONFIGTYPE              _problemType;
        Sampler*                _sampler;
        Planner*                _planner;
        vector<KthReal>         _currentRobControls;
        vector<KthReal>         _currentObsControls;
        string                  _filePath;


        /*!
     * \brief loads a robot node of the problem file,
     creates the robot and adds it to the workspace
     * \param robot_node robot node with the information of the robot
     to add to the workspace
     */
        bool addRobot2WSpace(xml_node *robot_node, bool useBBOX, progress_struct *progress = NULL);

        /*!
     * \brief loads an obstacle node of the problem file,
     creates the obstacle and adds it to the workspace
     * \param obstacle_node obstacle node with the information of the obstacle
     to add to the workspace
     */
        bool addObstacle2WSpace(xml_node *obstacle_node, bool useBBOX, progress_struct *progress = NULL);

        /*!
     * \brief isFileOK checks if all the information required
     to setup a problem is defined in the dile
     * \param doc problem file correctly parsed
     * \return true if the file seems to be OK
     */
        bool isFileOK (xml_document *doc);

        /*!
     * \brief exists checks if a spcified file exists
     * \param file is the file which existence is to be checked
     * \return true if the file exists
     */
        bool exists(string file);

        /*!
     * \brief checks the file, finds the files defined in the problem file and
     completes their absolute path
     * \param doc problem file to prepare before setting up the porblem
     * \param def_path default path list where files will be looked for
     * \return true if the file could be prepared
     */
        bool prepareFile (xml_document *doc, vector<string> def_path);

        /*!
     * \brief finds all the files defined in the atribute called \param attribute
     of the nodes called \param child that children of the \param parent node.
     Files will be looked for in the specified paths. If file is found, its absolute
     path will be completed
     * \param parent node that contains the files to be found
     * \param child name of the children nodes that contains the files to be found
     * \param attribute name of the atribute that contains the file to be found
     * \param path vector of paths where the file will be recursively looked for until
     the file is found
     * \return true if and only if all the files were found
     */
        bool findAllFiles(xml_node *parent, string child, string attribute,
                          vector<string> path);

        //! Counts the number of links of a robot
        int countRobotLinks(string robFile);

        //! Counts the number of links to load in a problem
        int countLinks2Load(xml_document *doc);
    };

    /** @}   end of Doxygen module "Problem" */
}

#endif  //_PROBLEM_H

