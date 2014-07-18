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


 
#if !defined(_KAUTHAMSHELL_H)
#define _KAUTHAMSHELL_H
#include <problem/problem.h>

using namespace std;
using namespace Kautham;

namespace Kautham {
    class kauthamshell {
    public:
        inline kauthamshell() {_problem = NULL;} //! Constructor
        inline ~kauthamshell(){} //! Destructor
        inline void closeProblem() {delete _problem;}
        inline bool problemOpened() {return(_problem != NULL);} //! Informs wether there is a problem opened;
        bool openProblem(istream* inputfile, vector<string> def_path = vector<string>());
        bool openProblem(string problemfilename, vector<string> def_path);
        bool checkCollision(vector<KthReal> smpcoords);
        void setRobotsConfig(vector<KthReal> smpcoords);
        void setObstaclesConfig(vector<KthReal> smpcoords);
        bool setQuery(vector<KthReal> init, vector<KthReal> goal);
        bool setInit(vector<KthReal> init);
        bool setGoal(vector<KthReal> goal);
        bool setInitObs(vector<KthReal> initObs);
        void clearSampleSet();
        bool setRobControls(istream* inputfile, vector<KthReal> init, vector<KthReal> goal);
        bool setRobControls(string controlsFile, vector<KthReal> init, vector<KthReal> goal);
        bool setDefaultRobControls(vector<KthReal> init, vector<KthReal> goal);
        bool setObsControls(istream* inputfile, vector<KthReal> initObs);
        bool setObsControls(string controlsFile, vector<KthReal> initObs);
        bool setFixedObsControls();
        bool setPlannerByName(string name);
        bool setPlanner(istream* inputfile);
        bool setPlanner(string problemfilename);
        bool setPlannerParameter(string parameter, string value);
        bool solve(ostream &graphVizPlannerDataFile);
        bool solve();
        int addRobot(string robFile, KthReal scale, vector<KthReal> home, vector< vector<KthReal> > limits,
                         vector< vector<KthReal> > mapMatrix, vector<KthReal> offMatrix);
        void removeRobot(int index);
        int addObstacle(string obsFile, KthReal scale, vector<KthReal> home, vector< vector<KthReal> > limits,
                         vector< vector<KthReal> > mapMatrix, vector<KthReal> offMatrix);
        void removeObstacle(int index);
        bool attachObstacle2RobotLink(string robot, string link, uint obs);
        bool detachObstacleFromRobotLink(string robot, string link, uint obs);

    private:
        Problem *_problem;
    };
}
#endif  //_KAUTHAMSHELL_H
