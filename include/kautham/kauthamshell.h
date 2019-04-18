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

#include <vector>
#include <string>

namespace Kautham {
    class kauthamshell {
    public:
        kauthamshell();//! Constructor
        ~kauthamshell(); //! Destructor
        bool problemOpened(); //! Informs wether there is a problem opened;
        void closeProblem();
        bool openProblem(std::istream *inputfile, std::vector<std::string> def_path = std::vector<std::string>());
        bool openProblem(std::string problemfilename, std::vector<std::string> def_path);
        bool checkCollisionObs(int index, std::vector<unsigned> *collObs, std::string *msg);
        bool checkCollisionRob(std::vector<float> smpcoords, std::vector<unsigned> *ObstColl);
        bool checkCollision(std::vector<float> smpcoords, bool *collisionFree, std::pair<std::pair<int, int>, std::pair<int, int> > *colliding_elements = NULL);
        bool checkCollision(std::vector<float> smpcoords, bool *collisionFree, std::string *msg, std::pair<std::pair<int, int>, std::pair<int, int> > *colliding_elements = NULL);
        bool setRobotsConfig(std::vector<float> smpcoords);
        bool setObstaclesConfig(std::vector<float> smpcoords);
        bool setQuery(std::vector<float> init, std::vector<float> goal);
        bool setInit(std::vector<float> init);
        bool setGoal(std::vector<float> goal);
        bool setInitObs(std::vector<float> initObs);
        bool clearSampleSet();
        bool setRobControls(std::istream* inputfile, std::vector<float> init, std::vector<float> goal);
        bool setRobControls(std::string controlsFile, std::vector<float> init, std::vector<float> goal);
        bool setDefaultRobControls(std::vector<float> init, std::vector<float> goal);
        bool setObsControls(std::istream* inputfile, std::vector<float> initObs);
        bool setObsControls(std::string controlsFile, std::vector<float> initObs);
        bool setFixedObsControls();
        bool setPlannerByName(std::string name);
        bool setPlanner(std::istream* inputfile);
        bool setPlanner(std::string problemfilename);
        bool setPlannerParameter(std::string parameter, std::string value);
        bool solve(std::ostream &graphVizPlannerDataFile);
        bool getPath(std::ostream &path);
        bool setInterpolatePath(bool interpolate);
        bool getInterpolatePath(bool &interpolate);
        bool solve();
        double getLastPlanComputationTime();
        int getNumEdges();
        int getNumVertices();
        bool connect(std::vector<float> smpcoords1, std::vector<float> smpcoords2);
        int addRobot(std::string robFile, double scale, std::vector<float> home, std::vector<std::vector<float> > limits,
                     std::vector<std::vector<float> > mapMatrix, std::vector<float> offMatrix);
        bool removeRobot(unsigned index);
        int addObstacle(std::string obsFile, double scale, std::vector<float> home);
        bool removeObstacle(unsigned index);
        bool attachObstacle2RobotLink(int robot, int link, int obs);
        bool detachObstacle(unsigned int obs);
        double cumDistCheck(std::vector<float> smpcoords);
	
    bool motionPlanner(std::vector <float> init, std::vector <float> goal, std::string root);

    bool setObstaclePos(int index, std::vector<float> pos);
    std::vector<float> getObstaclePos(int index);
    bool findIK(int robIndx, bool armType, std::vector<float> pos, std::vector<float> conf, bool maintSameWrist, std::vector<float> *solution);
    bool setRobPos(unsigned int index, std::vector<float> pos);
    bool getRobPos(unsigned int index, std::vector<float> &pos);

    private:
        void *memPtr_;
    };
}
#endif  //_KAUTHAMSHELL_H
