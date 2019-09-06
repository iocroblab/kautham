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

/* Author: Alexander Perez, Jan Rosell */

#include <string>
#include <sstream>
#include <boost/algorithm/string.hpp>
#include <kautham/planner/planner.h>
#include <pugixml.hpp>
#include <iostream>
#include <fstream>

using namespace pugi;

namespace Kautham{

  const KthReal RAD2GRAD=180.0/M_PI;
  Planner::Planner(SPACETYPE stype, Sample *init, Sample *goal, SampleSet *samples, WorkSpace *ws){
      _guiName = _idName = "";
      _spType = stype;
      _init.push_back(init);
      _goal.push_back(goal);
      _samples = samples;
      _wkSpace = ws;
      _path.clear();
      _parameters.clear();
      _solved = false;
      _hasCameraInformation = false;
      _sceneCspace = NULL;
      _scenePath = NULL;

  }

  bool Planner::solveAndInherit(){
    _solved = false;
    if(trySolve()){
      moveAlongPath(0);
      _wkSpace->inheritSolution(_simulationPath);
      return true;
    }
    return false;
  }
  
  void Planner::clearSimulationPath(){
    for(unsigned int i = 0; i < _simulationPath.size(); i++)
      delete _simulationPath[i];
    _simulationPath.clear();
    _wkSpace->eraseSolution();
  }

  void Planner::exportSimulationPath(){
    if( isSolved() ){
      ofstream  _theFile("Simulation_path_Rn.txt", ios::trunc | ios::out );

      if( getSimulationPath()->size() == 0 )
        moveAlongPath(0);

      for(size_t i = 0; i < getSimulationPath()->size(); i++){
        _wkSpace->moveRobotsTo(getSimulationPath()->at(i));
				std::vector<KthReal>& joints = _wkSpace->getRobot(0)->getCurrentPos()->getRn().getCoordinates();
        KthReal value=0;
        for(size_t j=0; j< joints.size(); j++){
          value = joints.at(j);
          _theFile << value << " ";
        }
        _theFile << std::endl;
      }

      _theFile.close();
    }else{
      std::cout << "The problem is not solved yet" << std::endl;
    }
  }
  
  uint Planner::moveAlongPath(unsigned int step){
    //Moves to the next step in the simulated path.
    //The path may be computed by the trySolve function to answer a motion planning query or
    //it may come from a taskmotion.xml file.
    //In this latter case the path is a sequence of transit and transfer subpaths and an attach operation
      //is needed before the transfer part starts and a detach once it is over.
      //the info of the steps where this attach/detach operation are to be done (and the info on the
      //object, the robot, and the link involved is stored in the _attachdetach struct.
    if(_solved){
        //here simulationPath coincides with path
        //this function is overloaded by PRM planners that do an interpolation between paths nodes to obtain a more finner simulationPath
        //TO BE REVISED: prior comment seems to be deprecated...(2019-09)
      if(_simulationPath.size() == 0 ){ // Then calculate the simulation path based on stepsize
        for(unsigned i=0; i<_path.size();i++)
        {
          Sample *s=new Sample(*_path.at(i));
          _simulationPath.push_back(s);
        }
      }
      if( _simulationPath.size() >= 2 )
      {
        //step = step % _simulationPath.size();
        //sets the step within simulated path bounds (to be returned by the function)
        if(step>=_simulationPath.size()){
            step = 0;
        }

        cout<<"step = "<<step<<endl;

        //verify if an attach/dettach has to be done
        if(_attachdetach.size())
        {
           /*
            for(uint i=0; i<_attachdetach.size();i++)
            {
              cout<<"step = "<<_attachdetach[i].step;
              cout<<" action = "<<_attachdetach[i].action;
              cout<<" robnumber = "<<_attachdetach[i].robnumber;
              cout<<" linknumber = "<<_attachdetach[i].linknumber;
              cout<<" objnumber = "<<_attachdetach[i].objnumber<<endl;
            }
            */
            //cout<<"step = "<<step<<" prevStep = "<<_simStep<<endl;


            //move the obstacles to their home poses at the beginning of the simulation
            //(needed because the robot will be transferring some objects during the simulation of the taskmotion path)
            if(step==0)
            { 
              cout<<"\n...Restarting the task-motion plan...."<<endl;
              cout<<"...Restoring Object poses...."<<endl;
              //The initial obejct poses were stored when opening the problem in Problem::createWSpaceFromFile()
              //now they are retreived
              wkSpace()->restoreInitialObjectPoses();
            }

            uint previousStep;
            uint currentStep;
            for(uint i=0; i<_attachdetach.size();i++)
            {
              //if an attach/detach is in between the last sim step (_simStep) and the current one (step) 
              //then the current one has to be changed to the attach/detach step
              //and the attach/detach action be performed
              
              //cout<<"_attachdetach["<<i<<"] "<<_attachdetach[i].step<<endl;
              //set limits for comparison
              if(_simStep<step)
              {
                //normal case
                previousStep = _simStep;
                currentStep = step;
              }
              else
              {
                //correction when restared
                previousStep = _simStep;
                currentStep = step + _simulationPath.size() -1;
              }
              //compare
              if(_attachdetach[i].step <= currentStep  && _attachdetach[i].step > previousStep)
              {
                //move robot and attach/detach
                step =_attachdetach[i].step;
                _wkSpace->moveRobotsTo(_simulationPath[step]);
                //cout<<"step modified to "<<step<<" action = "<<_attachdetach[i].action<<endl;
                if(_attachdetach[i].action.compare("attach") == 0)
                {
                  cout<<"...attaching obstacle "<<_attachdetach[i].objnumber<<endl;
                  _wkSpace->attachObstacle2RobotLink(_attachdetach[i].robnumber,_attachdetach[i].linknumber,_attachdetach[i].objnumber);
                }
                else if(_attachdetach[i].action.compare("detach") == 0)
                {
                  cout<<"...detaching obstacle "<<_attachdetach[i].objnumber<<endl;
                  _wkSpace->detachObstacle(_attachdetach[i].objnumber);
                }
                else{
                  cout<<"ERROR: _attachdetach.action is neither attach nor detach";
                }
                break;
              }
              else //move robot - do not attach/detach
                _wkSpace->moveRobotsTo(_simulationPath[step]);
            }
        }
        else 
          _wkSpace->moveRobotsTo(_simulationPath[step]);
        _simStep = step;//store the last simulated step
      }else
        std::cout << "The problem is wrong solved. The solution path has less than two elements." << std::endl;
    }
    return step;
  }


  void Planner::clearAttachData()
  {
    _attachdetach.clear();
  }
  
  void Planner::loadAttachData(int s, string a, int o, int r, int l)
  {
      //loads the attach/detach info read from the taskmotion.xml file
    attachData d;
    d.step = s; //step of the path where the attach/detach is taking place
    d.action = a; //string determining whether it is an attach or a detach
    d.objnumber = o; //object ot be attached/detached
    d.robnumber = r; //robot to which the object is being attached to/detached from
    d.linknumber = l; //link of the robot to which the object is being attached to/detached from
    _attachdetach.push_back(d);
  }

  void Planner::loadExternalPath(vector<Sample*> &p)
  {
      //loads the _path vector from the path read from a taskmotion.xml file
      //that is parsed in plannerwidget.cpp
      _solved = true;
      _path.clear();

      for(unsigned i=0; i<p.size();i++)
      {
         Sample *s=new Sample(*p.at(i));
         _path.push_back(s);
      }
  }


void Planner::moveAlongPathLoad(unsigned int step, std::string address)
{
    bool stopAll = false; //to stop main while loop
    bool stop = false; //to stop nested while loops
    std::string action; //action being processed, either transit or transfer
    int attachedObj; //index of object being attached
    int robIndex; //index of robot to where the object is attached
    int linkIndex; //index of robot link to where the object is attached
    unsigned int requested_line_number = 1;
    std::ifstream path;
    std::vector<Sample*> _path2;
    _path2.clear();
    std::string Rline;
    float value;

    unsigned int line_number = 1; //to loop along the file
    unsigned int line_number2 = 1; //to loop along the configurations while processing an action


    path.open (address);
    if (path.is_open())
    {
      std::cout<< " File successfully opened ! "<<std::endl;
    }
    else
    {
      std::cout << "Error opening file : " <<address<<std::endl;
      exit(1);
    }

    while(  std::getline(path, Rline) && !stopAll )
    {
        clearSimulationPath();

        std::cout << " Rline requested is : " << requested_line_number << " "<< Rline<< std::endl;

        if (line_number == requested_line_number)
        {
            std::vector <std::string> streamCheck;
            streamCheck.clear();
            std::string check;
            std::stringstream  RlineStreamCheck(Rline);

            while(RlineStreamCheck >> check)
            {
                streamCheck.push_back(check);
            }

            if(streamCheck[0] == "transit")
            {
                if(stopAll)
                    break;

                action = streamCheck[0];

                while( std::getline(path, Rline) && !stop)
                {
                    //std::cout << " Rline number is : " << line_number2 << std::endl;

                    if(Rline == "end")
                    { 
                        stop = true;
                        stopAll = true;
                        requested_line_number = line_number2+line_number+1;
                        std::cout << " Rline number is : " << requested_line_number << std::endl;
                        break;
                    }

                    Sample *Robsmp;
                    vector<RobConf> Robrc;
                    std::vector<float>   Rn;

                    Robsmp=new Sample(_wkSpace->getNumRobControls());

                    RobConf *Robrcj = new RobConf;

                    std::stringstream  RlineStream(Rline);


                    while(RlineStream >> value)
                    {
                        Rn.push_back(value);
                    }
                    Robrcj->setSE3(initSamp()->getMappedConf()[0].getSE3());
                    Robrcj->setRn(Rn);
                    Robrc.push_back(*Robrcj);
                    Robsmp->setMappedConf(Robrc);

                    _path2.push_back(Robsmp);
                    line_number2++;
                }
            }

            else if(streamCheck[0] == "transfer")
            {
                if(stopAll)
                    break;

                action = streamCheck[0];
                robIndex = atoi(streamCheck[1].c_str());
                linkIndex = atoi(streamCheck[2].c_str());
                attachedObj = atoi(streamCheck[3].c_str());

                _wkSpace->attachObstacle2RobotLink(robIndex,linkIndex,attachedObj);

                while( std::getline(path, Rline) && !stop)
                {
                    //std::cout << " Rline number is : " << line_number2 << std::endl;

                    if(Rline == "end")
                    { 
                        stop = true;
                        stopAll = true;
                        requested_line_number = line_number2+line_number+1;
                        std::cout << " Rline number is : " << requested_line_number << std::endl;
                        break;
                    }

                    Sample *Robsmp;
                    vector<RobConf> Robrc;
                    std::vector<float>   Rn;

                    Robsmp=new Sample(_wkSpace->getNumRobControls());

                    RobConf *Robrcj = new RobConf;

                    std::stringstream  RlineStream(Rline);


                    while(RlineStream >> value)
                    {
                        Rn.push_back(value);
                    }
                    Robrcj->setSE3(initSamp()->getMappedConf()[0].getSE3());
                    Robrcj->setRn(Rn);
                    Robrc.push_back(*Robrcj);
                    Robsmp->setMappedConf(Robrc);

                    _path2.push_back(Robsmp);
                    line_number2++;
                }
            }
            else
            {
                clearSimulationPath();
                std::cout << " Plan successfully executed !"<<std::endl;
                requested_line_number = 1;
            }
        }
        line_number++;
    }


    for(unsigned i=0; i<_path2.size();i++)
    {
        Sample *s=new Sample(*_path2.at(i));
        _simulationPath.push_back(s);
    }
    step = step % _simulationPath.size();
    _wkSpace->moveRobotsTo(_simulationPath[step]);

    std::cout<<step<<" SIMULATION FROM PATH "<<_simulationPath.size() <<" "<<action <<endl;

    if(step==_simulationPath.size()-1)
    {
        if(action=="transfer")
            _wkSpace->detachObstacle(attachedObj);

        stop=false;
        stopAll=false;
        stopC=1;
    }
}

}

