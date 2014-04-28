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

#include "problem.h"


namespace Kautham {
  const KthReal Problem::_toRad = (KthReal)(M_PI/180.0);
  Problem::Problem() {
      _wspace = NULL;
      _planner = NULL;
      _sampler = NULL;
      _cspace = new SampleSet();
      _cspace->clear();
      _currentRobControls.clear();
      _currentObsControls.clear();

  }

  Problem::~Problem(){
      delete _cspace; //must be deleted first, if not the program crashes...
      delete _wspace;
      delete _planner;
      delete _sampler;
  }


  // This is the new implementation trying to avoid the old strucparse and ProbStruc.
  bool Problem::createWSpace(xml_document *doc) {
      if(_wspace != NULL ) delete _wspace;
      _wspace = new IVWorkSpace();

      char *old = setlocale (LC_NUMERIC, "C");

      xml_node tmpNode;

      //add all robots to worskpace
      for (tmpNode = doc->child("Problem").child("Robot");
           tmpNode; tmpNode = tmpNode.next_sibling("Robot")) {
          if (!addRobot2WSpace(&tmpNode)) return false;
      }

      //add robot controls to worskpace
      if (!addRobotControls2WSpace(doc->child("Problem").child("Controls").
                              attribute("robot").as_string())) return false;

      //add all obstacles to worskpace
      for (tmpNode = doc->child("Problem").child("Obstacle");
           tmpNode; tmpNode = tmpNode.next_sibling("Obstacle")) {
          if (!addObstacle2WSpace(&tmpNode)) return false;
      }

      //add obstacle controls to workspace
      if (!addObstacleControls2WSpace(doc->child("Problem").child("Controls").
                              attribute("obstacle").as_string())) return false;

      //add all distance maps to worskpace
      for (tmpNode = doc->child("Problem").child("DistanceMap");
           tmpNode; tmpNode = tmpNode.next_sibling("DistanceMap")) {
          _wspace->addDistanceMapFile(tmpNode.attribute("distanceMap").as_string());
      }

      //add all dimensions files to worskpace
      for (tmpNode = doc->child("Problem").child("DimensionsFile");
           tmpNode; tmpNode = tmpNode.next_sibling("DimensionsFile")) {
          string file = tmpNode.attribute("filename").as_string();
          _wspace->addDimensionsFile(file);
          string dir = file.substr(0,file.find_last_of("/")+1);
          _wspace->addDirCase(dir);
      }

      _currentRobControls.clear();
      _currentRobControls.resize(_wspace->getNumRobControls());
      for(int i = 0; i<_currentRobControls.size(); i++)
          _currentRobControls[i] = (KthReal)0.0;

      _currentObsControls.clear();
      _currentObsControls.resize(_wspace->getNumObsControls());
      for(int i = 0; i<_currentObsControls.size(); i++)
          _currentObsControls[i] = (KthReal)0.0;

      setlocale (LC_NUMERIC, old);
      return true;
  }

  WorkSpace* Problem::wSpace() {
      return _wspace;
  }

  SampleSet* Problem::cSpace() {
      return _cspace;
  }

  void Problem::setHomeConf(Robot* rob, HASH_S_K* param) {
      Conf* tmpC = new SE3Conf();
      vector<KthReal> cords(7);
      string search[]={"X", "Y", "Z", "WX", "WY", "WZ", "TH"};
      HASH_S_K::iterator it;

      for(int i = 0; i < 7; i++){
          it = param->find(search[i]);
          if( it != param->end())
              cords[i]= it->second;
          else
              cords[i] = 0.0;
      }

      // Here is needed to convert from axis-angle to
      // quaternion internal represtantation.
      SE3Conf::fromAxisToQuaternion(cords);

      tmpC->setCoordinates(cords);
      rob->setHomePos(tmpC);
      delete tmpC;
      cords.clear();

      if(rob->getNumJoints() > 0){
          cords.resize(rob->getNumJoints());
          tmpC = new RnConf(rob->getNumJoints());
          for(int i = 0; i < tmpC->getDim(); i++){
              it = param->find(rob->getLink(i+1)->getName());
              if( it != param->end())
                  cords[i]= it->second;
              else
                  cords[i] = 0.0;
          }

          tmpC->setCoordinates(cords);
          rob->setHomePos(tmpC);
          delete tmpC;
      }
  }

  bool Problem::createPlanner( string name, ompl::geometric::SimpleSetup *ssptr ) {
      if(_planner != NULL )
          delete _planner;

      Sample *sinit=NULL;
      Sample *sgoal=NULL;
      if(_cspace->getSize()>=2)
      {
          sinit=_cspace->getSampleAt(0);
          sgoal=_cspace->getSampleAt(1);
      }

      if(name == "dummy") //Dummy if to start.
          cout<<"planer name is dummy?"<<endl;

#if defined(KAUTHAM_USE_IOC)
      else if(name == "PRM")
          _planner = new IOC::PRMPlanner(CONTROLSPACE, sinit, sgoal, _cspace, _sampler, _wspace);

      else if(name == "PRM Hand IROS")
          _planner = new IOC::PRMHandPlannerIROS(CONTROLSPACE, sinit, sgoal,
                                                 _cspace, _sampler, _wspace, 5, (KthReal)0.001 );
      else if(name == "PRM Hand ICRA")
          _planner = new IOC::PRMHandPlannerICRA(CONTROLSPACE, sinit, sgoal,
                                                 _cspace, _sampler, _wspace, 100, 5, (KthReal)0.010, 5);

      else if(name == "PRMAURO HandArm")
          _planner = new IOC::PRMAUROHandArmPlanner(CONTROLSPACE, sinit, sgoal, _cspace,
                                                    _sampler, _wspace, 10, (KthReal)0.0010, 10);

      else if(name == "PRM RobotHand-Const ICRA")
          _planner = new IOC::PRMRobotHandConstPlannerICRA(CONTROLSPACE, sinit, sgoal, _cspace,
                                                           _sampler, _wspace, 3, (KthReal)50.0);
      else if(name == "MyPlanner")
          _planner = new IOC::MyPlanner(CONTROLSPACE, sinit, sgoal, _cspace, _sampler, _wspace);

      else if(name == "MyPRMPlanner")
          _planner = new IOC::MyPRMPlanner(CONTROLSPACE, sinit, sgoal, _cspace, _sampler, _wspace);

      else if(name == "MyGridPlanner")
          _planner = new IOC::MyGridPlanner(CONTROLSPACE, sinit, sgoal, _cspace, _sampler, _wspace);

      else if(name == "NF1Planner")
          _planner = new IOC::NF1Planner(CONTROLSPACE, sinit, sgoal, _cspace, _sampler, _wspace);

      else if(name == "HFPlanner")
          _planner = new IOC::HFPlanner(CONTROLSPACE, sinit, sgoal, _cspace, _sampler, _wspace);

#if defined(KAUTHAM_USE_ARMADILLO)

      else if(name == "PRMPCA HandArm")
          _planner = new IOC::PRMPCAHandArmPlanner(CONTROLSPACE, sinit, sgoal, _cspace,
                                                   _sampler, _wspace, 10,0, (KthReal)0.0010, 10,0.0,0.0);
#endif
#endif

#if defined(KAUTHAM_USE_GUIBRO)
      else if(name == "GUIBROgrid")
          _planner = new GUIBRO::GUIBROgridPlanner(CONTROLSPACE, NULL, NULL, _cspace, _sampler, _wspace);
#endif

#if defined(KAUTHAM_USE_OMPL)

      else if(name == "omplDefault")
          _planner = new omplplanner::omplPlanner(CONTROLSPACE, sinit, sgoal, _cspace, _wspace, ssptr);

      else if(name == "omplPRM")
          _planner = new omplplanner::omplPRMPlanner(CONTROLSPACE, sinit, sgoal, _cspace, _wspace, ssptr);

      else if(name == "omplRRT")
          _planner = new omplplanner::omplRRTPlanner(CONTROLSPACE, sinit, sgoal, _cspace, _wspace, ssptr);

      else if(name == "omplRRTStar")
          _planner = new omplplanner::omplRRTStarPlanner(CONTROLSPACE, sinit, sgoal, _cspace, _wspace, ssptr);

      else if(name == "omplTRRT")
          _planner = new omplplanner::omplTRRTPlanner(CONTROLSPACE, sinit, sgoal, _cspace, _wspace, ssptr);

      else if(name == "omplpRRT")
          _planner = new omplplanner::omplpRRTPlanner(CONTROLSPACE, sinit, sgoal, _cspace, _wspace, ssptr);

      else if(name == "omplLazyRRT")
          _planner = new omplplanner::omplLazyRRTPlanner(CONTROLSPACE, sinit, sgoal, _cspace, _wspace, ssptr);

      else if(name == "omplRRTConnect")
          _planner = new omplplanner::omplRRTConnectPlanner(CONTROLSPACE, sinit, sgoal, _cspace, _wspace, ssptr);

      else if(name == "omplEST")
          _planner = new omplplanner::omplESTPlanner(CONTROLSPACE, sinit, sgoal, _cspace, _wspace, ssptr);

      else if(name == "omplSBL")
          _planner = new omplplanner::omplSBLPlanner(CONTROLSPACE, sinit, sgoal, _cspace, _wspace, ssptr);

      else if(name == "omplKPIECE")
          _planner = new omplplanner::omplKPIECEPlanner(CONTROLSPACE, sinit, sgoal, _cspace, _wspace, ssptr);

      else if(name == "omplBKPIECE")
          _planner = new omplplanner::omplKPIECEPlanner(CONTROLSPACE, sinit, sgoal, _cspace, _wspace, ssptr);

      else if(name == "omplcRRT")
          _planner = new omplcplanner::omplcRRTPlanner(CONTROLSPACE, sinit, sgoal, _cspace,_wspace);

      else if(name == "omplcRRTf16")
          _planner = new omplcplanner::omplcRRTf16Planner(CONTROLSPACE, sinit, sgoal, _cspace, _wspace);

      else if(name == "omplcRRTcar")
          _planner = new omplcplanner::omplcRRTcarPlanner(CONTROLSPACE, sinit, sgoal, _cspace, _wspace);

    //else
      //  cout<<"Planner "<< name <<" is unknow or not loaded (check the CMakeFiles.txt options)" << endl;

#endif
#if defined(KAUTHAM_USE_ODE)
    else if(name == "omplODERRTPlanner")
       {

        _planner  = new   omplcplanner::KauthamDERRTPlanner(CONTROLSPACE, sinit, sgoal, _cspace,_wspace);
       }
      else if(name == "omplODEKPIECEPlanner")
         {

          _planner  = new   omplcplanner::KauthamDEKPIECEPlanner(CONTROLSPACE, sinit, sgoal, _cspace,_wspace);
         }

#endif
      else
      cout<<"Planner "<< name <<" is unknow or not loaded (check the CMakeFiles.txt options)" << endl;



      if(_planner != NULL)
          return true;
      else
          return false;
  }

  bool Problem::createPlannerFromFile(xml_document *doc, ompl::geometric::SimpleSetup *ssptr) {
      if(_planner == NULL ){
          //Create the planner and set the parameters
          xml_node planNode = doc->child("Problem").child("Planner").child("Parameters");
          string name = planNode.child("Name").child_value();

          if( name != ""){
              if( createPlanner(name,ssptr) ){
                  xml_node::iterator it;
                  for(it = planNode.begin(); it != planNode.end(); ++it){
                      name = it->name();
                      try{
                          if( name == "Parameter" ){
                              name = it->attribute("name").value();
                              _planner->setParametersFromString( name.append("|").append(it->child_value()));
                          }
                      }catch(...){
                          std::cout << "Current planner doesn't have at least one of the parameters"
                                    << " found in the file (" << name << ")." << std::endl;
                          //return false; //changed, let it continue -
                      }
                  }
                  return true;
              }
          }
      }
      return false;
  }

  bool Problem::createCSpaceFromFile(xml_document *doc) {
      if( createCSpace() ){
          xml_node queryNode = doc->child("Problem").child("Planner").child("Queries");
          xml_node::iterator it;
          int i = 0;
          vector<string> tokens;
          string sentence = "";
          Sample* tmpSampPointer = NULL;
          unsigned int numRobCntr = _wspace->getNumRobControls();
          vector<KthReal> robCoords( numRobCntr );
          unsigned int numObsCntr = _wspace->getNumObsControls();
          vector<KthReal> obsCoords( numObsCntr );
          for( it = queryNode.begin(); it != queryNode.end(); ++it ){
              xml_node sampNode = it->child("InitObs");
              if (numObsCntr > 0 && sampNode) {
                  if( _wspace->getNumObsControls() == sampNode.attribute("dim").as_int() ){
                      sentence = sampNode.child_value();

                      boost::split(tokens, sentence, boost::is_any_of("| "));
                      if(tokens.size() != numObsCntr){
                          std::cout << "Dimension of a samples doesn't correspond with the problem's dimension.\n";
                          break;
                      }

                      tmpSampPointer = new Sample(numObsCntr);
                      for(char i=0; i<numObsCntr; i++)
                          obsCoords[i] = (KthReal)atof(tokens[i].c_str());

                      tmpSampPointer->setCoords(obsCoords);

                      _wspace->setInitObsSample(tmpSampPointer);

                  }else{
                      cout << "Sample doesn't have the right dimension.\n";
                      break;
                  }
              } else {
                  tmpSampPointer = new Sample(numObsCntr);
                  for(char i=0; i<numObsCntr; i++)
                      obsCoords[i] = (KthReal)0.5;

                  tmpSampPointer->setCoords(obsCoords);

                  _wspace->setInitObsSample(tmpSampPointer);
              }

              sampNode = it->child("Init");
              if( numRobCntr == sampNode.attribute("dim").as_int() ){
                  sentence = sampNode.child_value();

                  boost::split(tokens, sentence, boost::is_any_of("| "));
                  if(tokens.size() != numRobCntr){
                      std::cout << "Dimension of a samples doesn't correspond with the problem's dimension.\n";
                      break;
                  }

                  tmpSampPointer = new Sample(numRobCntr);
                  for(char i=0; i<numRobCntr; i++)
                      robCoords[i] = (KthReal)atof(tokens[i].c_str());

                  tmpSampPointer->setCoords(robCoords);
                  // Adding the mapping to configuration space with the collision test.
                  if(_wspace->collisionCheck(tmpSampPointer) == true)
                      cout<<"Init sample is in collision"<<endl;
                  _cspace->add(tmpSampPointer);

              }else{
                  cout << "Dimension of a samples doesn't correspond with the problem's dimension.\n";
                  break;
              }

              sampNode = it->child("Goal");
              if( numRobCntr == sampNode.attribute("dim").as_int() ){
                  sentence = sampNode.child_value();

                  boost::split(tokens, sentence, boost::is_any_of("| "));
                  if(tokens.size() != numRobCntr){
                      std::cout << "Dimension of a samples doesn't correspond with the problem's dimension.\n";
                      break;
                  }

                  tmpSampPointer = new Sample(numRobCntr);
                  for(char i=0; i<numRobCntr; i++)
                      robCoords[i] = (KthReal)atof(tokens[i].c_str());

                  tmpSampPointer->setCoords(robCoords);
                  // Adding the mapping to configuration space with the collision test.
                  if(_wspace->collisionCheck(tmpSampPointer)==true)
                      cout<<"Goal sample is in collision"<<endl;
                  _cspace->add(tmpSampPointer);

              }else{
                  cout << "Sample doesn't have the right dimension.\n";
                  break;
              }
          }
          if( _cspace->getSize() >= 2 ){
              _cspace->getSampleAt(0)->addNeigh(1);
              _cspace->getSampleAt(1)->addNeigh(0);
              return true;
          }
      }
      return false;
  }

  bool Problem::createCSpace() {
      try{
          if(_cspace == NULL) _cspace = new SampleSet();
          if(_sampler == NULL) _sampler = new RandomSampler(_wspace->getNumRobControls());
          _cspace->clear();
          return true;
      }catch(...){
          return false;
      }
  }

  bool Problem::setCurrentRobControls(vector<KthReal> &val) {
      try{
          if (val.size() !=  _currentRobControls.size()) return false;
          for(unsigned int i=0; i < val.size(); i++)
              _currentRobControls[i] = (KthReal)val[i];
          return true;
      }catch(...){
          return false;
      }

  }

  bool Problem::setCurrentObsControls(vector<KthReal> &val) {
      try{
          if (val.size() !=  _currentObsControls.size()) return false;
          for(unsigned int i=0; i < val.size(); i++)
              _currentObsControls[i] = (KthReal)val[i];
          return true;
      }catch(...){
          return false;
      }

  }

  bool Problem::exists(string file) {
      fstream fin;
      fin.open(file.c_str(),ios::in);
      if (fin.is_open()) {
          //the file exists
          fin.close();
          return (true);
      } else {
          //the file doesn't exist
          fin.close();
          return (false);
      }
  }

  bool Problem::isFileOK (xml_document *doc) {
      //a correct problem file must have a planner node
      if (!doc->child("Problem").child("Planner")) {
          return false;
      }

      //a correct problem file must have at least a robot node
      if (!doc->child("Problem").child("Robot").attribute("robot")) {
          return false;
      }

      //a correct problem file must have a robot controls node
      if (!doc->child("Problem").child("Controls").attribute("robot")) {
          return false;
      }

      //if all the required information exists
      return true;
  }

  bool Problem::findAllFiles(xml_node *parent, string child, string attribute,
                             vector<string> path) {
      if (path.size() > 0) {
          string file;
          bool found;
          int i;
          bool end = false;
          xml_node node = parent->child(child.c_str());
          while (node && !end) {
              //load file
              file = node.attribute(attribute.c_str()).as_string();

              //look for the file
              found = false;
              i = 0;
              while (!found && i < path.size()) {
                  if (exists(path.at(i)+file)) {
                      file = path.at(i)+file;
                      node.attribute(attribute.c_str()).set_value(file.c_str());
                      found = true;
                  } else {
                      i++;
                  }
              }

              if (found) {
                  node = node.next_sibling(child.c_str());
              } else {
                  end = true;
              }
          }

          return (!end);
      } else {
          return false;
      }
  }

  bool Problem::prepareFile (xml_document *doc) {
      if (isFileOK(doc)) {
          //get the relative paths where robots and obstacle files will be looked for
          string  dir = _filePath.substr(0,_filePath.find_last_of("/")+1);

          QSettings settings("IOC", "Kautham");
          string models_def_path = settings.value("default_path/models",QString(string(dir+string("/../../models")).c_str())).
                  toString().toStdString()+"/";

          vector <string> path;
          path.push_back(dir);
          path.push_back(models_def_path);

          xml_node prob_node = doc->child("Problem");

          //find all the robot files and set their complete path if found
          if (!findAllFiles(&prob_node,"Robot","robot",path)) return false;

          //find all the obstacle files and set their complete path if found
          if (!findAllFiles(&prob_node,"Obstacle","obstacle",path)) return false;

          //find the robot controls file and set its complete path if found
          if (!findAllFiles(&prob_node,"Controls","robot",path)) return false;

          //find the obstacle controls file and set its complete path if found
          if (!findAllFiles(&prob_node,"Controls","obstacle",path)) return false;

          //find all the distancemap files and set their complete path if found
          if (!findAllFiles(&prob_node,"DistanceMap","distanceMap",path)) return false;

          //find all the dimensions files and set their complete path if found
          if (!findAllFiles(&prob_node,"DimensionsFile","filename",path)) return false;

          return true;
      }
      return false;
  }

  bool Problem::setupFromFile(ifstream* xml_inputfile, string modelsfolder) {
      _filePath = modelsfolder.c_str();
      xml_document *doc = new xml_document;
      if(doc->load( *xml_inputfile )) {
          //if the file was correctly parsed
          if (prepareFile(doc)) {
              //if everything seems to be OK, try to setup the problem
              return setupFromFile(doc);
          } else {
              return false;
          }
      } else {
          return false;
      }
  }

  bool Problem::setupFromFile(string xml_doc) {
      _filePath = xml_doc;
      xml_document *doc = new xml_document;
      if(doc->load_file( xml_doc.c_str())) {
          //if the file was correctly parsed
          if (prepareFile(doc)) {
              //if everything seems to be OK, try to setup the problem
              return setupFromFile(doc);
          } else {
              return false;
          }
      } else {
          return false;
      }
  }

  bool Problem::setupFromFile(xml_document *doc) {
      if (!createWSpace(doc)) return false;

      if (!createCSpaceFromFile(doc)) return false;

      if (!createPlannerFromFile(doc)) return false;

      return true;
  }

  bool Problem::saveToFile(string file_path) {
      if( file_path == "" )  file_path = _filePath;
      if( _filePath != file_path ){ // If save as
          ifstream initialFile(_filePath.c_str(), ios::in|ios::binary);
          ofstream outputFile(file_path.c_str(), ios::out|ios::binary);

          //As long as both the input and output files are open...
          if(initialFile.is_open() && outputFile.is_open()){
              outputFile << initialFile.rdbuf() ;
          }else           //there were any problems with the copying process
              return false;

          initialFile.close();
          outputFile.close();
      }

      xml_document doc;
      xml_parse_result result = doc.load_file( file_path.c_str() );
      if( !result ) return false;

      if( _planner == NULL ) return false;
      if( _planner->initSamp() == NULL || _planner->goalSamp() == NULL )
          return false;

      if( doc.child("Problem").child("Planner") )
          doc.child("Problem").remove_child("Planner");

      xml_node planNode = doc.child("Problem").append_child();
      planNode.set_name("Planner");
      xml_node paramNode = planNode.append_child();
      paramNode.set_name("Parameters");
      xml_node planname = paramNode.append_child();
      planname.set_name("Name");
      planname.append_child(node_pcdata).set_value(_planner->getIDName().c_str());



      // Adding the parameters
      string param = _planner->getParametersAsString();
      vector<string> tokens;
      boost::split(tokens, param, boost::is_any_of("|"));

      for(int i=0; i<tokens.size(); i=i+2){
          xml_node paramItem = paramNode.append_child();
          paramItem.set_name("Parameter");
          paramItem.append_attribute("name") = tokens[i].c_str();
          paramItem.append_child(node_pcdata).set_value(tokens[i+1].c_str());
      }

      // Adding the Query information

      xml_node queryNode = planNode.append_child();
      queryNode.set_name("Queries");

      xml_node queryItem = queryNode.append_child();
      queryItem.set_name("Query");
      xml_node initNode = queryItem.append_child();
      initNode.set_name( "Init" );
      initNode.append_attribute("dim") = _wspace->getNumRobControls();
      initNode.append_child(node_pcdata).set_value( _planner->initSamp()->print(true).c_str() );
      xml_node goalNode = queryItem.append_child();
      goalNode.set_name( "Goal" );
      goalNode.append_attribute("dim") = _wspace->getNumRobControls();
      goalNode.append_child(node_pcdata).set_value( _planner->goalSamp()->print(true).c_str() );

      return doc.save_file(file_path.c_str());
  }

  bool Problem::inheritSolution(){
      if(_planner->isSolved()){
          _wspace->inheritSolution(*(_planner->getSimulationPath()));
          return true;
      }
      return false;
  }

  bool Problem::addRobot2WSpace(xml_node *robot_node) {
      Robot *rob;
      string name;

#ifndef KAUTHAM_COLLISION_PQP
      rob = new Robot(robot_node->attribute("robot").value(),
                      (KthReal)robot_node->attribute("scale").as_double(),INVENTOR);
#else
      rob = new Robot(robot_node->attribute("robot").value(),
                      (KthReal)robot_node->attribute("scale").as_double(),IVPQP);
#endif

      if (!rob->isArmed()) return false;

      // Setup the Inverse Kinematic if it has one.
      if(robot_node->child("InvKinematic")){
          name = robot_node->child("InvKinematic").attribute("name").value();
          if( name == "RR2D" )
              rob->setInverseKinematic( Kautham::RR2D );
          else if( name == "TX90")
              rob->setInverseKinematic( Kautham::TX90 );
          else if( name == "HAND")
              rob->setInverseKinematic( Kautham::HAND );
          else if( name == "TX90HAND")
              rob->setInverseKinematic( Kautham::TX90HAND );
          else if( name == "UR5")
              rob->setInverseKinematic( Kautham::UR5 );
          else if ( name == "")
              rob->setInverseKinematic( Kautham::NOINVKIN);
          else
              rob->setInverseKinematic(Kautham::UNIMPLEMENTED);
      }else
          rob->setInverseKinematic(Kautham::NOINVKIN);

      // Setup the Constrained Kinematic if it has one.
      if(robot_node->child("ConstrainedKinematic")){
          name = robot_node->child("ConstrainedKinematic").attribute("name").value();

          rob->setConstrainedKinematic( Kautham::UNCONSTRAINED );
#if defined(KAUTHAM_USE_GUIBRO)
          if( name == "BRONCHOSCOPY" ){
              rob->setConstrainedKinematic( Kautham::BRONCHOSCOPY );
              double amin = robot_node->child("ConstrainedKinematic").attribute("amin").as_double();
              double amax = robot_node->child("ConstrainedKinematic").attribute("amax").as_double();
              double bmin = robot_node->child("ConstrainedKinematic").attribute("bmin").as_double();
              double bmax = robot_node->child("ConstrainedKinematic").attribute("bmax").as_double();
              ((GUIBRO::ConsBronchoscopyKin*)rob->getCkine())->setAngleLimits(bmin*M_PI/180.0, bmax*M_PI/180.0, amin*M_PI/180.0, amax*M_PI/180.0);
          }
          else
              rob->setConstrainedKinematic( Kautham::UNCONSTRAINED );
#endif
      }else{
          rob->setConstrainedKinematic( Kautham::UNCONSTRAINED );
      }

      // Setup the limits of the moveable base
      for(xml_node_iterator itL = robot_node->begin(); itL != robot_node->end(); ++itL){
          name = (*itL).name();
          if( name == "Limits" ){
              name = (*itL).attribute("name").value();
              if( name == "X")
                  rob->setLimits(0, (KthReal)(*itL).attribute("min").as_double(),
                                 (KthReal)(*itL).attribute("max").as_double());
              else if( name == "Y")
                  rob->setLimits(1, (KthReal)(*itL).attribute("min").as_double(),
                                 (KthReal)(*itL).attribute("max").as_double());
              else if( name == "Z")
                  rob->setLimits(2, (KthReal)(*itL).attribute("min").as_double(),
                                 (KthReal)(*itL).attribute("max").as_double());
              else if( name == "WX")
                  rob->setLimits(3, (KthReal)(*itL).attribute("min").as_double(),
                                 (KthReal)(*itL).attribute("max").as_double());
              else if( name == "WY")
                  rob->setLimits(4, (KthReal)(*itL).attribute("min").as_double(),
                                 (KthReal)(*itL).attribute("max").as_double());
              else if( name == "WZ")
                  rob->setLimits(5, (KthReal)(*itL).attribute("min").as_double(),
                                 (KthReal)(*itL).attribute("max").as_double());
              else if( name == "TH")
                  rob->setLimits(6, (KthReal)(*itL).attribute("min").as_double() * _toRad,
                                 (KthReal)(*itL).attribute("max").as_double() * _toRad);
          }
          if( name == "Home" ){
              // If robot hasn't a home, it will be assumed in the origin.
              SE3Conf tmpC;
              vector<KthReal> cords(7);
              cords[0] = (KthReal)(*itL).attribute("X").as_double();
              cords[1] = (KthReal)(*itL).attribute("Y").as_double();
              cords[2] = (KthReal)(*itL).attribute("Z").as_double();
              cords[3] = (KthReal)(*itL).attribute("WX").as_double();
              cords[4] = (KthReal)(*itL).attribute("WY").as_double();
              cords[5] = (KthReal)(*itL).attribute("WZ").as_double();
              cords[6] = (KthReal)(*itL).attribute("TH").as_double() * _toRad;

              // Here is needed to convert from axis-angle to
              // quaternion internal represtantation.
              SE3Conf::fromAxisToQuaternion(cords);

              tmpC.setCoordinates(cords);

              //cout << tmpC.print();

              rob->setHomePos(&tmpC);
          }
      }

      _wspace->addRobot(rob);

      return true;
  }

  bool Problem::addRobotControls2WSpace(string cntrFile) {
      if (exists(cntrFile)) { // The file already exists.
          if (cntrFile.find(".cntr",0) != string::npos) { // It means that controls are defined by a *.cntr file
              //Opening the file with the new pugiXML library.
              xml_document doc;

              //Parse the cntr file
              if (doc.load_file(cntrFile.c_str())){
                  //Once the robots were added, the controls can be configured
                  int numControls = 0;
                  string controlsName = "";
                  xml_node tmpNode = doc.child("ControlSet").child("Control");
                  while (tmpNode) {
                      numControls++;
                      if (controlsName != "") controlsName.append("|");
                      controlsName.append(tmpNode.attribute("name").as_string());
                      tmpNode = tmpNode.next_sibling("Control");
                  }
                  _wspace->setNumRobControls(numControls);
                  _wspace->setRobControlsName(controlsName);

                  //Creating the mapping and offset Matrices between controls
                  //and DOF parameters and initializing them.
                  int numRob = _wspace->getNumRobots();
                  KthReal ***mapMatrix;
                  KthReal **offMatrix;
                  mapMatrix = new KthReal**[numRob];
                  offMatrix = new KthReal*[numRob];
                  int numDOFs;
                  for (int i = 0; i < numRob; i++) {
                      numDOFs = _wspace->getRobot(i)->getNumJoints()+6;
                      mapMatrix[i] = new KthReal*[numDOFs];
                      offMatrix[i] = new KthReal[numDOFs];
                      for (int j = 0; j < numDOFs; j++) {
                          mapMatrix[i][j] = new KthReal[numControls];
                          offMatrix[i][j] = (KthReal)0.5;
                          for (int k = 0; k < numControls; k++) {
                              mapMatrix[i][j][k] = (KthReal)0.0;
                          }
                      }
                  }

                  //Load the Offset vector
                  tmpNode = doc.child("ControlSet").child("Offset");
                  xml_node::iterator it;
                  string dofName, robotName, tmpstr;
                  for(it = tmpNode.begin(); it != tmpNode.end(); ++it) {// PROCESSING ALL DOF FOUND
                      tmpstr = (*it).attribute("name").as_string();
                      unsigned found = tmpstr.find_last_of("/");
                      if (found == string::npos) {
                          return (false);
                      }
                      dofName = tmpstr.substr(found+1);
                      robotName = tmpstr.substr(0,found);

                      //Find the robot index into the robots vector
                      int i = 0;
                      bool robot_found = false;
                      while (!robot_found && i < numRob) {
                          if (_wspace->getRobot(i)->getName() == robotName) {
                              robot_found = true;
                          } else {
                              i++;
                          }
                      }

                      if (!robot_found) {
                          return (false);
                      }

                      if( dofName == "X"){
                          _wspace->getRobot(i)->setSE3(true);
                          offMatrix[i][0] = (KthReal)(*it).attribute("value").as_double();
                      }else if( dofName == "Y"){
                          _wspace->getRobot(i)->setSE3(true);
                          offMatrix[i][1] = (KthReal)(*it).attribute("value").as_double();
                      }else if( dofName == "Z"){
                          _wspace->getRobot(i)->setSE3(true);
                          offMatrix[i][2] = (KthReal)(*it).attribute("value").as_double();
                      }else if( dofName == "X1"){
                          _wspace->getRobot(i)->setSE3(true);
                          offMatrix[i][3] = (KthReal)(*it).attribute("value").as_double();
                      }else if( dofName == "X2"){
                          _wspace->getRobot(i)->setSE3(true);
                          offMatrix[i][4] = (KthReal)(*it).attribute("value").as_double();
                      }else if( dofName == "X3"){
                          _wspace->getRobot(i)->setSE3(true);
                          offMatrix[i][5] = (KthReal)(*it).attribute("value").as_double();
                      }else{    // It's not a SE3 control and could have any name.
                          // Find the index orden into the links vector without the first static link.
                          for(int ind = 0; ind < _wspace->getRobot(i)->getNumJoints(); ind++)
                              if( dofName == _wspace->getRobot(i)->getLink(ind+1)->getName()){
                                  offMatrix[i][6 + ind ] = (KthReal)(*it).attribute("value").as_double();
                                  break;
                              }
                      }
                  }//End processing Offset vector

                  //Process the controls to load the mapMatrix
                  tmpNode = doc.child("ControlSet");
                  string nodeType = "";
                  int cont = 0;
                  for(it = tmpNode.begin(); it != tmpNode.end(); ++it){
                      nodeType = it->name();
                      if( nodeType == "Control" ){
                          xml_node::iterator itDOF;
                          KthReal eigVal = 1;
                          if ((*it).attribute("eigValue")) {
                              eigVal = (KthReal) (*it).attribute("eigValue").as_double();
                          }

                          for(itDOF = (*it).begin(); itDOF != (*it).end(); ++itDOF) {// PROCESSING ALL DOF FOUND
                              tmpstr = itDOF->attribute("name").as_string();
                              unsigned found = tmpstr.find_last_of("/");
                              if (found == string::npos) {
                                  return (false);
                              }
                              dofName = tmpstr.substr(found+1);
                              robotName = tmpstr.substr(0,found);

                              //Find the robot index into the robots vector
                              int i = 0;
                              bool robot_found = false;
                              while (!robot_found && i < numRob) {
                                  if (_wspace->getRobot(i)->getName() == robotName) {
                                      robot_found = true;
                                  } else {
                                      i++;
                                  }
                              }

                              if (!robot_found) {
                                  return (false);
                              }

                              if( dofName == "X"){
                                  _wspace->getRobot(i)->setSE3(true);
                                  mapMatrix[i][0][cont] = eigVal * (KthReal)itDOF->attribute("value").as_double();
                              }else if( dofName == "Y"){
                                  _wspace->getRobot(i)->setSE3(true);
                                  mapMatrix[i][1][cont] = eigVal * (KthReal)itDOF->attribute("value").as_double();
                              }else if( dofName == "Z"){
                                  _wspace->getRobot(i)->setSE3(true);
                                  mapMatrix[i][2][cont] = eigVal * (KthReal)itDOF->attribute("value").as_double();
                              }else if( dofName == "X1"){
                                  _wspace->getRobot(i)->setSE3(true);
                                  mapMatrix[i][3][cont] = eigVal * (KthReal)itDOF->attribute("value").as_double();
                              }else if( dofName == "X2"){
                                  _wspace->getRobot(i)->setSE3(true);
                                  mapMatrix[i][4][cont] = eigVal * (KthReal)itDOF->attribute("value").as_double();
                              }else if( dofName == "X3"){
                                  _wspace->getRobot(i)->setSE3(true);
                                  mapMatrix[i][5][cont] = eigVal * (KthReal)itDOF->attribute("value").as_double();
                              }else{  // It's not a SE3 control and could have any name.
                                  // Find the index orden into the links vector without the first static link.
                                  for(int ind = 0; ind < _wspace->getRobot(i)->getNumJoints(); ind++)
                                      if( dofName == _wspace->getRobot(i)->getLink(ind+1)->getName()){
                                          mapMatrix[i][6 + ind ][cont] = eigVal * (KthReal)itDOF->attribute("value").as_double();
                                          break;
                                      }
                              }
                          }

                          cont++;
                      }// closing if(nodeType == "Control" )
                  }//closing for(it = tmpNode.begin(); it != tmpNode.end(); ++it) for all ControlSet childs

                  for (int i = 0; i < numRob; i++) {
                      _wspace->getRobot(i)->setMapMatrix(mapMatrix[i]);
                      /*for (int j = 0; j < _wspace->getRobot(i)->getNumJoints()+6;j++) {
                      for (int k = 0; k < numControls; k++) {
                          cout << mapMatrix[i][j][k] << " ";
                      }
                      cout << endl;
                  }
                  cout << endl;*/
                      _wspace->getRobot(i)->setOffMatrix(offMatrix[i]);
                      /*for (int j = 0; j < _wspace->getRobot(i)->getNumJoints()+6;j++) {
                      cout << offMatrix[i][j] << endl;
                  }
                  cout << endl;*/
                  }
                  return (true);

              }else  {// the result of the file pasers is bad
                  cout << "The cntr file: " << cntrFile << " can not be read." << std::endl;
                  return (false);
              }
          }
      }else{ // File does not exists.
          cout << "The control file: " << cntrFile << "doesn't exist. Please confirm it." << endl;
          return (false);
      }

  }

  bool Problem::addObstacle2WSpace(xml_node *obstacle_node) {
      Robot *obs;
      string name;

#ifndef KAUTHAM_COLLISION_PQP
      obs = new Robot(obstacle_node->attribute("obstacle").value(),
                      (KthReal)obstacle_node->attribute("scale").as_double(),INVENTOR);
#else
      obs = new Robot(obstacle_node->attribute("obstacle").value(),
                      (KthReal)obstacle_node->attribute("scale").as_double(),IVPQP);
#endif

      if (!obs->isArmed()) return false;

      // Setup the limits of the moveable base
      for(xml_node_iterator itL = obstacle_node->begin(); itL != obstacle_node->end(); ++itL){
          name = (*itL).name();
          if( name == "Limits" ){
              name = (*itL).attribute("name").value();
              if( name == "X")
                  obs->setLimits(0, (KthReal)(*itL).attribute("min").as_double(),
                                 (KthReal)(*itL).attribute("max").as_double());
              else if( name == "Y")
                  obs->setLimits(1, (KthReal)(*itL).attribute("min").as_double(),
                                 (KthReal)(*itL).attribute("max").as_double());
              else if( name == "Z")
                  obs->setLimits(2, (KthReal)(*itL).attribute("min").as_double(),
                                 (KthReal)(*itL).attribute("max").as_double());
              else if( name == "WX")
                  obs->setLimits(3, (KthReal)(*itL).attribute("min").as_double(),
                                 (KthReal)(*itL).attribute("max").as_double());
              else if( name == "WY")
                  obs->setLimits(4, (KthReal)(*itL).attribute("min").as_double(),
                                 (KthReal)(*itL).attribute("max").as_double());
              else if( name == "WZ")
                  obs->setLimits(5, (KthReal)(*itL).attribute("min").as_double(),
                                 (KthReal)(*itL).attribute("max").as_double());
              else if( name == "TH")
                  obs->setLimits(6, (KthReal)(*itL).attribute("min").as_double() * _toRad,
                                 (KthReal)(*itL).attribute("max").as_double() * _toRad);
          }
          if( name == "Home" ){
              // If obstacle hasn't a home, it will be assumed in the origin.
              SE3Conf tmpC;
              vector<KthReal> cords(7);
              cords[0] = (KthReal)(*itL).attribute("X").as_double();
              cords[1] = (KthReal)(*itL).attribute("Y").as_double();
              cords[2] = (KthReal)(*itL).attribute("Z").as_double();
              cords[3] = (KthReal)(*itL).attribute("WX").as_double();
              cords[4] = (KthReal)(*itL).attribute("WY").as_double();
              cords[5] = (KthReal)(*itL).attribute("WZ").as_double();
              cords[6] = (KthReal)(*itL).attribute("TH").as_double() * _toRad;

              // Here is needed to convert from axis-angle to
              // quaternion internal represtantation.
              SE3Conf::fromAxisToQuaternion(cords);

              tmpC.setCoordinates(cords);

              //cout << tmpC.print();

              obs->setHomePos(&tmpC);
          }
          if(name == "Collision" ){
              if (itL->attribute("enabled")) {
                  obs->setCollisionable(itL->attribute("Enabled").as_bool());
              }
          }
      }

      _wspace->addObstacle(obs);

      return true;
  }

  bool Problem::addObstacleControls2WSpace(string cntrFile) {
      if (exists(cntrFile)) { // The file already exists.
          string::size_type loc = cntrFile.find( ".cntr", 0 );
          if( loc != string::npos ) { // It means that controls are defined by a *.cntr file
              //Opening the file with the new pugiXML library.
              xml_document doc;

              //Parse the cntr file
              if (doc.load_file(cntrFile.c_str())){
                  //Once the obstacles were added, the controls can be configured
                  int numControls = 0;
                  string controlsName = "";
                  xml_node tmpNode = doc.child("ControlSet").child("Control");
                  while (tmpNode) {
                      numControls++;
                      if(controlsName != "") controlsName.append("|");
                      controlsName.append(tmpNode.attribute("name").as_string());
                      tmpNode = tmpNode.next_sibling("Control");
                  }
                  _wspace->setNumObsControls(numControls);
                  _wspace->setObsControlsName(controlsName);

                  //Creating the mapping and offset Matrices between controls
                  //and DOF parameters and initializing them.
                  int numObs = _wspace->getNumObstacles();
                  int numDOFs;
                  KthReal ***mapMatrix;
                  KthReal **offMatrix;
                  mapMatrix = new KthReal**[numObs];
                  offMatrix = new KthReal*[numObs];
                  for (int i = 0; i < numObs; i++) {
                      numDOFs = _wspace->getObstacle(i)->getNumJoints()+6;
                      mapMatrix[i] = new KthReal*[numDOFs];
                      offMatrix[i] = new KthReal[numDOFs];
                      for (int j = 0; j < numDOFs; j++) {
                          mapMatrix[i][j] = new KthReal[numControls];
                          offMatrix[i][j] = (KthReal)0.5;
                          for (int k = 0; k < numControls; k++) {
                              mapMatrix[i][j][k] = (KthReal)0.0;
                          }
                      }
                  }

                  //Load the Offset vector
                  tmpNode = doc.child("ControlSet").child("Offset");
                  xml_node::iterator it;
                  string dofName, obstacleName, tmpstr;
                  for(it = tmpNode.begin(); it != tmpNode.end(); ++it) {// PROCESSING ALL DOF FOUND
                      tmpstr = (*it).attribute("name").as_string();
                      unsigned found = tmpstr.find_last_of("/");
                      if (found == string::npos) {
                          return (false);
                      }
                      dofName = tmpstr.substr(found+1);
                      obstacleName = tmpstr.substr(0,found);

                      //Find the obstacle index into the obstacles vector
                      int i = 0;
                      bool obstacle_found = false;
                      while (!obstacle_found && i < numObs) {
                          if (_wspace->getObstacle(i)->getName() == obstacleName) {
                              obstacle_found = true;
                          } else {
                              i++;
                          }
                      }

                      if (!obstacle_found) {
                          return (false);
                      }

                      if( dofName == "X"){
                          _wspace->getObstacle(i)->setSE3(true);
                          offMatrix[i][0] = (KthReal)(*it).attribute("value").as_double();
                      }else if( dofName == "Y"){
                          _wspace->getObstacle(i)->setSE3(true);
                          offMatrix[i][1] = (KthReal)(*it).attribute("value").as_double();
                      }else if( dofName == "Z"){
                          _wspace->getObstacle(i)->setSE3(true);
                          offMatrix[i][2] = (KthReal)(*it).attribute("value").as_double();
                      }else if( dofName == "X1"){
                          _wspace->getObstacle(i)->setSE3(true);
                          offMatrix[i][3] = (KthReal)(*it).attribute("value").as_double();
                      }else if( dofName == "X2"){
                          _wspace->getObstacle(i)->setSE3(true);
                          offMatrix[i][4] = (KthReal)(*it).attribute("value").as_double();
                      }else if( dofName == "X3"){
                          _wspace->getObstacle(i)->setSE3(true);
                          offMatrix[i][5] = (KthReal)(*it).attribute("value").as_double();
                      }else{    // It's not a SE3 control and could have any name.
                          // Find the index orden into the links vector without the first static link.
                          for(int ind = 0; ind < _wspace->getObstacle(i)->getNumJoints(); ind++)
                              if( dofName == _wspace->getObstacle(i)->getLink(ind+1)->getName()){
                                  offMatrix[i][6 + ind ] = (KthReal)(*it).attribute("value").as_double();
                                  break;
                              }
                      }
                  }//End processing Offset vector

                  //Process the controls to load the mapMatrix
                  tmpNode = doc.child("ControlSet");
                  string nodeType = "";
                  int cont = 0;
                  for(it = tmpNode.begin(); it != tmpNode.end(); ++it){
                      nodeType = it->name();
                      if( nodeType == "Control" ){
                          xml_node::iterator itDOF;
                          KthReal eigVal = 1;
                          if ((*it).attribute("eigValue")) {
                              eigVal = (KthReal) (*it).attribute("eigValue").as_double();
                          }

                          for(itDOF = (*it).begin(); itDOF != (*it).end(); ++itDOF) {// PROCESSING ALL DOF FOUND
                              tmpstr = itDOF->attribute("name").as_string();
                              unsigned found = tmpstr.find_last_of("/");
                              if (found == string::npos) {
                                  return (false);
                              }
                              dofName = tmpstr.substr(found+1);
                              obstacleName = tmpstr.substr(0,found);

                              //Find the obstacle index into the obstacles vector
                              int i = 0;
                              bool obstacle_found = false;
                              while (!obstacle_found && i < numObs) {
                                  if (_wspace->getObstacle(i)->getName() == obstacleName) {
                                      obstacle_found = true;
                                  } else {
                                      i++;
                                  }
                              }

                              if (!obstacle_found) {
                                  return (false);
                              }

                              if( dofName == "X"){
                                  _wspace->getObstacle(i)->setSE3(true);
                                  mapMatrix[i][0][cont] = eigVal * (KthReal)itDOF->attribute("value").as_double();
                              }else if( dofName == "Y"){
                                  _wspace->getObstacle(i)->setSE3(true);
                                  mapMatrix[i][1][cont] = eigVal * (KthReal)itDOF->attribute("value").as_double();
                              }else if( dofName == "Z"){
                                  _wspace->getObstacle(i)->setSE3(true);
                                  mapMatrix[i][2][cont] = eigVal * (KthReal)itDOF->attribute("value").as_double();
                              }else if( dofName == "X1"){
                                  _wspace->getObstacle(i)->setSE3(true);
                                  mapMatrix[i][3][cont] = eigVal * (KthReal)itDOF->attribute("value").as_double();
                              }else if( dofName == "X2"){
                                  _wspace->getObstacle(i)->setSE3(true);
                                  mapMatrix[i][4][cont] = eigVal * (KthReal)itDOF->attribute("value").as_double();
                              }else if( dofName == "X3"){
                                  _wspace->getObstacle(i)->setSE3(true);
                                  mapMatrix[i][5][cont] = eigVal * (KthReal)itDOF->attribute("value").as_double();
                              }else{  // It's not a SE3 control and could have any name.
                                  // Find the index orden into the links vector without the first static link.
                                  for(int ind = 0; ind < _wspace->getObstacle(i)->getNumJoints(); ind++)
                                      if( dofName == _wspace->getObstacle(i)->getLink(ind+1)->getName()){
                                          mapMatrix[i][6 + ind ][cont] = eigVal * (KthReal)itDOF->attribute("value").as_double();
                                          break;
                                      }
                              }
                          }

                          cont++;
                      }// closing if(nodeType == "Control" )
                  }//closing for(it = tmpNode.begin(); it != tmpNode.end(); ++it) for all ControlSet childs

                  for (int i = 0; i < numObs; i++) {
                      _wspace->getObstacle(i)->setMapMatrix(mapMatrix[i]);
                      /*for (int j = 0; j < _wspace->getObstacle(i)->getNumJoints()+6;j++) {
                      for (int k = 0; k < _wspace->getNumRobControls(); k++) {
                          cout << mapMatrix[i][j][k] << " ";
                      }
                      cout << endl;
                  }
                  cout << endl;*/
                      _wspace->getObstacle(i)->setOffMatrix(offMatrix[i]);
                      /*for (int j = 0; j < _wspace->getObstacle(i)->getNumJoints()+6;j++) {
                      cout << offMatrix[i][j] << endl;
                  }
                  cout << endl;*/
                  }
                  return (true);

              } else {// the result of the file pasers is bad
                  cout << "The cntr file: " << cntrFile << " can not be read." << std::endl;
                  return (false);
              }
          }
      } else { // File does not exists. All DOF must fixed.
          int numObs = _wspace->getNumObstacles();
          int numDOFs;
          KthReal **offMatrix;
          offMatrix = new KthReal*[numObs];
          for (int i = 0; i < numObs; i++) {
              numDOFs = _wspace->getObstacle(i)->getNumJoints()+6;
              offMatrix[i] = new KthReal[numDOFs];
              for (int j = 0; j < numDOFs; j++) {
                  offMatrix[i][j] = (KthReal)0.5;
              }
              _wspace->getObstacle(i)->setOffMatrix(offMatrix[i]);
          }

          return (true);
      }
  }

}
