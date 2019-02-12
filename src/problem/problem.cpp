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


#include <kautham/problem/problem.h>
#include <kautham/util/kthutil/kauthamexception.h>


namespace Kautham {
Problem::Problem() : _wspace(NULL), _cspace(new SampleSet()),
    _sampler(NULL), _planner(NULL)  {
}


Problem::~Problem(){
    delete _cspace; //must be deleted first, if not the program crashes...
    delete _wspace;
    delete _planner;
    delete _sampler;
}


bool Problem::createWSpaceFromFile(xml_document *doc, bool useBBOX,
                                   progress_struct *progress) {
    if (_wspace) delete _wspace;
    _wspace = new IVWorkSpace();

    char *old = setlocale(LC_NUMERIC, "C");

    xml_node tmpNode;

    //Add all robots to worskpace
    for (tmpNode = doc->child("Problem").child("Robot");
         tmpNode; tmpNode = tmpNode.next_sibling("Robot")) {
        if (!addRobot2WSpace(&tmpNode, useBBOX, progress)) return false;
    }

    //Set robot controls
    if (!setRobotControls(doc->child("Problem").child("Controls").
                          attribute("robot").as_string())) return false;

    //Add all obstacles to worskpace
    for (tmpNode = doc->child("Problem").child("Obstacle");
         tmpNode; tmpNode = tmpNode.next_sibling("Obstacle")) {
        if (!addObstacle2WSpace(&tmpNode, useBBOX, progress)) return false;
    }

    //Set obstacle controls
    if (!setObstacleControls(doc->child("Problem").child("Controls").
                             attribute("obstacle").as_string())) return false;

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
        if ( it != param->end())
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

    if (rob->getNumJoints() > 0){
        cords.resize(rob->getNumJoints());
        tmpC = new RnConf(rob->getNumJoints());
        for(unsigned i = 0; i < tmpC->getDim(); i++){
            it = param->find(rob->getLink(i+1)->getName());
            if ( it != param->end())
                cords[i]= it->second;
            else
                cords[i] = 0.0;
        }

        tmpC->setCoordinates(cords);
        rob->setHomePos(tmpC);
        delete tmpC;
    }
}

bool Problem::createPlanner( string name, std::string synergyTreeFilename ) {
    if (_planner != NULL )
        delete _planner;

    Sample *sinit=NULL;
    Sample *sgoal=NULL;
    if (_cspace->getSize()>=2)
    {
        sinit=_cspace->getSampleAt(0);
        sgoal=_cspace->getSampleAt(1);
    }

    if (name == "dummy") //Dummy if to start.
        cout<<"planer name is dummy?"<<endl;
#if defined(KAUTHAM_USE_IOC)
    else if (name == "MyGridPlanner")
        _planner = new IOC::MyGridPlanner(CONTROLSPACE, sinit, sgoal, _cspace, _wspace);

    else if (name == "NF1Planner")
        _planner = new IOC::NF1Planner(CONTROLSPACE, sinit, sgoal, _cspace, _wspace);

    else if (name == "HFPlanner")
        _planner = new IOC::HFPlanner(CONTROLSPACE, sinit, sgoal, _cspace, _wspace);
#endif

#if defined(KAUTHAM_USE_OMPL)
    else if (name == "omplDefault")
        _planner = new omplplanner::omplPlanner(CONTROLSPACE,sinit,sgoal,_cspace,_wspace,NULL);

    else if (name == "omplPRM")
        _planner = new omplplanner::omplPRMPlanner(CONTROLSPACE,sinit,sgoal,_cspace,_wspace,NULL);

    else if (name == "omplRRT")
        _planner = new omplplanner::omplRRTPlanner(CONTROLSPACE,sinit,sgoal,_cspace,_wspace,NULL);

    else if (name == "omplFOSRRT")
        _planner = new omplplanner::omplFOSRRTPlanner(CONTROLSPACE,sinit,sgoal,_cspace,_wspace,NULL);

    else if (name == "omplRRTStar")
        _planner = new omplplanner::omplRRTStarPlanner(CONTROLSPACE,sinit,sgoal,_cspace,_wspace,NULL);

    else if (name == "omplTRRT")
        _planner = new omplplanner::omplTRRTPlanner(CONTROLSPACE,sinit,sgoal,_cspace,_wspace,NULL);

    else if (name == "omplFOSTRRT")
        _planner = new omplplanner::omplFOSTRRTPlanner(CONTROLSPACE,sinit,sgoal,_cspace,_wspace,NULL);

    else if (name == "omplTRRTConnect")
        _planner = new omplplanner::omplTRRTConnectPlanner(CONTROLSPACE,sinit,sgoal,_cspace,_wspace,NULL);

    else if (name == "omplFOSTRRTConnect")
        _planner = new omplplanner::omplFOSTRRTConnectPlanner(CONTROLSPACE,sinit,sgoal,_cspace,_wspace,NULL);

    else if (name == "omplLazyTRRT")
        _planner = new omplplanner::omplLazyTRRTPlanner(CONTROLSPACE,sinit,sgoal,_cspace,_wspace,NULL);

    else if (name == "omplpRRT")
        _planner = new omplplanner::omplpRRTPlanner(CONTROLSPACE,sinit,sgoal,_cspace,_wspace,NULL);

    else if (name == "omplLazyRRT")
        _planner = new omplplanner::omplLazyRRTPlanner(CONTROLSPACE,sinit,sgoal,_cspace,_wspace,NULL);

    else if (name == "omplRRTConnect")
        _planner = new omplplanner::omplRRTConnectPlanner(CONTROLSPACE,sinit,sgoal,_cspace,_wspace,NULL);

    else if (name == "omplFOSVFRRT")
        _planner = new omplplanner::omplFOSVFRRTPlanner(CONTROLSPACE,sinit,sgoal,_cspace,_wspace,NULL,synergyTreeFilename);

    else if (name == "omplMyFOSVFRRT")
        _planner = new omplplanner::omplMyFOSVFRRTPlanner(CONTROLSPACE,sinit,sgoal,_cspace,_wspace,NULL,synergyTreeFilename);

    else if (name == "omplFOSBKPIECE1")
        _planner = new omplplanner::omplFOSBKPIECE1Planner(CONTROLSPACE,sinit,sgoal,_cspace,_wspace,NULL,synergyTreeFilename);

    else if (name == "omplFOSLBKPIECE1")
        _planner = new omplplanner::omplFOSLBKPIECE1Planner(CONTROLSPACE,sinit,sgoal,_cspace,_wspace,NULL,synergyTreeFilename);

    else if (name == "omplFOSKPIECE1")
        _planner = new omplplanner::omplFOSKPIECE1Planner(CONTROLSPACE,sinit,sgoal,_cspace,_wspace,NULL,synergyTreeFilename);

    else if (name == "omplFOSRRTStar")
        _planner = new omplplanner::omplFOSRRTStarPlanner(CONTROLSPACE,sinit,sgoal,_cspace,_wspace,NULL,synergyTreeFilename);

    else if (name == "omplFOSRRTConnect")
        _planner = new omplplanner::omplFOSRRTConnectPlanner(CONTROLSPACE,sinit,sgoal,_cspace,_wspace,NULL);

    else if (name == "omplRRTConnectEUROC")
        _planner = new omplplanner::omplRRTConnectPlannerEUROC(CONTROLSPACE,sinit,sgoal,_cspace,_wspace,NULL);

    else if (name == "omplProjEST")
        _planner = new omplplanner::omplProjESTPlanner(CONTROLSPACE,sinit,sgoal,_cspace,_wspace,NULL);

    else if (name == "omplSBL")
        _planner = new omplplanner::omplSBLPlanner(CONTROLSPACE,sinit,sgoal,_cspace,_wspace, NULL);

    else if (name == "omplKPIECE")
        _planner = new omplplanner::omplKPIECEPlanner(CONTROLSPACE,sinit,sgoal,_cspace,_wspace, NULL);


    else if (name == "omplcRRTf16")
        _planner = new omplcplanner::omplcRRTf16Planner(CONTROLSPACE, sinit, sgoal, _cspace, _wspace, NULL);

    else if (name == "omplcRRTdualdrive")
        _planner = new omplcplanner::omplcRRTdualdrivePlanner(CONTROLSPACE, sinit, sgoal, _cspace, _wspace, NULL);

    else if (name == "omplcRRTcar")
        _planner = new omplcplanner::omplcRRTcarPlanner(CONTROLSPACE, sinit, sgoal, _cspace, _wspace, NULL);


    else if (name == "omplcSSTcar")
        _planner = new omplcplanner::omplcSSTcarPlanner(CONTROLSPACE, sinit, sgoal, _cspace, _wspace, NULL);

    else if (name == "omplcSSTf16")
        _planner = new omplcplanner::omplcSSTf16Planner(CONTROLSPACE, sinit, sgoal, _cspace, _wspace, NULL);

    else if (name == "omplcSSTdualdrive")
        _planner = new omplcplanner::omplcSSTdualdrivePlanner(CONTROLSPACE, sinit, sgoal, _cspace, _wspace, NULL);
#endif

#if defined(KAUTHAM_USE_ODE)

    else if(name == "KPIECE2DPlanner")
        _planner  = new   omplcplanner::KPIECE2DPlanner(CONTROLSPACE, sinit, sgoal, _cspace,_wspace);

    else if (name == "KPIECECarPlanner")
        _planner = new omplcplanner::KPIECECarPlanner(CONTROLSPACE, sinit, sgoal, _cspace,_wspace);

    else if (name == "KPIECEChainPlanner")
        _planner = new omplcplanner::KPIECEChainPlanner(CONTROLSPACE, sinit, sgoal, _cspace,_wspace);

    else if(name == "RRT2DPlanner")
        _planner  = new   omplcplanner::RRT2DPlanner(CONTROLSPACE, sinit, sgoal, _cspace,_wspace);

    else if(name == "Syclop2DPlanner")
        _planner  = new   omplcplanner::Syclop2DPlanner(CONTROLSPACE, sinit, sgoal, _cspace,_wspace);

    else if(name == "KnowledgeOrientedKPIECE2DPlanner")
        _planner  = new   omplcplanner::KnowledgeOrientedKPIECE2DPlanner(CONTROLSPACE, sinit, sgoal, _cspace,_wspace);

    else if(name == "KnowledgeOrientedRRT2DPlanner")
        _planner  = new   omplcplanner::KnowledgeOrientedRRT2DPlanner(CONTROLSPACE, sinit, sgoal, _cspace,_wspace);

    else if(name == "KnowledgeOrientedSyclop2DPlanner")
        _planner  = new   omplcplanner::KnowledgeOrientedSyclop2DPlanner(CONTROLSPACE, sinit, sgoal, _cspace,_wspace);

    //      else if(name == "omplODELTL2DPlanner")
    //          _planner  = new   omplcplanner::KauthamDELTL2DPlanner(CONTROLSPACE, sinit, sgoal, _cspace,_wspace);
#endif
    else {
        cout<<"Planner "<< name <<" is unknow or not loaded (check the CMakeFiles.txt options)" << endl;
        string message = "Planner " + name + " is unknown or not loaded (check the CMakeFiles.txt options)";
        throw KthExcp(message);
    }

    if (_planner) {
        return true;
    } else {
        string message = "Planner couldn't be created";
        throw KthExcp(message);
        return false;
    }
}




bool Problem::createPlanner( string name, ompl::geometric::SimpleSetup *ssptr,
                             std::string synergyTreeFilename ) {
    if (_planner != NULL )
        delete _planner;

    Sample *sinit=NULL;
    Sample *sgoal=NULL;
    if (_cspace->getSize()>=2)
    {
        sinit=_cspace->getSampleAt(0);
        sgoal=_cspace->getSampleAt(1);
    }

    if (name == "dummy") //Dummy if to start.
        cout<<"planer name is dummy?"<<endl;

#if defined(KAUTHAM_USE_OMPL)
    else if (name == "omplDefault")
        _planner = new omplplanner::omplPlanner(CONTROLSPACE,sinit,sgoal,_cspace,_wspace,ssptr);

    else if (name == "omplPRM")
        _planner = new omplplanner::omplPRMPlanner(CONTROLSPACE,sinit,sgoal,_cspace,_wspace,ssptr);

    else if (name == "omplRRT")
        _planner = new omplplanner::omplRRTPlanner(CONTROLSPACE,sinit,sgoal,_cspace,_wspace,ssptr);

    else if (name == "omplFOSRRT")
        _planner = new omplplanner::omplFOSRRTPlanner(CONTROLSPACE,sinit,sgoal,_cspace,_wspace,ssptr);

    else if (name == "omplRRTStar")
        _planner = new omplplanner::omplRRTStarPlanner(CONTROLSPACE,sinit,sgoal,_cspace,_wspace,ssptr);

    else if (name == "omplTRRT")
        _planner = new omplplanner::omplTRRTPlanner(CONTROLSPACE,sinit,sgoal,_cspace,_wspace,ssptr);

    else if (name == "omplFOSTRRT")
        _planner = new omplplanner::omplFOSTRRTPlanner(CONTROLSPACE,sinit,sgoal,_cspace,_wspace,ssptr);

    else if (name == "omplTRRTConnect")
        _planner = new omplplanner::omplTRRTConnectPlanner(CONTROLSPACE,sinit,sgoal,_cspace,_wspace,ssptr);

    else if (name == "omplFOSTRRTConnect")
        _planner = new omplplanner::omplFOSTRRTConnectPlanner(CONTROLSPACE,sinit,sgoal,_cspace,_wspace,ssptr);

    else if (name == "omplLazyTRRT")
        _planner = new omplplanner::omplLazyTRRTPlanner(CONTROLSPACE,sinit,sgoal,_cspace,_wspace,ssptr);

    else if (name == "omplpRRT")
        _planner = new omplplanner::omplpRRTPlanner(CONTROLSPACE,sinit,sgoal,_cspace,_wspace,ssptr);

    else if (name == "omplLazyRRT")
        _planner = new omplplanner::omplLazyRRTPlanner(CONTROLSPACE,sinit,sgoal,_cspace,_wspace,ssptr);

    else if (name == "omplRRTConnect")
        _planner = new omplplanner::omplRRTConnectPlanner(CONTROLSPACE,sinit,sgoal,_cspace,_wspace,ssptr);

    else if (name == "omplFOSVFRRT")
        _planner = new omplplanner::omplFOSVFRRTPlanner(CONTROLSPACE,sinit,sgoal,_cspace,_wspace,ssptr,synergyTreeFilename);

    else if (name == "omplMyFOSVFRRT")
        _planner = new omplplanner::omplMyFOSVFRRTPlanner(CONTROLSPACE,sinit,sgoal,_cspace,_wspace,ssptr,synergyTreeFilename);

    else if (name == "omplFOSBKPIECE1")
        _planner = new omplplanner::omplFOSBKPIECE1Planner(CONTROLSPACE,sinit,sgoal,_cspace,_wspace,ssptr,synergyTreeFilename);

    else if (name == "omplFOSLBKPIECE1")
        _planner = new omplplanner::omplFOSLBKPIECE1Planner(CONTROLSPACE,sinit,sgoal,_cspace,_wspace,ssptr,synergyTreeFilename);

    else if (name == "omplFOSKPIECE1")
        _planner = new omplplanner::omplFOSKPIECE1Planner(CONTROLSPACE,sinit,sgoal,_cspace,_wspace,ssptr,synergyTreeFilename);

    else if (name == "omplFOSRRTStar")
        _planner = new omplplanner::omplFOSRRTStarPlanner(CONTROLSPACE,sinit,sgoal,_cspace,_wspace,ssptr,synergyTreeFilename);

    else if (name == "omplFOSRRTConnect")
        _planner = new omplplanner::omplFOSRRTConnectPlanner(CONTROLSPACE,sinit,sgoal,_cspace,_wspace,ssptr);

    else if (name == "omplRRTConnectEUROC")
        _planner = new omplplanner::omplRRTConnectPlannerEUROC(CONTROLSPACE,sinit,sgoal,_cspace,_wspace,ssptr);

    else if (name == "omplProjEST")
        _planner = new omplplanner::omplProjESTPlanner(CONTROLSPACE,sinit,sgoal,_cspace,_wspace,ssptr);

    else if (name == "omplSBL")
        _planner = new omplplanner::omplSBLPlanner(CONTROLSPACE,sinit,sgoal,_cspace,_wspace, ssptr);

    else if (name == "omplKPIECE")
        _planner = new omplplanner::omplKPIECEPlanner(CONTROLSPACE,sinit,sgoal,_cspace,_wspace, ssptr);

#endif

    else {
        cout<<"Planner "<< name <<" is unknow or not loaded (check the CMakeFiles.txt options)" << endl;
        string message = "Planner " + name + " is unknown or not loaded (check the CMakeFiles.txt options)";
        throw KthExcp(message);
    }

    if (_planner) {
        return true;
    } else {
        string message = "Planner couldn't be created";
        throw KthExcp(message);
        return false;
    }
}

bool Problem::createPlanner( string name, ompl::control::SimpleSetup *ssptr) {
    if (_planner != NULL )
        delete _planner;

    Sample *sinit=NULL;
    Sample *sgoal=NULL;
    if (_cspace->getSize()>=2)
    {
        sinit=_cspace->getSampleAt(0);
        sgoal=_cspace->getSampleAt(1);
    }

    if (name == "dummy") //Dummy if to start.
        cout<<"planer name is dummy?"<<endl;

#if defined(KAUTHAM_USE_OMPL)

    else if (name == "omplcRRTf16")
        _planner = new omplcplanner::omplcRRTf16Planner(CONTROLSPACE, sinit, sgoal, _cspace, _wspace, ssptr);

    else if (name == "omplcRRTdualdrive")
        _planner = new omplcplanner::omplcRRTdualdrivePlanner(CONTROLSPACE, sinit, sgoal, _cspace, _wspace, ssptr);

    else if (name == "omplcRRTcar")
        _planner = new omplcplanner::omplcRRTcarPlanner(CONTROLSPACE, sinit, sgoal, _cspace, _wspace, ssptr);


    else if (name == "omplcSSTcar")
        _planner = new omplcplanner::omplcSSTcarPlanner(CONTROLSPACE, sinit, sgoal, _cspace, _wspace, ssptr);

    else if (name == "omplcSSTf16")
        _planner = new omplcplanner::omplcSSTf16Planner(CONTROLSPACE, sinit, sgoal, _cspace, _wspace, ssptr);

    else if (name == "omplcSSTdualdrive")
        _planner = new omplcplanner::omplcSSTdualdrivePlanner(CONTROLSPACE, sinit, sgoal, _cspace, _wspace, ssptr);
#endif
    else {
        cout<<"Planner "<< name <<" is unknow or not loaded (check the CMakeFiles.txt options)" << endl;
        string message = "Planner " + name + " is unknown or not loaded (check the CMakeFiles.txt options)";
        throw KthExcp(message);
    }

    if (_planner) {
        return true;
    } else {
        string message = "Planner couldn't be created";
        throw KthExcp(message);
        return false;
    }
}






bool Problem::createPlannerFromFile(istream *xml_inputfile) {
    xml_document *doc = new xml_document;
    if (doc->load( *xml_inputfile )) {
        //if the file was correctly parsed
        return createPlannerFromFile(doc);
    } else {
        return false;
    }
}

bool Problem::createPlannerFromFile(istream *xml_inputfile, ompl::geometric::SimpleSetup *ssptr) {
    xml_document *doc = new xml_document;
    if (doc->load( *xml_inputfile )) {
        //if the file was correctly parsed
        return createPlannerFromFile(doc,ssptr);
    } else {
        return false;
    }
}

bool Problem::createPlannerFromFile(istream *xml_inputfile, ompl::control::SimpleSetup *ssptr) {
    xml_document *doc = new xml_document;
    if (doc->load( *xml_inputfile )) {
        //if the file was correctly parsed
        return createPlannerFromFile(doc,ssptr);
    } else {
        return false;
    }
}







bool Problem::createPlannerFromFile(string problemFile) {
    xml_document *doc = new xml_document;
    if (doc->load( problemFile.c_str() )) {
        //if the file was correctly parsed
        return createPlannerFromFile(doc);
    } else {
        return false;
    }
}

bool Problem::createPlannerFromFile(string problemFile, ompl::geometric::SimpleSetup *ssptr) {
    xml_document *doc = new xml_document;
    if (doc->load( problemFile.c_str() )) {
        //if the file was correctly parsed
        return createPlannerFromFile(doc,ssptr);
    } else {
        return false;
    }
}

bool Problem::createPlannerFromFile(string problemFile, ompl::control::SimpleSetup *ssptr) {
    xml_document *doc = new xml_document;
    if (doc->load( problemFile.c_str() )) {
        //if the file was correctly parsed
        return createPlannerFromFile(doc,ssptr);
    } else {
        return false;
    }
}




bool Problem::createPlannerFromFile(xml_document *doc) {
    if (!_planner) {
        //Create the planner and set the parameters
        xml_node planNode = doc->child("Problem").child("Planner").child("Parameters");
        string plannerName = planNode.child("Name").child_value();
        if (plannerName != "") {
            if (createPlanner(plannerName,
                              doc->child("Problem").child("Planner").child("Parameters").
                              child("SynergyTree").attribute("synergyTree").as_string())) {
                //Set especial parameters
                if (plannerName == "omplFOSRRT") {
                    ((omplplanner::omplFOSRRTPlanner*)_planner)->
                            setSynergyTree(doc->child("Problem").child("Planner").child("Parameters").
                                           child("SynergyTree").attribute("synergyTree").as_string());
                } else if (plannerName == "omplFOSTRRT") {
                    ((omplplanner::omplFOSTRRTPlanner*)_planner)->
                            setSynergyTree(doc->child("Problem").child("Planner").child("Parameters").
                                           child("SynergyTree").attribute("synergyTree").as_string());
                } else if (plannerName == "omplFOSRRTConnect") {
                    ((omplplanner::omplFOSRRTConnectPlanner*)_planner)->
                            setSynergyTree(doc->child("Problem").child("Planner").child("Parameters").
                                           child("SynergyTree").attribute("synergyTree").as_string());
                } else if (plannerName == "omplFOSTRRTConnect") {
                    ((omplplanner::omplFOSTRRTConnectPlanner*)_planner)->
                            setSynergyTree(doc->child("Problem").child("Planner").child("Parameters").
                                           child("SynergyTree").attribute("synergyTree").as_string());
                } else if (plannerName == "omplTRRT") {
                    ((omplplanner::omplTRRTPlanner*)_planner)->
                            setPotentialCost(doc->child("Problem").child("Planner").child("Parameters").
                                             child("Potential").attribute("potential").as_string());
                } else if (plannerName == "omplTRRTConnect") {
                    ((omplplanner::omplTRRTConnectPlanner*)_planner)->
                            setPotentialCost(doc->child("Problem").child("Planner").child("Parameters").
                                             child("Potential").attribute("potential").as_string());
                } else if (plannerName == "omplLazyTRRT") {
                    ((omplplanner::omplLazyTRRTPlanner*)_planner)->
                            setPotentialCost(doc->child("Problem").child("Planner").child("Parameters").
                                             child("Potential").attribute("potential").as_string());
                } else if (plannerName == "omplRRTStar") {
                    ((omplplanner::omplRRTStarPlanner*)_planner)->
                            setPotentialCost(doc->child("Problem").child("Planner").child("Parameters").
                                             child("Potential").attribute("potential").as_string());
                }

                //Set normal parameters
                xml_node::iterator it;
                string name;
                for (it = planNode.begin(); it != planNode.end(); ++it) {
                    name = it->name();
                    try {
                        if (name == "Parameter") {
                            name = it->attribute("name").as_string();
                            _planner->setParametersFromString(name.append("|").append(it->child_value()));
                        }
                    } catch(...) {
                        std::cout << "Current planner doesn't have at least one of the parameters"
                                  << " found in the file (" << name << ")." << std::endl;
                        //return false; //changed, let it continue -
                    }
                }
                return true;
            } else {
                return false;
            }
        } else {
            throw KthExcp("Planner name can't be empty");
            return false;
        }
    } else {
        return false;
    }
}


bool Problem::createPlannerFromFile(xml_document *doc, ompl::geometric::SimpleSetup *ssptr) {
    if (!_planner) {
        //Create the planner and set the parameters
        xml_node planNode = doc->child("Problem").child("Planner").child("Parameters");
        string plannerName = planNode.child("Name").child_value();
        if (plannerName != "") {
            if (createPlanner(plannerName,ssptr,
                              doc->child("Problem").child("Planner").child("Parameters").
                              child("SynergyTree").attribute("synergyTree").as_string())) {
                //Set especial parameters
                if (plannerName == "omplFOSRRT") {
                    ((omplplanner::omplFOSRRTPlanner*)_planner)->
                            setSynergyTree(doc->child("Problem").child("Planner").child("Parameters").
                                           child("SynergyTree").attribute("synergyTree").as_string());
                } else if (plannerName == "omplFOSTRRT") {
                    ((omplplanner::omplFOSTRRTPlanner*)_planner)->
                            setSynergyTree(doc->child("Problem").child("Planner").child("Parameters").
                                           child("SynergyTree").attribute("synergyTree").as_string());
                } else if (plannerName == "omplFOSRRTConnect") {
                    ((omplplanner::omplFOSRRTConnectPlanner*)_planner)->
                            setSynergyTree(doc->child("Problem").child("Planner").child("Parameters").
                                           child("SynergyTree").attribute("synergyTree").as_string());
                } else if (plannerName == "omplFOSTRRTConnect") {
                    ((omplplanner::omplFOSTRRTConnectPlanner*)_planner)->
                            setSynergyTree(doc->child("Problem").child("Planner").child("Parameters").
                                           child("SynergyTree").attribute("synergyTree").as_string());
                } else if (plannerName == "omplTRRT") {
                    ((omplplanner::omplTRRTPlanner*)_planner)->
                            setPotentialCost(doc->child("Problem").child("Planner").child("Parameters").
                                             child("Potential").attribute("potential").as_string());
                } else if (plannerName == "omplTRRTConnect") {
                    ((omplplanner::omplTRRTConnectPlanner*)_planner)->
                            setPotentialCost(doc->child("Problem").child("Planner").child("Parameters").
                                             child("Potential").attribute("potential").as_string());
                } else if (plannerName == "omplLazyTRRT") {
                    ((omplplanner::omplLazyTRRTPlanner*)_planner)->
                            setPotentialCost(doc->child("Problem").child("Planner").child("Parameters").
                                             child("Potential").attribute("potential").as_string());
                } else if (plannerName == "omplRRTStar") {
                    ((omplplanner::omplRRTStarPlanner*)_planner)->
                            setPotentialCost(doc->child("Problem").child("Planner").child("Parameters").
                                             child("Potential").attribute("potential").as_string());
                }

                //Set normal parameters
                xml_node::iterator it;
                string name;
                for (it = planNode.begin(); it != planNode.end(); ++it) {
                    name = it->name();
                    try {
                        if (name == "Parameter") {
                            name = it->attribute("name").as_string();
                            _planner->setParametersFromString(name.append("|").append(it->child_value()));
                        }
                    } catch(...) {
                        std::cout << "Current planner doesn't have at least one of the parameters"
                                  << " found in the file (" << name << ")." << std::endl;
                        //return false; //changed, let it continue -
                    }
                }
                return true;
            } else {
                return false;
            }
        } else {
            throw KthExcp("Planner name can't be empty");
            return false;
        }
    } else {
        return false;
    }
}


bool Problem::createPlannerFromFile(xml_document *doc, ompl::control::SimpleSetup *ssptr) {
    if (!_planner) {
        //Create the planner and set the parameters
        xml_node planNode = doc->child("Problem").child("Planner").child("Parameters");
        string plannerName = planNode.child("Name").child_value();
        if (plannerName != "") {
            std::cout<<"9"<<std::endl;
            if (createPlanner(plannerName,ssptr)) {

                std::cout<<"10"<<std::endl;
                //Set normal parameters
                xml_node::iterator it;
                string name;
                for (it = planNode.begin(); it != planNode.end(); ++it) {
                    name = it->name();
                    try {
                        if (name == "Parameter") {
                            name = it->attribute("name").as_string();
                            _planner->setParametersFromString(name.append("|").append(it->child_value()));
                        }
                    } catch(...) {
                        std::cout << "Current planner doesn't have at least one of the parameters"
                                  << " found in the file (" << name << ")." << std::endl;
                        //return false; //changed, let it continue -
                    }
                }
                return true;
            } else {
                return false;
            }
        } else {
            throw KthExcp("Planner name can't be empty");
            return false;
        }
    } else {
        return false;
    }
}




bool Problem::createCSpaceFromFile(xml_document *doc) {
    if (createCSpace()) {
        const unsigned int numRobCntr(_wspace->getNumRobControls());
        const unsigned int numObsCntr(_wspace->getNumObsControls());
        xml_node queries(doc->child("Problem").child("Planner").child("Queries"));
        for (xml_node::iterator query(queries.begin());
             query != queries.end(); ++query) {
            for (xml_node::iterator node(query->begin());
                 node != query->end(); ++node) {
                std::string type(node->name());
                if (type == "InitObs" && numObsCntr > 0) {
                    vector<string> tokens;
                    std::string sentence(node->child_value());
                    boost::split(tokens,sentence,boost::is_any_of(" "),
                                 boost::token_compress_on);
                    if (tokens.size() != numObsCntr) {
                        std::cout << "Dimension of a sample doesn't correspond \
                                     with the problem's dimension." << std::endl;
                                     string message("Error when creating CSpace: InitObs sample \
                                                    has an incorrect dimension");
                                                    stringstream details;
                                     details << "InitObs has dimension " << tokens.size()
                                  << " and should have dimension " << numObsCntr;
                        throw KthExcp(message, details.str());

                        return false;
                    }
                    std::vector<KthReal> coords;
                    for (std::vector<std::string>::const_iterator token(tokens.begin());
                         token != tokens.end(); ++token) {
                        coords.push_back(boost::lexical_cast<double>(*token));
                    }
                    Sample *smp(new Sample(numObsCntr));
                    smp->setCoords(coords);
                    _wspace->setInitObsSample(smp);
                } else if (type == "Init") {
                    vector<string> tokens;
                    std::string sentence(node->child_value());
                    boost::split(tokens,sentence,boost::is_any_of(" "),
                                 boost::token_compress_on);
                    if (tokens.size() != numRobCntr) {
                        std::cout << "Dimension of a sample doesn't correspond \
                                     with the problem's dimension." << std::endl;
                                     string message("Error when creating CSpace: Init sample \
                                                    has an incorrect dimension");
                                                    stringstream details;
                                     details << "Init has dimension " << tokens.size()
                                  << " and should have dimension " << numRobCntr;
                        throw KthExcp(message,details.str());

                        return false;
                    }
                    std::vector<KthReal> coords;
                    for (std::vector<std::string>::const_iterator token(tokens.begin());
                         token != tokens.end(); ++token) {
                        coords.push_back(boost::lexical_cast<double>(*token));
                    }
                    Sample *smp(new Sample(numRobCntr));
                    smp->setCoords(coords);
                    _cspace->addStart(smp);

                    //Add the mapping to configuration space with the collision test.
                    if (_wspace->collisionCheck(smp)) {
                        cout << "Init sample is in collision." << endl;
                    }
                } else if (type == "Goal") {
                    vector<string> tokens;
                    std::string sentence(node->child_value());
                    boost::split(tokens,sentence,boost::is_any_of(" "),
                                 boost::token_compress_on);
                    if (tokens.size() != numRobCntr) {
                        std::cout << "Dimension of a sample doesn't correspond \
                                     with the problem's dimension." << std::endl;
                                     string message("Error when creating CSpace: Goal sample has \
                                                    an incorrect dimension");
                                                    stringstream details;
                                     details << "Goal has dimension " << tokens.size()
                                  << " and should have dimension " << numRobCntr;
                        throw KthExcp(message,details.str());

                        return false;
                    }
                    std::vector<KthReal> coords;
                    for (std::vector<std::string>::const_iterator token(tokens.begin());
                         token != tokens.end(); ++token) {
                        coords.push_back(boost::lexical_cast<double>(*token));
                    }
                    Sample *smp(new Sample(numRobCntr));
                    smp->setCoords(coords);
                    _cspace->addGoal(smp);

                    //Add the mapping to configuration space with the collision test.
                    if (_wspace->collisionCheck(smp)) {
                        cout << "Goal sample is in collision" << endl;
                    }
                }
            }
        }

        if (!_wspace->getInitObsSample()) {//Default values
            Sample *smp(new Sample(numObsCntr));
            std::vector<KthReal> coords(numObsCntr,0.5);
            smp->setCoords(coords);
            _wspace->setInitObsSample(smp);
        }

        for (std::size_t i(0); i < _cspace->getSize(); ++i) {
            for (std::size_t j(i+1); j < _cspace->getSize(); ++j) {
                _cspace->getSampleAt(i)->addNeigh(j);
                _cspace->getSampleAt(j)->addNeigh(i);
            }
        }

        if (_cspace->getNumStarts() > 0 && _cspace->getNumGoals() > 0) return true;
    }
    throw KthExcp("Error when creating CSpace");

    return false;
}


bool Problem::createCSpace() {
    try{
        if (_cspace == NULL) _cspace = new SampleSet();
        if (_sampler == NULL) _sampler = new RandomSampler(_wspace->getNumRobControls());
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
            _currentRobControls[i] = val[i];
        return true;
    }catch(...){
        return false;
    }

}


bool Problem::setCurrentObsControls(vector<KthReal> &val) {
    try{
        if (val.size() !=  _currentObsControls.size()) return false;
        for(unsigned int i=0; i < val.size(); i++)
            _currentObsControls[i] = val[i];
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
        return true;
    } else {
        //the file doesn't exist
        fin.close();
        return false;
    }
}


bool Problem::isFileOK (xml_document *doc) {
    //a correct problem file must have a planner node
    if (!doc->child("Problem").child("Planner")) {
        string message = "Problem file " + _filePath + " doesn't have a planner node";
        throw KthExcp(message);
        return false;
    }

    //a correct problem file must have at least a robot node
    if (!doc->child("Problem").child("Robot").attribute("robot")) {
        string message = "Problem file " + _filePath + " doesn't have any robot node";
        throw KthExcp(message);
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
        unsigned i;
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

                string message = "File " + file + " couldn't be found";
                stringstream details;
                details << "The file was looked for in the following directories:" << endl;
                for (uint i = 0; i < path.size(); i++) {
                    details << "\t" << path.at(i) << endl;
                }
                details << "\nPease, consider updating the Default Path List if necessary";
                throw KthExcp(message, details.str());
            }
        }

        return (!end);
    } else {
        throw KthExcp("No directories where files must be looked for were specified");
        return false;
    }
}


bool Problem::prepareFile (xml_document *doc, vector <string> def_path) {
    if (isFileOK(doc)) {
        xml_node node = doc->child("Problem");

        //find all the robot files and set their complete path if found
        if (!findAllFiles(&node,"Robot","robot",def_path)) return false;

        //find all the obstacle files and set their complete path if found
        if (!findAllFiles(&node,"Obstacle","obstacle",def_path)) return false;

        //find the robot controls file and set its complete path if found
        if (!findAllFiles(&node,"Controls","robot",def_path)) return false;

        //find the obstacle controls file and set its complete path if found
        if (!findAllFiles(&node,"Controls","obstacle",def_path)) return false;

        node = node.child("Planner").child("Parameters");
        //find the PCAkdatree file and set its complete path if found
        if (!findAllFiles(&node,"SynergyTree","synergyTree",def_path)) return false;

        //find the potential file and set its complete path if found
        if (!findAllFiles(&node,"Potential","potential",def_path)) return false;

        return true;
    }
    return false;
}


bool Problem::setupFromFile(istream* xml_inputfile, vector <string> def_path, bool useBBOX) {
    _filePath = def_path.at(0);
    xml_document *doc = new xml_document;
    xml_parse_result result = doc->load( *xml_inputfile );
    if (result) {
        //if the file was correctly parsed
        if (prepareFile(doc, def_path)) {
            //if everything seems to be OK, try to setup the problem
            return setupFromFile(doc, useBBOX);
        } else {
            return false;
        }
    } else {
        string message = "Problem file " + _filePath + " couldn't be parsed";
        stringstream details;
        details << "Error: " << result.description() << endl <<
                   "Last successfully parsed character: " << result.offset;
        throw KthExcp(message,details.str());
        return false;
    }
}


bool Problem::setupFromFile(string xml_doc, vector <string> def_path, bool useBBOX) {
    _filePath = xml_doc;
    xml_document *doc = new xml_document;
    xml_parse_result result = doc->load_file( xml_doc.c_str());
    if (result) {
        //if the file was correctly parsed
        if (prepareFile(doc, def_path)) {
            //if everything seems to be OK, try to setup the problem
            return setupFromFile(doc, useBBOX);
        } else {
            return false;
        }
    } else {
        string message = "Problem file " + _filePath + " couldn't be parsed";
        stringstream details;
        details << "Error: " << result.description() << endl <<
                   "Last successfully parsed character: " << result.offset;
        throw KthExcp(message,details.str());
        return false;
    }
}

int Problem::countRobotLinks(string robFile) {
    //open the file
    fstream fin;
    fin.open(robFile.c_str(),ios::in);
    if (fin.is_open()){// The file already exists.
        fin.close();

        string extension = robFile.substr(robFile.find_last_of(".")+1);

        if (extension == "dh") {
            // Opening the file with the new pugiXML library.
            xml_document doc;

            //Parse the rob file
            xml_parse_result result = doc.load_file(robFile.c_str());
            if (result) {
                //Links of the robot
                return doc.child("Robot").child("Joints").attribute("size").as_int();
            } else {// the result of the file parser is bad
                cout << "Robot file: " << robFile << " can not be read." << endl;

                string message = "Robot file " + robFile + " couldn't be parsed";
                stringstream details;
                details << "Error: " << result.description() << endl <<
                           "Last successfully parsed character: " << result.offset;
                throw KthExcp(message,details.str());
                return -1;
            }
        } else if (extension == "urdf") {
            // Opening the file with the new pugiXML library.
            xml_document doc;

            //Parse the rob file
            xml_parse_result result = doc.load_file(robFile.c_str());
            if (result) {
                //Links of the robot
                xml_node tmpNode = doc.child("robot").child("link");
                int num_links = 0;

                while (tmpNode) {
                    num_links += 1;

                    tmpNode = tmpNode.next_sibling("link");
                }

                return num_links;
            } else {// the result of the file parser is bad
                cout << "Robot file: " << robFile << " can not be read." << endl;

                string message = "Robot file " + robFile + " couldn't be parsed";
                stringstream details;
                details << "Error: " << result.description() << endl <<
                           "Last successfully parsed character: " << result.offset;
                throw KthExcp(message,details.str());
                return -1;
            }
        } else {
            return 1;
        }
    } else {// File does not exists
        fin.close();
        cout << "Robot file: " << robFile << "doesn't exist. Please confirm it." << endl;
        string message = "Robot file " +robFile + " couldn't be found";
        throw KthExcp(message);
        return -1;
    }
}

int Problem::countLinks2Load(xml_document *doc) {
    int links2Load = 0;
    int numLinks;
    xml_node tmpNode;

    //count all robots links
    for (tmpNode = doc->child("Problem").child("Robot");
         tmpNode; tmpNode = tmpNode.next_sibling("Robot")) {
        numLinks = countRobotLinks(tmpNode.attribute("robot").as_string());
        if (numLinks <= 0) return 0;
        links2Load += numLinks;
    }

    //count all obstacles links
    for (tmpNode = doc->child("Problem").child("Obstacle");
         tmpNode; tmpNode = tmpNode.next_sibling("Obstacle")) {
        numLinks = countRobotLinks(tmpNode.attribute("obstacle").as_string());
        if (numLinks <= 0) return 0;
        links2Load += numLinks;
    }

    return links2Load;
}

xml_document *Problem::parseProblemFile(string filename, vector <string> def_path, int *links2Load) {
    _filePath = filename;
    xml_document *doc = new xml_document;
    xml_parse_result result = doc->load_file( filename.c_str());
    if (result) {
        //if the file was correctly parsed
        if (prepareFile(doc, def_path)) {
            //if everything seems to be OK

            *links2Load = countLinks2Load(doc);
            if (*links2Load > 0) {
                return doc;
            } else {
                delete doc;
                return NULL;
            }
        } else {
            *links2Load = -1;
            delete doc;
            return NULL;
        }
    } else {
        delete doc;
        *links2Load = -1;
        string message = "Problem file " + _filePath + " couldn't be parsed";
        stringstream details;
        details << "Error: " << result.description() << endl <<
                   "Last successfully parsed character: " << result.offset;
        throw KthExcp(message,details.str());
        return NULL;
    }
}


bool Problem::setupFromFile(xml_document *doc, bool useBBOX, progress_struct *progress) {
    if (progress != NULL)  {
        if (progress->abort || progress->mutex == NULL || progress->linksLoaded == NULL) {
            return false;
        }
    }

    if (!createWSpaceFromFile(doc, useBBOX, progress)) return false;

    if (!createCSpaceFromFile(doc)) return false;

    if (!createPlannerFromFile(doc)) return false;

    return true;
}


bool Problem::saveToFile(string file_path) {
    if ( file_path == "" )  file_path = _filePath;
    if ( _filePath != file_path ){ // If save as
        ifstream initialFile(_filePath.c_str(), ios::in|ios::binary);
        ofstream outputFile(file_path.c_str(), ios::out|ios::binary);

        //As long as both the input and output files are open...
        if (initialFile.is_open() && outputFile.is_open()){
            outputFile << initialFile.rdbuf() ;
        }else           //there were any problems with the copying process
            return false;

        initialFile.close();
        outputFile.close();
    }

    xml_document doc;
    xml_parse_result result = doc.load_file( file_path.c_str() );
    if ( !result ) return false;

    if ( _planner == NULL ) return false;
    if ( _planner->initSamp() == NULL || _planner->goalSamp() == NULL )
        return false;

    if ( doc.child("Problem").child("Planner") )
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

    for(unsigned i=0; i<tokens.size(); i=i+2){
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
    if (_planner->isSolved()){
        _wspace->inheritSolution(*(_planner->getSimulationPath()));
        return true;
    }
    return false;
}


bool Problem::addRobot2WSpace(xml_node *robot_node, bool useBBOX, progress_struct *progress) {
    Robot *rob = new Robot(robot_node->attribute("robot").as_string(),
                           robot_node->attribute("scale").as_double(1.),useBBOX,progress);

    if (!rob->isArmed()) return false;

    //Set the SE3 weights for the distance computations, in case of mobile base
    if (robot_node->child("WeightSE3")) {
        KthReal tra = 1.;
        KthReal rot = 1.;
        xml_node tmp = robot_node->child("WeightSE3");
        if( tmp.attribute("rho_t") )
            tra = (KthReal) tmp.attribute("rho_t").as_double();
        if( tmp.attribute("rho_r") )
            rot = (KthReal) tmp.attribute("rho_r").as_double();
        rob->getRobWeight()->setSE3Weight(tra,rot);
    } else {
        rob->getRobWeight()->setSE3Weight(1., 1.);
    }

    // Setup the Inverse Kinematic if it has one.
    string name;
    if (robot_node->child("InvKinematic")){
        name = robot_node->child("InvKinematic").attribute("name").as_string();
        if ( name == "RR2D" )
            rob->setInverseKinematic( Kautham::RR2D );
        else if ( name == "TX90")
            rob->setInverseKinematic( Kautham::TX90 );
        else if ( name == "HAND")
            rob->setInverseKinematic( Kautham::HAND );
        else if ( name == "TX90HAND")
            rob->setInverseKinematic( Kautham::TX90HAND );
        else if ( name == "UR5")
            rob->setInverseKinematic( Kautham::UR5 );
        else if ( name == "YUMI_RIGHT")
            rob->setInverseKinematic( Kautham::YUMI_RIGHT );
        else if ( name == "YUMI_LEFT")
            rob->setInverseKinematic( Kautham::YUMI_LEFT );
        else if ( name == "KUKA_LWR")
            rob->setInverseKinematic( Kautham::KUKA_LWR );
        else if ( name == "")
            rob->setInverseKinematic( Kautham::NOINVKIN);
        else
            rob->setInverseKinematic(Kautham::UNIMPLEMENTED);
    } else
        rob->setInverseKinematic(Kautham::NOINVKIN);

    // Setup the Constrained Kinematic if it has one.
    if (robot_node->child("ConstrainedKinematic")){
        name = robot_node->child("ConstrainedKinematic").attribute("name").as_string();

        rob->setConstrainedKinematic( Kautham::UNCONSTRAINED );

    }else{
        rob->setConstrainedKinematic( Kautham::UNCONSTRAINED );
    }

    // Setup the limits of the moveable base
    xml_node limits_node = robot_node->child("Limits");
    while (limits_node) {
        name = limits_node.attribute("name").as_string();
        if (name == "X") {
            rob->setLimits(0, limits_node.attribute("min").as_double(),
                           limits_node.attribute("max").as_double());
        } else if (name == "Y") {
            rob->setLimits(1, limits_node.attribute("min").as_double(),
                           limits_node.attribute("max").as_double());
        } else if (name == "Z") {
            rob->setLimits(2, limits_node.attribute("min").as_double(),
                           limits_node.attribute("max").as_double());
        }

        limits_node = limits_node.next_sibling("Limits");
    }

    xml_node home_node = robot_node->child("Home");
    if (home_node) {
        // If robot hasn't a home, it will be assumed in the origin.
        SE3Conf tmpC;
        vector<KthReal> cords(7);
        cords[0] = home_node.attribute("X").as_double();
        cords[1] = home_node.attribute("Y").as_double();
        cords[2] = home_node.attribute("Z").as_double();
        cords[3] = home_node.attribute("WX").as_double();
        cords[4] = home_node.attribute("WY").as_double();
        cords[5] = home_node.attribute("WZ").as_double();
        cords[6] = home_node.attribute("TH").as_double();

        // Here is needed to convert from axis-angle to
        // quaternion internal represtantation.
        SE3Conf::fromAxisToQuaternion(cords);

        tmpC.setCoordinates(cords);

        //cout << tmpC.print();

        rob->setHomePos(&tmpC);
    }

    _wspace->addRobot(rob);

    return true;
}


bool Problem::addRobot2WSpace(string robFile, KthReal scale, vector<KthReal> home,
                              vector< vector<KthReal> > limits) {
    Robot *rob = new Robot(robFile,scale,false);

    if (!rob->isArmed()) return false;

    rob->setInverseKinematic(Kautham::NOINVKIN);
    rob->setConstrainedKinematic(Kautham::UNCONSTRAINED);

    for (int i = 0; i < 3; ++i) {
        rob->setLimits(i,limits.at(i).at(0),limits.at(i).at(1));
    }

    SE3Conf tmpC;
    // Here is needed to convert from axis-angle to
    // quaternion internal represtantation.
    SE3Conf::fromAxisToQuaternion(home);
    tmpC.setCoordinates(home);
    //cout << tmpC.print();

    _wspace->addRobot(rob);

    return true;
}


bool Problem::setRobotControls(istream *inputfile) {
    xml_document *doc = new xml_document;
    if (!doc->load(*inputfile)) return false;
    return setRobotControls(doc);
}


bool Problem::setRobotControls(string cntrFile) {
    //Once the robots were added, the controls can be configured
    if (cntrFile != "") {//the robot control will be the ones in the controls file
        if (exists(cntrFile)) { // The file already exists.
            if (cntrFile.find(".cntr",0) != string::npos) { // It means that controls are defined by a *.cntr file
                //Opening the file with the new pugiXML library.
                xml_document *doc = new xml_document;
                //Parse the cntr file
                xml_parse_result result = doc->load_file(cntrFile.c_str());
                if (result) {
                    return (setRobotControls(doc));
                } else {// the result of the file parser is bad
                    cout << "The cntr file: " << cntrFile << " can not be read." << std::endl;
                    string message = "Robot controls file " + cntrFile + " couldn't be parsed";
                    stringstream details;
                    details << "Error: " << result.description() << endl <<
                               "Last successfully parsed character: " << result.offset;
                    throw KthExcp(message,details.str());
                    return false;
                }
            } else {//Incorrect extension
                string message = "Robot controls file " + cntrFile + " has an incorrect extension";
                string details = "Controls file must have cntr extension";
                throw KthExcp(message,details);
                return false;
            }
        } else { // File does not exists.
            cout << "The control file: " << cntrFile << "doesn't exist. Please confirm it." << endl;
            string message = "Robot controls file " + cntrFile + " couldn't be found";
            throw KthExcp(message);
            return false;
        }
    } else {//default controls will be set
        return (setDefaultRobotControls());
    }
}


bool Problem::setRobotControls(xml_document *doc) {
    int numControls = 0;
    string controlsName = "";
    xml_node tmpNode = doc->child("ControlSet").child("Control");
    while (tmpNode) {
        numControls++;
        if (controlsName != "") controlsName.append("|");
        controlsName.append(tmpNode.attribute("name").as_string());
        tmpNode = tmpNode.next_sibling("Control");
    }
    _wspace->setNumRobControls(numControls);
    _wspace->setRobControlsName(controlsName);
    _currentRobControls.clear();
    _currentRobControls.resize(_wspace->getNumRobControls());
    for(unsigned i = 0; i<_currentRobControls.size(); i++)
        _currentRobControls[i] = 0.0;

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
            offMatrix[i][j] = 0.5;
            for (int k = 0; k < numControls; k++) {
                mapMatrix[i][j][k] = 0.0;
            }
        }

        _wspace->getRobot(i)->setMapMatrix(mapMatrix[i]);
        _wspace->getRobot(i)->setOffMatrix(offMatrix[i]);
    }

    //Load the Offset vector
    tmpNode = doc->child("ControlSet").child("Offset");
    xml_node::iterator it;
    string dofName, robotName, tmpstr;
    for(it = tmpNode.begin(); it != tmpNode.end(); ++it) {// PROCESSING ALL DOF FOUND
        tmpstr = (*it).attribute("name").as_string();
        unsigned found = tmpstr.find_last_of("/");
        if (found >= tmpstr.length()) {
            string message = "Error when creating robot controls: DOF name " + tmpstr + " is incorrect";
            string details = "Name should be: robot_name + / + dof_name";
            throw KthExcp(message, details);
            return false;
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
            string message = "Error when creating robot controls: Robot " + robotName + " couldn't be found";
            throw KthExcp(message);
            return false;
        }

        if ( dofName == "X"){
            _wspace->getRobot(i)->setSE3(true);
            offMatrix[i][0] = (*it).attribute("value").as_double();
        }else if ( dofName == "Y"){
            _wspace->getRobot(i)->setSE3(true);
            offMatrix[i][1] = (*it).attribute("value").as_double();
        }else if ( dofName == "Z"){
            _wspace->getRobot(i)->setSE3(true);
            offMatrix[i][2] = (*it).attribute("value").as_double();
        }else if ( dofName == "X1"){
            _wspace->getRobot(i)->setSE3(true);
            offMatrix[i][3] = (*it).attribute("value").as_double();
        }else if ( dofName == "X2"){
            _wspace->getRobot(i)->setSE3(true);
            offMatrix[i][4] = (*it).attribute("value").as_double();
        }else if ( dofName == "X3"){
            _wspace->getRobot(i)->setSE3(true);
            offMatrix[i][5] = (*it).attribute("value").as_double();
        }else{    // It's not a SE3 control and could have any name.
            // Find the index orden into the links vector without the first static link.
            unsigned ind = 0;
            while (ind < _wspace->getRobot(i)->getNumJoints()) {
                if ( dofName == _wspace->getRobot(i)->getLink(ind+1)->getName()){
                    offMatrix[i][6 + ind ] = (*it).attribute("value").as_double();
                    break;
                } else {
                    ind++;
                }
            }
            if (ind >= _wspace->getRobot(i)->getNumJoints()) {
                string message = "Error when creating robot controls: DOF " + dofName +
                        " couldn't be found in robot " + robotName;
                throw KthExcp(message);
                return false;
            }
        }
    }//End processing Offset vector

    //Process the controls to load the mapMatrix
    tmpNode = doc->child("ControlSet");
    string nodeType = "";
    int cont = 0;
    for(it = tmpNode.begin(); it != tmpNode.end(); ++it){
        nodeType = it->name();
        if ( nodeType == "Control" ){
            xml_node::iterator itDOF;
            KthReal eigVal = 1;
            if ((*it).attribute("eigValue")) {
                eigVal =  (*it).attribute("eigValue").as_double();
            }

            for(itDOF = (*it).begin(); itDOF != (*it).end(); ++itDOF) {// PROCESSING ALL DOF FOUND
                tmpstr = itDOF->attribute("name").as_string();
                unsigned found = tmpstr.find_last_of("/");
                if (found >= tmpstr.length()) {
                    string message = "Error when creating robot controls: DOF name " + tmpstr + " is incorrect";
                    string details = "Name should be: robot_name + / + dof_name";
                    throw KthExcp(message, details);
                    return false;
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
                    string message = "Error when creating robot controls: Robot " + robotName + " couldn't be found";
                    throw KthExcp(message);
                    return false;
                }

                if ( dofName == "X"){
                    _wspace->getRobot(i)->setSE3(true);
                    mapMatrix[i][0][cont] = eigVal * itDOF->attribute("value").as_double();
                }else if ( dofName == "Y"){
                    _wspace->getRobot(i)->setSE3(true);
                    mapMatrix[i][1][cont] = eigVal * itDOF->attribute("value").as_double();
                }else if ( dofName == "Z"){
                    _wspace->getRobot(i)->setSE3(true);
                    mapMatrix[i][2][cont] = eigVal * itDOF->attribute("value").as_double();
                }else if ( dofName == "X1"){
                    _wspace->getRobot(i)->setSE3(true);
                    mapMatrix[i][3][cont] = eigVal * itDOF->attribute("value").as_double();
                }else if ( dofName == "X2"){
                    _wspace->getRobot(i)->setSE3(true);
                    mapMatrix[i][4][cont] = eigVal * itDOF->attribute("value").as_double();
                }else if ( dofName == "X3"){
                    _wspace->getRobot(i)->setSE3(true);
                    mapMatrix[i][5][cont] = eigVal * itDOF->attribute("value").as_double();
                }else{  // It's not a SE3 control and could have any name.
                    // Find the index orden into the links vector without the first static link.
                    unsigned ind = 0;
                    while (ind < _wspace->getRobot(i)->getNumJoints()) {
                        if ( dofName == _wspace->getRobot(i)->getLink(ind+1)->getName()){
                            mapMatrix[i][6 + ind ][cont] = eigVal * itDOF->attribute("value").as_double();
                            break;
                        } else {
                            ind++;
                        }
                    }
                    if (ind >= _wspace->getRobot(i)->getNumJoints()) {
                        string message = "Error when creating robot controls: DOF " + dofName +
                                " couldn't be found in robot " + robotName;
                        throw KthExcp(message);
                        return false;
                    }
                }
            }

            cont++;
        }// closing if (nodeType == "Control" )
    }//closing for(it = tmpNode.begin(); it != tmpNode.end(); ++it) for all ControlSet childs

    return true;
}


bool Problem::setDefaultRobotControls() {
    //get the number of really movable DOF for all the robots
    unsigned numRob = _wspace->getNumRobots();
    unsigned numControls = 0;
    Robot *rob;
    for (uint i = 0; i < numRob; i++) {
        rob = _wspace->getRobot(i);

        //for X, Y and Z
        for (uint j = 0; j < 3; j++) {
            if (rob->getLimits(j)[0] != rob->getLimits(j)[1]) {
                numControls++;
            }
        }

        //for X1, X2, X3
        numControls += 3;

        //for all the links but the base
        for (uint j = 1; j < rob->getNumLinks(); j++) {
            if (rob->getLink(j)->getMovable()) {
                numControls++;
            }
        }
    }
    _wspace->setNumRobControls(numControls);
    _currentRobControls.clear();
    _currentRobControls.resize(_wspace->getNumRobControls());
    for(unsigned i = 0; i<_currentRobControls.size(); i++)
        _currentRobControls[i] = 0.0;

    KthReal ***mapMatrix;
    KthReal **offMatrix;
    mapMatrix = new KthReal**[numRob];
    offMatrix = new KthReal*[numRob];
    string controlsName;
    string robotName;
    string DOFname;
    bool movDOF;
    unsigned numDOFs;
    Link *link;
    int c = 0;

    //create and set the matrices and get the name of the controls
    for (uint i = 0; i < numRob; i++) {
        rob = _wspace->getRobot(i);
        robotName = rob->getName();
        rob->setSE3(true);//At least X1, X2 and X3 are movable
        numDOFs = 6 + rob->getNumJoints();
        mapMatrix[i] = new KthReal*[numDOFs];
        offMatrix[i] = new KthReal[numDOFs];
        for (uint j = 0; j < numDOFs; j++) {
            offMatrix[i][j] = 0.5;
            mapMatrix[i][j] = new KthReal[numControls];

            for (uint k = 0; k < numControls; k++) {
                mapMatrix[i][j][k] = 0.0;
            }

            movDOF = false;
            switch (j) {
            case 0://X
                if (rob->getLimits(j)[0] != rob->getLimits(j)[1]) {
                    movDOF = true;
                    DOFname = "X";
                }
                break;
            case 1://Y
                if (rob->getLimits(j)[0] != rob->getLimits(j)[1]) {
                    movDOF = true;
                    DOFname = "Y";
                }
                break;
            case 2://Z
                if (rob->getLimits(j)[0] != rob->getLimits(j)[1]) {
                    movDOF = true;
                    DOFname = "Z";
                }
                break;
            case 3://X1
                movDOF = true;
                DOFname = "X1";
                break;
            case 4://X2
                movDOF = true;
                DOFname = "X2";
                break;
            case 5://X3
                movDOF = true;
                DOFname = "X3";
                break;
            default://a joint
                link = rob->getLink(j-5);
                if (link->getMovable()) {
                    movDOF = true;
                    DOFname = link->getName();
                }
                break;
            }
            if (movDOF) {
                if (controlsName != "") controlsName.append("|");
                controlsName.append(robotName+"/"+DOFname);
                mapMatrix[i][j][c] = 1.0;
                c++;
            }
        }

        rob->setMapMatrix(mapMatrix[i]);
        rob->setOffMatrix(offMatrix[i]);
    }
    _wspace->setRobControlsName(controlsName);

    return true;
}


bool Problem::addObstacle2WSpace(xml_node *obstacle_node, bool useBBOX, progress_struct *progress) {
    Robot *obs= new Robot(obstacle_node->attribute("obstacle").as_string(),
                          obstacle_node->attribute("scale").as_double(1.),useBBOX,progress);

    if (!obs->isArmed()) return false;

    // Setup the limits of the moveable base
    string name;
    xml_node limits_node = obstacle_node->child("Limits");
    while (limits_node) {
        name = limits_node.attribute("name").as_string();
        if (name == "X") {
            obs->setLimits(0, limits_node.attribute("min").as_double(),
                           limits_node.attribute("max").as_double());
        } else if (name == "Y") {
            obs->setLimits(1, limits_node.attribute("min").as_double(),
                           limits_node.attribute("max").as_double());
        } else if (name == "Z") {
            obs->setLimits(2, limits_node.attribute("min").as_double(),
                           limits_node.attribute("max").as_double());
        }

        limits_node = limits_node.next_sibling("Limits");
    }

    xml_node home_node = obstacle_node->child("Home");
    if (home_node) {
        // If robot hasn't a home, it will be assumed in the origin.
        SE3Conf tmpC;
        vector<KthReal> cords(7);
        cords[0] = home_node.attribute("X").as_double();
        cords[1] = home_node.attribute("Y").as_double();
        cords[2] = home_node.attribute("Z").as_double();
        cords[3] = home_node.attribute("WX").as_double();
        cords[4] = home_node.attribute("WY").as_double();
        cords[5] = home_node.attribute("WZ").as_double();
        cords[6] = home_node.attribute("TH").as_double();

        // Here is needed to convert from axis-angle to
        // quaternion internal represtantation.
        SE3Conf::fromAxisToQuaternion(cords);

        tmpC.setCoordinates(cords);

        //cout << tmpC.print();

        obs->setHomePos(&tmpC);
    }

    if (obstacle_node->child("Collision").attribute("enabled")){
        obs->setCollisionable(obstacle_node->child("Collision").attribute("enabled").as_bool());
    }

    _wspace->addObstacle(obs);

    return true;
}


bool Problem::addObstacle2WSpace(string robFile, KthReal scale, vector<KthReal> home) {
    Robot *obs = new Robot(robFile,scale,false);

    if (!obs->isArmed()) return false;

    SE3Conf* tmpC = new SE3Conf();
    // Here is needed to convert from axis-angle to
    // quaternion internal represtantation.
    SE3Conf::fromAxisToQuaternion(home);
    tmpC->setCoordinates(home);
    obs->setHomePos(tmpC);
    //cout << tmpC.print();
    std::cout<<"obstacle added at: "<< obs->getHomePos()->getSE3().getPos().at(0)<<" , "
            << obs->getHomePos()->getSE3().getPos().at(1)<<" , "
            << obs->getHomePos()->getSE3().getPos().at(2)<<std::endl;
    _wspace->addObstacle(obs);
    return true;
}


bool Problem::setObstacleControls(istream *inputfile) {
    xml_document *doc = new xml_document;
    if (!doc->load(*inputfile)) return false;
    return setObstacleControls(doc);
}


bool Problem::setObstacleControls(string cntrFile) {
    if (exists(cntrFile)) { // The file already exists.
        string::size_type loc = cntrFile.find( ".cntr", 0 );
        if ( loc != string::npos ) { // It means that controls are defined by a *.cntr file
            //Opening the file with the new pugiXML library.
            xml_document *doc = new xml_document;

            //Parse the cntr file
            xml_parse_result result = doc->load_file(cntrFile.c_str());
            if (result){
                //Once the obstacles were added, the controls can be configured
                return (setObstacleControls(doc));
            } else {// the result of the file pasers is bad
                cout << "The cntr file: " << cntrFile << " can not be read." << std::endl;
                string message = "Obstacle controls file " + cntrFile + " couldn't be parsed";
                stringstream details;
                details << "Error: " << result.description() << endl <<
                           "Last successfully parsed character: " << result.offset;
                throw KthExcp(message,details.str());
                return false;
            }
        } else {//Incorrect extension
            string message = "Obstacle controls file " + cntrFile + " has an incorrect extension";
            string details = "Controls file must have cntr extension";
            throw KthExcp(message,details);
            return false;
        }
    } else { // File does not exists. All DOF must be fixed.
        return(setFixedObstacleControls());
    }
}


bool Problem::setObstacleControls(xml_document *doc) {
    int numControls = 0;
    string controlsName = "";
    xml_node tmpNode = doc->child("ControlSet").child("Control");
    while (tmpNode) {
        numControls++;
        if (controlsName != "") controlsName.append("|");
        controlsName.append(tmpNode.attribute("name").as_string());
        tmpNode = tmpNode.next_sibling("Control");
    }
    _wspace->setNumObsControls(numControls);
    _wspace->setObsControlsName(controlsName);
    _currentObsControls.clear();
    _currentObsControls.resize(_wspace->getNumObsControls());
    for(unsigned i = 0; i<_currentObsControls.size(); i++)
        _currentObsControls[i] = 0.0;

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
            offMatrix[i][j] = 0.5;
            for (int k = 0; k < numControls; k++) {
                mapMatrix[i][j][k] = 0.0;
            }
        }

        _wspace->getObstacle(i)->setMapMatrix(mapMatrix[i]);
        _wspace->getObstacle(i)->setOffMatrix(offMatrix[i]);
    }

    //Load the Offset vector
    tmpNode = doc->child("ControlSet").child("Offset");
    xml_node::iterator it;
    string dofName, obstacleName, tmpstr;
    for(it = tmpNode.begin(); it != tmpNode.end(); ++it) {// PROCESSING ALL DOF FOUND
        tmpstr = (*it).attribute("name").as_string();
        unsigned found = tmpstr.find_last_of("/");
        if (found >= tmpstr.length()) {
            string message = "Error when creating obstacle controls: DOF name " + tmpstr + " is incorrect";
            string details = "Name should be: obstacle_name + / + dof_name";
            throw KthExcp(message, details);
            return false;
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
            string message = "Error when creating obstacle controls: Osbtacle " + obstacleName + " couldn't be found";
            throw KthExcp(message);
            return false;
        }

        if ( dofName == "X"){
            _wspace->getObstacle(i)->setSE3(true);
            offMatrix[i][0] = (*it).attribute("value").as_double();
        }else if ( dofName == "Y"){
            _wspace->getObstacle(i)->setSE3(true);
            offMatrix[i][1] = (*it).attribute("value").as_double();
        }else if ( dofName == "Z"){
            _wspace->getObstacle(i)->setSE3(true);
            offMatrix[i][2] = (*it).attribute("value").as_double();
        }else if ( dofName == "X1"){
            _wspace->getObstacle(i)->setSE3(true);
            offMatrix[i][3] = (*it).attribute("value").as_double();
        }else if ( dofName == "X2"){
            _wspace->getObstacle(i)->setSE3(true);
            offMatrix[i][4] = (*it).attribute("value").as_double();
        }else if ( dofName == "X3"){
            _wspace->getObstacle(i)->setSE3(true);
            offMatrix[i][5] = (*it).attribute("value").as_double();
        }else{    // It's not a SE3 control and could have any name.
            // Find the index orden into the links vector without the first static link.
            unsigned ind = 0;
            while (ind < _wspace->getObstacle(i)->getNumJoints()) {
                if ( dofName == _wspace->getObstacle(i)->getLink(ind+1)->getName()){
                    offMatrix[i][6 + ind ] = (*it).attribute("value").as_double();
                    break;
                } else {
                    ind++;
                }
            }
            if (ind >= _wspace->getObstacle(i)->getNumJoints()) {
                string message = "Error when creating obstacle controls: DOF " + dofName +
                        " couldn't be found in obstacle " + obstacleName;
                throw KthExcp(message);
                return false;
            }
        }
    }//End processing Offset vector

    //Process the controls to load the mapMatrix
    tmpNode = doc->child("ControlSet");
    string nodeType = "";
    int cont = 0;
    for(it = tmpNode.begin(); it != tmpNode.end(); ++it){
        nodeType = it->name();
        if ( nodeType == "Control" ){
            xml_node::iterator itDOF;
            KthReal eigVal = 1;
            if ((*it).attribute("eigValue")) {
                eigVal =  (*it).attribute("eigValue").as_double();
            }

            for(itDOF = (*it).begin(); itDOF != (*it).end(); ++itDOF) {// PROCESSING ALL DOF FOUND
                tmpstr = itDOF->attribute("name").as_string();
                unsigned found = tmpstr.find_last_of("/");
                if (found >= tmpstr.length()) {
                    string message = "Error when creating obstacle controls: DOF name " + tmpstr + " is incorrect";
                    string details = "Name should be: obstacle_name + / + dof_name";
                    throw KthExcp(message, details);
                    return false;
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
                    string message = "Error when creating obstacle controls: Obstacle " + obstacleName + " couldn't be found";
                    throw KthExcp(message);
                    return false;
                }

                if ( dofName == "X"){
                    _wspace->getObstacle(i)->setSE3(true);
                    mapMatrix[i][0][cont] = eigVal * itDOF->attribute("value").as_double();
                }else if ( dofName == "Y"){
                    _wspace->getObstacle(i)->setSE3(true);
                    mapMatrix[i][1][cont] = eigVal * itDOF->attribute("value").as_double();
                }else if ( dofName == "Z"){
                    _wspace->getObstacle(i)->setSE3(true);
                    mapMatrix[i][2][cont] = eigVal * itDOF->attribute("value").as_double();
                }else if ( dofName == "X1"){
                    _wspace->getObstacle(i)->setSE3(true);
                    mapMatrix[i][3][cont] = eigVal * itDOF->attribute("value").as_double();
                }else if ( dofName == "X2"){
                    _wspace->getObstacle(i)->setSE3(true);
                    mapMatrix[i][4][cont] = eigVal * itDOF->attribute("value").as_double();
                }else if ( dofName == "X3"){
                    _wspace->getObstacle(i)->setSE3(true);
                    mapMatrix[i][5][cont] = eigVal * itDOF->attribute("value").as_double();
                }else{  // It's not a SE3 control and could have any name.
                    // Find the index orden into the links vector without the first static link.
                    unsigned ind = 0;
                    while (ind < _wspace->getObstacle(i)->getNumJoints()) {
                        if ( dofName == _wspace->getObstacle(i)->getLink(ind+1)->getName()){
                            mapMatrix[i][6 + ind ][cont] = eigVal * itDOF->attribute("value").as_double();
                            break;
                        } else {
                            ind++;
                        }
                    }
                    if (ind >= _wspace->getObstacle(i)->getNumJoints()) {
                        string message = "Error when creating obstacle controls: DOF " + dofName +
                                " couldn't be found in obstacle " + obstacleName;
                        throw KthExcp(message);
                        return false;
                    }
                }
            }

            cont++;
        }// closing if (nodeType == "Control" )
    }//closing for(it = tmpNode.begin(); it != tmpNode.end(); ++it) for all ControlSet childs

    return true;
}


bool Problem::setFixedObstacleControls() {
    int numObs = _wspace->getNumObstacles();
    int numDOFs;
    KthReal *offMatrix;
    for (int i = 0; i < numObs; i++) {
        numDOFs = _wspace->getObstacle(i)->getNumJoints()+6;
        offMatrix = new KthReal[numDOFs];
        for (int j = 0; j < numDOFs; j++) {
            offMatrix[j] = 0.5;
        }
        _wspace->getObstacle(i)->setOffMatrix(offMatrix);
    }

    return true;
}
}
