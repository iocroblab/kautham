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

/* Author: Nestor Garcia Hidalgo */

#include "benchmark.h"
#include <fstream>
#include <sstream>
#include <planner/omplg/omplplanner.h>
#include <planner/omplg/omplValidityChecker.h>
#include <algorithm>

#define SSTR(x) dynamic_cast<std::ostringstream&>(std::ostringstream() << std::dec << x).str()

using namespace Kautham;


void preRunEvent(const ob::PlannerPtr &planner) {
    // Do whatever configuration we want to the planner,
    // including changing of problem definition (input states)
    // via planner->getProblemDefinition()

    omplplanner::ValidityChecker *validityChecker = dynamic_cast<omplplanner::ValidityChecker*>
            (planner->getSpaceInformation()->getStateValidityChecker().get());
    if (validityChecker) {
        validityChecker->resetCollisionChecks();
    }
}


void postRunEvent(const ob::PlannerPtr &planner, ompl::tools::Benchmark::RunProperties &run) {
    // Do any cleanup, or set values for upcoming run (or upcoming call to the pre-run event).
    // Adding elements to the set of collected run properties is also possible;
    // (the added data will be recorded in the log file. For example:
    // ob::PlannerDataPtr pdata(new ob::PlannerData(planner->getSpaceInformation()));
    // planner->getPlannerData(*pdata);
    // run["numEdges INTEGER"] = SSTR(pdata->numEdges());
    // run["numVertices INTEGER"] = SSTR(pdata->numVertices());*/
    // The format of added data is string key, string value pairs,
    // with the convention that the last word in string key is one of
    // REAL, INTEGER, BOOLEAN, STRING. (this will be the type of the field
    // when the log file is processed and saved as a database).
    // The values are always converted to string.
    // (the added data will be recorded in the log file)

    omplplanner::ValidityChecker *validityChecker = dynamic_cast<omplplanner::ValidityChecker*>
            (planner->getSpaceInformation()->getStateValidityChecker().get());
    if (validityChecker) {
        run["collision checks INTEGER"] = SSTR(validityChecker->getCollisionChecks());
    }

    ob::ProblemDefinitionPtr pdef = planner->getProblemDefinition();
    if (pdef->getOptimizationObjective().get()) {
        cout << "OPT" << endl;

        ob::OptimizationObjectivePtr opt = pdef->getOptimizationObjective();
        if (pdef->hasSolution() && !pdef->hasApproximateSolution()) {
            cout << "SOL" << endl;

            //Get solution path
            ob::PlannerDataPtr pdata(new ob::PlannerData(planner->getSpaceInformation()));
            planner->getPlannerData(*pdata);

            //Get start and goal
            unsigned int v1 = pdata->getStartIndex(0);
            double c1 = opt->stateCost(pdata->getVertex(v1).getState()).v;
            unsigned int v2 = pdata->getGoalIndex(0);
            double c2 = opt->stateCost(pdata->getVertex(v2).getState()).v;
            double d = planner->getSpaceInformation()->distance(pdata->getVertex(v1).getState(),
                                                                pdata->getVertex(v2).getState());
            //Set K's
            double KP = 1./3./d;
            double KI = 1./3./(0.5*(c1+c2)*d);
            double KD = 1./3./fabs(c2-c1);

            //Set initial costs
            double costMW(0.);
            double costPID(0.);
            double meanCost(0.);
            double maxCost(c2);
            double minCost(c2);

            //Iterate trough path from goal to start
            std::vector< unsigned int > edgeList;
            pdata->getIncomingEdges(v2,edgeList);
            while (edgeList.size() > 0) {
                //Get vertex parent
                v1 = edgeList[0];
                c1 = opt->stateCost(pdata->getVertex(v1).getState()).v;
                d = planner->getSpaceInformation()->distance(pdata->getVertex(v1).getState(),
                                                             pdata->getVertex(v2).getState());

                //Update costs
                costMW += max(c2-c1,0.);
                costPID += KP*d+KI*0.5*(c1+c2)*d+KD*fabs(c2-c1);
                maxCost = max(maxCost,c1);
                minCost = min(minCost,c1);
                meanCost += 0.5*(c1+c2)*d;

                //Next vertex
                swap(v1,v2);
                swap(c1,c2);
                pdata->getIncomingEdges(v2,edgeList);
            }

            run["best cost REAL"] = SSTR(pdef->getSolutionPath()->cost(opt));
            run["best costMW REAL"] = SSTR(costMW);
            run["best costPID REAL"] = SSTR(costPID);
            run["mean cost REAL"] = SSTR(meanCost/pdef->getSolutionPath()->length());
            run["maximum cost REAL"] = SSTR(maxCost);
            run["minimum cost REAL"] = SSTR(minCost);
        } else {
            cout << "NO SOL" << endl;

            run["best cost REAL"] = SSTR(opt->infiniteCost());
            run["best costMW REAL"] = SSTR(opt->infiniteCost());
            run["best costPID REAL"] = SSTR(opt->infiniteCost());
            run["mean cost REAL"] = SSTR(opt->infiniteCost());
            run["maximum cost REAL"] = SSTR(opt->infiniteCost());
            run["minimum cost REAL"] = SSTR(opt->infiniteCost());
            run["best cost REAL"] = SSTR(opt->infiniteCost());
        }
    } else {
        cout << "NO OPT" << endl;
    }


}


bool Benchmark::set(xml_node *bm_node, string dir, vector<string> def_path) {
    xml_node node;
    bool ok = true;

    if (bm_node->attribute("Name")) {
        name = bm_node->attribute("Name").as_string();
    } else {
        name = "NO_NAME";
    }

    node = bm_node->child("Problem");
    while (node && ok) {
        ok &= add_problem(dir + node.attribute("File").as_string(),def_path);

        node = node.next_sibling("Problem");
    }

    req = new ompl::tools::Benchmark::Request;
    node = bm_node->child("Parameter");
    while (node && ok) {
        ok &= add_parameter(&node);

        node = node.next_sibling("Parameter");
    }

    if (filename != "") {
        filename = dir + filename;
    }

    return (ok);
}


void Benchmark::run() {
    bm->benchmark(*req);
}


bool Benchmark::save() {
    if (filename != "") {
        return(bm->saveResultsToFile(filename.c_str()));
    } else {
        return(bm->saveResultsToFile());
    }
}


void Benchmark::clear() {
    delete bm;
    delete req;
    problem.clear();
    filename.clear();
}


bool Benchmark::add_problem(string prob_file, vector<string> def_path) {
    Problem *prob = new Problem();
    cout << "Adding problem number " << problem.size()+1 << endl;
    string dir = prob_file.substr(0,prob_file.find_last_of("/")+1);
    def_path.push_back(dir);
    def_path.push_back(dir+"../../models/");

    if (problem.size() == 0) {
        if (prob->setupFromFile(prob_file,def_path)) {
            if (prob->getPlanner()->getFamily() == Kautham::OMPLPLANNER) {
                problem.push_back(prob);
                bm = new ompl::tools::Benchmark(*(((omplplanner::omplPlanner*)prob->
                                                   getPlanner())->SimpleSetup()),name);
                bm->setPreRunEvent(boost::bind(&preRunEvent,_1));
                bm->setPostRunEvent(boost::bind(&postRunEvent,_1,_2));
                bm->addPlanner(((omplplanner::omplPlanner*)prob->getPlanner())->
                               SimpleSetupPtr()->getPlanner());

                return true;
            } else {
                return false;
            }
        } else {
            return false;
        }
    } else {
        prob->setWSpace(problem.at(0)->wSpace());
        prob->setCSpace(problem.at(0)->cSpace());
        xml_document *doc = new xml_document;
        if (!doc->load_file(prob_file.c_str())) return false;
        if (prob->createPlannerFromFile(
                    doc,((omplplanner::omplPlanner*)problem.at(0)->getPlanner())->
                    SimpleSetup())) {
            if (prob->getPlanner()->getFamily() == Kautham::OMPLPLANNER) {
                problem.push_back(prob);
                bm->addPlanner(((omplplanner::omplPlanner*)prob->getPlanner())->
                               SimpleSetupPtr()->getPlanner());

                return true;
            } else {
                return false;
            }
        } else {
            return false;
        }
    }
}


bool Benchmark::add_parameter(xml_node *param_node) {
    if (param_node->attribute("Value")) {
        string name = param_node->attribute("Name").as_string("");
        if (name == "maxTime") {
            req->maxTime = max(0.,param_node->attribute("Value").as_double(5.));
            return true;
        } else if (name == "maxMem") {
            req->maxMem = max(0.,param_node->attribute("Value").as_double(4096.));
            return true;
        } else if (name == "runCount") {
            req->runCount = max(1,param_node->attribute("Value").as_int(100));
            return true;
        } else if (name == "timeBetweenUpdates") {
            req->timeBetweenUpdates = max(0.,param_node->attribute("Value").as_double(0.05));
            return true;
        } else if (name == "displayProgress") {
            req->displayProgress = param_node->attribute("Value").as_bool(true);
            return true;
        } else if (name == "saveConsoleOutput") {
            req->saveConsoleOutput = param_node->attribute("Value").as_bool(true);
            return true;
        } else if (name == "useThreads") {
            req->useThreads = param_node->attribute("Value").as_bool(true);
            return true;
        } else if (name == "filename") {
            filename = param_node->attribute("Value").as_string("");
            return true;
        } else {
            cout << "Incorrect name: " << name << endl;
            return false;
        }
    } else {
        cout << "No value" << endl;
        return false;
    }
}


void benchmark(string file, vector <string> def_path) {
    cout << "Kautham is opening the benchmarking file: " << file << endl;
    string dir = file.substr(0,file.find_last_of("/")+1);
    fstream fin;
    fin.open(file.c_str(),ios::in);
    if (fin.is_open()){//the file already exists
        fin.close();
        cout << "The file exists" << endl;

        xml_document doc;
        if (doc.load_file(file.c_str())) {//the file was correctly parsed
            cout << "The file was correctly parsed" << endl;

            unsigned int i = 0;
            Benchmark bm;

            xml_node bm_node = doc.child("Benchmark");
            while (bm_node) {
                i++;
                cout << "Reading benchmark number " << i << endl;

                if (bm.set(&bm_node,dir,def_path)) {//benchmark could be set
                    cout << "Running benchmark number " << i << endl;
                    bm.run();

                    if (!bm.save()) {
                        cout << "Benchmark could'nt be saved." << endl;
                    }

                    bm.clear();
                } else {//benchmark couldn't be set
                    cout << "Benchmark number " << i << " is incorrectly defined." << endl;
                }

                bm_node = bm_node.next_sibling("Benchmark");
            }

            cout << i << " benchmarkings were done" << endl;

        } else {//the file wasn't correctly parsed
            cout << "The benchmarking file couldn't be read." << endl;
        }
    } else {//the file doesn't exists
        fin.close();
        cout << "The benchmarking file doesn't exist. Please confirm it." << endl;
    }
}
