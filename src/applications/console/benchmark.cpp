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
        ok &= add_problem(dir + node.attribute("File").as_string(), def_path);

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
        xml_parse_result result = doc->load_file( prob_file.c_str() );
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
        string name = param_node->attribute("Name").as_string();
        if (name == "maxTime") {
            req->maxTime = param_node->attribute("Value").as_double();
            return true;
        } else if (name == "maxMem") {
            req->maxMem = param_node->attribute("Value").as_double();
            return true;
        } else if (name == "runCount") {
            req->runCount = (uint)param_node->attribute("Value").as_int();
            return true;
        } else if (name == "timeBetweenUpdates") {
            req->timeBetweenUpdates = param_node->attribute("Value").as_double();
            return true;
        } else if (name == "displayProgress") {
            req->displayProgress = param_node->attribute("Value").as_bool();
            return true;
        } else if (name == "saveConsoleOutput") {
            req->saveConsoleOutput = param_node->attribute("Value").as_bool();
            return true;
        } else if (name == "useThreads") {
            req->useThreads = param_node->attribute("Value").as_bool();
            return true;
        } else if (name == "filename") {
            filename = param_node->attribute("Value").as_string();
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
    if(fin.is_open()){//the file already exists
        fin.close();
        cout << "The file exists" << endl;

        xml_document doc;
        if (doc.load_file(file.c_str())) {//the file was correctly parsed
            cout << "The file was correctly parsed" << endl;

            int i = 0;
            Benchmark bm;

            xml_node bm_node = doc.child("Benchmark");
            while (bm_node) {
                i++;
                cout << "Reading benchmark number " << i << endl;

                if(bm.set(&bm_node, dir, def_path)) {//benchmark could be set
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
