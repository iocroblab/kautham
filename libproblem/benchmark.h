/***************************************************************************
 *   Copyright (C) 2014 by Nestor Garcia Hidalgo                           *
 *   nestor.garcia.hidalgo@upc.edu                                         *
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


#include <iostream>
#include <fstream>
#include <pugixml.hpp>
#include <ompl/tools/benchmark/Benchmark.h>
#include "problem.h"
#include "libsplanner/libompl/omplplanner.h"


using namespace std;
using namespace pugi;
using namespace Kautham;


class Benchmark {
public:
    bool set(xml_node *bm_node, string dir);
    void run();
    bool save();
    void clear();
private:
    ompl::tools::Benchmark::Request *req;
    ompl::tools::Benchmark *bm;
    vector<Problem*> problem;
    string filename;
    string name;

    bool add_problem(string prob_file);
    bool add_parameter(xml_node *param_node);
};


void benchmark(string file);
