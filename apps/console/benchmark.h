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


#include <iostream>

#include <pugixml.hpp>

#include <ompl/tools/benchmark/Benchmark.h>

#include <kautham/problem/problem.h>


using namespace std;
using namespace pugi;


/** \addtogroup Application
 *  @{
 */


/*!
 * \brief The Benchmark class contains all data needed to run a benchmarking
 */
class Benchmark {
public:
    /*!
     * \brief initializes a benchmarking
     * \param bm_node benchmarking node where problems to be run and the
     benchmarking parameters are defined
     * \param dir relative path where problems files will be looked for
     * \param def_path is the set of folders where the models will be looked for
     * \return true if benchmarking setting succeed
     */
    bool set(xml_node *bm_node, string dir, vector<string> def_path);

    /*!
     * \brief runs the benchmarking previously set
     */
    void run();

    /*!
     * \brief saves the benchmarking results in the file specified when
     the benchmarking was set. If no filename was definied, default value will be used.
     * \return true if saving succeed
     */
    bool save();

    /*!
     * \brief clears all data created when the benchmarking was set
     */
    void clear();
private:
    /*!
     * \brief parameters for the execution of the benchmark
     *
     A request can be configured setting some of the next parameters in the benchmarking file:

     -maxTime: the maximum amount of time a planner is allowed to run (seconds); 5.0 by default.

     -maxMem: the maximum amount of memory a planner is allowed to use (MB); 4096.0 by default.

     -runCount: the number of times to run each planner; 100 by default.

     -timeBetweenUpdates: When collecting time-varying data from a planner during its execution,
     the planner's progress will be queried every timeBetweenUpdates seconds; 0.001 by default.

     -displayProgress: flag indicating whether progress is to be displayed or not; true by default.

     -saveConsoleOutput: flag indicating whether console output is saved
     (in an automatically generated filename); true by default.

     -useThreads: flag indicating whether planner runs should be run in a separate thread.
     It is advisable to set this to true, so that a crashing planner doesn't result in a crash
     of the benchmark program.
     */
    ompl::tools::Benchmark::Request *req;

    /*!
     * \brief set of planners on a problem instance
     */
    ompl::tools::Benchmark *bm;

    /*!
     * \brief list of problems to be run in the benchmarking
     */
    vector<Problem*> problem;

    /*!
     * \brief file where results will be saved
     *
     Default value is the current date and time
     */
    string filename;

    /*!
     * \brief name of the benchmarking
     */
    string name;

    /*!
     * \brief def_path is the set of folders where the models will be looked for
     */
    vector <string> def_path;

    /*!
     * \brief adds a problem to the benchmarking
     * \param prob_file problem description file of the problem to be added
     * \param def_path is the set of folders where the models will be looked for
     * \param planner_alias is an optional name for the planner
     * \return true if the problem could be added
     */
    bool add_problem(string prob_file, vector<string> def_path, string planner_alias = string());

    /*!
     * \brief sets a request parameter
     * \param param_node node where parameter's name and value are defined
     * \return true if the parameter could be added
     */
    bool add_parameter(xml_node *param_node);
};

/*!
 * \brief loads a the benchmarking to be run and tries to run them
 * \param file benchmarking file where all benchmarking to be run are defined
 * \param def_path is the set of folders where the models will be looked for
 */
void benchmark(string file, vector<string> def_path = vector<string>());


/** @}   end of Doxygen module "Application" */
