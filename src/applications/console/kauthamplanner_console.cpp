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


#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include "kauthamshell.h"
#include <problem/problem.h>
#include "benchmark.h"
#include <planner/ioc/kthquery.h>
#include <planner/ioc/iocplanner.h>
#include <Inventor/SoDB.h>
#include "util/kthutil/kauthamexception.h"

#ifdef KAUTHAM_USE_MPI
#include <mpi.h>
#endif

using namespace std;

class membuf : public basic_streambuf<char> {
public:
    membuf(char* p, size_t n) {
        setg(p, p, p + n);
        setp(p, p + n);
    }
};


int main(int argc, char* argv[]){
#ifdef KAUTHAM_USE_MPI
    // Initializing MPI
    int numProcs, myId;
    MPI_Status stat;
    MPI_Init(&argc,&argv); // all MPI programs start with MPI_Init; all 'N' processes exist thereafter //
    MPI_Comm_size(MPI_COMM_WORLD,&numProcs); // find out how big the SPMD world is //
    MPI_Comm_rank(MPI_COMM_WORLD,&myId); // and this processes' rank is //
#endif


    if ((argc > 1 && string(argv[1]) == "-h") ||
            (argc < 3 || argc > 5) ||
            (argc == 3 && string(argv[1]) != "-t" && string(argv[1]) != "-s") ||
            (argc == 4 && string(argv[1]) != "-s" && string(argv[1]) != "-b") ||
            (argc == 5 && string(argv[1]) != "-b")){//print help info
        std::cout << "\nKautham console has been called with an invalid number of parameters "
                  << "or you want to read this help.\n"
                  << "The correct way to run this program is as follow:\n\n"
                  << "If you want to benchmark OMPL planners:\n"
                  << "\t KauthamConsole -b abs_path_xml_benchmarking_file [abs_path_models_folder]\n"
                  << "where xml_benchmarking_file is a relative path of the benchmark to be solved and "
                  << "models_folder is an optional parameter to specify where the models must be looked for.\n\n"
                  << "Or if you want to benchmark IOC planners:\n"
                  << "\t KauthamConsole -b abs_path_xml_problem_file number_of_runs [abs_path_models folder]\n"
                  << "where xml_problem_file is a relative path of the problem to be solved and "
                  << "the number_of_runs is the integer number that the problem is trying to be solved.\n\n"
                  << "Or if you want to test the kauthamshell utilities:\n"
                  << "\t KauthamConsole -t abs_path_of_models_folder\n\n"
                  << "Or if you want to execute a single problem:\n"
                  << "\t KauthamConsole -s abs_path_xml_problem_file [abs_path_of_models_folder]\n\n";

        return 0;
    }

    //single problem execution (from shell)
    //call: KauthamConsole -s abs_path_xml_problem_file [abs_path_of_models_folder]
    //e.g. KauthamConsole -s /home/jan.rosell/kautham.git/demos/OMPL_demos/2DRR/OMPL_RRT_2DRR_columns.xml /home/jan.rosell/kautham.git/demos/models
    if(string(argv[1]) == "-s")
    {
        //=====================
        SoDB::init();
        string absPath = argv[2];

        //directory containing the models
        vector <string> def_path;
        if (argc == 4) {
            def_path.push_back(argv[3]);
        }
        string dir = absPath.substr(0,absPath.find_last_of("/")+1);
        def_path.push_back(dir);
        def_path.push_back(dir+"../../models/");

        try{
            kauthamshell* ksh = new kauthamshell();

            ifstream inputfile;
            inputfile.open(absPath.c_str());
            if(ksh->openProblem(&inputfile, def_path)==true)
                cout << "The problem file has been loaded successfully.\n";
            else{
                cout << "The problem file has not been loaded successfully."
                     << "Please take care with the problem definition.\n";
                throw(1);
            }

            if(ksh->solve(std::cout)){
                cout << "The problem has been solved successfully.\n";
            }
            else{
                cout << "The problem has NOT been solved successfully.\n";
            }
        } catch (const KthExcp& excp) {
            cout << "Error: " << excp.what() << endl << excp.more() << endl;
            return 1;
        } catch (const exception& excp) {
            cout << "Error: " << excp.what() << endl;
            return 1;
        } catch(...) {
            cout << "Something is wrong with the problem. Please run the "
                 << "problem with the Kautham2 application at less once in order "
                 << "to verify the correctness of the problem formulation.\n";
            return 1;
        }
        return 0;
    }
    //test of kauthamshell options
    //call: KauthamConsole -t abs_path_of_models_folder
    //e.g. KauthamConsole -t /home/jan.rosell/kautham.git/demos/models/
    else if(string(argv[1]) == "-t")
    {
        //=====================
        SoDB::init();

        //directory containing the models
        vector <string> def_path;
        def_path.push_back(argv[2]);

        try{
            kauthamshell* ksh = new kauthamshell();

            //
            //Set the problem as a string.
            //Do not add control file, unless it is available in the models folder
            //If no control file is added then the default controls are defined
            //which are 3 for the translation of the base (if the X, Y, Z limits are defined), 3 for rotation of the base
            //plus the dof of each joint
            //Then, take care of the query dimensions!
            //
            cout << "\n TESTING THE SETTING OF A PROBLEM WITH A STREAM - no controls defined, thus default ones are generated.\n";
            string buffstring = "<?xml version=\"1.0\"?>\n"
                    "<Problem name=\"OMPL_RRTstar_2DRR_columns\">\n"
                    "    <Robot robot=\"robots/2DRR.dh\" scale=\"1.0\">\n"
                    "        <InvKinematic name=\"RR2D\"/>\n"
                    "    </Robot>\n"
                    "    <Obstacle obstacle=\"obstacles/columns.iv\" scale=\"0.5\">\n"
                    "        <Home TH=\"0.0\" WZ=\"1.0\" WY=\"0.0\" WX=\"0.0\" Z=\"0.0\" Y=\"50.0\" X=\"150.0\" />\n"
                    "    </Obstacle>\n"
                    "    <Planner>\n"
                    "      <Parameters>\n"
                    "            <Name>omplRRTStar</Name>\n"
                    "            <Parameter name=\"Max Planning Time\">1.0</Parameter>\n"
                    "            <Parameter name=\"Speed Factor\">1</Parameter>\n"
                    "            <Parameter name=\"Range\">0.05</Parameter>\n"
                    "            <Parameter name=\"Goal Bias\">0.05</Parameter>\n"
                    "            <Parameter name=\"Optimize none(0)/dist(1)/clear(2)/PMD(3)\">3</Parameter>\n"
                    "            <Parameter name=\"Simplify Solution\">1</Parameter>\n"
                    "        </Parameters>\n"
                    "        <Queries>\n"
                    "            <Query>\n"
                    "                <Init dim=\"5\">0.5 0.5 0.5 0.574 0.687</Init>\n"
                    "                <Goal dim=\"5\">0.5 0.5 0.5 0.431 0.69</Goal>\n"
                    "           </Query>\n"
                    "        </Queries>\n"
                    "    </Planner>\n"
                    "</Problem>\n";
            size_t size = buffstring.size();
            char *buffer = new char[size];
            for(int i=0; i<size;i++)
                buffer[i] = buffstring[i];

            membuf mb(buffer, size);
            istream reader(&mb);
            cout.write (buffer,size);

            if(ksh->openProblem(&reader, def_path)==true)
                cout << "The problem file has been loaded successfully.\n";
            else{
                cout << "The problem file has not been loaded successfully."
                     << "Please take care with the problem definition.\n";
                throw(1);
            }

            if(ksh->solve(std::cout))
                cout << "The problem has been solved successfully.\n";
            else
                cout << "The problem has NOT been solved successfully.\n";

            //TESTING THE SETTING OF A PROBLEM WITH A FILE
            cout << "\n TESTING THE SETTING OF A PROBLEM WITH A STREAM - controls defined with a control file (that exists in the models folder).\n";
            buffstring = "<?xml version=\"1.0\"?>\n"
                    "<Problem name=\"OMPL_RRTstar_2DRR_columns\">\n"
                    "    <Robot robot=\"robots/2DRR.dh\" scale=\"1.0\">\n"
                    "        <InvKinematic name=\"RR2D\"/>\n"
                    "    </Robot>\n"
                    "    <Obstacle obstacle=\"obstacles/columns.iv\" scale=\"0.5\">\n"
                    "        <Home TH=\"0.0\" WZ=\"1.0\" WY=\"0.0\" WX=\"0.0\" Z=\"0.0\" Y=\"50.0\" X=\"150.0\" />\n"
                    "    </Obstacle>\n"
                    "    <Controls robot=\"controls/2DRR/2DRR_2dof.cntr\" />\n"
                    "    <Planner>\n"
                    "      <Parameters>\n"
                    "            <Name>omplRRTStar</Name>\n"
                    "            <Parameter name=\"Max Planning Time\">1.0</Parameter>\n"
                    "            <Parameter name=\"Speed Factor\">1</Parameter>\n"
                    "            <Parameter name=\"Range\">0.05</Parameter>\n"
                    "            <Parameter name=\"Goal Bias\">0.05</Parameter>\n"
                    "            <Parameter name=\"Optimize none(0)/dist(1)/clear(2)/PMD(3)\">3</Parameter>\n"
                    "            <Parameter name=\"Simplify Solution\">1</Parameter>\n"
                    "        </Parameters>\n"
                    "        <Queries>\n"
                    "            <Query>\n"
                    "                <Init dim=\"2\">0.574 0.687</Init>\n"
                    "                <Goal dim=\"2\">0.431 0.69</Goal>\n"
                    "           </Query>\n"
                    "        </Queries>\n"
                    "    </Planner>\n"
                    "</Problem>\n";
            size_t size2 = buffstring.size();
            char *buffer2 = new char[size2];
            for(int i=0; i<size2;i++)
                buffer2[i] = buffstring[i];

            membuf mb2(buffer2, size2);
            istream reader2(&mb2);
            cout.write (buffer2,size2);

            if(ksh->openProblem(&reader2, def_path)==true)
                cout << "The problem file has been loaded successfully.\n";
            else{
                cout << "The problem file has not been loaded successfully."
                     << "Please take care with the problem definition.\n";
                throw(1);
            }

            if(ksh->solve(std::cout))
                cout << "The problem has been solved successfully.\n";
            else
                cout << "The problem has NOT been solved successfully.\n";

            //TESTING THE SETTING OF A NEW QUERY
            //kauthamshell::setQuery(vector<KthReal> init, vector<KthReal> goal)
            cout << "\n TESTING THE SETTING OF A NEW QUERY.\n";
            vector<KthReal> init;
            init.resize(2);
            init[0] = 0.3;
            init[1] = 0.3;
            vector<KthReal> goal;
            goal.resize(2);
            goal[0] = 0.7;
            goal[1] = 0.7;
            ksh->setQuery(init, goal);
            if(ksh->solve(std::cout))
                cout << "The problem has been solved successfully.\n";
            else
                cout << "The problem has NOT been solved successfully.\n";


            //TESTING THE SETTING OF NEW CONTROLS FROM FILE
            //bool setRobControls(istream* inputfile, vector<KthReal> init, vector<KthReal> goal);
            cout << "\n TESTING THE CHANGE OF ROBOT CONTROLS WITH A NEW FILE.\n";
            string new_control_file = string(argv[2])+"controls/2DRR/2DRR_2PMD.cntr";
            ifstream inputfile;
            inputfile.open(new_control_file.c_str());
            init.resize(3);
            init[0] = 0.5;
            init[1] = 0.3574;
            init[2] = 0.687;
            goal.resize(3);
            goal[0] = 0.5;
            goal[1] = 0.431;
            goal[2] = 0.69;
            ksh->setRobControls(&inputfile, init, goal);
            if(ksh->solve(std::cout))
                cout << "The problem has been solved successfully.\n";
            else
                cout << "The problem has NOT been solved successfully.\n";

            //TESTING THE SETTING OF NEW CONTROLS FROM STREAM
            cout << "\n TESTING THE SETTING OF NEW CONTROLS FROM STREAM.\n";
            buffstring = "<?xml version=\"1.0\"?>\n"
                    "<ControlSet>\n"
                    "  <Offset>\n"
                    "    <DOF name=\"2D-Robot/link1\" value=\"0.5\"></DOF>\n"
                    "    <DOF name=\"2D-Robot/link2\" value=\"0.5\"></DOF>\n"
                    "  </Offset>\n"
                    "  <Control name=\"PMD1\" eigValue=\"1.0\">\n"
                    "    <DOF name=\"2D-Robot/link1\" value=\"0.70710678118654752440084436210485\"/>\n"
                    "    <DOF name=\"2D-Robot/link2\" value=\"0.70710678118654752440084436210485\"/>\n"
                    "  </Control>\n"
                    "  <Control name=\"Joint1\" eigValue=\"1.0\">\n"
                    "    <DOF name=\"2D-Robot/link1\" value=\"1.0\"/>\n"
                    "  </Control>\n"
                    "  <Control name=\"Joint2\" eigValue=\"1.0\">\n"
                    "    <DOF name=\"2D-Robot/link2\" value=\"1.0\"/>\n"
                    "  </Control>\n"
                    "</ControlSet>\n";
            size_t size3 = buffstring.size();
            char *buffer3 = new char[size3];
            for(int i=0; i<size3;i++)
                buffer3[i] = buffstring[i];

            membuf mb3(buffer3, size3);
            istream reader3(&mb3);
            cout.write (buffer3,size3);

            init.resize(3);
            init[0] = 0.5;
            init[1] = 0.3574;
            init[2] = 0.687;
            goal.resize(3);
            goal[0] = 0.5;
            goal[1] = 0.431;
            goal[2] = 0.69;
            ksh->setRobControls(&reader3, init, goal);
            if(ksh->solve(std::cout))
                cout << "The problem has been solved successfully.\n";
            else
                cout << "The problem has NOT been solved successfully.\n";

            //TESTING THE SETTING OF NEW PLANNER FROM STREAM
            //Comment: changes the planner. Needs the tag Problem.
            //Does not process any query written inside the planner tag.
            cout << "\n TESTING THE SETTING OF NEW PLANNER FROM STREAM.\n";
            buffstring =
                    "<Problem name=\"OMPL_PRM_2DRR_columns\">\n"
                    " <Planner>\n"
                    "  <Parameters>\n"
                    "    <Name>omplPRM</Name>\n"
                    "    <Parameter name=\"Max Planning Time\">10.0</Parameter>\n"
                    "    <Parameter name=\"Speed Factor\">1</Parameter>\n"
                    "    <Parameter name=\"MaxNearestNeighbors\">10</Parameter>\n"
                    "    <Parameter name=\"DistanceThreshold\">1.0</Parameter>\n"
                    "  </Parameters>\n"
                    " </Planner>\n"
                    "</Problem>\n";

            size_t size4 = buffstring.size();
            char *buffer4 = new char[size4];
            for(int i=0; i<size4;i++)
                buffer4[i] = buffstring[i];

            membuf mb4(buffer4, size4);
            istream reader4(&mb4);
            cout.write (buffer4,size4);

            if(ksh->setPlanner(&reader4))
            {
                if(ksh->solve(std::cout))
                    cout << "The problem has been solved successfully.\n";
                else
                    cout << "The problem has NOT been solved successfully.\n";
            }
            else
                cout<<"New planner could not be loaded\n";
        } catch (const KthExcp& excp) {
            cout << "Error: " << excp.what() << endl << excp.more() << endl;
            return 1;
        } catch (const exception& excp) {
            cout << "Error: " << excp.what() << endl;
            return 1;
        } catch(...) {
            cout << "Something is wrong with the problem. Please run the "
                 << "problem with the Kautham2 application at less once in order "
                 << "to verify the correctness of the problem formulation.\n";
            return 1;
        }
        return 0;
    }
    //benchmarking
    else if(string(argv[1]) == "-b")
    {
        //IOC benchmarking
        //call: KauthamConsole -b abs_path_xml_problem_file number_of_runs [abs_path_models folder]
        if (argc>3 && atoi(argv[3]) != 0) {//IOC planners benchmarking
            //=====================
            SoDB::init();


            string soluFile = argv[2];
            soluFile.append("_solution_");
            std::cout << "Kautham is opening a problem file: " << argv[2] << endl;

            try{
                int tryTimes = atoi( argv[3] );
#ifdef KAUTHAM_USE_MPI 
                tryTimes /= numProcs;
#endif
                cout << "The problem will be tried to be solved " << tryTimes << " times.\n" ;
                int badSol = 0;
                Problem* _problem = new Problem();
                stringstream ss;
                ss.str(soluFile);


                string absPath = argv[2];

                //directory containing the models
                vector <string> def_path;
                if (argc == 5) {
                    def_path.push_back(argv[4]);
                }
                string dir = absPath.substr(0,absPath.find_last_of("/")+1);
                def_path.push_back(dir);
                def_path.push_back(dir+"../../models/");


                if( _problem->setupFromFile( absPath, def_path ) ){
                    cout << "The problem file has been loaded successfully.\n";
                    IOC::iocPlanner* _planner = (IOC::iocPlanner*)_problem->getPlanner();
                    SampleSet* _samples = _problem->getSampleSet();
                    unsigned int d =  _samples->getSampleAt(0)->getDim();
                    //vector<KthReal> init(d), goal(d);
                    vector<KthReal> init( _samples->getSampleAt(0)->getCoords());
                    vector<KthReal> goal( _samples->getSampleAt(1)->getCoords());
                    KthReal times[2]={0., 0.};
                    int sampCount[3]={0, 0, 0};
                    unsigned int checks=0;
                    unsigned int worldchecks=0;
                    for(int i = 0; i < tryTimes; i++){
                        _samples->clear();
                        Sample* smp = new Sample(d);
                        smp->setCoords( init );
                        _problem->wSpace()->collisionCheck( smp);
                        _samples->add( smp );
                        smp = new Sample(d);
                        smp->setCoords( goal );
                        _problem->wSpace()->collisionCheck( smp);
                        _samples->add( smp );
                        _planner->setInitSamp( _samples->getSampleAt(0) );
                        _planner->setGoalSamp( _samples->getSampleAt(1) );

                        if(_planner->solveAndInherit()){
                            IOC::KthQuery& tmp = _planner->getQueries().at( _planner->getQueries().size() - 1 );
                            times[0] += tmp.getTotalTime();
                            times[1] += tmp.getSmoothTime();
                            sampCount[0] += tmp.getGeneratedSamples();
                            sampCount[1] += tmp.getConnectedSamples();
                            sampCount[2] += tmp.getPath().size();
                            worldchecks += tmp.getWorldCollCheckCalls();
                            checks += tmp.getCollCheckCalls();
                            //ss << i << ".kps";
                            //_planner->saveData( soluFile.c_str() );
                        }else{
                            cout << "The problem has not been solve successfully.\n";
                            badSol++;
                            /*
          KthQuery& tmp = _planner->getQueries().at( _planner->getQueries().size() - 1 );
          times[0] += tmp.getTotalTime();
          times[1] += tmp.getSmoothTime();
          sampCount[0] += tmp.getGeneratedSamples();
          sampCount[1] += tmp.getConnectedSamples();
          sampCount[2] += tmp.getPath().size();
          checks += tmp.getCollCheckCalls();
*/
#ifndef KAUTHAM_USE_MPI
                            //if( badSol >= tryTimes / 5 )
                            //  throw(1);
#endif
                        }
                    }

#ifdef KAUTHAM_USE_MPI
                    // Now use the REDUCE MPI function to collect the data from the other active process.
                    // Reducing: buff2 will contain the sum of all buff
                    KthReal allTimes[2]={0., 0.};
                    int allSampCount[3]={0, 0, 0};
                    int allBadSol = 0;
                    if( typeid( KthReal ) == typeid( float ) )
                        MPI_Reduce(times, allTimes, 2, MPI_FLOAT, MPI_SUM, 0, MPI_COMM_WORLD);
                    else
                        MPI_Reduce(times, allTimes, 2, MPI_DOUBLE, MPI_SUM, 0, MPI_COMM_WORLD);

                    MPI_Reduce(sampCount, allSampCount, 3, MPI_INT, MPI_SUM, 0, MPI_COMM_WORLD);
                    MPI_Reduce(&badSol, &allBadSol, 1, MPI_INT, MPI_SUM, 0, MPI_COMM_WORLD);

                    //  Now it saves the data to a file in the root process or the unique one
                    //  if it doesn't use the MPI parallelization
                    //
                    if( myId == 0 ){ // it means the root process
                        // unifying the data between some parallel process and unique one.
                        tryTimes *= numProcs;
                        badSol = allBadSol;
                        for( int i = 0; i < 2; i++ ){
                            times[i] = allTimes[i];
                            sampCount[i] = allSampCount[i];
                        }
                        sampCount[2] = allSampCount[2];
#endif
                        //ofstream outputFile("stats.kth", ios::out|ios::trunc);
                        ofstream outputFile("stats.kth", ios::out|ios::app);

                        if( outputFile.is_open() ){
                            /*
          outputFile << "Problem solved:\t"     << absPath                << endl;
          outputFile << "TryTimes: \t"          << tryTimes               << endl;
          outputFile << "BadSolved: \t"         << badSol                 << endl;
          outputFile << "Total time: \t"        << times[0]/tryTimes      << endl;
          outputFile << "Smooth time: \t"       << times[1]/tryTimes      << endl;
          outputFile << "Tried samples: \t"     << sampCount[0]/(float) tryTimes  << endl;
          outputFile << "Connected samples: \t" << sampCount[1]/(float) tryTimes  << endl;
          outputFile << "Nodes in solution path: \t"   << sampCount[2]/(float) tryTimes  << endl;
          outputFile << "Collision-check calls: \t"   << checks/tryTimes << endl;
          */
                            outputFile << "Problem solved:\t"     << absPath                << endl;
                            outputFile << "NumExecutions SuccesRate Time Samples PRMnodes PathNodes WorldCollChecks TotalPQPCollChecks" << endl;
                            outputFile << tryTimes  << "\t";
                            //outputFile << badSol  << "\t";
                            outputFile << 100.0*(float)(tryTimes-badSol)/tryTimes  << "\t";
                            if(badSol!=tryTimes)
                            {
                                outputFile << times[0]/(float) (tryTimes-badSol)  << "\t";
                                outputFile << sampCount[0]/(float) (tryTimes-badSol)  << "\t";
                                outputFile << sampCount[1]/(float) (tryTimes-badSol)  << "\t";
                                outputFile << sampCount[2]/(float) (tryTimes-badSol)  << "\t";
                                outputFile << worldchecks/(float) (tryTimes-badSol)  <<  "\t";
                                outputFile << checks/(float) (tryTimes-badSol)  << endl;
                            }
                            else outputFile << endl;
                        }else           //there were any problems with the copying process
                            throw(1);

                        outputFile.close();

#ifdef KAUTHAM_USE_MPI
                    }
#endif

                }else{
                    cout << "The problem file has not been loaded successfully. "
                         << "Please take care with the problem definition.\n";
                    throw(1);
                }
                delete _problem;
            } catch (const KthExcp& excp) {
                cout << "Error: " << excp.what() << endl << excp.more() << endl;
#ifdef KAUTHAM_USE_MPI
                MPI_Finalize();
#endif
                return 1;
            } catch (const exception& excp) {
                cout << "Error: " << excp.what() << endl;
#ifdef KAUTHAM_USE_MPI
                MPI_Finalize();
#endif
                return 1;
            } catch(...) {
                cout << "Something is wrong with the problem. Please run the "
                     << "problem with the Kautham2 application at less once in order "
                     << "to verify the correctness of the problem formulation.\n";
#ifdef KAUTHAM_USE_MPI
                MPI_Finalize();
#endif
                return 1;
            }

#ifdef KAUTHAM_USE_MPI
            // Ending MPI
            MPI_Finalize();
#endif


            return 0;
        }//end ioc benchmarking
        else
        {//OMPL planners benchmarking
            //call: KauthamConsole -b abs_path_xml_benchmarking_file [abs_path_models_folder]
            SoDB::init();
            string absPath = argv[2];
            //directory containing the models
            vector <string> def_path;
            if (argc == 4) {
                def_path.push_back(argv[3]);
            }
            string dir = absPath.substr(0,absPath.find_last_of("/")+1);
            def_path.push_back(dir);
            def_path.push_back(dir+"../../models/");

            try {
                benchmark(string(argv[2]),def_path);
            } catch (const KthExcp& excp) {
                cout << "Error: " << excp.what() << endl << excp.more() << endl;
                return 1;
            } catch (const exception& excp) {
                cout << "Error: " << excp.what() << endl;
                return 1;
            } catch(...) {
                cout << "Something is wrong with the problem. Please run the "
                     << "problem with the Kautham2 application at less once in order "
                     << "to verify the correctness of the problem formulation.\n";
                return 1;
            }

            return (0);
        }//end ompl benchmarking
    }//End benchmarking.
}
