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


#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include "kauthamshell.h"
#include "libproblem/problem.h"
#include "libproblem/benchmark.h"
#include "libsplanner/libioc/kthquery.h"
#include "libsplanner/libioc/iocplanner.h"
#include <Inventor/SoDB.h>

#ifdef KAUTHAM_USE_MPI
#include <mpi.h>
#endif

using namespace std;

int main(int argc, char* argv[]){   

#ifdef KAUTHAM_USE_MPI
  // Initializing MPI
	int numProcs, myId;
	MPI_Status stat; 
	MPI_Init(&argc,&argv); // all MPI programs start with MPI_Init; all 'N' processes exist thereafter //
	MPI_Comm_size(MPI_COMM_WORLD,&numProcs); // find out how big the SPMD world is //
	MPI_Comm_rank(MPI_COMM_WORLD,&myId); // and this processes' rank is //
#endif


  if( (argc > 1 && string(argv[1]) == "-h") ||
      (argc != 3 && argc != 2) ||
      (argc > 1 && string(argv[1]) != "-h" && string(argv[1]) != "-s" && string(argv[1]) != "-b")){//print help info
    std::cout << "\nKautham console has been called with an invalid number of parameters "
      << "or you want to read this help. "
      << "The correct way to run this program is as follow:\n\n"
      << "If you want to benchmark OMPL planners:\n"
      << "\t KauthamConsole -b xml_benchmarking_file\n"
      << "where xml_benchmarking_file is a relative path of the benchamrk to be solved \n\n"
      << "Or if you want to benchmark IOC planners:\n"
      << "\t KauthamConsole -b xml_problem_file number_of_runs\n"
      << "where xml_problem_file is a relative path of the problem to be solved "
      << "and the number_of_runs is the integer number that the problem is trying to be "
      << "solved.\n\n"
      << "Or if you want to execute a single problem:\n"
      << "\t KauthamConsole -s xml_problem_file\n\n"
      << "Please take care with the relative paths where robots files and "
      << "scenes files are located relative to the problem file.\n\n";

    return 0;
  }

  //single problem execution (from shell)
  if(string(argv[1]) == "-s")
  {
      //=====================
      SoDB::init();
      string dir = argv[0];
      dir.erase(dir.find_last_of("/") + 1, dir.length());
      string absPath = dir;
      absPath.append( argv[2] );

      //directory containing the models
      dir=absPath;
      dir.erase(dir.find_last_of("/") + 1, dir.length());

      try{
          kauthamshell* ksh = new kauthamshell();

          //if(ksh->openProblem(absPath.c_str())==true)

          ifstream inputfile;
          inputfile.open(absPath.c_str());
          if(ksh->openProblem(&inputfile, dir.c_str())==true)
              cout << "The problem file has been loaded successfully.\n";
          else{
              cout << "The problem file has not been loaded successfully."
                   << "Please take care with the problem definition.\n";
              throw(1);
           }

            if(ksh->solve(std::cout)){
                cout << "The problem has been solve successfully.\n";
            }
            else{
              cout << "The problem has NOT been solve successfully.\n";
            }


      }catch(...){
        cout << "Something is wrong with the problem. Please run the "
          << "problem with the Kautham2 application al less once in order "
          << "to verify the correctness of the problem formulation.\n";
        return 1;
      }
      return 0;
  }
  //benchmarking
  else if(string(argv[1]) == "-b")
  {
    if (argc == 4) {//IOC planners benchmarking
      //=====================
      SoDB::init();

      string dir = argv[0];

      // If this is a windows executable, the path is unix format converted, changing  "\" for "/"
      size_t found;
      found = dir.find_first_of("\\");
      while (found != string::npos){
          dir[found]='/';
          found = dir.find_first_of("\\",found+1);
      }

      dir.erase(dir.find_last_of("/") + 1, dir.length());
      string absPath = dir;
      absPath.append( argv[2] );

      found = absPath.find_first_of("\\");
      while (found != string::npos){
          absPath[found]='/';
          found = absPath.find_first_of("\\",found+1);
      }

      string baseName = absPath;
      baseName.erase(0,baseName.find_last_of("/") + 1);
      baseName.erase(baseName.find_first_of("."), baseName.length());

      string soluFile = dir + baseName;
      soluFile.append("_solution_");
      std::cout << "Kautham is opening a problem file: " << absPath << endl;

      try{
          int tryTimes = atoi( argv[3] );
#ifdef KAUTHAM_USE_MPI 
          tryTimes /= numProcs;
#endif
          cout << "The problem will be trying to solve " << tryTimes << " times.\n" ;
          int badSol = 0;
          Problem* _problem = new Problem();
          stringstream ss;
          ss.str(soluFile);

          if( _problem->setupFromFile( absPath ) ){
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
      }catch(...){
          cout << "Something is wrong with the problem. Please run the "
               << "problem with the Kautham2 application al less once in order "
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
      SoDB::init();

      benchmark(string(argv[2]));

      return (0);
  }//end ompl benchmarking
 }//End benchmarking.
}

