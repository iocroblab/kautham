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

#include <Inventor/SoDB.h>

#include <kautham/kauthamshell.h>
#include <kautham/problem/problem.h>
#include <kautham/util/kthutil/kauthamexception.h>

#include "benchmark.h"


using namespace std;

class membuf : public basic_streambuf<char> {
public:
    membuf(char* p, size_t n) {
        setg(p, p, p + n);
        setp(p, p + n);
    }
};


int main(int argc, char* argv[]){

    if ((argc > 1 && string(argv[1]) == "-h") ||
            (argc < 3 || argc > 4) ||
            (argc == 3 && string(argv[1]) != "-t" && string(argv[1]) != "-s" && string(argv[1]) != "-b") ||
            (argc == 4 && string(argv[1]) != "-s" && string(argv[1]) != "-b") ||
            (argc == 5 && string(argv[1]) != "-b")){//print help info
        std::cout << "\nKautham console has been called with an invalid number of parameters "
                  << "or you want to read this help.\n"
                  << "The correct way to run this program is as follow:\n\n"
                  << "If you want to benchmark OMPL planners:\n"
                  << "\t KauthamConsole -b abs_path_xml_benchmarking_file [abs_path_models_folder]\n"
                  << "where xml_benchmarking_file is a relative path of the benchmark to be solved and "
                  << "models_folder is an optional parameter to specify where the models must be looked for.\n\n"
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
            for(unsigned i=0; i<size;i++)
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
            for(unsigned i=0; i<size2;i++)
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
            for(unsigned i=0; i<size3;i++)
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
            for(unsigned i=0; i<size4;i++)
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
      //OMPL planners benchmarking
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
    }//End benchmarking.
}
