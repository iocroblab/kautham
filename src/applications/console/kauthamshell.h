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


 
#if !defined(_KAUTHAMSHELL_H)
#define _KAUTHAMSHELL_H
#include <problem/problem.h>

using namespace std;
using namespace Kautham;

namespace Kautham {

    class kauthamshell {

    public:
        inline kauthamshell(){_problem = NULL;}; //! Constructor
        inline ~kauthamshell(){}; //! Destructor
        inline void closeProblem(){delete _problem;};
        inline bool problemOpened(){return(_problem !=NULL);}//! Informs wether there is a problem opened;

        bool openProblem(ifstream* inputfile, string modelsfolder);
        bool openProblem(string problemfilename);
        bool checkCollision(vector<KthReal> smpcoords);
        void setRobotsConfig(vector<KthReal>);
        bool setQuery(vector<KthReal> init, vector<KthReal> goal);
        bool solve(ostream &graphVizPlannerDataFile);
        bool solve();

    private:
        Problem* _problem;
    };

}
#endif  //_KAUTHAMSHELL_H
