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



#include <kautham/problem/workspace.h>
#include <kautham/sampling/sampling.h>
#include <kautham/planner/ioc/NF1planner.h>


namespace Kautham {
  namespace IOC{
	//! Constructor
    NF1Planner::NF1Planner(SPACETYPE stype, Sample *init, Sample *goal, SampleSet *samples, WorkSpace *ws):
              gridPlanner(stype, init, goal, samples, ws)
	{
		//set intial values
	  
        _guiName = _idName =  "NF1Planner";
		removeParameter("Max. Samples");
		removeParameter("Step Size");

    }

	//! void destructor
	NF1Planner::~NF1Planner(){
			
	}
	
	//! setParameters sets the parameters of the planner
    bool NF1Planner::setParameters(){
      try{
        HASH_S_K::iterator it = _parameters.find("Speed Factor");
        if(it != _parameters.end())
          _speedFactor = it->second;
        else
          return false;

        it = _parameters.find("Show labels (0/1)");
        if(it != _parameters.end())
          _showLabels = it->second;
        else
          return false;

		char *str = new char[20];
        for(unsigned i=0; i<_wkSpace->getNumRobControls();i++)
		{
			sprintf(str,"Discr. Steps %d",i);
			it = _parameters.find(str);
			if(it != _parameters.end())
			{
				setStepsDiscretization(it->second,i);
                //recompute NF1 fucntion because grid has changed
                computeNF1(indexgoal);
			}
			else
				return false;
		}


      }catch(...){
        return false;
      }
      return true;
    }

	

	void NF1Planner::computeNF1(gridVertex  vgoal)
	{

        boost::graph_traits<gridGraph>::vertex_iterator vi, vi_end;

		/* Prints adjacencies 

		graph_traits<gridGraph>::adjacency_iterator avi, avi_end;
		for(tie(vi,vi_end)=vertices(*g); vi!=vi_end; ++vi)
		{
			cout<<*vi<< " adjacent to: ";
			for(tie(avi,avi_end)=adjacent_vertices(*vi, *g); avi!=avi_end; ++avi)
			{
				cout<<*avi<<" ,";
			}
			cout<<endl;
		}
		*/

		//initialize potential to -1 and goal to 0
		for(tie(vi,vi_end)=vertices(*g); vi!=vi_end; ++vi)	setPotential(*vi, -1);
		setPotential(vgoal, 0);
		//propagate potential
		breadth_first_search(*fg, vgoal, visitor(bfs_distance_visitor<PotentialMap>(getpotmat())));


        //print dist info
        /*
         *graph_traits<filteredGridGraph>::vertex_iterator i, end;
		for(tie(i,end)=vertices(*fg); i!=end; ++i)
		{
			cout<<"vertex "<< *i<<" dist "<<getPotential(*i)<<endl;
		}
        */
	}

	  
	//! function to find a solution path
		bool NF1Planner::trySolve()
		{
			//cout << "MyGridPlanner::trySolve - now a simple rectilinear connection without collision checking..."<<endl<<flush;

			if(goalSamp()->isFree()==false || initSamp()->isFree()==false) 
			{
				cout<<"init or goal configuration are in COLLISION!"<<endl;
				return false;
			}

            indexgoal = _samples->indexOf(goalSamp());
            indexinit = _samples->indexOf(initSamp());

			static int svg = -1;
			
            if (svg != int(indexgoal)){
				//recompute navigation function because goal has changed
                svg = indexgoal;
                computeNF1(indexgoal);
			}
			
            gridVertex vc = indexinit;
			gridVertex vmin;

			
			PotentialMap pm = getpotmat();

			//if navigation function did'nt arrive at initial cell, return false
            if(pm[indexinit] == -1)
			{
				cout<<"CONNECTION NOT POSSIBLE: Init and goal configurations not on the same connected component..."<<endl;
				drawCspace();
				return false;
			}

			//otherwise follow the negated values
			_path.clear();
			clearSimulationPath();
            boost::graph_traits<filteredGridGraph>::adjacency_iterator avi, avi_end;

            while(vc != indexgoal)
			{
				_path.push_back(locations[vc]);
				vmin = vc;
				for(tie(avi,avi_end)=adjacent_vertices(vc, *fg); avi!=avi_end; ++avi)
				{
					KthReal pneigh = pm[*avi];
					KthReal pcurr = pm[vmin];
					if(pneigh < pcurr) vmin = *avi; 
					//if(pm[*avi] < pm[vmin]) vmin = *avi; 
                }
                //init control code:
                //the following code should never be executed. NF1 is suposed not to have local minima!
                //something wrong might be happening...
                if(vc==vmin)
                {
                    cout<<"something wrong is happening. NF1 encontered a local minima ¿?"<<endl;
                    _solved = false;
                    return _solved;
                }
                //end control code


                vc = vmin;
			}
            _path.push_back(locations[indexgoal]);
			_solved = true;
			drawCspace();
			return _solved;
			
		}
      }
}


