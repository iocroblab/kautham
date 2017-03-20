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
#include <kautham/planner/ioc/HFplanner.h>


namespace Kautham {

  namespace IOC{
	//! Constructor
    HFPlanner::HFPlanner(SPACETYPE stype, Sample *init, Sample *goal, SampleSet *samples, WorkSpace *ws):
              gridPlanner(stype, init, goal, samples, ws)
	{
		//set intial values
	  
        _guiName = _idName =  "HFPlanner";
		removeParameter("Max. Samples");
		removeParameter("Step Size");
      
		_hfiter=10;
		_mainiter=10;
		addParameter("HF relax iter", _hfiter);
		addParameter("main iter", _mainiter);
		_dirichlet=0;
		addParameter("(1)dirichlet (0)neumann", _dirichlet);
		_goalPotential=-1000;
        addParameter("Goal potential", _goalPotential);

    }

	//! void destructor
	HFPlanner::~HFPlanner(){
			
	}
	
	//! setParameters sets the parameters of the planner
    bool HFPlanner::setParameters(){
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

		it = _parameters.find("Goal potential");
		if(it != _parameters.end())
          _goalPotential = it->second;
		else
          return false;
		
		it = _parameters.find("(1)dirichlet (0)neumann");
        if(it != _parameters.end()){
          _dirichlet = it->second;
          //reset HF values
          setRandValues();
        }
		else
          return false;

		it = _parameters.find("main iter");
		if(it != _parameters.end())
          _mainiter = it->second;
		else
          return false;

		it = _parameters.find("HF relax iter");
		if(it != _parameters.end())
          _hfiter = it->second;
		else
          return false;

        it = _parameters.find("num decimals");
        if(it != _parameters.end())
        {
          _decimals = it->second;
          if(_decimals<0) _decimals = 0;
          else if(_decimals>6) _decimals=6;
        }
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
                setRandValues();
                computeHF(indexgoal);
			}
			else
				return false;
		}


      }catch(...){
        return false;
      }
      return true;
    }

	

	void HFPlanner::computeHF(gridVertex  vgoal)
	{           
		//initialize potential to -1 and goal to 0
		setPotential(vgoal, _goalPotential);
		//relax potential
        int numneighs = _wkSpace->getNumRobControls()*2;
        boost::graph_traits<gridGraph>::vertex_iterator vi, vend;
        boost::graph_traits<gridGraph>::adjacency_iterator avi, avi_end;
		for(int i=0; i<_hfiter; i++)
		{
			for(tie(vi,vend)=vertices(*g); vi!=vend; ++vi)
			{
				if(getPotential(*vi) == _goalPotential ||
				   getPotential(*vi) == _obstaclePotential) continue;
				
				//cout << "cell "<<*vi<< " neighs = ";
				KthReal p=0;
				int count=0;
				int totalcount=0;
				for(tie(avi,avi_end)=adjacent_vertices(*vi, *g); avi!=avi_end; ++avi)
				{
					totalcount++;
					//cout<<*avi<<" ";
					if(_dirichlet != 0)
					{
						//dirichlet 
						count++;
						p+=getPotential(*avi);
					}
					else {
						//neumann
						//cout<<"("<<getPotential(*avi)<<") ";
						if(getPotential(*avi) != _obstaclePotential)
						{
							count++;
							p+=getPotential(*avi);
						}
					}
				}
				//the borders of the cspace fixed to high
				for(;totalcount<numneighs;totalcount++)
				{
					count++;
					p+=_obstaclePotential;
				}
				//cout<<" POT= "<<p<<"/"<<count<<"= "<<p/count<<endl;
				if(count) setPotential(*vi, p/count);
			}
		}
	}

	  
	//! function to find a solution path
		bool HFPlanner::trySolve()
		{
			//verify init and goal samples
			if(goalSamp()->isFree()==false || initSamp()->isFree()==false) 
			{
				cout<<"init or goal configuration are in COLLISION!"<<endl;
				drawCspace();
				return false;
			}

			//set init and goal vertices
            indexgoal = _samples->indexOf(goalSamp());
            indexinit = _samples->indexOf(initSamp());
			
            static int svg = -1;

            if (svg != int(indexgoal)) {
                //reset HF function because goal has changed
                svg = indexgoal;
                setRandValues();
            }

            gridVertex vc = indexinit;
			gridVertex vmin;

			_path.clear();
			clearSimulationPath();
            boost::graph_traits<gridGraph>::adjacency_iterator avi, avi_end;

			//relax HF
            computeHF(indexgoal);
			int count = 0;
			int countmax = _mainiter;

			std::vector<int> cellpath;
            cellpath.push_back(indexinit);
            _path.push_back(locations[indexinit]);

            while(vc != indexgoal && count < countmax)
			{
				vmin = vc;
				for(tie(avi,avi_end)=adjacent_vertices(vc, *g); avi!=avi_end; ++avi)
				{
					KthReal pneigh = getPotential(*avi);
					KthReal pcurr = getPotential(vmin);
					if(pneigh < pcurr) {
                        vmin = *avi;
					}
				}
                cellpath.push_back(vmin);
                _path.push_back(locations[vmin]);
				if(vc == vmin) {
					//relax HF again and resume
                    computeHF(indexgoal);
					_path.clear();
					cellpath.clear();
                    vc = indexinit;
					count++;
				}
				else vc = vmin;
			}
			if(count < countmax) _solved = true;
			else
			{
				_path.clear();
				cellpath.clear();
				_solved = false;
			}
			if(_solved)
			{
				cout<<"PATH:";
                for(unsigned i=0;i<cellpath.size();i++)
				{
					cout<<" "<<cellpath[i]<<"("<<getPotential(cellpath[i])<<"), ";
				}
				cout<<endl;
			}

			drawCspace();
			return _solved;
		}
      }
}


