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

 

#include <kautham/problem/ivworkspace.h>
#include <kautham/problem/workspace.h>
#include <kautham/sampling/sampling.h>
#include <kautham/planner/ioc/gridplanner.h>

#include <Inventor/nodes/SoCube.h>
#include <Inventor/nodes/SoText3.h>
#include <Inventor/nodes/SoMaterial.h>
#include <Inventor/nodes/SoScale.h>

using namespace boost;

namespace Kautham {



//! Namespace IOC contains the planners developed at the IOC
  namespace IOC{

    gridPlanner::gridPlanner(SPACETYPE stype, Sample *init, Sample *goal, SampleSet *samples, WorkSpace *ws):
              Planner(stype, init, goal, samples, ws)
	{
		//set intial values
		_obstaclePotential = 10.0;
		_goalPotential = 0.0;
        _showLabels = 1;
        _decimals = 0;

        //set intial values from parent class data
        _speedFactor = 1;
        _solved = false;

        _guiName = "Grid Planner";
        addParameter("Speed Factor", _speedFactor);
        addParameter("Show labels (0/1)", _showLabels);
        addParameter("num decimals", _decimals);

        //set step discretization
        _stepsDiscretization.resize(_wkSpace->getNumRobControls());
		char *str = new char[20];
		KthReal step=0;
        for(unsigned i=0;i<_wkSpace->getNumRobControls();i++){
			_stepsDiscretization[i] = 4;
			sprintf(str,"Discr. Steps %d",i);
			addParameter(str, _stepsDiscretization[i]);
			step+=_stepsDiscretization[i];
		}



        //set max samples
		_maxNumSamples = 1;
        for(unsigned i=0; i<_wkSpace->getNumRobControls();i++){
			_maxNumSamples = _maxNumSamples * _stepsDiscretization[i];
		}


        //store the init and goal config
        tmpSamInit = new Sample(_wkSpace->getNumRobControls());
        tmpSamGoal = new Sample(_wkSpace->getNumRobControls());
        tmpSamGoal->setCoords(goalSamp()->getCoords());
        tmpSamInit->setCoords(initSamp()->getCoords());
        tmpSamGoal->setFree(goalSamp()->getcolor());
        tmpSamInit->setFree(initSamp()->getcolor());

        _samples->clear();
		discretizeCspace();

        //recompute the init and goal to coincide with a cell of the grid
        int stepd=1;
        indexgoal= (gridVertex)(tmpSamGoal->getCoords()[0]*_stepsDiscretization[0]);
        indexinit= (gridVertex)(tmpSamInit->getCoords()[0]*_stepsDiscretization[0]);
        int r;
        for(unsigned i=1; i<_wkSpace->getNumRobControls();i++)
        {
            stepd = stepd * _stepsDiscretization[i-1];
            r=tmpSamGoal->getCoords()[i]*_stepsDiscretization[i];
            indexgoal += (gridVertex)(stepd*r);
            r=tmpSamInit->getCoords()[i]*_stepsDiscretization[i];
            indexinit += (gridVertex)(stepd*r);
        }
        setGoalSamp(_samples->getSampleAt(indexgoal));
        if (_wkSpace->collisionCheck(_goal.at(0)))
            cout<<"Goal sample is in collision"<<endl;
        else{
            cout<<"GOAL sample must be set to sample number "<< indexgoal <<endl;
            setGoalSamp(_samples->getSampleAt(indexgoal));
        }
        setInitSamp(_samples->getSampleAt(indexinit));
        if (_wkSpace->collisionCheck(_init.at(0)))
            cout<<"Init sample is in collision"<<endl;
        else{
            cout<<"INIT sample must be set to sample number "<< indexinit <<endl;
            setInitSamp(_samples->getSampleAt(indexinit));
        }

    }

	gridPlanner::~gridPlanner(){
			
	}


	SoSeparator *gridPlanner::getIvCspaceScene()
	{
        if(_wkSpace->getNumRobControls()==2)
		{
			//_sceneCspace = ((IVWorkSpace*)_wkSpace)->getIvScene();
			_sceneCspace = new SoSeparator();
            _sceneCspace->ref();
		}
		else _sceneCspace=NULL;
		return Planner::getIvCspaceScene();
		
	}


	void gridPlanner::drawCspace()
	{
		if(_sceneCspace==NULL) return;
        if(_wkSpace->getNumRobControls()==2)
		{
			//first delete whatever is already drawn
			while (_sceneCspace->getNumChildren() > 0)
			{
				_sceneCspace->removeChild(0);
			}

			KthReal cellside = 10.0;

			//draw cells
			SoSeparator *gsep = new SoSeparator();
			graph_traits<gridGraph>::vertex_iterator vi, vi_end;
			KthReal minPot=100000000.0;
			KthReal maxPot=-100000000.0;
			for(tie(vi,vi_end)=vertices(*g); vi!=vi_end; ++vi)
			{
				if(getPotential(*vi)<minPot) minPot=getPotential(*vi);
				if(getPotential(*vi)>maxPot) maxPot=getPotential(*vi);
			}
			KthReal rangePot = maxPot-minPot;
			for(tie(vi,vi_end)=vertices(*g); vi!=vi_end; ++vi)
			{
				SoSeparator *cellsep = new SoSeparator();
				SoCube *cellcube = new SoCube();
				cellcube->width = 0.99*cellside;
				cellcube->depth =  1 + 2*cellside*(getPotential(*vi) - minPot)/rangePot;
				cellcube->height = 0.99*cellside;

				KthReal x=cellside*_stepsDiscretization[0]*locations[*vi]->getCoords()[0];
				KthReal y=cellside*_stepsDiscretization[1]*locations[*vi]->getCoords()[1];
				KthReal z=(cellcube->depth.getValue())/2;

				SoTransform *cell_transf = new SoTransform;
				SbVec3f centre;
				centre.setValue(x,y,z);
				cell_transf->translation.setValue(centre);
				cell_transf->recenter(centre);	
			
				SoMaterial *cell_color = new SoMaterial;
				cell_color->diffuseColor.setValue(0.7,0.7,0.7);
				if(getPotential(*vi)!=minPot)
				{
					KthReal red = (getPotential(*vi) - minPot)/rangePot;
					KthReal green = 0.5;
					KthReal blue = 1-red;
					cell_color->diffuseColor.setValue(red,green,blue);
				}
				if(_solved)
				{
                    for(unsigned i=0;i<_path.size();i++)
					{
						if(_path[i]==locations[*vi])
							cell_color->diffuseColor.setValue(0.2,0.7,0.2);
					}
				}

				cellsep->addChild(cell_color);
				cellsep->addChild(cell_transf);
				cellsep->addChild(cellcube);

                if(_showLabels)
                {
                    //Add text to the cells wiht the potential value
                    //text position
                    SoTransform *text_transf = new SoTransform;
                    text_transf->translation.setValue(-cellside/4,-cellside/4,cellcube->depth.getValue());
                    //text scale
                    SoScale *textsc = new SoScale();
                    textsc->scaleFactor.setValue(0.2,0.2,1.0);
                    //text color
                    SoMaterial *text_color = new SoMaterial;
                    text_color->diffuseColor.setValue(0.2,0.2,0.2);
                    //text string
                    SoText3 *cell_text = new SoText3();
                    char textpot[20];
                    ostringstream s_decimals;
                    s_decimals << _decimals;
                    std::string s="%."+s_decimals.str()+"f";
                    sprintf(textpot,s.c_str(),getPotential(*vi));//e.g. s.c_str()="%.2f"
                    cell_text->string.setValue(textpot);
                    //text separator
                    SoSeparator *textsep = new SoSeparator();
                    textsep->addChild(text_transf);
                    textsep->addChild(textsc);
                    textsep->addChild(text_color);
                    textsep->addChild(cell_text);
                    cellsep->addChild(textsep);
                }
				gsep->addChild(cellsep);
			}
			_sceneCspace->addChild(gsep);
			//draw path
			/*
			if(_solved)
			{
				SoSeparator *pathsep = new SoSeparator();
				for(int i=0;i<_path.size();i++)
				{
					//cout<<" "<<vpath[i]<<"("<<getPotential(vpath[i])<<"), ";
					SoSeparator *cellpathsep = new SoSeparator();
					SoCube *cellpathcube = new SoCube();
					cellpathcube->width = 0.99*cellside;
					cellpathcube->depth =  1 + 2*cellside*(getPotential(*vi) - minPot)/rangePot;
					cellpathcube->height = 0.99*cellside;

					KthReal x=cellside*_stepsDiscretization[0]*_path[i]->getCoords()[0];
					KthReal y=cellside*_stepsDiscretization[1]*_path[i]->getCoords()[1];
					KthReal z=(cellpathcube->depth.getValue())/2;

					SoTransform *cellpath_transf = new SoTransform;
					SbVec3f centre;
					centre.setValue(x,y,z);
					cellpath_transf->translation.setValue(centre);
					cellpath_transf->recenter(centre);	
			
					SoMaterial *cellpath_color = new SoMaterial;
					cellpath_color->diffuseColor.setValue(0.2,0.7,0.2);
					

					cellpathsep->addChild(cellpath_color);
					cellpathsep->addChild(cellpath_transf);
					cellpathsep->addChild(cellpathcube);
					pathsep->addChild(cellpathsep);
				}
				_sceneCspace->addChild(pathsep);
			}
			*/

			//draw floor
			KthReal xmin=0.0;
			KthReal xmax=cellside*_stepsDiscretization[0];
			KthReal ymin=0.0;
			KthReal ymax=cellside*_stepsDiscretization[1];

			SoSeparator *floorsep = new SoSeparator();
			SoCube *cs = new SoCube();
			cs->width = xmax-xmin;
			cs->depth = (xmax-xmin)/50.0;
			cs->height = ymax-ymin;
			
			SoTransform *cub_transf = new SoTransform;
			SbVec3f centre;
			centre.setValue(xmin+(xmax-xmin)/2,ymin+(ymax-ymin)/2,-cs->depth.getValue());
			cub_transf->translation.setValue(centre);
			cub_transf->recenter(centre);	
			
			SoMaterial *cub_color = new SoMaterial;
			cub_color->diffuseColor.setValue(0.2,0.2,0.2);

			floorsep->addChild(cub_color);
			floorsep->addChild(cub_transf);
			floorsep->addChild(cs);
			_sceneCspace->addChild(floorsep);
		}
	}



	void gridPlanner::setStepsDiscretization(int numsteps, int axis)
	{
		//changes the discretization in a given axis
		if(_stepsDiscretization[axis]!=numsteps)
		{
			_stepsDiscretization[axis] = numsteps;

			_maxNumSamples = 1;
            for(unsigned i=0; i<_wkSpace->getNumRobControls();i++){
				_maxNumSamples = _maxNumSamples * _stepsDiscretization[i];
			}
			if(_isGraphSet) clearGraph();

           _samples->clear();
			discretizeCspace();

            //recompute the init and goal to coincide with a cell of the grid
            int stepd=1;
            indexgoal=(gridVertex)(tmpSamGoal->getCoords()[0]*_stepsDiscretization[0]);
            indexinit=(gridVertex)(tmpSamInit->getCoords()[0]*_stepsDiscretization[0]);
            int r;
            for(unsigned i=1; i<_wkSpace->getNumRobControls();i++)
            {
                stepd = stepd * _stepsDiscretization[i-1];
                r=tmpSamGoal->getCoords()[i]*_stepsDiscretization[i];
                indexgoal += (gridVertex)(stepd*r);
                r=tmpSamInit->getCoords()[i]*_stepsDiscretization[i];
                indexinit += (gridVertex)(stepd*r);
            }
            setGoalSamp(_samples->getSampleAt(indexgoal));
            if(_wkSpace->collisionCheck(_goal.at(0)))
                cout<<"Goal sample is in collision"<<endl;
            else{
                cout<<"GOAL sample must be set to sample number "<< indexgoal <<endl;
                setGoalSamp(_samples->getSampleAt(indexgoal));
            }

            setInitSamp(_samples->getSampleAt(indexinit));
            if(_wkSpace->collisionCheck(_init.at(0)))
                cout<<"Init sample is in collision"<<endl;
            else{
                cout<<"INIT sample must be set to sample number "<< indexinit <<endl;
                setInitSamp(_samples->getSampleAt(indexinit));
            }

        }

        drawCspace();
   }
		
    void  gridPlanner::loadgrid(vector<KthReal> &coords, unsigned coord_i)
	{
			KthReal delta = 1.0/_stepsDiscretization[coord_i];
			KthReal offset = delta/2;
			//for coordinate coord_i, loop for all discretization steps 
			for(int j=0; j<_stepsDiscretization[coord_i]; j++) 
			{
				//set the value of the coordinate corresponding to step j
				coords[coord_i] = offset + j*delta;
				//if not last coordinate, continue  
				//i.e. by means of a recursive call, all the coordinates are swept
				//until the last coordinate is reached
				if(coord_i != 0)   
				{
					loadgrid(coords, coord_i - 1);	 
				}
				//if coord_i is the last coordinate, then the coords of the cell are completed
				//and the sample created and collision-checked
				else
				{
                    Sample *smp = new Sample(_wkSpace->getNumRobControls());
					smp->setCoords(coords);
					_wkSpace->collisionCheck(smp);
					_samples->add(smp);
					/* Print INFO: collision-cehck nature of samples  
					cout<<"sample "<<_samples->getSize()<<": ";
                    for(int i=0;i<_wkSpace->getNumRobControls();i++) cout<<coords[i]<<", ";
					cout<<"SAMPLE COLOR = "<<smp->getcolor()<<endl;
					*/
				}
			}
		}


        void  gridPlanner::connectgrid(vector<int> &index, unsigned coord_i)
		{
			//for coordinate coord_i, loop for all discretization steps 
			for(int j=0; j<_stepsDiscretization[coord_i]; j++) 
			{
				index[coord_i] = j;
				//if not last coordinate, continue  
				//i.e. by means of a recursive call, all the coordinates are swept
				//until the last coordinate is reached
                if(coord_i != _wkSpace->getNumRobControls()-1)
				{
					connectgrid(index, coord_i + 1);	
				}
				//if coord_i is the last coordinate, then the indeices of the cell are completed
				//and the edges connecting it t its neighbors are computed
				else
				{
					//find sample label from indices
					int smplabel = 0;
					int coef;
                    for(unsigned k=0;k<_wkSpace->getNumRobControls();k++){
						if(k==0) coef=1;
						else coef = coef * _stepsDiscretization[k-1];
						smplabel += coef*index[k];
					}
					
					//sweep for all directions to find (Manhattan) neighbors
					//neighbors are looked for in the positive drection of the axis
					//i.e. incrementing the index of the current sample
                    for(unsigned n=0;n<_wkSpace->getNumRobControls();n++)
					{
						//find the label of the neighbor samples from indices
						//a Manhattan neighbor (in the positive direction of an axis)
						//has all the indices equal, minus one that has 
						//the value of the index incremented by one
						int smplabelneighplus = 0;
						bool plusneighexists = true;
						
                        for(unsigned k=0;k<_wkSpace->getNumRobControls();k++)
						{
							if(k==0) coef=1;
							else coef = coef*_stepsDiscretization[k-1];
							
							if(k==n) 
							{
								if(index[k]+1 >= _stepsDiscretization[k]) plusneighexists = false;
								smplabelneighplus  += coef*(index[k]+1);
							}
							else
							{
								smplabelneighplus  += coef*index[k];
							}
						}
						//connect samples (if neighbor sample did exist)
						if(plusneighexists==true)
						{
							//edges are defined as pairs of ints indicating the label of the vertices
							gridEdge *e;
							e = new gridEdge(smplabel, smplabelneighplus);
							edges.push_back(e);
							//put a weight to the edge, depending on the collison nature of the samples
							//edges linking  at least one collison sample are weighted -1
							//edges linking free samples are weighted +1
							if(_samples->getSampleAt(smplabel)->isFree()==false || 
							   _samples->getSampleAt(smplabelneighplus)->isFree()==false) 
								weights.push_back(-1.0);
							else 
								weights.push_back(1.0);
						}
					}
				}
			}


		}


		
		void  gridPlanner::prunegrid()
		{
			//the filtered graph is obtained using the negative_edge_weight function
			//that filters out edges with a negative weight
			negative_edge_weight<WeightMap> filter(get(edge_weight, *g));

			fg = new filteredGridGraph(*g, filter);
		}

		void  gridPlanner::discretizeCspace()
		{
			//create graph vertices, i.e. sample and collision-check at grid cell centers
            vector<KthReal> coords(_wkSpace->getNumRobControls());
            loadgrid(coords, _wkSpace->getNumRobControls()-1);
            //connect neighbor grid cells
            vector<int> index(_wkSpace->getNumRobControls());
			connectgrid(index, 0);
			//create grid as graph (alse sets initial potential values)
			loadGraph();

			//Print INFO: edges of graph
			/*graph_traits<gridGraph>::edge_iterator i, end;
			for(tie(i,end)=boost::edges(*g); i!=end; ++i)
			{
				gridVertex s=source(*i,*g);
				gridVertex t=target(*i,*g);
				cout<<"edge "<<*i<< " from "<< s<< " to "<<t<<endl;
			}
			*/
			

			/*Print INFO: verification intial values of potential 
			cout<<"POTENTIAL:";
			for(unsigned int i=0;i<num_vertices(*g); i++) 
				cout<<getPotential(i)<<", ";
			cout<<endl;
			cout<<endl;
			*/

			//compute the filtered graph that do not consider edges with
			//negative weight (those connecting to a collision vertex)
			prunegrid();

			//Print INFO: edges of filtered graph
			/*
			graph_traits<filteredGridGraph>::edge_iterator i, end;
			for(tie(i,end)=boost::edges(*fg); i!=end; ++i)
			{
				gridVertex s=source(*i,*fg);
				gridVertex t=target(*i,*fg);
				cout<<"edge "<<*i<< " from "<< s<< " to "<<t<<endl;
			}
			*/
		}


    void gridPlanner::clearGraph()
	{
  		weights.clear();
        edges.clear();
	    if(_isGraphSet){
		    locations.clear();
		    delete g;
		    delete fg;
	    }
	    _isGraphSet = false;
		_solved = false;
    }

    void gridPlanner::setRandValues()
    {
        LCPRNG* rgen = new LCPRNG(3141592621, 1, 0, ((unsigned int)time(NULL) & 0xfffffffe) + 1);//LCPRNG(15485341);//15485341 is a big prime number
        for(unsigned int i=0;i<num_vertices(*g); i++)
        {
            if(locations[i]->isFree()==true)
            {
                setPotential(i, rgen->d_rand()); //random initialization of potential
            }
            else setPotential(i, _obstaclePotential);
        }
  }

  
    void gridPlanner::loadGraph()
	{
	    int maxNodes = this->_samples->getSize();
		unsigned int num_edges = edges.size(); 
      
		// create graph with maxNodes nodes
		g = new gridGraph(maxNodes);
		WeightMap weightmap = get(edge_weight, *g);

		//load edges stored in edges array, and put the
		//corresponding weight, stored in the weights array
		for(std::size_t j = 0; j < num_edges; ++j) 
		{
			edge_descriptor e; 
			bool inserted;//when the efge already exisits or is a self-loop
						  //then this flag is set to false and the edge is not inserted 
			tie(e, inserted) = add_edge(edges[j]->first,edges[j]->second, *g);
			if(inserted) weightmap[e] = weights[j];
		}

		//locations are the pointer to the samples of cspace that 
		//are at each node of the graph 
		for(unsigned int i=0;i<num_vertices(*g); i++)
		    locations.push_back( _samples->getSampleAt(i) );

		//set initial potential vañues with random numbers in the range [0,1]
		//if the cell is free, or to value 10 if it is in collsion
		potmap = get(potential_value_t(), *g);
		
        setRandValues();

		_isGraphSet = true;
	}

  }
}


