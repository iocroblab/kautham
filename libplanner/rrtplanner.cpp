#include <libproblem/workspace.h>
#include <libsampling/sampling.h>
#include <boost/graph/astar_search.hpp>
#include <boost/graph/adjacency_list.hpp>
#include "localplanner.h"
#include "rrtplanner.h"
#include <stdio.h>
//#include <libGUI/sampleswidget.h>

#include <libsampling/lcprng.h>

namespace libPlanner{
    namespace RRT{
      RRTPlanner::RRTPlanner(SPACETYPE stype, Sample *init, Sample *goal, SampleSet *samples, Sampler *sampler,
                               WorkSpace *ws, LocalPlanner *lcPlan, KthReal ssize)
                                   :Planner(stype, init, goal, samples, sampler, ws, lcPlan, ssize){
			
			  _wkSpace = ws;			
			  _neighThress = 50.0;
        _kNeighs = 1;
        _isGraphSet = false;
        _maxNumSamples = 1000;
        _speedFactor = 1;
			  _extDist = 5;
        _ssize=ssize;
        //ssize=0.01;

        _solved = false;
			  setStepSize(0.01);//also changes stepsize of localplanner

        _idName = _guiName = "RRT";
        addParameter("DELTA", ssize);
			  //removeParameter("Max. Neighs");
        addParameter("SAMPLES", _maxNumSamples);
        addParameter("SPEED", _speedFactor);
			  addParameter("EXTEND_DISTANCE", _extDist); // how far the smp_new is taken

        _labelCC=0;

        _samples->setTypeSearch(ANNMETHOD);//(BRUTEFORCE);//
        _samples->setWorkspacePtr(_wkSpace);
        _samples->setANNdatastructures(_kNeighs, _maxNumSamples);

        }

        RRTPlanner::~RRTPlanner(){
        }

		// Set the GUI parameters 
        bool RRTPlanner::setParameters(){
            try{
                HASH_S_K::iterator it = _parameters.find("DELTA");
                if(it != _parameters.end()){
                    setStepSize(it->second);//also changes stpssize of localplanner
                    _ssize=it->second;
                }
                else
                    return false;
                it = _parameters.find("SPEED");
                if(it != _parameters.end())
                    _speedFactor = it->second;
                else
                    return false;
                it = _parameters.find("SAMPLES");
                if(it != _parameters.end()){
                    _maxNumSamples = it->second;
                    _samples->setANNdatastructures(_kNeighs, _maxNumSamples);
                }else
                    return false;
                //it = _parameters.find("Neigh Thresshold");
                //if(it != _parameters.end())
                //    _neighThress = it->second;
                //else
                //    return false;
                it = _parameters.find("Max. Neighs");
                if(it != _parameters.end()){
                    _kNeighs = (int)it->second;
                    _samples->setANNdatastructures(_kNeighs, _maxNumSamples);
                }else
                    return false;
				it = _parameters.find("EXTEND_DISTANCE");
                if(it != _parameters.end())
                    _extDist = it->second;
                else
                    return false;
            }catch(...){
                return false;
            }
            return true;
        }

		
		// TODO: how to save the data
        void RRTPlanner::saveData(){
   //         //cout << "RRTPlanner::saveData not implemented"<<endl<<flush;
			//float Distinitgoal = _init->getDistance(_goal,BRONCOSPACE);
			//cout<<" Distinitgoal: "<<Distinitgoal<<endl<<flush;
        }


		//! The actual planner 
		bool RRTPlanner::trySolve(){

      // Sample a new sample
      Sample* smp_rand = NULL;//_sampler->nextSample();
			Sample* smp_near = NULL;
			unsigned int idx_near;
      Sample* smp_new = NULL;
			//int idx_new;
			Sample* smp_inter = NULL;

			KthReal dist2goal=_extDist+1;
			//float dist2goal_0; it's not necessary a comparatory dist. Distance to goal will be always done with the last sample (smp_new) taken: 
			//if the algorithm has not stopped so far, it means that the distance has not been small enough to connect with the goal.

      unsigned int hms=0;
      bool nearGoalConnect=false;

			
      if ((_init==NULL)||(_goal==NULL)){
        cout << "A problem occured, maybe init and goal configuration you set does not exist anymore. \n"; 
        return false;
      }

			// delete all the elements from the sampleList and insert just the _init
			if( _samples->getSize() > 0){
				char sampleDIM = _wkSpace->getDimension();
				SmpInit = new Sample(sampleDIM);
				SmpInit->setCoords(_init->getCoords());
				//_wkSpace->collisionCheck(SmpInit);
        SmpInit->setMappedConf(_wkSpace->getConfigMapping(_init));
				SmpGoal = new Sample(sampleDIM);
				SmpGoal->setCoords(_goal->getCoords());
				//_wkSpace->collisionCheck(SmpGoal);
        SmpGoal->setMappedConf(_wkSpace->getConfigMapping(_goal));
				_samples->clear();
				_init=SmpInit;
				_goal=SmpGoal;
				if(typeid(*_sampler) == typeid(SDK::GaussianLikeSampler))
        {
          int i=9;
          ((SDK::GaussianLikeSampler*)_sampler)->clear();
				}
				_samples->add(SmpInit);
				if(_samples->isAnnSet()) _samples->loadAnnData();
			}
			else
			{
				return false;
			}
			//libGUI::SamplesWidget::updateSampleList();
			
			clearGraph();

			//// Assuming that the goal sample is always free of collisions
   //   _locPlanner->setInitSamp(SmpInit);
   //   _locPlanner->setGoalSamp(SmpGoal);
   //   
   //   if ((SmpInit->getDistance(SmpGoal,CONFIGSPACE))<_extDist && _locPlanner->canConect()){ //If goal is already inside the accepted radius 
   //     _solved=true;
   //     _samples->add(SmpGoal);
   //     SmpGoal->addNeigh(_samples->indexOf(_init));
   //    	connect2samples(_init,SmpGoal,SmpGoal->getDistance(SmpInit,CONFIGSPACE));
   //      if (findPath()) {
   //         cout << "The path size is " << _path.size() << "\n" ;
   //         cout << "The number of samples used is " << _samples->getSize() << "\n" ;
   //         return true;
   //       }
			//	  else 
			//		  return false;
   //   }
   //   else{
        do{
          if (hms%3==0) // every 3 sample the Goal is used to grow the tree.
            smp_rand=SmpGoal;
          else
            smp_rand = _sampler->nextSample();

          cout << "Total number of samples " << ++hms <<"\n"; 
          
          // Search the nearest sample in the existing RRT graph
				  vector<unsigned int> * ann_neigh=_samples->findAnnNeighs(smp_rand, 1000000);
				  idx_near=ann_neigh->at(0);
				  smp_near=_samples->getSampleAt(idx_near);

				  // Control if the distance between smp_rand and smp_near is < _extDist 
				  KthReal dist=smp_rand->getDistance(smp_near,CONFIGSPACE);

				  if (dist<=_extDist){
            _locPlanner->setInitSamp(smp_near);
            _locPlanner->setGoalSamp(smp_rand);
            if(!_wkSpace->collisionCheck(smp_rand) && _locPlanner->canConect()){
						  smp_new=smp_rand;
						  _samples->add(smp_new);
						  //idx_new=_samples->indexOf(smp_new);
              smp_new->clearNeighs();
						  smp_new->addNeigh(idx_near); // considering the father as the neighbour so as to do backtracking when building the path.
						  connect2samples(smp_near,smp_new,dist);	// building the edge
              dist2goal=(_samples->getSampleAt((_samples->getSize())-1))->getDistance(SmpGoal,CONFIGSPACE);
              if (dist2goal==0) nearGoalConnect=true;
					  }
					  else if (smp_rand!=SmpGoal)
						  delete smp_rand; 
            else
               smp_rand=NULL;

				  }
				  else { // if it is outside the radius, an interpolation must be done (from smp_near till smp_near+_extDist)
					  _wkSpace->collisionCheck(smp_rand);
            smp_inter=smp_near->interpolate(smp_rand,_extDist/dist);
            _locPlanner->setInitSamp(smp_near);
            _locPlanner->setGoalSamp(smp_inter);
            if(!_wkSpace->collisionCheck(smp_inter) && _locPlanner->canConect()){
						  smp_new=smp_inter;
						  _samples->add(smp_new);
						  //idx_new=_samples->indexOf(smp_new);
              smp_new->clearNeighs();
              smp_new->addNeigh(idx_near); // considering the father as the neighbour so as to do backtracking when building the path.
              connect2samples(smp_near,smp_new,_extDist); // building the edge 
              dist2goal=(_samples->getSampleAt((_samples->getSize())-1))->getDistance(SmpGoal,CONFIGSPACE);
              if (dist2goal==0) nearGoalConnect=true;
            }
            else {
						  if (smp_rand!=SmpGoal) 
                delete smp_rand; 
              else
                smp_rand=NULL;
						  delete smp_inter; // smp_inter=NULL;
					  } 
				  }
				
          if (_samples->changed() && dist2goal<_extDist && !nearGoalConnect) {
            _locPlanner->setInitSamp((_samples->getSampleAt((_samples->getSize())-1)));
            _locPlanner->setGoalSamp(SmpGoal);
            if (_locPlanner->canConect()) 
              nearGoalConnect=true;
          }

			  }while(((_samples->getSize())<_maxNumSamples)&&(!nearGoalConnect));

        if (nearGoalConnect){ 
          _solved=true;
          if (dist2goal!=0){
            SmpGoal->clearNeighs();
            SmpGoal->addNeigh(_samples->indexOf(smp_new));
       		  connect2samples(smp_new,SmpGoal,dist2goal);
            _samples->add(SmpGoal);
          }
          if (findPath()) {
            cout << "The path size is " << _path.size() << "\n" ;
            cout << "The number of samples used is " << _samples->getSize() << "\n" ;
            return true;
          }
				  else 
					  return false;
        } //  if (near Goal Connect) 
        else {
				  cout << "The RRT reaches the maximum number of samples and did not reach the goal";
				  return false;
        }
      //} // closing if - else of "smp_goal inside the accepted radius"
      }//trySolve

		
		//! add an edge to the graph between two samples in _samples
		void RRTPlanner::connect2samples(Sample* sampleA, Sample* sampleB, KthReal dist)
		{
			rrtEdge *e;
			e= new rrtEdge(_samples->indexOf(sampleA),_samples->indexOf(sampleB));
			edges.push_back(e);
			weights.push_back(dist);
		}

		bool RRTPlanner::findPath(){
//      clearSimulationPath();
			Sample* nextPathSmp;
			//_solved = false;
 			_path.clear();
			_path.push_back(SmpGoal);
      //_simulationPath.push_back(SmpGoal);
			Sample * currentSmp=_goal;
			do {
				nextPathSmp=_samples->getSampleAt((currentSmp->getNeighs())->at(0));
				_path.push_back(nextPathSmp);
        //_simulationPath.push_back(nextPathSmp);
				currentSmp=nextPathSmp;
			//}while((currentSample!=_init) && ((currentSmp->getNeighs())->at(0)!=_samples->indexOf(_init))); 
    /*_path.push_back(_init);
      _simulationPath.push_back(_init);*/
      }while(currentSmp!=_init); // when the _init sample has already been inserted.
      
      return true;
		}

     
    //!Delete the graph g
    void RRTPlanner::clearGraph(){
            weights.clear();
            edges.clear();
            _samples->clearNeighs();
            if(_isGraphSet){
                locations.clear();
                delete g;
            }
            _isGraphSet = false;
            _solved = false;
            _labelCC = 0;
            _ccMap.clear();
            for(int i=0;i<_samples->getSize();i++)
                _samples->getSampleAt(i)->setConnectedComponent(-1);
        }

        //!Print connected components
        void RRTPlanner::printConnectedComponents(){
			//cout << "RRTPlanner::printConnectedComponents not implemented"<<endl<<flush;
        }

        //!Smooths the path.
        //!If maintainfirst is set then the first edge is maintained in the smoothed path
        //!If maintainlast is set then the last edge is maintained in the smoothed path
        //! They are both initialized to false
        void RRTPlanner::smoothPath(bool maintainfirst, bool maintainlast){
            if(!_solved){
                cout<<"Cannot smooth path - path is not yet solved"<<endl;
                return;
            }
            //START CREATING AN AUXILIAR GRAPH
            //Create a graph with all the samples but with edges connecting those samples
            //of the original path (if collision-free)
            int maxNodesPath = _path.size();
            vector<rrtEdge*> edgesPath;
            vector<cost> weightsPath;

            int last;
            if(maintainlast==true) last=maxNodesPath-1;//do not try to connect other nodes to the final
            else last=maxNodesPath;

            for(int i=0;i<maxNodesPath-1;i++){
                //connect node i with the next sample (i+1) in path (known to be connectable)
                rrtEdge *e = new rrtEdge(_samples->indexOf(_path[i]),_samples->indexOf(_path[i+1]));
                edgesPath.push_back(e);
                weightsPath.push_back(_locPlanner->distance(_path[i],_path[i+1]));
                //try to connect with the other samples in the path
                if(i==0 && maintainfirst==true) continue; //do not try to connect initial node with others
                for(int n=i+2;n<last;n++){
                    _locPlanner->setInitSamp(_path[i]);
                    _locPlanner->setGoalSamp(_path[n]);
                    if(_locPlanner->canConect()){
                        rrtEdge *e = new rrtEdge(_samples->indexOf(_path[i]),_samples->indexOf(_path[n]));
                        edgesPath.push_back(e);
                        weightsPath.push_back(_locPlanner->distance(_path[i],_path[n]));
                    }
                }
            }

            // create graph
            rrtGraph *gPath = new rrtGraph(_samples->getSize());
            WeightMap weightmapPath = get(edge_weight, *gPath);

            for(std::size_t j = 0; j < edgesPath.size(); ++j){
                edge_descriptor e;
                bool inserted;
                tie(e, inserted) = add_edge(edgesPath[j]->first, edgesPath[j]->second, *gPath);
                weightmapPath[e] = weightsPath[j];
            }
            //END CREATING AN AUXILIAR GRAPH

            //START FINDING A SMOOTHER PATH
            //Find path from cini to cgoal along the nodes of gPath
            rrtVertex start = shortest_path.front();
            rrtVertex goal = shortest_path.back();

            //Now clear the unsmoothed solution
            shortest_path.clear(); //path as a vector of rrtvertex
            _path.clear();//path as a vector of samples

            //vector to store the parent information of each vertex
            vector<rrtGraph::vertex_descriptor> p(num_vertices(*gPath));
            //vector with cost of reaching each vertex
            vector<cost> d(num_vertices(*gPath));

            try {
                // call astar named parameter interface
                astar_search(*gPath, start,
                             distance_heuristic<rrtGraph, cost, vector<location> >(locations, goal, _locPlanner),
                             predecessor_map(&p[0]).distance_map(&d[0]).
                             visitor(astar_goal_visitor<rrtVertex>(goal)));

            }catch(RRT::found_goal fg) { // found a path to the goal
                //list<rrtVertex> shortest_path; now is a class parameter
                for( rrtVertex v = goal; ; v = p[v] ){
                    shortest_path.push_front(v);
                    if( p[v] == v ) break;
                }
                cout << "Smoothed path from " << start << " to " << goal << ": ";
                cout << start;
                list<rrtVertex>::iterator spi = shortest_path.begin();
                for(++spi; spi != shortest_path.end(); ++spi)
                    cout << " -> " << *spi;
                cout << endl;

                //set solution vector
                list<rrtVertex>::iterator spi2 = shortest_path.begin();
                _path.push_back(_samples->getSampleAt(start));

                for(++spi2; spi2 != shortest_path.end(); ++spi2){
                    _path.push_back(_samples->getSampleAt(*spi2));
                }
            }
        }
    } //namespace RRTPlanner
} //namespace libPlanner


