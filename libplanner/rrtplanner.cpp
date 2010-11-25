#include <libproblem/workspace.h>
#include <libsampling/sampling.h>
#include <boost/graph/astar_search.hpp>
#include <boost/graph/adjacency_list.hpp>
#include "localplanner.h"
#include "rrtplanner.h"
#include <stdio.h>

#include <libsampling/lcprng.h>

namespace libPlanner{
    namespace RRT{

        RRTPlanner::RRTPlanner(SPACETYPE stype, Sample *init, Sample *goal, SampleSet *samples, Sampler *sampler,
                               WorkSpace *ws, LocalPlanner *lcPlan, KthReal ssize)
                                   :Planner(stype, init, goal, samples, sampler, ws, lcPlan, ssize){
			
			_wkSpace = ws;			
			_neighThress = 50.0;
            _kNeighs = 10;
            _isGraphSet = false;
            _maxNumSamples = 200;
            _speedFactor = 10;
            _solved = false;
			      ssize = 5;
            setStepSize(ssize);//also changes stpssize of localplanner

            _idName = "RRT Planner";
            addParameter("DELTA", ssize);
            addParameter("SAMPLES", _maxNumSamples);
            addParameter("SPEED", _speedFactor);

            _labelCC=0;

            _samples->setTypeSearch(ANNMETHOD);//(BRUTEFORCE);//
            _samples->setWorkspacePtr(_wkSpace);
            _samples->setANNdatastructures(_kNeighs, _maxNumSamples);
        }

        RRTPlanner::~RRTPlanner(){
        }

        bool RRTPlanner::setParameters(){
            try{
                HASH_S_K::iterator it = _parameters.find("DELTA");
                if(it != _parameters.end())
                    setStepSize(it->second);//also changes stpssize of localplanner
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
                it = _parameters.find("Neigh Thresshold");
                if(it != _parameters.end())
                    _neighThress = it->second;
                else
                    return false;
                it = _parameters.find("Max. Neighs");
                if(it != _parameters.end()){
                    _kNeighs = (int)it->second;
                    _samples->setANNdatastructures(_kNeighs, _maxNumSamples);
                }else
                    return false;
            }catch(...){
                return false;
            }
            return true;
        }

        void RRTPlanner::saveData(){
            //cout << "RRTPlanner::saveData not implemented"<<endl<<flush;
			float Distinitgoal = _init->getDistance(_goal,BRONCOSPACE);
			cout<<" Distinitgoal: "<<Distinitgoal<<endl<<flush;
        }

        void RRTPlanner::setIniGoal(){
            //cout << "RRTPlanner::setIniGoal not implemented"<<endl<<flush;
			vector<float> coordnew;
			coordnew.resize(7);
			_init = _sampler->nextSample();
			_goal = _sampler->nextSample();
			coordnew[0] = 0.52;
			coordnew[1] = 0.5;
			coordnew[2] = 0.64;
			coordnew[3] = 1;
			coordnew[4] = 0.5;
			coordnew[5] = 0.5;
			coordnew[6] = 1;
			_init->setCoords (coordnew);
			coordnew[0] = 0.32;
			coordnew[1] = 0.5;
			coordnew[2] = 0.37;
			coordnew[3] = 0.75;
			coordnew[4] = 0.75;
			coordnew[5] = 0.5;
			coordnew[6] = 0.5;
			_goal->setCoords (coordnew);
			_samples->add(_goal);
			_samples->add(_init);
        }

        bool RRTPlanner::trySolve(){

            Sample* samplerand = _sampler->nextSample();
            Sample* samplenear = _init;
            Sample* samplenew = NULL;
			Sample* currentsample = NULL;
			float Distrandcurrent_i = 0;
			float Distrandcurrent_i0 = 0;
			float Distgoalnew;
			int max = _maxNumSamples;
			clearGraph();

			int index_init = _samples->indexOf(_init);
			int index_goal = _samples->indexOf(_goal);
			int index_rand;
			int index_near;
			int index_new;
			int index_total[1000];
			index_total[1] = index_init;
			index_total[0] = index_goal;

			int imax = 2;
			do{
				//do{
				samplerand = NULL;
				samplerand = _sampler->nextSample();
				
				for(int i=1;i<imax;i++){
					currentsample = NULL;
					currentsample = _samples->getSampleAt(index_total[i]);
					Distrandcurrent_i = samplerand->getDistance(currentsample,CONFIGSPACE);
					if(Distrandcurrent_i<Distrandcurrent_i0){
						samplenear = NULL;
						samplenear = currentsample;
					}
					index_near = _samples->indexOf(samplenear);
					//cout<<" index_near: "<<index_near<<endl<<flush;
					//cout<<" imax: "<<imax<<endl<<flush;
					Distrandcurrent_i0 = Distrandcurrent_i;
				}

				do{
					samplenew = NULL;
					samplenew = predictor(samplenear,samplerand);
				}while(_wkSpace->collisionCheck(samplenew)==true);

				//	_locPlanner->setInitSamp(samplenear);
				//	_locPlanner->setGoalSamp(samplenew);

				//}while(_locPlanner->canConect()==false);

				//samplenew = NULL;
				//samplenew = samplerand;
				_samples->add(samplenew);
				index_new = _samples->indexOf(samplenew);
				samplenear->addNeigh(index_new);
				index_total[imax] = index_new;
				imax = imax + 1;				
				//_locPlanner->setInitSamp(_goal);
				//_locPlanner->setGoalSamp(samplenew);
				Distgoalnew = _goal->getDistance(samplenew,CONFIGSPACE);
				//float dist = samplenew->getDistance(samplenear,CONFIGSPACE);
				//cout<<" dist: "<<dist<<endl<<flush;
			}while(_samples->getSize() < _maxNumSamples);// && Distgoalnew>=10);//&& _locPlanner->canConect()==false);
			
			_samples->findNeighs(100, 10);
			connectSamples();

            loadGraph();

            if( findPath() )
            {
                //smoothPath();
                return true;
            }
            else
            {
                return false;
            }
        }

        Sample* RRTPlanner::predictor(Sample* sampleA, Sample* sampleB){

            //cout << "RRTPlanner::predictor not implemented"<<endl<<flush;

            Sample *Samplenew = _sampler->nextSample();
			Sample *Samplecurr = NULL;
			Robot *_robot = _wkSpace->getRobot(0);
						
			vector<float> coordA = sampleA->getCoords(); 
			
			vector<KthReal> values;
			values.resize(3);
			vector<KthReal> bestvalues;
			values.resize(3);
			vector<float> coordnew;
			static LCPRNG* gen = new LCPRNG(15485341);
			float rand0;
			float rand1;
			float rand2;
			float dist_i = 0;
			float dist_i0 = 100000;
			
			for (int i=0; i<10; i++){				
				
				rand0 = (KthReal)gen->d_rand();
				rand1 = (KthReal)gen->d_rand();
				rand2 = (KthReal)gen->d_rand();
				values[0] = rand0;//rand0/100;
				values[1] = rand1;//coordA[6]+rand1/100;
				values[2] = -rand2/100;//(rand2-0.5)/50;
				coordnew = _robot->constrainedinterpolate(coordA,values);
					
				Samplenew->setCoords(coordnew);	
				dist_i = Samplenew->getDistance(sampleB,CONFIGSPACE);
				if (dist_i<dist_i0){
					Samplecurr = NULL;
					Samplecurr = Samplenew;
					dist_i0 = dist_i;
				}				
			}
			
			return Samplecurr;

        }

        //!Finds a solution path in the graph using A*
        bool RRTPlanner::findPath(){
            _solved = false;

            if(_init->getConnectedComponent() != _goal->getConnectedComponent()) return false;
            clearSimulationPath();
            shortest_path.clear(); //path as a vector of rrtvertex
            _path.clear();//path as a vector of samples

            rrtVertex start = _samples->indexOf(_init);
            rrtVertex  goal = _samples->indexOf(_goal);

            //vector to store the parent information of each vertex
            vector<rrtGraph::vertex_descriptor> p(num_vertices(*g));
            //vector with cost of reaching each vertex
            vector<cost> d(num_vertices(*g));

            try {
                // call astar named parameter interface
                astar_search(*g, start,
                             distance_heuristic<rrtGraph, cost, vector<location> >(locations, goal, _locPlanner),
                             predecessor_map(&p[0]).distance_map(&d[0]).
                             visitor(astar_goal_visitor<rrtVertex>(goal)));
            }catch(RRT::found_goal fg){ // found a path to the goal
                _solved = true;
                //Load the vector shortest_path that represents the solution as a sequence of rrtvertex
                for( rrtVertex v = goal; ; v = p[v] ){
                    shortest_path.push_front(v);
                    if( p[v] == v ) break;
                }
                //Print solution path
                cout << "Shortest path from " << start << " to " << goal << ": ";
                cout << start;
                list<rrtVertex>::iterator spi = shortest_path.begin();
                for(++spi; spi != shortest_path.end(); ++spi)
                    cout << " -> " << *spi;
                cout << endl;
                //Load the vector _path that represents the solution as a sequence of samples
                list<rrtVertex>::iterator spi2 = shortest_path.begin();
                _path.push_back(_samples->getSampleAt(start));
                for(++spi2; spi2 != shortest_path.end(); ++spi2){
                    _path.push_back(_samples->getSampleAt(*spi2));
                }
                return true;
            }
            cout << "Didn't find a path from " << start << " to " << goal << "!" << endl;
            return false;
        }

        //!Load boost graph data
        void RRTPlanner::loadGraph(){
            int maxNodes = this->_samples->getSize();
            unsigned int num_edges = edges.size();

            // create graph
            g = new rrtGraph(maxNodes);
            WeightMap weightmap = get(edge_weight, *g);

            for(std::size_t j = 0; j < num_edges; ++j){
                edge_descriptor e;
                bool inserted;//when the efge already exisits or is a self-loop
                //then this flag is set to false and the edge is not inserted
                tie(e, inserted) = add_edge(edges[j]->first,edges[j]->second, *g);
                if(inserted) weightmap[e] = weights[j];
            }

            //locations are the pointer to the samples of cspace that
            //are at each node of the graph and is used to compute the
            //distance when using the heuristic (function distance_heuristic)
            for(unsigned int i=0;i<num_vertices(*g); i++)
                locations.push_back( _samples->getSampleAt(i) );
            _isGraphSet = true;
        }

        //!connect samples - put weights
        bool RRTPlanner::connectSamples(bool assumeAllwaysFree){
            int n;
            Sample *smpFrom;
            Sample *smpTo;
            rrtEdge *e; //rrtEdge is a type defined in RRT.h as std::pair<int, int>

            cout << "CONNECTING  " << _samples->getSize() << " FREE SAMPLES" << endl;

            typedef std::pair<int, SampleSet*> ccPair;
            int ccFrom; //label co connected component of intial sample
            int ccTo;   //label co connected component of goal sample
            unsigned int max;

            if(_samples->getSize() < _maxNumSamples)
                max = _samples->getSize();
            else {
                max = _maxNumSamples;
                cout<<"connectSamples::Using a maximum of "<<max<<" samples"<<endl;
            }

            for(unsigned int i=0; i<max; i++){
                smpFrom = _samples->getSampleAt(i);
                //set initial sample of local planner
                _locPlanner->setInitSamp(smpFrom);
                //if not yet labeled, labelwith a new connected component
                if(smpFrom->getConnectedComponent() == -1)
                {
                    smpFrom->setConnectedComponent(_labelCC);	//label sample with connected component
                    SampleSet *tmpSS = new SampleSet();			//create new sample set
                    tmpSS->add( smpFrom );						//add sample to sample set
                    _ccMap.insert(ccPair(_labelCC,tmpSS));		//create connected component as a labeled sample set
                    _labelCC++;
                }
                else continue;//already in a connected component

                //srtart connecting with neighs
                for(unsigned int j=0; j<smpFrom->getNeighs()->size(); j++){
                    n = smpFrom->getNeighs()->at(j);
                    smpTo = _samples->getSampleAt(n);
                    //if same connected component do no try to connect them
                    if(smpFrom->getConnectedComponent() == smpTo->getConnectedComponent()) continue;
                    //set goal sample of local planner
                    _locPlanner->setGoalSamp(smpTo);
                    //local planner collision checks the edge (if required)
                    if(assumeAllwaysFree || _locPlanner->canConect())
                    {
                        e = new rrtEdge(i,n);
                        edges.push_back(e);
                        weights.push_back(_locPlanner->distance(smpFrom,smpTo));
                        //set neigh sample with same label as current sample
                        ccTo = smpTo->getConnectedComponent();
                        ccFrom = smpFrom->getConnectedComponent();
                        if(ccTo == -1)
                        {
                            smpTo->setConnectedComponent( ccFrom );
                            _ccMap[ccFrom]->add( smpTo );
                        }
                        //set all samples of the connected component of the neigh sample to
                        //the same label as the current sample
                        else
                        {
                            //unify lists under the lowest label
                            if( ccFrom < ccTo)
                            {
                                vector<Sample*>::iterator itera = _ccMap[ccTo]->getBeginIterator();
                                while((itera != _ccMap[ccTo]->getEndIterator()))
                                {
                                    (*itera)->setConnectedComponent( ccFrom );
                                    _ccMap[ccFrom]->add( (*itera) );
                                    itera++;
                                }
                                _ccMap.erase(ccTo);
                            }
                            else
                            {
                                vector<Sample*>::iterator itera = _ccMap[ccFrom]->getBeginIterator();
                                while((itera != _ccMap[ccFrom]->getEndIterator()))
                                {
                                    (*itera)->setConnectedComponent( ccTo );
                                    _ccMap[ccTo]->add( (*itera) );
                                    itera++;
                                }
                                _ccMap.erase(ccFrom);
                            }
                        }
                    }
                    else
                    {
                        //cout<<": FAILED"<<endl;
                        //cout << "edge from " << i << " to " << n << " is NOT free" << endl;
                    }
                }
            }
            cout << "END CONNECTING  " << max << " FREE SAMPLES" << endl;
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
