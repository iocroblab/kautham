#if !defined (_RRTPLANNER_H)
#define _RRTPLANNER_H

#include <libproblem/workspace.h>
#include <libsampling/sampling.h>
#include <boost/property_map/property_map.hpp>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/astar_search.hpp>
#include "localplanner.h"
#include "planner.h"

using namespace std;
using namespace libSampling;
using namespace boost;

namespace libPlanner{
    namespace RRT{

        typedef Sample* location;
        typedef KthReal cost;
        typedef std::pair<int, int> rrtEdge;
        typedef boost::adjacency_list<boost::listS, boost::vecS, boost::undirectedS, boost::no_property, boost::property<boost::edge_weight_t, cost> > rrtGraph;
        typedef rrtGraph::vertex_descriptor rrtVertex;
        typedef property_map<rrtGraph, boost::edge_weight_t>::type WeightMap;
        typedef rrtGraph::edge_descriptor edge_descriptor;
        typedef rrtGraph::vertex_iterator vertex_iterator;

        //CLASS distance heuristic
        template <class Graph, class CostType, class LocMap>
        class distance_heuristic : public astar_heuristic<Graph, CostType>
        {
        public:
            typedef typename graph_traits<Graph>::vertex_descriptor rrtVertex;

            distance_heuristic(LocMap l, rrtVertex goal, LocalPlanner* lPlan) : m_location(l), m_goal(goal), locPlan(lPlan) {};
            CostType operator()(rrtVertex u)
            {
                //returns euclidean distance form vertex u to vertex goal
                return locPlan->distance(m_location[u],m_location[m_goal]);
            }
        private:
            LocMap m_location;
            rrtVertex m_goal;
            LocalPlanner* locPlan;
        };

        struct found_goal{}; // exception for termination

        //CLASS astar_goal_visitor
        // visitor that terminates when we find the goal
        template <class rrtVertex>
        class astar_goal_visitor : public boost::default_astar_visitor
        {
        public:
            astar_goal_visitor(rrtVertex goal) : m_goal(goal) {};
            template <class Graph>
                    void examine_vertex(rrtVertex u, Graph& g)
            {
                if(u == m_goal)throw found_goal();
            }
                    private:
                    rrtVertex m_goal;
                };

        class RRTPlanner:public Planner{

        public:

            RRTPlanner(SPACETYPE stype, Sample *init, Sample *goal, SampleSet *samples, Sampler *sampler,
                       WorkSpace *ws, LocalPlanner *lcPlan, KthReal ssize);
            ~RRTPlanner();

            bool    setParameters();
            void    saveData();
            void    setIniGoal();
			      void    connect2samples(Sample* sampleA, Sample* sampleB, float dist);
            bool    trySolve();


            //!find path
            bool findPath();
            //!load boost graph data
            void loadGraph();
            //!connect samples
            //bool connectSamples(bool assumeAllwaysFree = false);
            //! connects last sampled configuration & adds to graph
            //void connectLastSample(Sample* connectToSmp = NULL);
            //!delete g
            void clearGraph();
            void updateGraph();
            void smoothPath(bool maintainfirst=false, bool maintainlast=false);
            bool isGraphSet(){return _isGraphSet;}
            void printConnectedComponents();
            //Sample* predictor(Sample* sampleA, Sample* sampleB);

            protected:
            vector<rrtEdge*> edges;
            //!edge weights
            vector<cost> weights;
            //!bool to determine if the graph has been loaded
            bool _isGraphSet;
            KthReal _neighThress;
            int     _kNeighs;
            std::map<int, SampleSet*> _ccMap;
            int _labelCC;
      			float _extDist;

            private:
			      Sample *SmpInit;
			      Sample *SmpGoal;
            RRTPlanner();
            KthReal _ssize;
            //!boost graph
            rrtGraph *g;
            //!solution to query
            list<rrtVertex> shortest_path;
            //!pointer to the samples of cspace to be used by the distance_heuristic function used in A*
            vector<location> locations;
        };
    }
};

#endif // _RRTPLANNER_H
