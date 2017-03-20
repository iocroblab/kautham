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

 

#if !defined(_GRIDPLANNER_H)
#define _GRIDPLANNER_H

#include <kautham/problem/workspace.h>
#include <kautham/sampling/sampling.h>
#include <boost/property_map/property_map.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/filtered_graph.hpp>
#include <kautham/planner/planner.h>

using namespace std;

namespace Kautham {
/** \addtogroup GridPlanners
 *  @{
 */

   namespace IOC{
	//Typedefs
	//!The location associated to a vertex of the graph will be a sample pointer
    typedef Sample* location;
	//!The cost of an edge of the graph will be a KthReal
    typedef KthReal cost;
	//!Edge of a graph represented as a pair of ints (the vertices)
    typedef std::pair<int, int> gridEdge;
	 
	//!Definition of the property of vertices to store the value of a potential function
	struct potential_value_t {
        typedef boost::vertex_property_tag kind;
	};
	//!Graph representing the grid, defined as an adjacency_list with 
	//!a potential value associated to the vertices and a cost associated
	//!to the edges (both defined as KthReals)
    typedef boost::adjacency_list<boost::listS, boost::vecS, 
		boost::undirectedS, boost::property<potential_value_t, KthReal>, 
		boost::property<boost::edge_weight_t, cost> > gridGraph;

    typedef gridGraph::vertex_descriptor gridVertex;
    typedef boost::property_map<gridGraph, boost::edge_weight_t>::type WeightMap;
    typedef boost::property_map<gridGraph, potential_value_t>::type PotentialMap;
    typedef gridGraph::edge_descriptor edge_descriptor;
    typedef gridGraph::vertex_iterator vertex_iterator;

	
	//!Function that filters out edges with negative weigh
	template <typename EdgeWeightMap>
		struct negative_edge_weight {
			negative_edge_weight() { }
			negative_edge_weight(EdgeWeightMap weight) : m_weight(weight) { }
			template <typename Edge>
				bool operator()(const Edge& e) const {
					return get(m_weight, e) > 0.0;
				}
			EdgeWeightMap m_weight;
	};

	//!Graph representing the subgraph of the grid without the edges that have the source or target
	//!vertex associated to a collision sample
    typedef boost::filtered_graph<gridGraph, negative_edge_weight<WeightMap> > filteredGridGraph;


  //! This class is an abstract class that represents a discretization of the configuration space.
	//!It contains a graph representing the whole regular grid with both collision and free samples
	//!associated to the vertices, and a subgraph that contains only those free.
	//!Its derived classes must implement the trysolve function to implement any grid-based 
	//!planning method .
    class gridPlanner:public Planner {
	    public:
		//!Constructor
        gridPlanner(SPACETYPE stype, Sample *init, Sample *goal, SampleSet *samples, WorkSpace *ws);
        ~gridPlanner();
        
		//!Abstract class to be implemented by the derived classes with planning methods based on grids
		bool trySolve() = 0;

		protected:

		//!Boost graph representing the whole grid
		gridGraph *g;

		//!Boost graph representing the filtered grid
		filteredGridGraph *fg;	
		
		//!Vector of C-space sample pointers associated to the vertices
		vector<location> locations;

		//!Potential values at the cells of the grid
		PotentialMap potmap;

		//!Deiscretization step of the grid, per axis
		vector<int> _stepsDiscretization;

		//!Potential value of the obstacles
		KthReal _obstaclePotential;

		//!Potential value of the goal cell
		KthReal _goalPotential;

		//!Vector of edges used in the construction of the grid
		vector<gridEdge*> edges;

        //!Vector of edge weights used in the construction of the grid
        vector<cost> weights;

		//!Bool to determine if the graph has been loaded
        bool _isGraphSet;

        //!temporal storage of init config. Used when discretization steps change
        Sample *tmpSamInit;

        //!temporal storage of goal config. Used when discretization steps change
        Sample *tmpSamGoal;

        //!index of init cell in the current discretization of the cspace
        gridVertex indexinit;

        //!index of goal cell in the current discretization of the cspace
        gridVertex indexgoal;

        //!flag to set/unset the visualization of the potential values of the cells
        int _showLabels;

        //!number of decimals to be shown. The derived classes may change the defaul value using setParameters.
        int _decimals;

	    //!Function to load the boost graph data
	    void loadGraph();	 

		//!Function to delete the graphs
	    void clearGraph();

		//!Function to construct the grid graphs
		void discretizeCspace();

		//!Function to compute and collision-check the samples of the grid vertices
        void loadgrid(vector<KthReal> &coords, unsigned coord_i);

		//!Function to create the edges connecting the vertices of the grid
        void connectgrid(vector<int> &index, unsigned coord_i);
		
		//!Function to filter those edges of the grid connecting collision samples
		void  prunegrid();

		//!Function to set the potential value at a given vertex
        inline void setPotential(int i, KthReal value){potmap[i]=value;}

		//!Function to obtain the potential value at a given vertex
        inline KthReal getPotential(int i){return potmap[i];}

		//!Function that retruns the vector of potential values
        inline PotentialMap getpotmat(){return potmap;}

		//!Function to setup the value of _stepsDiscretization in the given axis, and recompute the grid
		void setStepsDiscretization(int numsteps, int axis);

        //!set random values to alla the cells of the grid
        void setRandValues();

		//!function that returns the pointer to the cspace separator in the case of 2D
		SoSeparator *getIvCspaceScene();//reimplemented
		//!Function that draws the cspace (loads the cspace separator)
		void drawCspace();
	  };
   }
   /** @}   end of Doxygen module "Planner */
}

#endif  //_GRIDPLANNER_H

