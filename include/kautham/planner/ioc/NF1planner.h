/*************************************************************************\
   Copyright 2014-2024  Institute of Industrial and Control Engineering (IOC)
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


#if !defined(_NF1PLANNER_H)
#define _NF1PLANNER_H


#include <boost/graph/visitors.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/breadth_first_search.hpp>

#include <kautham/problem/workspace.h>
#include <kautham/planner/planner.h>
#include <kautham/planner/ioc/gridplanner.h>

using namespace std;

namespace Kautham {
/** \addtogroup GridPlanners
 *  @{
 */

  namespace IOC{

    //CLASS bfs_distance_visitor
    // visitor that terminates when we find the goal
    template <class DistanceMap>
    class bfs_distance_visitor : public boost::default_bfs_visitor 
	{
		public:
			bfs_distance_visitor(DistanceMap dist) : d(dist) {};

			template <typename Edge, typename Graph> 
			void tree_edge(Edge e, Graph& g)
			{
                typename boost::graph_traits<Graph>::vertex_descriptor s=source(e,g);
                typename boost::graph_traits<Graph>::vertex_descriptor t=target(e,g);
				d[t] = d[s] + 1;
				//potmap[t] = d[t];
			}

		private:
			DistanceMap d;
    };

    class NF1Planner:public gridPlanner {
	    public:
        NF1Planner(SPACETYPE stype, Sample *init, Sample *goal, SampleSet *samples, WorkSpace *ws);
        ~NF1Planner();
        
		bool trySolve();
		bool setParameters();
		//Add public data and functions
		

		protected:
		//Add protected data and functions

		
	    private:
		//Add private data and functions	
		void computeNF1(gridVertex  vgoal);

		

	  };
   }
  /** @}   end of Doxygen module "Planner */
}

#endif  //_MYGRIDPLANNER_H

