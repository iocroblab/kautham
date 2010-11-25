//-------------------------------------------------------------------------
//                  The Nearest Neighbor Library for Motion Planning
//-------------------------------------------------------------------------
//
// Copyright (c) 2005 University of Illinois and Anna Yershova
// All rights reserved.
//
// Developed by:                Motion Strategy Laboratory
//                              University of Illinois
//                              http://msl.cs.uiuc.edu/msl/
//
// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation files (the
// "Software"), to deal with the Software without restriction, including
// without limitation the rights to use, copy, modify, merge, publish,
// distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject to
// the following conditions:
// 
//     * Redistributions of source code must retain the above copyright 
//       notice, this list of conditions and the following disclaimers.
//     * Redistributions in binary form must reproduce the above copyright 
//       notice, this list of conditions and the following disclaimers in 
//       the documentation and/or other materials provided with the 
//       distribution.
//     * Neither the names of the Motion Strategy Laboratory, University
//       of Illinois, nor the names of its contributors may be used to 
//       endorse or promote products derived from this Software without 
//       specific prior written permission.
//
// The software is provided "as is", without warranty of any kind,
// express or implied, including but not limited to the warranties of
// merchantability, fitness for a particular purpose and
// noninfringement.  In no event shall the contributors or copyright
// holders be liable for any claim, damages or other liability, whether
// in an action of contract, tort or otherwise, arising from, out of or
// in connection with the software or the use of other dealings with the
// software.
//
//-------------------------------------------------------------------------

#ifndef DNN_NN_H
#define DNN_NN_H

#include "ANN.h"

class ANN {

 protected:
  int numPoints;  				// Total number currently stored in the ANN tree
  int dimension;				// dimension of the space
  ANNpointArray	data_pts;			// data points
  ANNpoint query_pt;		        	// query point
  ANNidxArray nn_idx;				// near neighbor indices
  ANNdistArray dists;				// near neighbor distances
  ANNkd_tree *the_tree;		        	// search structure
  int *node_indices;        			// indices to the points in MultiANN
  double epsilon;        			// the error bound for the search

 public:
  ANN() {};
  ANN(						// constructor
      ANNpointArray points, 			// array of data points
      int n1, 					// starting index in points	
      int n2, 					// ending index in points
      int size, 				// size of the chunk of points to be used
      int dim, 					// dimension of the space
      int *topology, 				// topology of the space
      ANNpoint scaling, 			// scaling of the coordinates
      int NumNeighbors);			// number of nearest neighbors to be returned

  ~ANN();					// destructor

  int  NearestNeighbor(  			// 1-nearest neighbor query
		       int *topology, 		// topology of the space
		       ANNpoint scaling, 	// scaling of the coordinates
		       const ANNpoint &x,	// query point 
		       double &d_best); 	// distance from the nearest neighbor to x (returned)

  void NearestNeighbor(  			// k-nearest neighbor query
		       int *topology,  		// topology of the space
		       ANNpoint scaling, 	// scaling of the coordinates 
		       const ANNpoint &x, 	// query point 
		       int NumNeighbors, 	// the number of nearest neighbors to be returned (k)
		       ANNpoint &d_best, 	// array of distances from the k nearest neighbors to x (returned) 
		       int *&n_best); 		// array of k nearest neighbors to x (returned) 

  int LastNode();  				// The index of the last node in MultiANN that is stored in the ANN
  int FirstNode(); 				// The index of the first node in MultiANN that is stored in the ANN
};
#endif

