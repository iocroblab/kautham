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

#ifndef DNN_MULTIANN_H
#define DNN_MULTIANN_H

#include <math.h>
#include <stdio.h>

#include "nn.h"

#define ANN_STARTING_INDEX 7 //4  			// Start a new ANN for each 2^n samples
#define ANN_MAXIMUM_INDEX 30  			// Absolute max on number of nodes (2^n)

class MultiANN{
 public:

  int size;					// the number of points in MultiANN
  int dimension;				// dimension of the space
  int LastNodeCount;				// the index of the last node
  ANNpointArray points_coor;			// array of the coordinates of data points
  void **points_ptr;				// array of the pointers to data points
  int NumNeighbors;				// number of nearest neighbors to be returned (k) 
  int *topology;				// topology of the space 
  ANNpoint scaling;				// scaling of the coordinates

  ANN  *AnnArray[ANN_MAXIMUM_INDEX+1];		// array of ANNs to hold data points

  MultiANN() {};				// constructor
  MultiANN(					// constructor
	   int dim, 				// dimension of the space
	   ANNpoint x_coor, 		       	// coordinate of the initial point in the data structure
	   void *x_ptr, 		       	// pointer to the initial point in the data structure
	   int k, 				// number of nearest neighbors to be returned
	   int *topology, 			// topology of the space
	   ANNpoint scaling);			// scaling of the coordinates

  MultiANN(					// constructor
	   int dim,  				// dimension of the space
	   int k, 				// number of nearest neighbors to be returned 
	   int *topology, 			// topology of the space 
	   ANNpoint scaling);			// scaling of the coordinates

  ~MultiANN();					// destructor
  void ResetMultiANN();				// destroys all the arrays of data points
  void AddPoint(				// dynamic update of the data points
		ANNpoint x_coor,		// the point's coordinates 
		void *x_ptr);			// the pointer to the point 

  void UpdateAnnArray();			// updates the arrays of data points

  void *NearestNeighbor(  			// 1-nearest neighbor query
		       const ANNpoint &x,	// query point  
		       int &best_idx, 		// distance from the nearest neighbor to x (returned)
		       double &best_dist); 	// distance from the nearest neighbor to x (returned)

  void NearestNeighbor(  			// k-nearest neighbor query
		       const ANNpoint &x, 	// query point 
		       ANNpoint &best_dist,  	// array of distances from the k nearest neighbors to x (returned) 
		       int *&best_idx, 		// array of indices of k nearest neighbors to x (returned) 
		       void **&best_ptr);      	// array of pointers to k nearest neighbors to x (returned) 
};
#endif
