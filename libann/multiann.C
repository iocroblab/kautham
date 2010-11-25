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

#include "DNN/ANN.h"					// ANN declarations
#include "DNN/multiann.h"

#ifndef INFINITY
#define INFINITY 1.0e40
#endif

#ifndef PI
#define PI 3.1415926535897932385
#endif

#ifndef MAXPOINTS
#define MAXPOINTS 1000000
#endif

MultiANN::MultiANN(					// constructor
	   int dim, 					// dimension of the space
	   ANNpoint x_coor, 				// coordinates of the initial point in the data structure
	   void *x_ptr,					// pointer to the object holding the initial point
	   int k, 					// number of nearest neighbors to be returned
	   int *tt, 					// topology of the space
	   ANNpoint ss)					// scaling of the coordinates
{
  NumNeighbors = k;
  dimension = dim;
  LastNodeCount = 1;
  size = 1;

  topology = new int[dimension]; 			// allocate topology array
  scaling = annAllocPt(dimension); 			// allocate scaling array
  points_coor = annAllocPts(MAXPOINTS, dimension);     	// allocate coordinates of the data points
  points_ptr = new void*[MAXPOINTS]; 	       	// allocate pointers to the data points

  for (int i = 0; i < dimension; i++) {
   topology[i] = tt[i];
   scaling[i] = ss[i];
   points_coor[size - 1][i] = x_coor[i];
  }
  points_ptr[size - 1] = (void *)x_ptr;   

  for (int k = 0; k < ANN_MAXIMUM_INDEX+1; k++) {
    AnnArray[k] = NULL;
  }
}


MultiANN::MultiANN(					// constructor
	   int dim,  					// dimension of the space
	   int k, 					// number of nearest neighbors to be returned 
	   int *tt, 					// topology of the space 
	   ANNpoint ss)					// scaling of the coordinates
{
  NumNeighbors = k;
  dimension = dim;
  LastNodeCount = 0;
  size = 0;
	
  topology = new int[dimension];			// allocate topology array	
  scaling = annAllocPt(dimension); 			// allocate scaling array
  points_coor = annAllocPts(MAXPOINTS, dimension); 	// allocate data points
  points_ptr = new void*[MAXPOINTS]; 			// allocate pointers to the data points

  for (int i = 0; i < dimension; i++) {
   topology[i] = tt[i];
   scaling[i] = ss[i];
  }


  for (int k = 0; k < ANN_MAXIMUM_INDEX+1; k++) {
    AnnArray[k] = NULL;
  }
}

MultiANN::~MultiANN()					// destructor
{
  annDeallocPts(points_coor); 				// deallocate coordinates of data points
  delete []points_ptr; 					// deallocate pointers to data points
  annDeallocPt(scaling); 				// deallocate scaling
  delete []topology; 					// deallocate topology
}

void MultiANN::AddPoint(				// dynamic update of the data points
			ANNpoint x_coor,		// coordinates of the point
			void *x_ptr)			// pointer to the point
{
  for (int i = 0; i < dimension; i++) {
    points_coor[size][i] = x_coor[i];
  }
  points_ptr[size] = (void *)x_ptr;
  size++;

  if ((size != LastNodeCount)&&
      (size % ((int) pow((double)2,ANN_STARTING_INDEX)) == 0)) {		// Check if it is time to update the ANNs
    LastNodeCount = size;
    UpdateAnnArray();
  }
}

void MultiANN::ResetMultiANN()				// destroys all the arrays of data points
{
  for (int k = 0; k < ANN_MAXIMUM_INDEX+1; k++) {
    if (AnnArray[k]) delete(AnnArray[k]);
    AnnArray[k] = NULL;    
  }
  annDeallocPts(points_coor);				// deallocate coordinates of data points
  delete []points_ptr;					// deallocate pointers to data points
  annDeallocPt(scaling);				// deallocate scaling
  delete []topology; 					// deallocate topology
}

void MultiANN::UpdateAnnArray()				// updates the arrays of data points
{
  int j,k,num; 
  int n,n_last;
  bool done;

  num = size;
  n_last = 0;	 					// Just to make warnings go away

  if ((num > 0)&&
      (num % ((int) pow((double)2,ANN_STARTING_INDEX)) == 0)) { // Check if it is time to update the ANNs

    for (k = ANN_MAXIMUM_INDEX; k >= ANN_STARTING_INDEX; k--) {
      if (AnnArray[k])
	n_last = AnnArray[k]->LastNode();			// Make sure update has not been performed
    }
    n_last++;

    if (n_last != size) { 					// If equal, then update already performed

      k = ANN_STARTING_INDEX - 1;
      done = false;
      while ((k <= ANN_MAXIMUM_INDEX)&&(!done)) {
	k++;
	if (!AnnArray[k]) { 					// If there is no tree, then make one
	  done = true;
	}
	else { 
	  delete(AnnArray[k]);  				// Blow this one away; these nodes are moved up
	  AnnArray[k] = NULL;
	}
      }

      if ((num == ((int) pow((double)2,ANN_STARTING_INDEX))))	// k should be the location for making the new ANN tree
	n = 0;							// Need to find the starting node in G for the new ANN tree
      else {
	j = k; done = false;
	while ((j <= ANN_MAXIMUM_INDEX)&&(!done)) {
	  if (AnnArray[j]) {
	    n = AnnArray[j]->LastNode();
	    n++;
	    done = true;
	  }
	  j++;
	}
      }

      if (!done)
	n = 0;

      AnnArray[k] = new ANN(points_coor, n, size, (int) pow((double)2,k), dimension, 
      		    topology, scaling, NumNeighbors); 		 		// Finally, make the new ANN tree

      //cout << "New ANN Tree:  k: " << k << " num: " << num << "\n";
      //for (j = ANN_MAXIMUM_INDEX; j >= ANN_STARTING_INDEX; j--) {
      // 	cout << (AnnArray[j] ? "X" : ".");
      //}
      //cout << "\n";
    }
  }

}


void *MultiANN::NearestNeighbor(  				// 1-nearest neighbor query
				   const ANNpoint &x,		// query point
                                   int &best_idx,		// index of the nearest neighbor to x (returned)
				   double &best_dist) 		// distance from the nearest neighbor to x (returned)
{
  int n,n_last;
  int k,num;
  double d,d_min;

  num = size;
  n = n_last = 0; 						// Keeps the warnings away
  d_min = INFINITY; d = 0.0;

  for (k = ANN_MAXIMUM_INDEX; k >= ANN_STARTING_INDEX; k--) {   // Find the nearest neighbor
    if (AnnArray[k]) {  					// First check the ANN trees
      d = 0.0;
      n = AnnArray[k]->NearestNeighbor(topology, scaling, x, d);
      if (d < d_min) {
        d_min = d; best_idx = n; 
      }
      n_last = AnnArray[k]->LastNode();
      if (n_last != size) 
	n_last++;
    }
  }

  for (n = n_last; n != size; n++ ) {			// Check the new nodes, which are not yet in the ANN tree
    ANNpoint x1 = points_coor[n];
    d = 0.0;
    for (int i = 0; i < dimension; i++) {
      if (topology[i] == 1) {
	d += ANN_POW(scaling[i]*(x1[i] - x[i]));
      }
      else if (topology[i] == 2) {
	  double t = fabs(x1[i]-x[i]);
	  double t1 = ANN_MIN(t,2.0*PI - t);
	  d += ANN_POW(scaling[i]*t1);
      }
      else if (topology[i] == 3) {
	double fd = x1[i]*x[i] + x1[i+1]*x[i+1] + x1[i+2]*x[i+2] + x1[i+3]*x[i+3];
	if (fd > 1) {
	  double norm1 = x1[i]*x1[i] + x1[i+1]*x1[i+1] + x1[i+2]*x1[i+2] + x1[i+3]*x1[i+3];
	  double norm2 = x[i]*x[i] + x[i+1]*x[i+1] + x[i+2]*x[i+2] + x[i+3]*x[i+3];
	  fd = fd/(norm1*norm2);
	}
	double dtheta = ANN_MIN(acos(fd), acos(-fd));
	d += ANN_POW(scaling[i]*dtheta);
	i = i + 3;
      }
    }
    d = sqrt(d);
    if (d < d_min) {
      d_min = d; best_idx = n; 
    }
  }

  best_dist = d_min;
  return points_ptr[best_idx];
}

void MultiANN::NearestNeighbor(const ANNpoint &x,      	// query point
			       ANNpoint &best_dist,    	// distances from the  nearest neighbors to x (returned) 
			       int *&best_idx, 		// indices of the nearest neighbors to x (returned)
			       void **&best_ptr)       	// pointers to the nearest neighbors to x (returned)
{
  int n_best, n, n_last, j, k, l, m, num;
  double d;

  int *node_list = new int[NumNeighbors];
  int *help_list = new int[NumNeighbors];
  double *d1 = new double[NumNeighbors];
  double *d_help = new double[NumNeighbors];

  for (int i = 0; i < NumNeighbors; i++) {
    best_dist[i] = INFINITY;
    d1[i] = 0.0;
    d_help[i] = 0.0;
    best_idx[i] = 0;
    node_list[i] = 0;
    help_list[i] = 0;
  }

  num = size;
  n_best = n_last = n = 0; // Keeps the warnings away


  // Find the nearest neighbor
  // First check the ANN trees
  for (k = ANN_MAXIMUM_INDEX; k >= ANN_STARTING_INDEX; k--) {
    if (AnnArray[k]) {
      AnnArray[k]->NearestNeighbor(topology, scaling, x, NumNeighbors, d1, node_list);
      m = 0; j = 0; l = 0;
      while (m < NumNeighbors && j < NumNeighbors && l < NumNeighbors) {
	if (best_dist[m] > d1[j]) {
	  d_help[l] = d1[j];
	  help_list[l] = node_list[j];
	  j++; l++;
	}
	else {
	  d_help[l] = best_dist[m];
	  help_list[l] = best_idx[m];
	  m++; l++;
	}
      }
      for (int i = 0; i < NumNeighbors; i++) {
	best_dist[i] = d_help[i];
	best_idx[i] = help_list[i];
      }
      n_last = AnnArray[k]->LastNode();
      if (n_last != size) 
	n_last++;
    }
  }

  // Check the new nodes, which are not yet in the ANN tree
  for (n = n_last; n != size; n++ ) {
    ANNpoint x1 = points_coor[n];
    d = 0.0;
    for (int i = 0; i < dimension; i++) {
      if (topology[i] == 1) {
	d += ANN_POW(scaling[i]*(x1[i] - x[i]));
      }
      else if (topology[i] == 2) {
	  double t = fabs(x1[i]-x[i]);
	  double t1 = ANN_MIN(t,2.0*PI - t);
	  d += ANN_POW(scaling[i]*t1);
      }
      else if (topology[i] == 3) {
	double fd = x1[i]*x[i] + x1[i+1]*x[i+1] + x1[i+2]*x[i+2] + x1[i+3]*x[i+3];
	if (fd > 1) {
	  double norm1 = x1[i]*x1[i] + x1[i+1]*x1[i+1] + x1[i+2]*x1[i+2] + x1[i+3]*x1[i+3];
	  double norm2 = x[i]*x[i] + x[i+1]*x[i+1] + x[i+2]*x[i+2] + x[i+3]*x[i+3];
	  fd = fd/(norm1*norm2);
	}
	double dtheta = ANN_MIN(acos(fd), acos(-fd));
	d += ANN_POW(scaling[i]*dtheta);
	i = i + 3;
      }
    }
    d = sqrt(d);
    bool flag = true;
    int ind = NumNeighbors + 2;
    j = 0;
    while (flag && j < NumNeighbors) {
      if (d < best_dist[j]) {
	flag = false;
	ind = j;
      }
      j++;
    }
    if (ind < NumNeighbors) {
      for (int m = (NumNeighbors - 1); m > ind; m--) {
	best_dist[m] = best_dist[m - 1];
	best_idx[m] = best_idx[m - 1];
      }
      best_idx[ind] = n;
      best_dist[ind] = d;
    }
  }

  for (int i = 0; i < NumNeighbors; i++) {
    best_ptr[i] = points_ptr[best_idx[i]];
  }

  delete []node_list;
  delete []help_list;
  delete []d_help;
  delete []d1;
}

