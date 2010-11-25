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

// This file is modified from:
//----------------------------------------------------------------------
//	File:		kd_search.cc
//	Programmer:	Sunil Arya and David Mount
//	Last modified:	03/04/98 (Release 0.1)
//	Description:	Standard kd-tree search
//----------------------------------------------------------------------
// Copyright (c) 1997-1998 University of Maryland and Sunil Arya and David
// Mount.  All Rights Reserved.
// 
// This software and related documentation is part of the 
// Approximate Nearest Neighbor Library (ANN).
// 
// Permission to use, copy, and distribute this software and its 
// documentation is hereby granted free of charge, provided that 
// (1) it is not a component of a commercial product, and 
// (2) this notice appears in all copies of the software and
//     related documentation. 
// 
// The University of Maryland (U.M.) and the authors make no representations
// about the suitability or fitness of this software for any purpose.  It is
// provided "as is" without express or implied warranty.
//----------------------------------------------------------------------

#include "DNN/kd_search.h"			// kd-search declarations

//----------------------------------------------------------------------
//  Approximate nearest neighbor searching by kd-tree search
//	The kd-tree is searched for an approximate nearest neighbor.
//	The point is returned through one of the arguments, and the
//	distance returned is the squared distance to this point.
//
//	The method used for searching the kd-tree is an approximate
//	adaptation of the search algorithm described by Friedman,
//	Bentley, and Finkel, ``An algorithm for finding best matches
//	in logarithmic expected time,'' ACM Transactions on Mathematical
//	Software, 3(3):209-226, 1977).
//
//	The algorithm operates recursively.  When first encountering a
//	node of the kd-tree we first visit the child which is closest to
//	the query point.  On return, we decide whether we want to visit
//	the other child.  If the box containing the other child exceeds
//	1/(1+eps) times the current best distance, then we skip it (since
//	any point found in this child cannot be closer to the query point
//	by more than this factor.)  Otherwise, we visit it recursively.
//	The distance between a box and the query point is computed exactly
//	(not approximated as is often done in kd-tree), using incremental
//	distance updates, as described by Arya and Mount in ``Algorithms
//	for fast vector quantization,'' Proc.  of DCC '93: Data Compression
//	Conference, eds. J. A. Storer and M. Cohn, IEEE Press, 1993,
//	381-390.
//
//	The main entry points is annkSearch() which sets things up and
//	then call the recursive routine ann_search().  This is a recursive
//	routine which performs the processing for one node in the kd-tree.
//	There are two versions of this virtual procedure, one for splitting
//	nodes and one for leaves.  When a splitting node is visited, we
//	determine which child to visit first (the closer one), and visit
//	the other child on return.  When a leaf is visited, we compute
//	the distances to the points in the buckets, and update information
//	on the closest points.
//
//	Some trickery is used to incrementally update the distance from
//	a kd-tree rectangle to the query point.  This comes about from
//	the fact that which each successive split, only one component
//	(along the dimension that is split) of the squared distance to
//	the child rectangle is different from the squared distance to
//	the parent rectangle.
//----------------------------------------------------------------------

//----------------------------------------------------------------------
//	To keep argument lists short, a number of global variables
//	are maintained which are common to all the recursive calls.
//	These are given below.
//----------------------------------------------------------------------
#define PI 3.1415926535897932385

int		ANNkdDim;		// dimension of space
ANNpoint	ANNkdQ;			// query point
double		ANNkdMaxErr;		// max tolerable squared error
ANNpointArray	ANNkdPts;		// the points
ANNmin_k	*ANNkdPointMK;		// set of k closest points
double		*ANNScale;		// scaling array
int		*ANNTopology;		// topology array
 
//----------------------------------------------------------------------
//  annkSearch - search for the k nearest neighbors
//----------------------------------------------------------------------

void ANNkd_tree::annkSearch(
    ANNpoint		q,		// the query point
    int			k,		// number of near neighbors to return
    ANNidxArray		nn_idx,		// nearest neighbor indices (returned)
    ANNdistArray	dd,		// the approximate nearest neighbor
    double		eps)		// the error bound
{

    ANNkdDim = dim;			// copy arguments to static equivs
    ANNScale = new double [dim];
    ANNTopology = new int [dim];
    for (int i = 0; i < dim; i++){
      ANNScale[i] = Scale[i];
      ANNTopology[i] = Topology[i];
    }
    ANNkdQ = q;
    ANNkdPts = pts;
    ANNptsVisited = 0;			// initialize count of points visited

    if (k > n_pts) {			// too many near neighbors?
	annError("Requesting more near neighbors than data points", ANNabort);
    }

    ANNkdMaxErr = ANN_POW(1.0 + eps);
    //    FLOP(2)				// increment floating op count

    ANNkdPointMK = new ANNmin_k(k);	// create set for closest k points
					// search starting at the root
    root->ann_search(annBoxDistance(q, bnd_box_lo, bnd_box_hi, dim, ANNScale, ANNTopology));

    for (int i = 0; i < k; i++) {	// extract the k-th closest points
	dd[i] = ANNkdPointMK->ith_smallest_key(i);
	nn_idx[i] = ANNkdPointMK->ith_smallest_info(i);
    }
    delete [] ANNScale;
    delete [] ANNTopology;
    delete ANNkdPointMK;		// deallocate closest point set
}

//----------------------------------------------------------------------
//  kd_split::ann_search - search a splitting node
//----------------------------------------------------------------------

void ANNkd_split::ann_search(ANNdist box_dist)
{

  					// check dist calc termination condition
  if (ANNmaxPtsVisited && ANNptsVisited > ANNmaxPtsVisited) return;

  if (ANNTopology[cut_dim] == 1){
					// distance to cutting plane
    ANNcoord cut_diff = ANNkdQ[cut_dim] - cut_val;

    if (cut_diff < 0) {			// left of cutting plane
	child[LO]->ann_search(box_dist);// visit closer child first

	ANNcoord box_diff = cd_bnds[LO] - ANNkdQ[cut_dim];
	if (box_diff < 0)		// within bounds - ignore
	    box_diff = 0;
					// distance to further box
	box_dist = (ANNdist) ANN_SUM(box_dist,
		ANN_DIFF(ANN_POW(box_diff), ANN_POW(cut_diff)));

					// visit further child if close enough
        if (box_dist * ANNkdMaxErr < ANNkdPointMK->max_key())
            child[HI]->ann_search(box_dist);

    }
    else {				// right of cutting plane
	child[HI]->ann_search(box_dist);// visit closer child first

	ANNcoord box_diff = ANNkdQ[cut_dim] - cd_bnds[HI];
	if (box_diff < 0)		// within bounds - ignore
	    box_diff = 0;
					// distance to further box
	box_dist = (ANNdist) ANN_SUM(box_dist,
		ANN_DIFF(ANN_POW(box_diff), ANN_POW(cut_diff)));

					// visit further child if close enough
        if (box_dist * ANNkdMaxErr < ANNkdPointMK->max_key())
            child[LO]->ann_search(box_dist);

    }
    //FLOP(10)				// increment floating ops
    //SPL(1)				// one more splitting node visited
      }
  else if (ANNTopology[cut_dim] == 2) {
    ANNdist box_dist1, box_dist2;

					// distance to cutting plane
    ANNcoord cut_diff1 = ANNkdQ[cut_dim] - cut_val;

    if (cut_diff1 < 0) {       		// left of cutting plane

	ANNcoord cut_diff2 = 2*PI*ANNScale[cut_dim] + (ANNkdQ[cut_dim] - cut_val);

	ANNcoord box_diff1 = cd_bnds[LO] - ANNkdQ[cut_dim];
	if (box_diff1 < 0)		// within bounds - ignore
	    box_diff1 = 0;

	ANNcoord box_diff2 = 2*PI*ANNScale[cut_dim] - (cd_bnds[HI] - ANNkdQ[cut_dim]);
	if (box_diff2 < 0)		// within bounds - ignore
	    box_diff2 = 0;

	if (box_diff1 < box_diff2) {
	  child[LO]->ann_search(box_dist);// visit closer child first
					// distance to further box
	  box_dist1 = (ANNdist) ANN_SUM(box_dist,
		ANN_DIFF(ANN_POW(box_diff1), ANN_POW(cut_diff1)));

	
					// distance to further box
	  box_dist2 = (ANNdist) ANN_SUM(box_dist,
		ANN_DIFF(ANN_POW(box_diff1), ANN_POW(box_diff2)));
	  if (box_dist1 < box_dist2) {
	  
	  	 	 	 	 // visit further child if close enough
	    if (box_dist1 * ANNkdMaxErr < ANNkdPointMK->max_key())
	      child[HI]->ann_search(box_dist1);
	  }
	  else {
	    if (box_dist2 * ANNkdMaxErr < ANNkdPointMK->max_key())
	      child[HI]->ann_search(box_dist2);
	  }
	}
	else {
	  child[HI]->ann_search(box_dist);// visit closer child first
					// distance to further box
	  box_dist1 = (ANNdist) ANN_SUM(box_dist,
		ANN_DIFF(ANN_POW(box_diff2), ANN_POW(cut_diff2)));
	
					// distance to further box
	  box_dist2 = (ANNdist) ANN_SUM(box_dist,
		ANN_DIFF(ANN_POW(box_diff2), ANN_POW(box_diff1)));
	  if (box_dist1 < box_dist2) {
	  
	  	 	 	 	 // visit further child if close enough
	    if (box_dist1 * ANNkdMaxErr < ANNkdPointMK->max_key())
	      child[LO]->ann_search(box_dist1);
	  }
	  else {
	    if (box_dist2 * ANNkdMaxErr < ANNkdPointMK->max_key())
	      child[LO]->ann_search(box_dist2);
	  }
	
	}
    }
    else {				// right of cutting plane
	ANNcoord cut_diff2 = 2*PI*ANNScale[cut_dim] - (ANNkdQ[cut_dim] - cut_val);

	ANNcoord box_diff1 =  ANNkdQ[cut_dim] - cd_bnds[HI];
	if (box_diff1 < 0)		// within bounds - ignore
	    box_diff1 = 0;

	ANNcoord box_diff2 = 2*PI*ANNScale[cut_dim] - (ANNkdQ[cut_dim] - cd_bnds[LO]);
	if (box_diff2 < 0)		// within bounds - ignore
	    box_diff2 = 0;

	if (box_diff1 < box_diff2) {
	  child[HI]->ann_search(box_dist);// visit closer child first
					// distance to further box
	  box_dist1 = (ANNdist) ANN_SUM(box_dist,
		ANN_DIFF(ANN_POW(box_diff1), ANN_POW(cut_diff1)));

	
					// distance to further box
	  box_dist2 = (ANNdist) ANN_SUM(box_dist,
		ANN_DIFF(ANN_POW(box_diff1), ANN_POW(box_diff2)));
	  if (box_dist1 < box_dist2) {
	  
	  	 	 	 	 // visit further child if close enough
	    if (box_dist1 * ANNkdMaxErr < ANNkdPointMK->max_key())
	      child[LO]->ann_search(box_dist1);
	  }
	  else {
	    if (box_dist2 * ANNkdMaxErr < ANNkdPointMK->max_key())
	      child[LO]->ann_search(box_dist2);
	  }
	}
	else {
	  child[LO]->ann_search(box_dist);// visit closer child first
					// distance to further box
	  box_dist1 = (ANNdist) ANN_SUM(box_dist,
		ANN_DIFF(ANN_POW(box_diff2), ANN_POW(cut_diff2)));

	
					// distance to further box
	  box_dist2 = (ANNdist) ANN_SUM(box_dist,
		ANN_DIFF(ANN_POW(box_diff2), ANN_POW(box_diff1)));
	  if (box_dist1 < box_dist2) {
	  
	  	 	 	 	 // visit further child if close enough
	    if (box_dist1 * ANNkdMaxErr < ANNkdPointMK->max_key())
	      child[HI]->ann_search(box_dist1);
	  }
	  else {
	    if (box_dist2 * ANNkdMaxErr < ANNkdPointMK->max_key())
	      child[HI]->ann_search(box_dist2);
	  }
	
	}
    }
    //FLOP(10)				// increment floating ops
    //SPL(1)				// one more splitting node visited
  }

  else {
    
    ANNdist box_dist1, box_dist2;
    
    ANNRectangle *RectLO = new ANNRectangle(cd_bnds[LO], cut_val, bnds1[LO], bnds1[HI], 
					    bnds2[LO], bnds2[HI], bnds3[LO], bnds3[HI]);
    ANNRectangle *RectHI = new ANNRectangle(cut_val, cd_bnds[HI], bnds1[LO], bnds1[HI], 
					    bnds2[LO], bnds2[HI], bnds3[LO], bnds3[HI]);
    ANNdist distLO, distLO1, distHI, distHI1;
    distLO = RectLO->DistPointRectangle (ANNkdQ[cut_dim], ANNkdQ[dim1], ANNkdQ[dim2], ANNkdQ[dim3]);
    distLO1 = RectLO->DistPointRectangle (-ANNkdQ[cut_dim], -ANNkdQ[dim1], -ANNkdQ[dim2], -ANNkdQ[dim3]);
    if (distLO1 < distLO) distLO = distLO1;

    distHI = RectHI->DistPointRectangle (ANNkdQ[cut_dim], ANNkdQ[dim1], ANNkdQ[dim2], ANNkdQ[dim3]);
    distHI1 = RectHI->DistPointRectangle (-ANNkdQ[cut_dim], -ANNkdQ[dim1], -ANNkdQ[dim2], -ANNkdQ[dim3]);
    if (distHI1 < distHI) distHI = distHI1;

    if (distLO < distHI) {
      box_dist1 = box_dist;
      box_dist2 = box_dist - distLO + distHI;
      child[LO]->ann_search(box_dist1);
      
      if (box_dist2 * ANNkdMaxErr < ANNkdPointMK->max_key())
	child[HI]->ann_search(box_dist2);
    }
    else {
      box_dist1 = box_dist;
      box_dist2 = box_dist - distHI + distLO;
      child[HI]->ann_search(box_dist1);

      if (box_dist2 * ANNkdMaxErr < ANNkdPointMK->max_key())
	child[LO]->ann_search(box_dist2);
    }
    delete RectLO;
    delete RectHI;
    //FLOP(10)				// increment floating ops
    //SPL(1)				// one more splitting node visited
    
  }
}

//----------------------------------------------------------------------
//  kd_leaf::ann_search - search points in a leaf node
//	Note: The unreadability of this code is the result of
//	some fine tuning to replace indexing by pointer operations.
//----------------------------------------------------------------------

void ANNkd_leaf::ann_search(ANNdist box_dist)
{

    register ANNdist dist;		// distance to data point
    register ANNcoord* pp;		// data coordinate pointer
    register ANNcoord* qq;		// query coordinate pointer
    register ANNdist min_dist;		// distance to k-th closest point
    register int d;
    
    register ANNcoord t, t1;
    double cosTheta, dotProduct, norm1, norm2;

    min_dist = ANNkdPointMK->max_key();	// k-th smallest distance so far

    for (int i = 0; i < n_pts; i++) {	// check points in bucket

	pp = ANNkdPts[bkt[i]];		// first coord of next data point
	qq = ANNkdQ; 	    		// first coord of query point
	dist = 0;

	for(d = 0; d < ANNkdDim; d++) {
	  //COORD(1)			// one more coordinate hit
	      //FLOP(4)			// increment floating ops

	    if (ANNTopology[d] == 2) {
	      t = fabs(*(qq) - *(pp));    	// compute length and adv coordinate
	      t1 = 2*PI*ANNScale[d] - t;
	      if (t > t1) t = t1;
	      t = ANN_POW(t);
	    }
	    else if (ANNTopology[d] == 1){
	      t = ANN_POW(*(qq) - *(pp));	// compute length and adv coordinate
	    }
	    else {
	      dotProduct = *(qq) * *(pp) + *(qq+1) * *(pp+1) + *(qq+2) * *(pp+2) + *(qq+3) * *(pp+3);
	      cosTheta = dotProduct/ANN_POW(ANNScale[d]);
	      
	      if (cosTheta > 1) {
		norm1 = *(qq) * *(qq) + *(qq+1) * *(qq+1) + *(qq+2) * *(qq+2) + *(qq+3) * *(qq+3);
		norm2 = *(pp) * *(pp) + *(pp+1) * *(pp+1) + *(pp+2) * *(pp+2) + *(pp+3) * *(pp+3);
		cosTheta = dotProduct/(sqrt(norm1*norm2));
	      }
	      t = acos(cosTheta);
	      t1 = acos(-cosTheta);
	      if (t > t1) t = t1;
	      t = ANN_POW(ANNScale[d]*t);
	      d = d + 3; qq++;qq++;qq++; pp++;pp++;pp++;
	    }

	    dist = ANN_SUM(dist, t);
	    qq++; pp++;

					// exceeds dist to k-th smallest?
	    if( dist > min_dist) {
	      break;
	    }
	}

	if (d >= ANNkdDim &&			// among the k best?
	   (ANN_ALLOW_SELF_MATCH || dist!=0)) {	// and no self-match problem
						// add it to the list
	    ANNkdPointMK->insert(dist, bkt[i]);
	    min_dist = ANNkdPointMK->max_key();
	}
    }
    //LEAF(1)				// one more leaf node visited
    //PTS(n_pts)				// increment points visited
    ANNptsVisited += n_pts;		// increment number of points visited
}
