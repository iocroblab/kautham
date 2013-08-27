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

// This file is modified from
//----------------------------------------------------------------------
//      File:           ANN.cc
//      Programmer:     Sunil Arya and David Mount
//      Last modified:  03/04/98 (Release 0.1)
//      Description:    Methods for ANN.h and ANNx.h
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

#include "DNN/ANNx.h"			// all ANN includes

//----------------------------------------------------------------------
//  Point methods
//----------------------------------------------------------------------

//----------------------------------------------------------------------
//  Distance utility.
//	(Note: In the nearest neighbor search, most distances are
//	computed using partial distance calculations, not this
//	procedure.)
//----------------------------------------------------------------------

ANNdist annDist(			// interpoint squared distance
    int			dim,
    ANNpoint		p,
    ANNpoint		q)
{
    register int d;
    register ANNcoord diff;
    register ANNcoord dist;

    dist = 0;
    for (d = 0; d < dim; d++) {
	diff = p[d] - q[d];
	dist = ANN_SUM(dist, ANN_POW(diff));
    }
    return dist;
}

//----------------------------------------------------------------------
//  annPrintPoint() prints a point to a given output stream.
//----------------------------------------------------------------------

void annPrintPt(			// print a point
    ANNpoint		pt,		// the point
    int			dim,		// the dimension
    std::ostream		&out)		// output stream
{
    for (int j = 0; j < dim; j++) {
       	out << pt[j];
	if (j < dim-1) out << " ";
    }
}

//----------------------------------------------------------------------
//  Point allocation/deallocation:
//
//	Because points (somewhat like strings in C) are stored
//	as pointers.  Consequently, creating and destroying
//	copies of points may require storage allocation.  These
//	procedures do this.
//
//	annAllocPt() and annDeallocPt() allocate a deallocate
//	storage for a single point, and return a pointer to it.
//
//	annAllocPts() allocates an array of points as well a place
//	to store their coordinates, and initializes the points to
//	point to their respective coordinates.  It allocates point
//	storage in a contiguous block large enough to store all the
//	points.  It performs no initialization.
//
// 	annDeallocPts() should only be used on point arrays allocated
//	by annAllocPts since it assumes that points are allocated in
//	a block.
//
//	annCopyPt() copies a point taking care to allocate storage
//	for the new point.
//
//	annAssignRect() assigns the coordinates of one rectangle to
//	another.  The two rectangles must have the same dimension
//	(and it is not possible to test this here).
//----------------------------------------------------------------------

ANNpoint annAllocPt(int dim, ANNcoord c)	// allocate 1 point
{
    ANNpoint p = new ANNcoord[dim];
    for (int i = 0; i < dim; i++) p[i] = c;
    return p;
}
   
ANNpointArray annAllocPts(int n, int dim)	// allocate n pts in dim
{
    ANNpointArray pa = new ANNpoint[n];		// allocate points
    ANNpoint	  p  = new ANNcoord[n*dim];	// allocate space for coords
    for (int i = 0; i < n; i++) {
	pa[i] = &(p[i*dim]);
    }
    return pa;
}

void annDeallocPt(ANNpoint &p)			// deallocate 1 point
{
  delete [] p;
  p = NULL;
}
   
void annDeallocPts(ANNpointArray &pa)		// deallocate points
{
  delete [] pa[0];				// dealloc coordinate storage
  delete [] pa;				// dealloc points

  pa = NULL;
}
   
ANNpoint annCopyPt(int dim, ANNpoint source)	// copy point
{
    ANNpoint p = new ANNcoord[dim];
    for (int i = 0; i < dim; i++) p[i] = source[i];
    return p;
}
   
						// assign one rect to another
void annAssignRect(int dim, ANNorthRect &dest, const ANNorthRect &source)
{
    for (int i = 0; i < dim; i++) {
	dest.lo[i] = source.lo[i];
	dest.hi[i] = source.hi[i];
    }
}

						// is point inside rectangle?
ANNbool ANNorthRect::inside(int dim, ANNpoint p)
{
    for (int i = 0; i < dim; i++) {
	if (p[i] < lo[i] || p[i] > hi[i]) return ANNfalse;
    }
    return ANNtrue;
}

//----------------------------------------------------------------------
//  Error handler
//----------------------------------------------------------------------

void annError(char *msg, ANNerr level)
{
    if (level == ANNabort) {
	std::cerr << "ANN: ERROR------->" << msg << "<-------------ERROR\n";
		//JAN 2010-10-15 (exit commented)
	std::cerr << "max number of allowed neighs is 2^(ANN_STARTING_INDEX)\n";
	std::cerr << "change the value of ANN_STARTING_INDEX in multiann.h\n";
	exit(1);
    }
    else {
	std::cerr << "ANN: WARNING----->" << msg << "<-------------WARNING\n";
    }
}

//----------------------------------------------------------------------
//  Limit on number of points visited
//	We have an option for terminating the search early if the
//	number of points visited exceeds some threshold.  If the
//	threshold is 0 (its default)  this means there is no limit
//	and the algorithm applies its normal termination condition.
//	This is for applications where there are real time constraints
//	on the running time of the algorithm.
//----------------------------------------------------------------------

int		ANNmaxPtsVisited = 0;	// maximum number of pts visited
int		ANNptsVisited;		// number of pts visited in search

//----------------------------------------------------------------------
//  Global function declarations
//----------------------------------------------------------------------

void annMaxPtsVisit(		// set limit on max. pts to visit in search
    int                 maxPts)		// the limit
{
    ANNmaxPtsVisited = maxPts;
}
