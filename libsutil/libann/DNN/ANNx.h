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
//	File:		ANNx.h
//	Programmer:	Sunil Arya and David Mount
//	Last modified:	03/04/98 (Release 0.1)
//	Description:	Internal include file for ANN
//
//	These declarations are of use in manipulating some of
//	the internal data objects appearing in ANN, but are not
//	needed for applications just using the nearest neighbor
//	search.
//
//	Typical users of ANN should not need to access this file.
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

#ifndef ANNx_H
#define ANNx_H

#include <iomanip>			// I/O manipulators
#include "ANN.h"			// ANN includes

//----------------------------------------------------------------------
//  Global constants and types
//----------------------------------------------------------------------
enum	{LO=0, HI=1};			// splitting indices
enum	{IN=0, OUT=1};			// shrinking indices

					// what to do in case of error
enum ANNerr {ANNwarn = 0, ANNabort = 1};

//----------------------------------------------------------------------
//  Maximum number of points to visit
//	We have an option for terminating the search early if the
//	number of points visited exceeds some threshold.  If the
//	threshold is 0 (its default)  this means there is no limit
//	and the algorithm applies its normal termination condition.
//----------------------------------------------------------------------

extern int		ANNmaxPtsVisited;// maximum number of pts visited
extern int		ANNptsVisited;	// number of pts visited in search

//----------------------------------------------------------------------
//  Global function declarations
//----------------------------------------------------------------------

void annError(			// ANN error routine
    char		*msg,		// error message
    ANNerr		level);		// level of error

void annPrintPt(		// print a point
    ANNpoint		pt,		// the point
    int			dim,		// the dimension
	std::ostream		&out);		// output stream

//----------------------------------------------------------------------
//  Orthogonal (axis aligned) rectangle
//	Orthogonal rectangles are represented by two points, one
//	for the lower left corner (min coordinates) and the other
//	for the upper right corner (max coordinates).
//
//	The constructor initializes from either a pair of coordinates,
//	pair of points, or another rectangle.  Note that all constructors
//	allocate new point storage.  The destructor deallocates this
//	storage.
//
//	BEWARE: Orthogonal rectangles should be passed ONLY BY REFERENCE.
//	(C++'s default copy constructor will not allocate new point
//	storage, then on return the destructor free's storage, and then
//	you get into big trouble in the calling procedure.)
//----------------------------------------------------------------------

class ANNorthRect {
public:
    ANNpoint	lo;			// rectangle lower bounds
    ANNpoint	hi;			// rectangle upper bounds
//
    ANNorthRect(			// basic constructor
	int dd,				// dimension of space
	ANNcoord l=0,			// default is empty
	ANNcoord h=0)
    {  lo = annAllocPt(dd, l);  hi = annAllocPt(dd, h);  }

    ANNorthRect(			// (almost a) copy constructor
	int dd,				// dimension
	const ANNorthRect &r)		// rectangle to copy
    {  lo = annCopyPt(dd, r.lo);  hi = annCopyPt(dd, r.hi);  }

    ANNorthRect(			// construct from points
	int dd,				// dimension
	ANNpoint l,			// low point
	ANNpoint h)			// hight point
    {  lo = annCopyPt(dd, l);  hi = annCopyPt(dd, h);  }

    ~ANNorthRect()			// destructor
    {  delete lo;  delete hi;  lo = hi = NULL;  }

    ANNbool inside(int dim, ANNpoint p);// is point p inside rectangle?
};

void annAssignRect(		// assign one rect to another
    int			dim,		// dimension (both must be same)
    ANNorthRect		&dest,		// destination (modified)
    const ANNorthRect	&source);	// source

//----------------------------------------------------------------------
//  Orthogonal (axis aligned) halfspace
//	An orthogonal halfspace is represented by an integer cutting
//	dimension cd, coordinate cutting value, cv, and side, sd, which
//	is either +1 or -1.  Our convention is that point q lies in the
//	(closed) halfspace if (q[cd] - cv)*sd >= 0.
//----------------------------------------------------------------------

class ANNorthHalfSpace {
public:
    int		cd;			// cutting dimension
    ANNcoord	cv;			// cutting value
    int		sd;			// which side
//
    ANNorthHalfSpace()			// default constructor
    {  cd = 0;  cv = 0;  sd = 0;  }

    ANNorthHalfSpace(			// basic constructor
	int		cdd,		// dimension of space
	ANNcoord	cvv,		// cutting value
	int		sdd)		// side
    {  cd = cdd;  cv = cvv;  sd = sdd;  }

    ANNbool in(ANNpoint q) const	// is q inside halfspace?
    {  return  (ANNbool) ((q[cd] - cv)*sd >= 0);  }

    ANNbool out(ANNpoint q) const	// is q outside halfspace?
    {  return  (ANNbool) ((q[cd] - cv)*sd < 0);  }

    ANNdist dist(ANNpoint q) const	// (squared) distance from q
    {  return  (ANNdist) ANN_POW(q[cd] - cv);  }

    void setLowerBound(int d, ANNpoint p)// set to lower bound at p[i]
    {  cd = d;  cv = p[d];  sd = +1;  }

    void setUpperBound(int d, ANNpoint p)// set to upper bound at p[i]
    {  cd = d;  cv = p[d];  sd = -1;  }

    void project(ANNpoint &q)		// project q (modified) onto halfspace
    {  if (out(q)) q[cd] = cv;  }
};

					// array of halfspaces
typedef ANNorthHalfSpace *ANNorthHSArray;

#endif
