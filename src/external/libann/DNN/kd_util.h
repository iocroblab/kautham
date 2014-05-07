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
//	File:		kd_util.h
//	Programmer:	Sunil Arya and David Mount
//	Last modified:	03/04/98 (Release 0.1)
//	Description:	Common utilities for kd- trees
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

#ifndef DNN_kd_util_H
#define DNN_kd_util_H

#include "kd_tree.h"			// kd-tree declarations

//----------------------------------------------------------------------
//  externally accessible functions
//----------------------------------------------------------------------

double annAspectRatio(			// compute aspect ratio of box
    int			dim,		// dimension
    const ANNorthRect	&bnd_box);	// bounding cube

void annEnclRect(			// compute smallest enclosing rectangle
    ANNpointArray	pa,		// point array
    ANNidxArray		pidx,		// point indices
    int			n,		// number of points
    int			dim,		// dimension
    ANNorthRect	&bnds);			// bounding cube (returned)

void annEnclCube(			// compute smallest enclosing cube
    ANNpointArray	pa,		// point array
    ANNidxArray		pidx,		// point indices
    int			n,		// number of points
    int			dim,		// dimension
    ANNorthRect	&bnds);			// bounding cube (returned)

class ANNRectangle
{
  ANNcoord     	bnds1[2];		// bounding values
  ANNcoord      bnds2[2];		// of the rectangle
  ANNcoord     	bnds3[2];		// in four dimensions
  ANNcoord     	bnds4[2];
public:
    ANNRectangle(			// constructor
		 ANNcoord lb1, ANNcoord hb1,
		 ANNcoord lb2, ANNcoord hb2,
		 ANNcoord lb3, ANNcoord hb3,
		 ANNcoord lb4, ANNcoord hb4 );

    ~ANNRectangle();			// destructor

    ANNdist DistPointRectangle (ANNcoord p1, ANNcoord p2, ANNcoord p3, ANNcoord p4);      
};

ANNdist annBoxDistance(			// compute distance from point to box
    const ANNpoint	q,		// the point
    const ANNpoint	lo,		// low point of box
    const ANNpoint	hi,		// high point of box
    int			dim,		// dimension of space	
    double 		*Scale,		// scaling coefficients
    int 		*Topology);	// topology of space

ANNcoord annSpread(			// compute point spread along dimension
    ANNpointArray	pa,		// point array
    ANNidxArray		pidx,		// point indices
    int			n,		// number of points
    int			d);		// dimension to check

void annMinMax(				// compute min and max coordinates along dim
    ANNpointArray	pa,		// point array
    ANNidxArray		pidx,		// point indices
    int			n,		// number of points
    int			d,		// dimension to check
    ANNcoord&		min,		// minimum value (returned)
    ANNcoord&		max);		// maximum value (returned)

int annMaxSpread(			// compute dimension of max spread
    ANNpointArray	pa,		// point array
    ANNidxArray		pidx,		// point indices
    int			n,		// number of points
    int			dim);		// dimension of space

void annMedianSplit(			// split points along median value
    ANNpointArray	pa,		// points to split
    ANNidxArray		pidx,		// point indices
    int			n,		// number of points
    int			d,		// dimension along which to split
    ANNcoord		&cv,		// cutting value
    int			n_lo);		// split into n_lo and n-n_lo

void annPlaneSplit(			// split points by a plane
    ANNpointArray	pa,		// points to split
    ANNidxArray		pidx,		// point indices
    int			n,		// number of points
    int			d,		// dimension along which to split
    ANNcoord		cv,		// cutting value
    int			&br1,		// first break (values < cv)
    int			&br2);		// second break (values == cv)

void annBoxSplit(			// split points by a box
    ANNpointArray	pa,		// points to split
    ANNidxArray		pidx,		// point indices
    int			n,		// number of points
    int			dim,		// dimension of space
    ANNorthRect		&box,		// the box
    int			&n_in);		// number of points inside (returned)

int annSplitBalance(			// determine balance factor of a split
    ANNpointArray	pa,		// points to split
    ANNidxArray		pidx,		// point indices
    int			n,		// number of points
    int			d,		// dimension along which to split
    ANNcoord		cv);		// cutting value

void annBox2Bnds(			// convert inner box to bounds
    const ANNorthRect	&inner_box,	// inner box
    const ANNorthRect	&bnd_box,	// enclosing box
    int			dim,		// dimension of space
    int			&n_bnds,	// number of bounds (returned)
    ANNorthHSArray	&bnds);		// bounds array (returned)

void annBnds2Box(			// convert bounds to inner box
    const ANNorthRect	&bnd_box,	// enclosing box
    int			dim,		// dimension of space
    int			n_bnds,		// number of bounds
    ANNorthHSArray	bnds,		// bounds array
    ANNorthRect		&inner_box);	// inner box (returned)

#endif
