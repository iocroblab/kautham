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
//	File:		kd_split.h
//	Programmer:	Sunil Arya and David Mount
//	Last modified:	03/04/98 (Release 0.1)
//	Description:	Methods for splitting kd-trees
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

#ifndef DNN_KD_SPLIT_H
#define DNN_KD_SPLIT_H

#include "kd_tree.h"			// kd-tree definitions

//----------------------------------------------------------------------
//  External entry points
//	These are all splitting procedures for kd-trees.
//----------------------------------------------------------------------

void kd_split(				// standard (optimized) kd-splitter
    ANNpointArray	pa,		// point array (unaltered)
    ANNidxArray		pidx,		// point indices (permuted on return)
    const ANNorthRect	&bnds,		// bounding rectangle for cell
    int			n,		// number of points
    int			dim,		// dimension of space
    int			&cut_dim,	// cutting dimension (returned)
    ANNcoord		&cut_val,	// cutting value (returned)
    int			&n_lo);		// num of points on low side (returned)

void midpt_split(			// midpoint kd-splitter
    ANNpointArray	pa,		// point array (unaltered)
    ANNidxArray		pidx,		// point indices (permuted on return)
    const ANNorthRect	&bnds,		// bounding rectangle for cell
    int			n,		// number of points
    int			dim,		// dimension of space
    int			&cut_dim,	// cutting dimension (returned)
    ANNcoord		&cut_val,	// cutting value (returned)
    int			&n_lo);		// num of points on low side (returned)

void sl_midpt_split(			// sliding midpoint kd-splitter
    ANNpointArray	pa,		// point array (unaltered)
    ANNidxArray		pidx,		// point indices (permuted on return)
    const ANNorthRect	&bnds,		// bounding rectangle for cell
    int			n,		// number of points
    int			dim,		// dimension of space
    int			&cut_dim,	// cutting dimension (returned)
    ANNcoord		&cut_val,	// cutting value (returned)
    int			&n_lo);		// num of points on low side (returned)

void fair_split(			// fair-split kd-splitter
    ANNpointArray	pa,		// point array (unaltered)
    ANNidxArray		pidx,		// point indices (permuted on return)
    const ANNorthRect	&bnds,		// bounding rectangle for cell
    int			n,		// number of points
    int			dim,		// dimension of space
    int			&cut_dim,	// cutting dimension (returned)
    ANNcoord		&cut_val,	// cutting value (returned)
    int			&n_lo);		// num of points on low side (returned)

void sl_fair_split(			// sliding fair-split kd-splitter
    ANNpointArray	pa,		// point array (unaltered)
    ANNidxArray		pidx,		// point indices (permuted on return)
    const ANNorthRect	&bnds,		// bounding rectangle for cell
    int			n,		// number of points
    int			dim,		// dimension of space
    int			&cut_dim,	// cutting dimension (returned)
    ANNcoord		&cut_val,	// cutting value (returned)
    int			&n_lo);		// num of points on low side (returned)

#endif
