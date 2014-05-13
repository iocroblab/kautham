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

/* Author: Alexander Perez, Jan Rosell */


#if !defined(_SDKGAUSSIANSAMPLER_H)
#define _SDKGAUSSIANSAMPLER_H

#include "sampler.h"
#include "sdksampler.h"
#include <external/lcprng.h>
//#include "sdksample.h"
#include "haltonsampler.h"
#include "randomsampler.h"
#include "sampleset.h"
#include <problem/workspace.h>


namespace Kautham{


/** \addtogroup Sampling
 *  @{
 */


  class GaussianLikeSampler : public Sampler {
  public:
    //! This is the unique constructor. In order to create an Gaussian like SDK sampler
    //! is needed to provide the dimension and the maximum partition level.
    //! Is not possible create an SDKSampler without this information.
    GaussianLikeSampler(char dim, char maxLevel, WorkSpace *w);

    //! Destructor.
    ~GaussianLikeSampler();

    //! Implements the virtual function. \sa Sample. For ommision, the samples
    //! will be created with their coordinates randomly into their respective cell.
    Sample* nextSample();
	Sample *setcolor(Sample *smp, vector<unsigned int> *N);
    Sample* nextSample(bool random);
	void print();
	//!delete samples from sampleset ss
	inline void clear(){ss->clear();};

    //! Pointer to object that generates a random number sequence.
    LCPRNG* generator;

	SDKSampler *sdkgen;
	RandomSampler *randgen;
	HaltonSampler *haltgen;

	SampleSet *ss;
	vector<Sample*> vectfree;
	int itfree;
	WorkSpace *ws;
	int	_maxNeighs;
	int	_maxSamples;
  };


  /** @}   end of Doxygen module "Sampling" */
}

#endif  //_SDKSAMPLER_H
