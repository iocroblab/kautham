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


#if !defined(_GAUSSIANSAMPLER_H)
#define _GAUSSIANSAMPLER_H

#include <kautham/sampling/sampler.h>
#include <external/lcprng.h>
#include <kautham/sampling/randomsampler.h>



#include <kautham/problem/workspace.h>

namespace Kautham{


/** \addtogroup Sampling
 *  @{
 */

  //! This is the Sampler implementation using a random numbers generator.
  class GaussianSampler : public Sampler {
  public:
    //! Unique constructor.
    GaussianSampler(char dim, KthReal s, WorkSpace *w);

    //! getSample tries to obtain a gaussian sample, returns NULL otherwise
    Sample* getSample();

    //! Implements the virtual Sample method.   
    Sample* nextSample();

  private:
	  LCPRNG* genRand;
	  RandomSampler *randgen;
	  WorkSpace *ws;
	  KthReal sigma;
  };


  /** @}   end of Doxygen module "Sampling" */
}

#endif  //_RANDOMSAMPLER_H
