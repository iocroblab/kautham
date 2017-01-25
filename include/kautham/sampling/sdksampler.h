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


#if !defined(_SDKSAMPLER_H)
#define _SDKSAMPLER_H

#include <kautham/sampling/sampler.h>
#include <kautham/sampling/sequence.h>
#include <external/lcprng.h>
#include <kautham/sampling/sdksample.h>
#include <kautham/sampling/wmat.h>
#include <kautham/sampling/tmat.h>


namespace Kautham{
/** \addtogroup Sampling
 *  @{
 */
  class SDKSampler : public Sampler {
  public:
      //! This is the unique constructor. In order to create an SDK sampler
      //! is needed to provide the dimension and the maximum partition level.
      //! Is not possible create an SDKSampler without this information.
      SDKSampler(int dim, int maxLevel);

      //! Destructor.
      ~SDKSampler();

      //! Implements the virtual function. \sa Sample. For ommision, the samples
      //! will be created with their coordinates randomly into their respective cell.
      Sample* nextSample();

      //! This is provided for convenience. In the SDK strategy, the samples
      //! could be created with their coordinates randomly or centered into
      //! their respective cells.
      Sample* nextSample(bool random);

      //! Overwites the Sample method in order to retrieves a known sample.
      //! Really, this method create a new sample whose code will be the code parameter.
      //! Take care with this method because it could create a new sample
      //! with the same code as an existing sample
      Sample* getSample(unsigned long int code, bool random = true);

      //! Returns a pointer to T matrix.
      TMat* getTMat(){return _tMat;}

      //! Returns a pointer to W matrix.
      WMat* getWMat(){return _wMat;}

      Sequence* getSeqGenerator(){return sdkSequence;}

  private:
      Sequence* sdkSequence;
      //! This is the size of an M-Cell.
      double sizeContainer;

      //! This is the grid partition level.
      int M;

      //! This is a static pointer to the W matrix. This is a unique object used for any sample in sampleset.
      WMat* _wMat;

      //! Pointer to T matrix.
      TMat *_tMat;

      //! Pointer to object that generates a random number sequence.
      LCPRNG* generator;
  };
  /** @}   end of Doxygen module "Sampling" */
}
#endif  //_SDKSAMPLER_H
