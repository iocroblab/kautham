/*************************************************************************\
   Copyright 2014-2024  Institute of Industrial and Control Engineering (IOC)
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


#include <kautham/sampling/sdksampler.h>
#include <external/lcprng.h>			//class for linear congruential generators

namespace Kautham {
  int SDKSample::M = 0;
  float SDKSample::sizeContainer = 0.0;
  
  SDKSampler::SDKSampler(int dim, int maxLevel){
      generator = new LCPRNG(3141592621, 1, 0, ((unsigned int)time(NULL) & 0xfffffffe) + 1);//LCPRNG(15485341);//15485341 is a big prime number
      setDim(dim);
      _tMat = new TMat(dim);
      _wMat = new WMat(dim,maxLevel);
      SDKSample::sizeContainer = 1.0f/(0x01<<maxLevel);
      SDKSample::M = maxLevel;
      sdkSequence = new Sequence(dim, maxLevel);
      sdkSequence->setT(*_tMat);
      sdkSequence->setW(*_wMat);
      SDKSample::gen = generator;
  }

  Sample* SDKSampler::nextSample() {
      return nextSample(true);
  }

  Sample* SDKSampler::nextSample(bool random) {
      return getSample(sdkSequence->getSequenceCode(),random);
  }

  Sample* SDKSampler::getSample(unsigned long int code, bool random ){
      _current = new SDKSample(dimension,code,sdkSequence->getIndexes(code),random);

      return _current;
  }

  //! Destructor.
  SDKSampler::~SDKSampler()
  {

  }
}
