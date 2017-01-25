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


#include <kautham/sampling/randomsampler.h>
#include <kautham/sampling/randomsample.h>
#include <vector>

namespace Kautham{



  LCPRNG* RandomSample::gen = NULL;

  RandomSampler::RandomSampler(char dim){
    genRand = new LCPRNG(3141592621, 1, 0, ((unsigned int)time(NULL) & 0xfffffffe) + 1);//LCPRNG(15485341);//15485341 is a big prime number
    setDim(dim);
    RandomSample::gen=genRand;
  }

  Sample* RandomSampler::nextSample(){
    return _current= new RandomSample(dimension);
  }

  //Sample* RandomSampler::getSample(){
  //  return _current;
  //}

}

