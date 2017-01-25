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


#include <kautham/sampling/gaussiansampler.h>
#include <kautham/sampling/randomsample.h>
#include <vector>

namespace Kautham{



  GaussianSampler::GaussianSampler(char dim, KthReal s, WorkSpace *w){
	ws = w;
    genRand = new LCPRNG(3141592621, 1, 0, ((unsigned int)time(NULL) & 0xfffffffe) + 1);//LCPRNG(15485341);//15485341 is a big prime number
    setDim(dim);
	randgen = new RandomSampler(dim);
	sigma = s;
  }

  Sample* GaussianSampler::nextSample(){
	//cout<<"Sorry: GAUSSIAN SAMPLER not truly implemented!!"<<endl;
    //return _current= new RandomSample(dimension);
	/**/
	  Sample *s;
	  do{
		  s = getSample();
	  }while(s==NULL);
	  return s;
  }

  Sample* GaussianSampler::getSample(){
    	    int dim = (int)getDim();
		Sample *firstSample = randgen->nextSample();
		Sample *secondSample = new Sample(dim);
		std::vector<KthReal> *firstcoords;
		
		
		firstcoords = &(firstSample->getCoords());
		
		std::vector<KthReal> secondcoords(dim);


		KthReal v=-1.0;
		for(int j = 0; j < dim ; j++)
		{
			v=-1.0;
			while(v<0.0 || v>1.0)
				v = firstcoords->at(j) + 2*sigma*(genRand->d_rand()-0.5);
			secondcoords[j]=v;
		}
		secondSample->setCoords(secondcoords);
  
		//if first in collision...
		if(ws->collisionCheck(firstSample))
		{
			//and second free sample...
			if(!ws->collisionCheck(secondSample))
			{
				//return free sample
				delete firstSample;
				return secondSample;
				//_current = secondSample;
			}
		}
		//else first free sample..
		else
		{
			//and second in collision...
			if(ws->collisionCheck(secondSample))
			{
				//return free sample
				delete secondSample;
				return firstSample;
				//_current = firstSample;
			}
		}

	delete firstSample;
	delete secondSample;
	return NULL;
  }


}

