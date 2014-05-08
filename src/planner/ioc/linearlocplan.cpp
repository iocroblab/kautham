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

/* Author: Alexander Perez, Jan Rosell, Nestor Garcia Hidalgo */

 

#include <problem/workspace.h>
#include <sampling/sampling.h>
#include "linearlocplan.h"


namespace Kautham {

namespace IOC{

  LinearLocalPlanner::LinearLocalPlanner(SPACETYPE stype, Sample *init, Sample *goal, WorkSpace *ws, KthReal st )
    :LocalPlanner(stype,init,goal,ws,st) {
    vanderMethod = true;

    _idName = "Linear";
	}

	bool LinearLocalPlanner::canConect(){
    if(initSamp() == NULL || goalSamp() == NULL) return false; //problem not set.

    if(initSamp()->getDim() != wkSpace()->getNumRobControls()) return false;  //sample is not come from the workspace.
    
    KthReal dist = 0;
    int wkDim = wkSpace()->getNumRobControls();
    Sample *tmpSample = NULL;
    vector<KthReal> steps;
    steps.resize(wkDim);
    vector<KthReal> coord;
    coord.resize(wkDim);


    dist = distance(initSamp(), goalSamp());

    int maxsteps = (dist/stepSize())+2; //the 2 is necessary to always reduce the distance...

    if( !vanderMethod )		{
//      for(int i = 0; i < wkDim; i++)
//        steps[i] = goalSamp()->getCoords()[i] - initSamp()->getCoords()[i];
//
//      for(int k=0;k<wkDim;k++)
//        steps[k] = steps[k]/maxsteps; //this is the delta step for each dimension
		
      for(int i=0; i<maxsteps; i++)	{

//        for(int k = 0; k < wkDim; k++)
//          coord[k] = initSamp()->getCoords()[k] + i*steps[k];
//
//        tmpSample.setCoords(coord);

        tmpSample = initSamp()->interpolate(goalSamp(),i/(KthReal)maxsteps);

        if(wkSpace()->collisionCheck(tmpSample)) return false; 
      }
    }else{ //method == VANDERCORPUT
			//find how many bits are needed to code the maxsteps
			int b= ceil(log10( (double) maxsteps) / log10( 2.0 ));
			int finalmaxsteps = (0x01<<b);
			//cout<<"maxsteps= "<<maxsteps<<" b= "<<b<<" finalmaxsteps = "<<finalmaxsteps<<endl;

			//index is the index of the Van der Corput sequence, using b bites the sequence has 2^b elements
			//dj is the bit j of the binary representation of the index
			//rj are the elements of the sequence
			//deltarj codes the jumps between successive elements of the sequence
			double rj=0;
			for(int index = 0; index < finalmaxsteps ; index++){
				int dj;
				double deltaj;
				double newrj=0;
				for(int j = 0; j < b ; j++){
					dj = (index >> j) & 0x01;
					newrj += ((double)dj /  (double)(0x01<<(j+1)) );
				}
				deltaj = newrj - rj;
				
				rj = newrj;

//        for(int k=0; k < wkDim; k++)
//					coord[k] = initSamp()->getCoords()[k] + rj*steps[k];;
//
//        tmpSample.setCoords(coord);


        tmpSample = initSamp()->interpolate(goalSamp(),(KthReal)rj);

        if(wkSpace()->collisionCheck(tmpSample))
          return false; 
			}
		}
    return true;
	}

  KthReal LinearLocalPlanner::distance(Sample* from, Sample* to)
  {
    return _wkSpace->distanceBetweenSamples(*from, *to, CONFIGSPACE);
  }
}
}



