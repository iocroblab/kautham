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

 
#include <kautham/sampling/sdksample.h>
#include <external/lcprng.h>
#include <sstream>
#include <iostream>
#include <time.h>

using namespace std;

namespace Kautham {
  LCPRNG* SDKSample::gen = NULL;

  SDKSample::SDKSample(unsigned d, unsigned long int code, int *indexes, bool random):Sample(d){
      KthReal r=0.0;
      this->code = code;
      _coords.resize(_dim);
      index = new int[_dim];
      for(unsigned int j=0; j< _dim; j++){
          if(random)
              r = (KthReal)gen->d_rand();
          else
              r=0.5;
          index[j]=indexes[j];
          _coords[j]=(indexes[j]+r)*SDKSample::sizeContainer;
      }
  }


  SDKSample::~SDKSample() {
      delete[] index;
      _coords.clear();
  }


  SDKSample::SDKSample(SDKSample *s):Sample(s)
  {
      code = s->code;
      index = new int[_dim];
      for(unsigned int j = 0; j < _dim; j++)
          index[j] = s->index[j];
  }


  string SDKSample::print(bool extend) {
      std::ostringstream s;
      if(extend)s << "code: " << code << "\t";
      for(unsigned i=0; i < _dim; i++){
          if(extend) {
              s << " coor[" ;
              s << i ;
              s << "]= ";
          }
          s << _coords[i] ;
          s << " "  ;
      }
      s << std::endl;
      return s.str();
  }


  string SDKSample::printNeighs() {
      std::ostringstream s;
      s << "code: " << code << "\tNeighs: ";
      s << _neighset.size() << " Codes: ";
      for(unsigned int i=0; i < _neighset.size(); i++){
          //s << ((SDKSample*)_neighset[i])->getCode() << ", "  ;
          s << _neighset[i] << ", "  ;
      }

      s << std::endl;
      return s.str();
  }
}
