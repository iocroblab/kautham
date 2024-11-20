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

#include <kautham/sampling/robweight.h>

namespace Kautham{



  RobWeight::RobWeight(unsigned int dim) {
    setRnDim(dim);
    for(unsigned int i =0; i < dim; i++)
      _rn.at(i) = (KthReal) 1.0;
    _se3[0] = (KthReal) 1.0;
    _se3[1] = (KthReal) 1.0;
  }

  void RobWeight::setRnWeigh(unsigned int i, KthReal w){
    if( i < _rn.size())
      _rn.at(i) = w;
  }

}
