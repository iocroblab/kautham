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

/* Author: Nestor Garcia Hidalgo */


#include <kautham/sampling/state.h>


namespace Kautham{

  bool State::setRob(vector<KthReal>& robcoords) {
      try {
          first = new Sample(robcoords.size());
          first->setCoords(robcoords);
          return true;
      } catch(...) {
          return false;
      }
  }


  bool State::setRob(Sample& robsample) {
      try {
          first = &robsample;
          return true;
      } catch(...) {
          return false;
      }
  }


  bool State::setObs(vector<KthReal>& obscoords) {
      try {
          second = new Sample(obscoords.size());
          second->setCoords(obscoords);
          return true;
      } catch(...) {
          return false;
      }
  }


  bool State::setObs(Sample& obssample) {
      try {
          second = &obssample;
          return true;
      } catch(...) {
          return false;
      }
  }

}
