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

 
#include <kautham/sampling/conf.h>
#include <stdexcept>

using namespace std;


namespace Kautham {






	LCPRNG* Conf::genRand = new LCPRNG(3141592621, 1, 0, ((unsigned int)time(NULL) & 0xfffffffe) + 1);//LCPRNG(15485341);//15485341 is a big prime number
	Conf::Conf(CONFIGTYPE typ):type(typ){

	}

  Conf::~Conf(){
    coord.clear();
  }
	
  bool Conf::setCoordinates(std::vector<KthReal>& coordinates) {
    if(coordinates.size() == dim ){
      for(unsigned int i=0;i<dim;i++)
				this->coord[i]=coordinates[i];

      return true;
    }else{
		  return false;	
		}
	}

  KthReal Conf::getCoordinate(unsigned int index) {
      if (index<dim) return this->coord[index];

      throw out_of_range("");
	}

  //! Returns the distance to a configuration in the respective space metric.
  KthReal Conf::getDistance(Conf* conf){
    return sqrt(getDistance2(conf));
  }

  //! Returns the weighted distance to a configuration in the respective space metric.
  KthReal Conf::getDistance(Conf* conf, std::vector<KthReal>& weights){
    return sqrt(getDistance2(conf, weights));
  }


  /** @}   end of Doxygen module "libSampling" */
}


