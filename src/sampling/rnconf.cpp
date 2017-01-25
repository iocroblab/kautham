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

 
 
#include <kautham/sampling/rnconf.h>
#include <stdexcept>

using namespace std;

namespace Kautham {



//  RnConf::RnConf():RnConf(1){
//
//  }

  RnConf::RnConf(unsigned int d):Conf(Rn){
		dim = d;
		coord.resize(dim);
	}
	
//	RnConf::~RnConf() {
//		coord.clear();
//	}

  void RnConf::reDim(unsigned int dim){
    this->dim = dim;
    coord.clear();
    coord.resize(dim);
  }

	std::string RnConf::print() {
			std::ostringstream s;
            for(unsigned i=0;i < dim; i++){
				s << "coord[" << i << "]=" << coord[i] << ";" << std::endl ;
			}
			return s.str();
	}
	
  KthReal RnConf::getDistance2(Conf* conf){
    return getDistance2(*((RnConf*) conf));
  }

//! Returns the squared distance to a configuration in the respective space metric.
  KthReal RnConf::getDistance2(RnConf& conf){
//    if( conf == NULL ) return -1.0;
    if(conf.getType() == Rn && conf.getDim() == this->dim){
      KthReal dist = (KthReal) 0.0;
      for(unsigned int i = 0; i < coord.size(); i++ )
        //pow((coord.at(i) - conf.getCoordinate(i)),2);
        dist += ( coord.at(i) - conf.getCoordinate(i)) * (coord.at(i) - conf.getCoordinate(i));

      return dist;
    }
    return -1;
  }

  KthReal RnConf::getDistance2(Conf* conf, std::vector<KthReal>& weights){
    return getDistance2(*(RnConf*)conf, weights);
  }

  //! Returns the weighted squared distance to a configuration in the respective space metric.
  KthReal RnConf::getDistance2(RnConf& conf, std::vector<KthReal>& weights){
//    if( conf == NULL ) return -1.0;
    if(conf.getType() == Rn){
      KthReal dist = (KthReal) 0.0;
      KthReal diff = (KthReal) 0.0;
      if(weights.size() == 1){
        // Using the same weight for each coordinate component
        for(unsigned int i = 0; i < coord.size(); i++ ){
          diff = coord.at(i) - conf.getCoordinate(i);
          dist += diff * weights.at(0) * diff * weights.at(0); //pow((diff * weights.at(0)),2);
        }
      }else if(weights.size() == this->dim ){
        // Each coordinate has their own weigth to will be applied
        for(unsigned int i = 0; i < coord.size(); i++ ){
          diff = coord.at(i) - conf.getCoordinate(i);
          dist += diff * weights.at(i) * diff * weights.at(i);  //pow((diff*weights.at(i)),2);
        }
      }else
        dist = -1;
      return dist;
    }
    return -1;
  }

  RnConf RnConf::interpolate(RnConf& rn, KthReal fraction){
    RnConf tmpC(rn.getDim());

    // Interpolation in space.
    vector<KthReal>& other = rn.getCoordinates();
    for(unsigned int i = 0; i < coord.size(); i++)
      tmpC.coord.at(i) = coord.at(i) + fraction*(other.at(i) - coord.at(i));
   
    return tmpC;
  }

}

