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

  
#include <kautham/sampling/se2conf.h>
#include <cmath>

#if !defined(M_PI)
#define M_PI 3.1415926535897932384626433832795
#endif



namespace Kautham {


	SE2Conf::SE2Conf():Conf(SE2) {
		dim = 3;
		coord.resize(dim);	
	}
	
	SE2Conf::~SE2Conf() {
		coord.clear();
	}

  void SE2Conf::setPos(double pos[2]){
		for(int i=0;i<3;i++)
			coord[i] = pos[i];
	}

	double* SE2Conf::getPos() {
		double *p = new double[2];
		for(int i=0;i<2;i++)
			p[i] = coord[i];
		return p;
	}
	
	void SE2Conf::setAngle(double angle){
		coord[3] = angle;
	}

	double SE2Conf::getAngle() {
		return coord[3];
	}
	
	std::string SE2Conf::print() {
		std::ostringstream s;
        for(unsigned i=0;i < dim; i++)
			s << "x = " << coord[i] << ";" << std::endl ;

		return s.str();
	}

	//bool SE2Conf::setCoordinates(char indexes[], char dimP, char dimS, double length){
	//	return true;
	//}

  double SE2Conf::getDistance2(Conf* conf){
    if( conf == NULL ) return -1.0;
    if(conf->getType() == SE2){
      double dist = (conf->getCoordinate(0)- coord.at(0)) * (conf->getCoordinate(0)- coord.at(0));
      dist += (conf->getCoordinate(1)- coord.at(1)) * (conf->getCoordinate(1)- coord.at(1));
      double t = fabs(conf->getCoordinate(2)- coord.at(2));
      double t1 = std::min(t,(double)(2.0*M_PI - t));
      return dist += t1 * t1;
    }
    return -1;
  }

    //! Returns the weighted squared distance to a configuration in the respective space metric.
  double SE2Conf::getDistance2(Conf* conf, std::vector<double>& weights){
    if( conf == NULL ) return -1.0;

    double dist = (double) 0.0;
    if(conf->getType() == SE2){
      if(weights.size() == 1){
        double dist = (conf->getCoordinate(0)- coord.at(0)) * (conf->getCoordinate(0)- coord.at(0));
        dist += (conf->getCoordinate(1)- coord.at(1)) * (conf->getCoordinate(1)- coord.at(1));
        double t = fabs(conf->getCoordinate(2)- coord.at(2));
	      double t1 = min(t, (double)(2.0*M_PI - t));
        return dist += (t1 * weights.at(0)) * (t1 * weights.at(0));
      }else if(weights.size() == this->dim ){
        // Each coordinate has their own weigth to will be applied
        double dist = (conf->getCoordinate(0)- coord.at(0)) * weights.at(0)
                     * (conf->getCoordinate(0)- coord.at(0)) * weights.at(0);
        dist += (conf->getCoordinate(1)- coord.at(1))* weights.at(1) 
              * (conf->getCoordinate(1)- coord.at(1))* weights.at(1);

        double t = fabs(conf->getCoordinate(2)- coord.at(2));
        double t1 = min(t, (double)(2.0*M_PI - t));
        return dist += (t1 * weights.at(2)) * (t1 * weights.at(2));
      }else
        dist = (double)-1.0;
      return dist;
    }
    return (double)-1.0;
  }

  SE2Conf  SE2Conf::interpolate(SE2Conf& se2, double fraction){
    SE2Conf tmpC;
    tmpC.coord.at(0) = coord.at(0) + fraction*(se2.coord.at(0) - coord.at(0));
    tmpC.coord.at(1) = coord.at(1) + fraction*(se2.coord.at(1) - coord.at(1));
    tmpC.coord.at(2) = coord.at(2) + fraction*(se2.coord.at(2) - coord.at(2));
    tmpC.coord.at(2) = min(tmpC.coord.at(2),(double)(2.0* M_PI - tmpC.coord.at(2)));
    return tmpC;
  }

}

