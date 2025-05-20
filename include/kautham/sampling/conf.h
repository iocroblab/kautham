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

#if !defined(_CONF_H)
#define _CONF_H

#include <vector>
#include <string>
#include <kautham/util/kthutil/kauthamdefs.h>
#include <sstream>
#include <iostream>

class LCPRNG;
using namespace std;

namespace Kautham {


/** \addtogroup Sampling
 *  @{
 */


	class Conf {
	public:
		Conf(CONFIGTYPE typ);
    virtual ~Conf();
    virtual bool      setCoordinates(std::vector<double>& coordinates);

    //! Returns the distance
    double               getDistance(Conf* conf);
    double               getDistance(Conf* conf, std::vector<double>& weights );

    //! Returns the squared distance.
    virtual double       getDistance2(Conf* conf)=0;
    virtual double       getDistance2(Conf* conf, std::vector<double>& weights )=0;

    virtual std::string print()= 0;
    inline  std::vector<double>& getCoordinates(){return coord;}
    double               getCoordinate(unsigned int index ) const;
    inline unsigned int   getDim(){return dim;}
    static LCPRNG*        genRand;
    inline CONFIGTYPE     getType(){return type;}
	protected:
    vector<double>       coord;
    unsigned int          dim;
    CONFIGTYPE            type;
	};


    /** @}   end of Doxygen module "Sampling" */

}

#endif  //_CONF_H



