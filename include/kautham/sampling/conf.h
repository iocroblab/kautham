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

#if !defined(_CONF_H)
#define _CONF_H

#include <vector>
#include <string>
#include "external/lcprng.h"
#include <kautham/util/kthutil/kauthamdefs.h>
#include <sstream>
#include <iostream>


using namespace std;

namespace Kautham {


/** \addtogroup Sampling
 *  @{
 */


	class Conf {
	public:
		Conf(CONFIGTYPE typ);
    virtual ~Conf();
    virtual bool      setCoordinates(std::vector<KthReal>& coordinates);

    //! Returns the distance
    KthReal               getDistance(Conf* conf);
    KthReal               getDistance(Conf* conf, std::vector<KthReal>& weights );

    //! Returns the squared distance.
    virtual KthReal       getDistance2(Conf* conf)=0;
    virtual KthReal       getDistance2(Conf* conf, std::vector<KthReal>& weights )=0;

    virtual std::string print()= 0;
    inline  std::vector<KthReal>& getCoordinates(){return coord;}
    KthReal               getCoordinate(unsigned int index );
    inline unsigned int   getDim(){return dim;}
    static LCPRNG*        genRand;
    inline CONFIGTYPE     getType(){return type;}
	protected:
    vector<KthReal>       coord;
    unsigned int          dim;
    CONFIGTYPE            type;
	};


    /** @}   end of Doxygen module "Sampling" */

}

#endif  //_CONF_H



