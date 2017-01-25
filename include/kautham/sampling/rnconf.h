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

 

#if !defined(_RNCONF_H)
#define _RNCONF_H

#include <kautham/sampling/conf.h>
#include <string>
#include <sstream>
#include <iostream>

using namespace std;

namespace Kautham {


/** \addtogroup Sampling
 *  @{
 */

	class RnConf : public Conf {
	public:
//    RnConf();
    RnConf(unsigned int d = 0);
//		~RnConf();
    void        reDim(unsigned int dim);
		std::string print();
    KthReal     getDistance2(Conf* conf);
    KthReal     getDistance2(RnConf& conf);
    KthReal     getDistance2(Conf* conf, std::vector<KthReal>& weights);
    KthReal     getDistance2(RnConf& conf, std::vector<KthReal>& weights);

    //! Returns the interpolated configuration based on coordinates.
    RnConf     interpolate(RnConf& rn, KthReal fraction);
	private:
//		RnConf();
	};

    /** @}   end of Doxygen module "Sampling" */
}

#endif  //_RNCONF_H

