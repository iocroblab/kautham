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

 
#if !defined(_SE2CONF_H)
#define _SE2CONF_H

#include <kautham/sampling/conf.h>
#include <string>
#include <sstream>
#include <iostream>
#include <kautham/util/kthutil/kauthamdefs.h>

using namespace std;

namespace Kautham {

/** \addtogroup Sampling
 *  @{
 */

	class SE2Conf : public Conf {
	public:

		SE2Conf();
		~SE2Conf();
    bool    setCoordinates(std::vector<KthReal> coordinates);
    KthReal getDistance2(Conf* conf);
    KthReal getDistance2(Conf* conf, std::vector<KthReal>& weights);

    //! Returns the interpolated configuration based on coordinates.
    SE2Conf     interpolate(SE2Conf& se2, KthReal fraction);
		std::string print();
    void		setPos(KthReal pos[3]);
		KthReal*	getPos();

		void		  setAngle(KthReal angle);
		KthReal		getAngle();
	};

    /** @}   end of Doxygen module "Sampling" */
}

#endif  //_SE2CONF_H

