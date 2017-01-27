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


#if !defined(_SE3CONF_H)
#define _SE3CONF_H


#include <string>
#include <sstream>
#include <iostream>

#include <mt/mt.h>

#include <external/lcprng.h>

#include <kautham/sampling/se2conf.h>


using namespace std;

namespace Kautham {


/** \addtogroup Sampling
 *  @{
 */

	//!	 SE3Conf is an abstractions of a configuration of the SE3 space.
	//!  This class stores their coordinates internally as a 3 values for 
  //!  translational part and quaternion (qx, qy, qz, w) as rotational part.
	class SE3Conf : public Conf {
	public:
		SE3Conf();
		//~SE3Conf();
    bool              setCoordinates(SE2Conf* conf);
    bool              setCoordinates(std::vector<KthReal>& coordinates);

    KthReal           getDistance2(Conf* conf);
    KthReal           getDistance2(SE3Conf& conf);

    //! Redefininig this method in order to include the rotational weight instead of
    //! the same weight for each coordinate component.
    KthReal           getDistance2( Conf* conf, KthReal& transWeight, KthReal& rotWeight );
    KthReal           getDistance2( SE3Conf& conf, KthReal& transWeight, KthReal& rotWeight );
    inline KthReal    getDistance( Conf* conf, KthReal& transWeight, KthReal& rotWeight ){
                          return sqrt(getDistance2(conf, transWeight, rotWeight));}
    KthReal           getDistance2( Conf* conf, std::vector<KthReal>& weights );
    KthReal           getDistance2( SE3Conf& conf, std::vector<KthReal>& weights );
    string            print();
    void              setPos(vector<KthReal>& pos);
	vector<KthReal>&  getPos();
    void              setOrient(vector<KthReal>& ori);
    vector<KthReal>&  getOrient();
    bool              setAngle(KthReal angle);
    KthReal           getAngle();
    bool              setAxis(vector<KthReal>& axis);
	vector<KthReal>&  getAxisAngle();
    bool              setAxisAngle(vector<KthReal>& axis, KthReal angle);
    
    //! Returns the interpolated configuration based on coordinates.
    SE3Conf     interpolate(SE3Conf& se3, KthReal fraction);

    //! Returns the sampled parameters.
    vector<KthReal> getParams();

    //! Changes the representation of the rotational part of the configuration
    //! from axis-angle to Quaternion. This methos is useful because the XML
    //! files provide de most understandable axis-angle format.
    static void fromAxisToQuaternion(vector<KthReal> &values);

    static void fromAxisToQuaternion(KthReal values[]);
  private:
    void normalizeQ();
    vector<KthReal>  _pos;
    vector<KthReal>  _ori;
    vector<KthReal>  _axisAn;
	};

    /** @}   end of Doxygen module "Sampling" */
}

#endif  //_SE3CONF_H

