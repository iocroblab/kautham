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

/* Author: Alexander Perez, Jan Rosell, Nestor Garcia Hidalgo */

 

#if !defined(_IVKIN2DRR_H)
#define _IVKIN2DRR_H

#include "inversekinematic.h"


/** \addtogroup libKin
 *  @{
 */
class IvKin2DRR : public Kautham::InverseKinematic{
  public:
    IvKin2DRR(Robot* const rob);
    ~IvKin2DRR();
    INVKINECLASS    type() {return RR2D;}
    string          name() {return "RR2D";}
    bool            solve();
    bool            setParameters();
    RobLayout&      getRobLayout(vector<KthReal> &target);
    inline void     setConfiguration(const bool lefty){_robLefty = lefty;}
  private:
    IvKin2DRR();
    bool            _robLefty;
    KthReal         _tcp[2];
    KthReal         _llong[2];
  };

/** @}   end of Doxygen module "Util */
#endif  //_IVKIN2DRR_H
