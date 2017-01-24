
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

/* Author: Alexander Perez, Jan Rosell and Nestor Garcia Hidalgo */
 

#if !defined(_IVKINTXHAND_H)
#define _IVKINTXHAND_H

#include "inversekinematic.h"

/** \addtogroup libKin
 *  @{
 */

  class IvKinTxHand:public Kautham::InverseKinematic{
  public:
    IvKinTxHand(Robot* const rob);
    ~IvKinTxHand();
    INVKINECLASS type() {return TX90HAND;}
    string       name() {return "TX90Hand";}
    bool solve();
    bool setParameters();
  private:
    IvKinTxHand();
  };

  /** @}   end of Doxygen module "Util */
#endif  //_IVKINTXHAND_H
