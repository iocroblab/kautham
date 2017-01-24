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

#ifndef ROBWEIGHT_H
#define ROBWEIGHT_H

#include <kautham/util/kthutil/kauthamdefs.h>

namespace Kautham{


/** \addtogroup Sampling
 *  @{
 */

  class RobWeight{
  public:
      RobWeight(unsigned int dim);
      inline void                   setRnDim(unsigned int d){_rn.resize(d);}
      inline void                   setSE3Weight(KthReal t, KthReal r){_se3[0] = t;_se3[1] = r;}
      inline KthReal*               getSE3Weight(){return _se3;}
      inline std::vector<KthReal>&  getRnWeights(){return _rn;}
      void                          setRnWeigh(unsigned int i, KthReal w);
  private:
      //! This _se3 attribute is an array where the 
      //! first component is translational weight and the
      //! second one is the rotational weight.
      KthReal                       _se3[2];

      //! This attribute is a vector that contains
      //! the respective weight for each joint.
      std::vector<KthReal>          _rn;
  };

  /** @}   end of Doxygen module "Sampling" */
}

#endif // ROBWEIGHT_H
