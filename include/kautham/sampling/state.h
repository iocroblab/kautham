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

/* Author: Nestor Garcia Hidalgo */



#if !defined(_STATE_H)
#define _STATE_H

#include <kautham/sampling/sample.h>


namespace Kautham{

/** \addtogroup Sampling
 *  @{
 */

  //! Class State is used to represent a configuration of the State Space.
  //! The first element contains a Sample of the robot controls.
  //! The second element contains a Sample of the obstacle controls.
  class State: public std::pair<Sample*,Sample*> {
  public:

      //! Initializes the robot sample with the robcoords contents.
      bool    setRob(vector<KthReal>& robcoords);

      //! Copies the robot sample.
      bool    setRob(Sample& robsample);

      //! Initializes the obstacle sample with the obscoords contents.
      bool    setObs(vector<KthReal>& obscoords);

      //! Copies the obstacle sample.
      bool    setObs(Sample& obssample);

  private:

  };

/** @}   end of Doxygen module "Sampling" */

}

#endif  //_STATE_H
