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

/* Author: Alexander Perez, Jan Rosell, Nestor Garcia Hidalgo */

 

#if !defined(_ELEMENT_H)
#define _ELEMENT_H

#include <kautham/util/kthutil/kauthamdefs.h>
#include <kautham/problem/odeelement.h>

namespace Kautham {
/** \addtogroup Problem
 *  @{
 */

  class Element {
  public:

      //! Used for dynamic simulation.
      ode_element         ode;

    virtual void          setPosition(double pos[3]) = 0;

    //! Sets the orientation. Remember that orientation is a quaternion
    virtual void          setOrientation(double ori[4]) = 0;
    virtual bool          collideTo(Element* other) const = 0;
    virtual double       getDistanceTo(Element* other) const = 0;
    inline double*       getOrientation(){return orientation;}
    inline double*       getPosition(){return position;}
    inline double        getScale() const {return scale;}
    static void           resetCollCheckCounter();
    static unsigned int   getCollCheckCounter();
    static void           increaseCollCheckCounter();
  protected:
	  double position[3];
	  double orientation[4];
	  double scale;
  private:
    static unsigned int   _countCollCheck;
  };

  /** @}   end of Doxygen module "Problem" */
}

#endif  //_ELEMENT_H
