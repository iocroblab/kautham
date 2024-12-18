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

/* Author: Nestor Garcia Hidalgo */


#if !defined(_IVFCLELEMENT_H)
#define _IVFCLELEMENT_H

#include <kautham/problem/ivelement.h>
#include <fcl/narrowphase/collision_object.h>
#include <external/lcprng.h>

namespace Kautham {

/** \addtogroup Problem
 *  @{
 */

class IVFCLElement:public IVElement {
public:
    IVFCLElement(string visFile, string collFile, double sc, bool useBBOX);

    IVFCLElement(SoSeparator *visModel, SoSeparator *collModel,
                 double sc, bool useBBOX);

    ~IVFCLElement();

    bool collideTo(Element* other) const override ;

    double getDistanceTo(Element* other) const override ;

    SoSeparator* getIvFromFCLModel(bool tran = true);

    const fcl::CollisionObjectd *getFCLModel() {return FCLModel;}

    void setPosition(double *pos);

    void setOrientation(double *ori);

private:
    fcl::CollisionObjectd *FCLModel;

    bool makeFCLModel();

    static LCPRNG gen;
};

/** @}   end of Doxygen module "Problem" */
}

#endif  //_IVFCLELEMENT_H
