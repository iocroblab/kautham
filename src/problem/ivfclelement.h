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


#if !defined(_IVFCLELEMENT_H)
#define _IVFCLELEMENT_H

#include "ivelement.h"
#include <fcl/BVH/BVH_model.h>
#include <fcl/collision_object.h>

#include <util/kthutil/kauthamdefs.h>
#include <Inventor/SoPrimitiveVertex.h>
#include <external/lcprng.h>
#include <Inventor/actions/SoCallbackAction.h>
#include <Inventor/SbLinear.h>
#include <Inventor/nodes/SoVertexProperty.h>
#include <Inventor/nodes/SoFaceSet.h>


namespace Kautham {

/** \addtogroup Problem
 *  @{
 */

typedef fcl::BVHModel<fcl::OBBRSS> FCL_Model;

class IVFCLElement:public IVElement {
public:
    IVFCLElement(string visFile, string collFile, KthReal sc, bool useBBOX);
    IVFCLElement(SoSeparator *visual_model,SoSeparator *collision_model, KthReal sc, bool useBBOX);
    SoSeparator* getIvFromFCLModel(bool tran = true);
    bool collideTo(Element* other);
    KthReal getDistanceTo(Element* other);
    inline FCL_Model *fclModel() {return fclmodel;}
    fcl::CollisionObject *getCollisionObject();

private:
    FCL_Model *fclmodel;
    bool makeFCLModel();
    struct tri_info {
        vector<fcl::Vec3f> vertices;
        vector<fcl::Triangle> triangles;
    };
    tri_info info;
    static void triang_CB(void *data, SoCallbackAction *action,
                          const SoPrimitiveVertex *vertex1,
                          const SoPrimitiveVertex *vertex2,
                          const SoPrimitiveVertex *vertex3);
};

/** @}   end of Doxygen module "Problem" */
}

#endif  //_IVFCLELEMENT_H
