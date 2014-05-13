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

#if !defined(_IVELEMENT_H)
#define _IVELEMENT_H


#include <sstream>
#include <fstream>
#include "element.h"
#include <external/gdiam/gdiam.hpp>
#include <Inventor/SoPrimitiveVertex.h>
#include <Inventor/fields/SoSFVec3f.h>
#include <Inventor/fields/SoSFRotation.h>
#include <Inventor/nodes/SoTranslation.h>
#include <Inventor/nodes/SoRotation.h>
#include <Inventor/nodes/SoMaterial.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoScale.h>
#include <Inventor/nodes/SoSphere.h>
#include <Inventor/actions/SoCallbackAction.h>
#include <Inventor/nodes/SoCoordinate3.h>
#include <Inventor/nodes/SoIndexedFaceSet.h>
#include <Inventor/actions/SoWriteAction.h>
#include <Inventor/SbLinear.h>
#include <util/kthutil/kauthamdefs.h>


namespace Kautham {

/** \addtogroup Problem
 *  @{
 */

class IVElement : public Element {
  public:

      IVElement(string ivfile, string collision_ivfile, float sc, bool useBBOX);
      IVElement(SoSeparator *visual_model, SoSeparator *collision_model, float sc, bool useBBOX);
	  void setColor(KthReal c[3]);
    void setPosition(KthReal pos[3]);
	  void setOrientation(KthReal ori[4]);
	  SbMatrix orientationMatrix();
	  SoSeparator* ivModel(bool tran=false);
      SoSeparator* collision_ivModel(bool tran=false);
      bool collideTo(Element* other);
      static void point_CB(void *data, SoCallbackAction *action,
                           const SoPrimitiveVertex *v);
      static void triangle_CB(void *data, SoCallbackAction *action,
                                         const SoPrimitiveVertex *v1,
                                         const SoPrimitiveVertex *v2,
                                         const SoPrimitiveVertex *v3);
      SoSeparator *BBOX(SoSeparator *model, float sc, string filename = "");
	  KthReal getDistanceTo(Element* other);
    inline SoTranslation* getTrans(){return trans;}
    inline SoRotation* getRot(){return rot;}
    inline SoMaterial* getMaterial(){return color;}

  private:
	  SoSeparator   *ivmodel;
      SoSeparator   *collision_ivmodel;
	  SoSFVec3f     *scaVec;
	  SoSFVec3f     *posVec;
	  SoSFRotation  *rotVec;
	  SoTranslation *trans;
	  SoRotation    *rot;
	  SoMaterial    *color;
      SoScale       *sca;
  };

/** @}   end of Doxygen module "Problem" */
}

#endif  //_IVELEMENT_H