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
 
 
#if !defined(_IVPQPELEMENT_H)
#define _IVPQPELEMENT_H


#include <Inventor/SoPrimitiveVertex.h>
#include <Inventor/actions/SoCallbackAction.h>
#include <Inventor/SbLinear.h>
#include <Inventor/nodes/SoVertexProperty.h>
#include <Inventor/nodes/SoFaceSet.h>

#include <external/pqp/PQP.h>
#include <external/lcprng.h>

#include <kautham/problem/ivelement.h>
#include <kautham/util/kthutil/kauthamdefs.h>


namespace Kautham {

/** \addtogroup Problem
 *  @{
 */

  class IVPQPElement:public IVElement {
  public:

      IVPQPElement(string visFile, string collFile, KthReal sc, bool useBBOX);
      IVPQPElement(SoSeparator *visual_model,SoSeparator *collision_model, KthReal sc, bool useBBOX);
	  SoSeparator* getIvFromPQPModel(bool tran = true);
	  bool collideTo(Element* other);
	  KthReal getDistanceTo(Element* other);
      inline PQP_Model* pqpModel(){return pqpmodel;}
  private:
	  PQP_Model *pqpmodel;
	  bool makePQPModel();
      struct tri_info {
          tri_info(PQP_Model* model, int cnt = 0) {
              pqp_model = model;
              tri_cnt = cnt;
          }
          PQP_Model* pqp_model;
          int tri_cnt;
      };
      static void triang_CB(void *data, SoCallbackAction *action,
                            const SoPrimitiveVertex *vertex1,
                            const SoPrimitiveVertex *vertex2,
                            const SoPrimitiveVertex *vertex3);
  };

  /** @}   end of Doxygen module "Problem" */
}

#endif  //_IVPQPELEMENT_H
