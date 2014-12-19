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


#include "ivfclelement.h"
#include <fcl/collision_data.h>
#include <fcl/collision.h>
#include <fcl/distance.h>

namespace Kautham {

IVFCLElement::IVFCLElement(string visFile, string collFile, KthReal sc, bool useBBOX)
    :IVElement(visFile,collFile,sc,useBBOX) {
    fclmodel = NULL;
    makeFCLModel();
}


IVFCLElement::IVFCLElement(SoSeparator *visual_model,SoSeparator *collision_model, KthReal sc, bool useBBOX)
    :IVElement(visual_model,collision_model,sc,useBBOX) {
    fclmodel = NULL;
    makeFCLModel();
}


// get triangles from inventor models
void IVFCLElement::triang_CB(void *data, SoCallbackAction *action,
                             const SoPrimitiveVertex *vertex1,
                             const SoPrimitiveVertex *vertex2,
                             const SoPrimitiveVertex *vertex3) {
    tri_info* info = (tri_info*)data;

    SbVec3f vertex[] = {vertex1->getPoint(),
                        vertex2->getPoint(),
                        vertex3->getPoint()};

    const SbMatrix  matrix = action->getModelMatrix();
    for (int i = 0; i < 3; ++i) {
        matrix.multVecMatrix(vertex[i],vertex[i]);
        info->vertices.push_back(fcl::Vec3f(vertex[i][0],
                                            vertex[i][1],
                                            vertex[i][2]));
    }

    info->triangles.push_back(fcl::Triangle(info->vertices.size()-3,
                                            info->vertices.size()-2,
                                            info->vertices.size()-1));
}


SoSeparator* IVFCLElement::getIvFromFCLModel(bool tran) {
    LCPRNG* gen = new LCPRNG(3141592621, 1, 0, ((unsigned int)time(NULL) & 0xfffffffe) + 1);//LCPRNG(15485341);//15485341 is a big prime number
    float r,g;
    float x,y,z;
    int npunts = 3;

    SbVec3f *vertices = new SbVec3f[3];
    SoSeparator *root = new SoSeparator;
    int numTriangles = info.triangles.size();
    SoSeparator **triangle = new SoSeparator*[numTriangles];
    SoVertexProperty **vertexProperty = new SoVertexProperty*[numTriangles];
    SoFaceSet **faceSet = new SoFaceSet*[numTriangles];

    int k;
    for (int i = 0; i < numTriangles; ++i) {
        for (int j = 0; j < 3; ++j) {
            k = ((fcl::Triangle)info.triangles.at(i))[j];

            x = ((fcl::Vec3f)info.vertices.at(k))[0];
            y = ((fcl::Vec3f)info.vertices.at(k))[1];
            z = ((fcl::Vec3f)info.vertices.at(k))[2];

            vertices[j].setValue(x,y,z);
        }

        triangle[i] = new SoSeparator();
        triangle[i]->ref();

        vertexProperty[i] = new SoVertexProperty;
        vertexProperty[i]->normalBinding = SoNormalBinding::PER_FACE;
        vertexProperty[i]->vertex.setValues(0, 3, vertices);
        r = 0.5f +((float)gen->d_rand()/2.0f);
        g = 0.5f +((float)gen->d_rand()/2.0f);
        vertexProperty[i]->orderedRGBA.setValue(SbColor(r,g,(float)0.0).getPackedValue());

        faceSet[i] = new SoFaceSet;
        faceSet[i]->numVertices.setValues(0, 1, &npunts);
        faceSet[i]->vertexProperty.setValue(vertexProperty[i]);
        triangle[i]->addChild(faceSet[i]);

        triangle[i]->unrefNoDelete();

        root->addChild(triangle[i]);
    }
    if (tran) {
        SoSeparator* temp = new SoSeparator;
        temp->addChild(getTrans());
        temp->addChild(getRot());
        temp->addChild(root);
        return temp;
    } else {
        return root;
    }
}


fcl::CollisionObject *IVFCLElement::getCollisionObject() {
    float *ori = getOrientation();
    mt::Scalar y, p, r;
    mt::Rotation(ori[0],ori[1],ori[2],ori[3]).getYpr(y,p,r);
    fcl::Matrix3f mat;
    mat.setEulerYPR(y,p,r);

    float *pos = getPosition();
    fcl::Vec3f vect(pos[0],pos[1],pos[2]);
    fcl::Transform3f pose(mat,vect);

    const boost::shared_ptr<fcl::CollisionGeometry> geom(fclmodel);
    fcl::CollisionObject *obj = new fcl::CollisionObject(geom,pose);
    return obj;
}


bool IVFCLElement::collideTo(Element* other) {
    Element::increaseCollCheckCounter();
    try {
        fcl::CollisionRequest request;
        fcl::CollisionResult result;

        fcl::collide(getCollisionObject(),
                     ((IVFCLElement*)other)->getCollisionObject(),
                     request,result);

        return result.isCollision();
    } catch (...) {
        return true;
    }
}


KthReal IVFCLElement::getDistanceTo(Element* other) {
    Element::increaseCollCheckCounter();
    try {
        fcl::DistanceRequest request;
        fcl::DistanceResult result;

        fcl::distance(getCollisionObject(),
                     ((IVFCLElement*)other)->getCollisionObject(),
                     request,result);

        return (KthReal)result.min_distance;
    } catch (...) {
        return (KthReal)0.;
    }
}


bool IVFCLElement::makeFCLModel() {
    if (collision_ivModel() != NULL) {
        if (this->fclmodel == NULL) {
            SoCallbackAction triAction;

            triAction.addTriangleCallback(SoShape::getClassTypeId(),
                                          triang_CB, (void*)&info);
            triAction.apply(collision_ivModel());

            fclmodel = new FCL_Model();
            fclmodel->beginModel();
            fclmodel->addSubModel(info.vertices, info.triangles);
            fclmodel->endModel();

            return true;
        }
    }
    return false;
}
}
