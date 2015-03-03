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


#ifdef KAUTHAM_USE_FCL

#include "ivfclelement.h"
#include <fcl/collision_data.h>
#include <fcl/collision.h>
#include <fcl/distance.h>


struct tri_info {
    vector<fcl::Vec3f> vertices;
    vector<fcl::Triangle> triangles;
};


void triang_CB(void *data, SoCallbackAction *action,
                             const SoPrimitiveVertex *vertex1,
                             const SoPrimitiveVertex *vertex2,
                             const SoPrimitiveVertex *vertex3) {
    tri_info* info = (tri_info*)data;

    const SbVec3f vertex[] = {vertex1->getPoint(),
                              vertex2->getPoint(),
                              vertex3->getPoint()};
    SbVec3f point;

    const SbMatrix  matrix = action->getModelMatrix();
    for (int i = 0; i < 3; ++i) {
        matrix.multVecMatrix(vertex[i],point);

        info->vertices.push_back(fcl::Vec3f(point[0],point[1],point[2]));
    }

    info->triangles.push_back(fcl::Triangle(info->vertices.size()-3,
                                            info->vertices.size()-2,
                                            info->vertices.size()-1));
}


namespace Kautham {

IVFCLElement::IVFCLElement(string visFile, string collFile, KthReal sc, bool useBBOX)
    : IVElement(visFile,collFile,sc,useBBOX) {
    if (!makeFCLModel()) throw invalid_argument("FCL model could not be initialized");
}


IVFCLElement::IVFCLElement(SoSeparator *visModel, SoSeparator *collModel, KthReal sc,
                           bool useBBOX) : IVElement(visModel,collModel,sc,useBBOX) {
    if (!makeFCLModel()) throw invalid_argument("FCL model could not be initialized");
}


IVFCLElement::~IVFCLElement() {
    delete FCLModel;
}


bool IVFCLElement::collideTo(Element* other) {
    Element::increaseCollCheckCounter();
    try {
        fcl::CollisionRequest request;
        fcl::CollisionResult result;

        fcl::collide(((IVFCLElement*)other)->getFCLModel(),FCLModel,request,result);

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

        fcl::distance(((IVFCLElement*)other)->getFCLModel(),FCLModel,request,result);

        return result.min_distance;
    } catch (...) {
        return 0.;
    }
}


SoSeparator *IVFCLElement::getIvFromFCLModel(bool tran) {
    float r,g,b,x,y,z;
    int k;
    LCPRNG gen(3141592621,1,0,((unsigned int)time(NULL)&0xfffffffe)+1);
    SbVec3f vertices[3];
    SoSeparator *root = new SoSeparator;
    SoSeparator *triangle;
    SoVertexProperty *vertexProperty;
    SoFaceSet *faceSet;

    fcl::BVHModel<fcl::OBBRSS> *geom;
    geom = (fcl::BVHModel<fcl::OBBRSS>*)FCLModel->collisionGeometry().get();

    for (int i = 0; i < geom->num_tris; ++i) {
        for (int j = 0; j < 3; ++j) {
            k = geom->tri_indices[i][j];

            x = geom->vertices[k][0];
            y = geom->vertices[k][1];
            z = geom->vertices[k][2];

            vertices[j].setValue(x,y,z);
        }

        vertexProperty = new SoVertexProperty;
        vertexProperty->normalBinding = SoNormalBinding::PER_FACE;
        vertexProperty->vertex.setValues(0,3,vertices);
        r = gen.d_rand();
        g = gen.d_rand();
        b = gen.d_rand();
        vertexProperty->orderedRGBA.setValue(SbColor(r,g,b).getPackedValue());

        faceSet = new SoFaceSet;
        faceSet->numVertices.setValue(3);
        faceSet->vertexProperty.setValue(vertexProperty);

        triangle = new SoSeparator();
        triangle->addChild(faceSet);

        root->addChild(triangle);
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


void IVFCLElement::setOrientation(float *ori) {
    IVElement::setOrientation(ori);

    mt::Scalar y,p,r;
    mt::Rotation(ori[0],ori[1],ori[2],ori[3]).getYpr(y,p,r);
    fcl::Matrix3f rotation;
    rotation.setEulerYPR(y,p,r);

    FCLModel->setRotation(rotation);
}


void IVFCLElement::setPosition(float *pos) {
    IVElement::setPosition(pos);

    fcl::Vec3f translation(pos[0],pos[1],pos[2]);

    FCLModel->setTranslation(translation);
}


bool IVFCLElement::makeFCLModel() {
    try {
        if (collision_ivModel()) {
            SoCallbackAction triAction;
            tri_info info;
            triAction.addTriangleCallback(SoShape::getClassTypeId(),
                                          triang_CB, (void*)&info);
            triAction.apply(collision_ivModel());

            fcl::BVHModel<fcl::OBBRSS> *geom = new fcl::BVHModel<fcl::OBBRSS>;
            geom->beginModel();
            geom->addSubModel(info.vertices, info.triangles);
            geom->endModel();

            const boost::shared_ptr<fcl::CollisionGeometry> cgeom(geom);

            FCLModel = new fcl::CollisionObject(cgeom);

            return true;
        }
    } catch(...) {
    }

    delete FCLModel;
    FCLModel = NULL;

    return false;
}
}
#endif
