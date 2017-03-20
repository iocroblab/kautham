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

#include <kautham/problem/ivfclelement.h>
#include <kautham/util/kthutil/kauthamdefs.h>
#include <map>

#include <fcl/collision.h>
#include <fcl/distance.h>
#include <fcl/BVH/BVH_model.h>

#include <Inventor/SoPrimitiveVertex.h>
#include <Inventor/SbColor.h>
#include <Inventor/actions/SoCallbackAction.h>
#include <Inventor/nodes/SoVertexProperty.h>
#include <Inventor/nodes/SoIndexedFaceSet.h>

struct GeomData {
    vector<fcl::Vec3f> vertices;
    vector<fcl::Triangle> triangles;
    unsigned int addVertex(const float *point) {
        fcl::Vec3f vertex(point[0],point[1],point[2]);
        map<fcl::Vec3f,unsigned int>::const_iterator it = verticesMap.find(vertex);
        if (it != verticesMap.end()) {
            return it->second;
        } else {
            unsigned int index = vertices.size();
            vertices.push_back(vertex);
            verticesMap.insert(pair<fcl::Vec3f,unsigned int>(vertex,index));
            return index;
        }
    }
private:
    struct cmpVertex {
        bool operator()(const fcl::Vec3f& a, const fcl::Vec3f& b) const {
            return (a[0] < b[0]) ||
                   (a[1] < b[1] && a[0] == b[0]) ||
                   (a[2] < b[2] && a[1] == b[1] && a[0] == b[0]);
        }
    };
    map<fcl::Vec3f,unsigned int,cmpVertex> verticesMap;
};

void triangleCB(void *data, SoCallbackAction *action,
                const SoPrimitiveVertex *vertex1,
                const SoPrimitiveVertex *vertex2,
                const SoPrimitiveVertex *vertex3) {
    GeomData *geomData = (GeomData*)data;
    SbVec3f point;
    const SbVec3f vertex[] = {vertex1->getPoint(),
                              vertex2->getPoint(),
                              vertex3->getPoint()};
    const SbMatrix  matrix = action->getModelMatrix();
    unsigned int index[3];
    for (int i = 0; i < 3; ++i) {
        matrix.multVecMatrix(vertex[i],point);

        index[i] = geomData->addVertex(point.getValue());
    }

    geomData->triangles.push_back(fcl::Triangle(index[0],index[1],index[2]));
}


namespace Kautham {

LCPRNG IVFCLElement::gen = LCPRNG(3141592621,1,0,((unsigned int)time(NULL)&0xfffffffe)+1);

IVFCLElement::IVFCLElement(string visFile, string collFile, KthReal sc, bool useBBOX)
    : IVElement(visFile,collFile,sc,useBBOX),FCLModel(NULL) {
    if (!makeFCLModel()) throw invalid_argument("FCL model could not be initialized");
}


IVFCLElement::IVFCLElement(SoSeparator *visModel, SoSeparator *collModel, KthReal sc,
                           bool useBBOX) : IVElement(visModel,collModel,sc,useBBOX),
    FCLModel(NULL) {
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
    fcl::BVHModel<fcl::OBBRSS> *geom;
    geom = (fcl::BVHModel<fcl::OBBRSS>*)FCLModel->collisionGeometry().get();
    unsigned int nVerts = geom->num_vertices;
    unsigned int nTris = geom->num_tris;

    if (nVerts > 2 && nTris > 0) {
        SbVec3f *vertices = new SbVec3f[nVerts];
        for (unsigned int i = 0; i < nVerts; ++i) {
            for (unsigned int j = 0; j < 3; ++j) {
                vertices[i][j] = geom->vertices[i][j];
            }
        }

        uint32_t *colors = new uint32_t[nTris];
        int32_t *coordIndices = new int32_t[nTris*4];
        int32_t *materialIndices = new int32_t[nTris];
        for (unsigned int i = 0; i < nTris; ++i) {
            for (unsigned int j = 0; j < 3; ++j) {
                coordIndices[4*i+j] = geom->tri_indices[i][j];
            }
            coordIndices[4*i+3] = SO_END_FACE_INDEX;
            materialIndices[i] = i;
            colors[i] = SbColor(gen.d_rand(),gen.d_rand(),gen.d_rand()).getPackedValue();
        }

        SoVertexProperty *vertexProperty(new SoVertexProperty);
        vertexProperty->vertex.setValues(0,nVerts,vertices);
        vertexProperty->orderedRGBA.setValues(0,nTris,colors);
        vertexProperty->materialBinding.setValue(SoVertexProperty::PER_FACE);

        SoIndexedFaceSet *indexedFaceSet(new SoIndexedFaceSet());
        indexedFaceSet->coordIndex.setValues(0,nTris*4,coordIndices);
        indexedFaceSet->materialIndex.setValues(0,nTris,materialIndices);
        indexedFaceSet->vertexProperty.setValue(vertexProperty);

        SoSeparator *root(new SoSeparator);

        if (tran) {
            root->addChild(getTrans());
            root->addChild(getRot());
        }
        root->addChild(indexedFaceSet);

        return root;
    } else {
        return NULL;
    }
}


void IVFCLElement::setOrientation(float *ori) {
    IVElement::setOrientation(ori);

    fcl::Matrix3f rotation;
    fcl::Quaternion3f(ori[3],ori[0],ori[1],ori[2]).toRotation(rotation);

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
            GeomData geomData;
            triAction.addTriangleCallback(SoShape::getClassTypeId(),
                                          triangleCB,(void*)&geomData);
            triAction.apply(collision_ivModel());

            fcl::BVHModel<fcl::OBBRSS> *model = new fcl::BVHModel<fcl::OBBRSS>;
            model->beginModel();
            model->addSubModel(geomData.vertices,geomData.triangles);
            model->endModel();

            const std::shared_ptr<fcl::CollisionGeometry> geom(model);

            FCLModel = new fcl::CollisionObject(geom);

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
