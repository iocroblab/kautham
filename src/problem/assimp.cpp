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

#include "assimp.h"

#if defined(KAUTHAM_USE_ASSIMP)

#include <assimp/scene.h>
#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <iostream>
#include <Inventor/nodes/SoTransform.h>
#include <Inventor/nodes/SoCoordinate3.h>
#include <Inventor/nodes/SoIndexedFaceSet.h>
#include <Inventor/nodes/SoMaterial.h>

using namespace std;



SoTransform *getTransform(aiMatrix4x4 matrix) {
    aiVector3D scaling;
    aiQuaternion rotation;
    aiVector3D position;
    matrix.Decompose(scaling,rotation,position);

    SoTransform *transform = new SoTransform;
    transform->translation.setValue((float)position[0],(float)position[1],(float)position[2]);
    transform->rotation.setValue((float)rotation.x,(float)rotation.y,(float)rotation.z,(float)rotation.w);
    transform->scaleFactor.setValue((float)scaling[0],(float)scaling[1],(float)scaling[2]);

    return transform;
}


SoMaterial *getMaterial(aiMaterial *material) {
    SoMaterial *soMat = new SoMaterial;

    aiColor3D color;
    float value;

    //Add diffuse color
    if (AI_SUCCESS == material->Get(AI_MATKEY_COLOR_DIFFUSE,color)) {
        soMat->diffuseColor.setValue((float)color.r,(float)color.g,(float)color.b);
    }

    //Add specular color
    if (AI_SUCCESS == material->Get(AI_MATKEY_COLOR_SPECULAR,color)) {
        soMat->specularColor.setValue((float)color.r,(float)color.g,(float)color.b);
    }

    //Add ambient color
    if (AI_SUCCESS == material->Get(AI_MATKEY_COLOR_SPECULAR,color)) {
        soMat->ambientColor.setValue((float)color.r,(float)color.g,(float)color.b);
    }

    //Add ambient color
    if (AI_SUCCESS == material->Get(AI_MATKEY_COLOR_EMISSIVE,color)) {
        soMat->emissiveColor.setValue((float)color.r,(float)color.g,(float)color.b);
    }

    //Add transparency
    if (AI_SUCCESS == material->Get(AI_MATKEY_OPACITY,value)) {
        soMat->transparency.setValue(1.0-(float)value);
    }

    //Add shininess
    if (AI_SUCCESS == material->Get(AI_MATKEY_SHININESS_STRENGTH,value)) {
        soMat->shininess.setValue((float)value);
    }

    return soMat;
}


SoCoordinate3 *getCoords(aiMesh *mesh) {
    float vertices[mesh->mNumVertices][3];
    for (int j = 0; j < mesh->mNumVertices; ++j) {
        for (int k = 0; k < 3; ++k) {
            vertices[j][k] = (float)mesh->mVertices[j][k];
        }
    }

    SoCoordinate3 *coords = new SoCoordinate3;
    coords->point.setValues(0,mesh->mNumVertices,vertices);

    return coords;
}


SoIndexedFaceSet *getFaceSet(aiMesh *mesh) {
    int indices[4*mesh->mNumFaces];
    for (int j = 0; j < mesh->mNumFaces; ++j) {
        if (mesh->mFaces[j].mNumIndices != 3) {
            cout << "error; numIndeces different from 3" << endl;
        }
        for (int k = 0; k < 3; ++k) {
            indices[4*j+k] = mesh->mFaces[j].mIndices[k];
        }
        indices[4*j+3] = -1;
    }

    SoIndexedFaceSet *faceSet = new SoIndexedFaceSet;
    faceSet->coordIndex.setValues(0,4*mesh->mNumFaces,indices);

    return faceSet;
}


SoSeparator *getMesh(aiMesh *mesh, aiMaterial *material) {
    SoSeparator *meshSep = new SoSeparator;
    meshSep->setName(SbName(mesh->mName.C_Str()));

    //Add material
    meshSep->addChild(getMaterial(material));

    //Add vertices
    if (mesh->HasPositions()) meshSep->addChild(getCoords(mesh));

    //Add faces
    if (mesh->HasFaces()) meshSep->addChild(getFaceSet(mesh));

    return meshSep;
}


void addNode(SoSeparator *parent, aiNode *node, aiMaterial **materials, aiMesh **meshes) {
    //Create separator
    SoSeparator *nodeSep = new SoSeparator;
    nodeSep->setName(SbName(node->mName.C_Str()));
    parent->addChild(nodeSep);

    //Add transform
    if (node->mParent) nodeSep->addChild(getTransform(node->mTransformation));

    //Add meshes
    for (int i = 0; i < node->mNumMeshes; ++i) {
        nodeSep->addChild(getMesh(meshes[node->mMeshes[i]],
                materials[meshes[node->mMeshes[i]]->mMaterialIndex]));
    }

    //Add children nodes
    for (int i = 0; i < node->mNumChildren; ++i) {
        addNode(nodeSep,node->mChildren[i],materials,meshes);
    }
}


SoSeparator* ivFromAssimp(string file) {
    cout << "Importing from assimp" << endl;

    Assimp::Importer importer;
    const aiScene* scene = importer.ReadFile(file,
                                             aiProcess_Triangulate |
                                             aiProcess_JoinIdenticalVertices |
                                             aiProcess_SortByPType);

    SoSeparator *root;
    if (scene) {
        cout << "File read" << endl;

        cout << "Importing to iv" << endl;

        root = new SoSeparator;
        root->ref();
        addNode(root,scene->mRootNode,scene->mMaterials,scene->mMeshes);
    } else {
        root = NULL;
        cout << "File not read" << endl;
        cout << "Error parsing: " << importer.GetErrorString() << endl;
    }

    return root;
}
#else
SoSeparator* ivFromAssimp(std::string file) {
    return NULL;
}
#endif


