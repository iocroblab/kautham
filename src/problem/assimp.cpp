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

#ifdef KAUTHAM_USE_ASSIMP

#include <assimp/scene.h>
#include <assimp/Importer.hpp>
#include <assimp/importerdesc.h>
#include <assimp/postprocess.h>
#include <assimp/matrix4x4.h>

#include <iostream>
#include <algorithm>

#include <Inventor/nodes/SoTransform.h>
#include <Inventor/nodes/SoIndexedPointSet.h>
#include <Inventor/nodes/SoIndexedLineSet.h>
#include <Inventor/nodes/SoIndexedTriangleStripSet.h>
#include <Inventor/nodes/SoMaterial.h>

using namespace std;


SoTransform *getTransform(aiMatrix4x4 matrix) {
    aiVector3D scaling;
    aiQuaternion rotation;
    aiVector3D position;
    matrix.Decompose(scaling,rotation,position);

    SoTransform *transform = new SoTransform;
    transform->translation.setValue((float)position[0],
                                    (float)position[1],
                                    (float)position[2]);
    transform->rotation.setValue((float)rotation.x,
                                 (float)rotation.y,
                                 (float)rotation.z,
                                 (float)rotation.w);
    transform->scaleFactor.setValue((float)scaling[0],
                                    (float)scaling[1],
                                    (float)scaling[2]);

    return transform;
}


SoMaterial *getMaterial(aiMaterial *material) {
    SoMaterial *soMat = new SoMaterial;

    aiColor3D color;
    float value;

    //Add diffuse color
    if (AI_SUCCESS == material->Get(AI_MATKEY_COLOR_DIFFUSE,color)) {
        soMat->diffuseColor.setValue((float)color.r,
                                     (float)color.g,
                                     (float)color.b);
    }

    //Add specular color
    if (AI_SUCCESS == material->Get(AI_MATKEY_COLOR_SPECULAR,color)) {
        soMat->specularColor.setValue((float)color.r,
                                      (float)color.g,
                                      (float)color.b);
    }

    //Add ambient color
    if (AI_SUCCESS == material->Get(AI_MATKEY_COLOR_SPECULAR,color)) {
        soMat->ambientColor.setValue((float)color.r,
                                     (float)color.g,
                                     (float)color.b);
    }

    //Add ambient color
    if (AI_SUCCESS == material->Get(AI_MATKEY_COLOR_EMISSIVE,color)) {
        soMat->emissiveColor.setValue((float)color.r,
                                      (float)color.g,
                                      (float)color.b);
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


SoIndexedShape *getShape(aiMesh *mesh) {
    if (mesh->mNumVertices == 0 ||
            mesh->mNumFaces == 0) return NULL;

    SoIndexedShape *shape;
    unsigned numIndices;
    switch (mesh->mPrimitiveTypes) {
    case aiPrimitiveType_POINT:
        shape = new SoIndexedPointSet;
        numIndices = 1;
        break;
    case aiPrimitiveType_LINE:
        shape = new SoIndexedLineSet;
        numIndices = 2;
        break;
    case aiPrimitiveType_TRIANGLE:
        shape = new SoIndexedTriangleStripSet;
        numIndices = 3;
        break;
    default:
        return NULL;
        break;
    }

    SoVertexProperty *vertexProperty = new SoVertexProperty;
    shape->vertexProperty.setValue(vertexProperty);

    float vertices[mesh->mNumVertices][3];
    for (unsigned i = 0; i < mesh->mNumVertices; ++i) {
        //A rotation of pi/2 rad around (1 0 0) is needed
        vertices[i][0] = (float)mesh->mVertices[i][0];
        vertices[i][1] = -(float)mesh->mVertices[i][2];
        vertices[i][2] = (float)mesh->mVertices[i][1];
    }
    vertexProperty->vertex.setValues(0,mesh->mNumVertices,vertices);

    int indices[mesh->mNumFaces*(numIndices+1)];
    for (unsigned i = 0; i < mesh->mNumFaces; ++i) {
            for (unsigned j = 0; j < numIndices; ++j) {
                indices[i*(numIndices+1)+j] = mesh->mFaces[i].mIndices[j];
            }
            indices[i*(numIndices+1)+numIndices] = -1;
    }
    shape->coordIndex.setValues(0,mesh->mNumFaces*(numIndices+1),indices);

    return shape;
}


SoSeparator *getMesh(aiMesh *mesh, aiMaterial *material,
                     SoSeparator *meshSep = NULL) {
    if (!meshSep) {
        meshSep = new SoSeparator;
        meshSep->setName(SbName(mesh->mName.C_Str()));
    }

    //Add material
    meshSep->addChild(getMaterial(material));

    //Add shape
    SoIndexedShape* shape = getShape(mesh);
    if (shape) meshSep->addChild(shape);

    return meshSep;
}


bool hasMesh(aiNode *node) {
    if (node->mNumMeshes > 0) return true;
    for (unsigned i = 0; i < node->mNumChildren; ++i) {
        if (hasMesh(node->mChildren[i])) return true;
    }
    return false;
}


void addNode(SoSeparator *parent, aiNode *node,
             aiMaterial **materials, aiMesh **meshes) {
    if (hasMesh(node)) {
        SoSeparator *nodeSep;
        if ((!node->mParent || node->mTransformation.IsIdentity()) &&
                node->mNumMeshes == 0) {
            nodeSep = parent;
        } else {
            //Create separator
            nodeSep = new SoSeparator;
            nodeSep->setName(SbName(node->mName.C_Str()));
            parent->addChild(nodeSep);

            //Add transform
            if (node->mParent && !node->mTransformation.IsIdentity())
                nodeSep->addChild(getTransform(node->mTransformation));

            //Add meshes
            aiMesh *mesh;
            if (node->mNumMeshes == 1 && node->mNumChildren == 0) {
                getMesh(meshes[node->mMeshes[0]],
                        materials[meshes[node->mMeshes[0]]->mMaterialIndex],
                        nodeSep);
            } else {
                for (unsigned i = 0; i < node->mNumMeshes; ++i) {
                    mesh = meshes[node->mMeshes[i]];
                    nodeSep->addChild(getMesh(mesh,
                                          materials[mesh->mMaterialIndex]));
                }
            }
        }
        //Add children nodes
        for (unsigned i = 0; i < node->mNumChildren; ++i) {
            addNode(nodeSep,node->mChildren[i],materials,meshes);
        }
    }
}


SoSeparator* ivFromAssimp(string file) {
    Assimp::Importer importer;
    const aiScene* scene = importer.ReadFile(file,
                                             aiProcess_Triangulate | //Everything will be triangles, lines or points
                                             aiProcess_JoinIdenticalVertices | //No repeated vertices
                                             aiProcess_RemoveRedundantMaterials | //Check for redundant materials
                                             aiProcess_PreTransformVertices | //Pretransform vertices with the local transformation
                                             aiProcess_OptimizeMeshes | //Reduce the number of meshes
                                             aiProcess_OptimizeGraph | //Optimize the scene hierarchy
                                             aiProcess_Debone | //Remove bones
                                             aiProcess_SortByPType | //No meshes with more than one primitive
                                             aiProcess_RemoveComponent | //Remove the following components
                                             aiComponent_ANIMATIONS |
                                             aiComponent_BONEWEIGHTS |
                                             aiComponent_CAMERAS |
                                             aiComponent_COLORS |
                                             aiComponent_LIGHTS |
                                             aiComponent_NORMALS |
                                             //aiComponent_TANGENTS_AND_BITANGENTS |
                                             aiComponent_TEXCOORDS |
                                             aiComponent_TEXTURES);
    SoSeparator *root;
    if (scene) {
        root = new SoSeparator;
        root->ref();
        addNode(root,scene->mRootNode,scene->mMaterials,scene->mMeshes);
    } else {
        root = NULL;
    }

    return root;
}
#else
SoSeparator* ivFromAssimp(string file) {
    return NULL;
}
#endif

vector<string> tokenize(string str, string token) {
    vector<string> tokenized;
    size_t from = 0, size = str.size();
    for (size_t to = min(str.find(";",from),size);
         from < to; to = min(str.find(";",from),size)) {
        tokenized.push_back(str.substr(from,to-from));
        from = to + token.size();
    }
    return tokenized;
}

vector<string> assimpSupportedExtensions() {
#ifdef KAUTHAM_USE_ASSIMP
    aiString tmp;
    Assimp::Importer importer;
    importer.GetExtensionList(tmp);
    string extensions = tmp.C_Str();
    return tokenize(extensions.substr(2,string::npos),";*.");
#else
    vector<string> supportedExtensions;
    return supportedExtensions;
#endif
}

vector<pair<string,vector<string> > > assimpSupportedFormats() {
    vector<pair<string,vector<string> > > supportedFormats;
#ifdef KAUTHAM_USE_ASSIMP
    const aiImporterDesc* importerDesc;
    Assimp::Importer importer;
    string name;
    vector<string> extensions;
    unsigned k;
    size_t pos;
    for (unsigned i = 0; i < importer.GetImporterCount(); ++i) {
        importerDesc = importer.GetImporterInfo(i);

        name = importerDesc->mName;
        pos = name.find(" Importer");
        if (pos != string::npos) name.erase(pos,9);
        pos = name.find(" Reader");
        if (pos != string::npos) name.erase(pos,7);
        pos = name.find("\n");
        if (pos != string::npos) name.erase(pos,string::npos);
        while (name.substr(name.size()-1) == " ") {
            name.erase(name.size()-1,1);
        }
        extensions = tokenize(importerDesc->mFileExtensions," ");

        k = 0;
        while (k < supportedFormats.size() &&
               supportedFormats.at(k).first != name) {
            k++;
        }
        if (k < supportedFormats.size()) {
            for (unsigned j = 0; j < extensions.size(); ++j) {
                supportedFormats.at(k).second.push_back(extensions.at(j));
            }
        } else {
            pair< string,vector<string> > format;
            format.first = name;
            format.second = extensions;
            supportedFormats.push_back(format);
        }
    }
#endif
    return supportedFormats;
}
