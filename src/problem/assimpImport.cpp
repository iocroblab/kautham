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


#include <kautham/problem/assimpImport.h>

#ifdef KAUTHAM_USE_ASSIMP
#include <assimp/scene.h>
#include <assimp/Importer.hpp>
#include <assimp/importerdesc.h>
#include <assimp/postprocess.h>

#include <Inventor/nodes/SoTransform.h>
#include <Inventor/nodes/SoIndexedPointSet.h>
#include <Inventor/nodes/SoIndexedLineSet.h>
#include <Inventor/nodes/SoIndexedTriangleStripSet.h>
#include <Inventor/nodes/SoMaterial.h>
#include <Inventor/nodes/SoTexture2.h>

#include <iostream>

#include <boost/algorithm/string/replace.hpp>


SoTransform *getTransform(const aiMatrix4x4 &matrix) {
    aiVector3D scaling;
    aiQuaternion rotation;
    aiVector3D position;
    matrix.Decompose(scaling,rotation,position);

    SoTransform *transform(new SoTransform);
    transform->translation.setValue(position.x,
                                    position.y,
                                    position.z);
    transform->rotation.setValue(rotation.x,
                                 rotation.y,
                                 rotation.z,
                                 rotation.w);
    transform->scaleFactor.setValue(scaling.x,
                                    scaling.y,
                                    scaling.z);

    return transform;
}


SoTexture2 *getTexture(const aiTexture *const texture) {
    if (texture->mHeight == 0) {//Compressed texture
        std::cout << "Found a compressed embedded texture. "
                  << "It will be ignored." << std::endl;
        ///texture->pcData is a pointer to a memory buffer of
        ///size mWidth containing the compressed texture data
        return NULL;
    } else {//Uncompressed texture
        unsigned char *pixels = new unsigned char[texture->mWidth*texture->mHeight*4];
        for (std::size_t i(0); i < texture->mWidth; ++i) {
            for (std::size_t j(0); j < texture->mHeight; ++j) {
                pixels[4*(texture->mHeight*i+j)+0] = texture->pcData[texture->mHeight*i+j].r;
                pixels[4*(texture->mHeight*i+j)+1] = texture->pcData[texture->mHeight*i+j].g;
                pixels[4*(texture->mHeight*i+j)+2] = texture->pcData[texture->mHeight*i+j].b;
                pixels[4*(texture->mHeight*i+j)+3] = texture->pcData[texture->mHeight*i+j].a;
            }
        }
        SoTexture2 *soTexture(new SoTexture2);
        soTexture->image.setValue(SbVec2s(texture->mWidth,texture->mHeight),4,pixels);

        return soTexture;
    }
}


SoTexture *getTexture(const aiMaterial * const material, const std::string &scenePath) {
    std::vector<aiTextureType> types;
    types.push_back(aiTextureType_NONE);
    types.push_back(aiTextureType_DIFFUSE);
    types.push_back(aiTextureType_SPECULAR);
    types.push_back(aiTextureType_AMBIENT);
    types.push_back(aiTextureType_EMISSIVE);
    types.push_back(aiTextureType_HEIGHT);
    types.push_back(aiTextureType_NORMALS);
    types.push_back(aiTextureType_SHININESS);
    types.push_back(aiTextureType_OPACITY);
    types.push_back(aiTextureType_DISPLACEMENT);
    types.push_back(aiTextureType_LIGHTMAP);
    types.push_back(aiTextureType_REFLECTION);
    types.push_back(aiTextureType_UNKNOWN);

    SoTexture2 *texture(NULL);
    ///Am I really interested in all texture types or only in DIFFUSE?
    for (std::vector<aiTextureType>::const_iterator type(types.begin());
         type != types.end(); ++type) {
        for (std::size_t i(0); i < material->GetTextureCount(*type); ++i) {
            aiString path;
            aiTextureMapping mapping;
            unsigned int uvIndex;
            float blendFactor;
            aiTextureOp operation;
            aiTextureMapMode mapMode[3];
            if (aiReturn_SUCCESS == material->GetTexture(*type,i,&path,&mapping,&uvIndex,
                                                   &blendFactor,&operation,mapMode)) {
                std::cout << "The " << i << "th texture (out of "
                          << material->GetTextureCount(*type) << ") is of type " << *type
                          << " is stored in " << path.C_Str() << " and has UV index "
                          << uvIndex << ", blend factor " << blendFactor << ", operation "
                          << operation << " and map mode [" << mapMode[0] << ", "
                          << mapMode[1] << ", " << mapMode[2] << "]" << std::endl;

                //AI_MATKEY_UVTRANSFORM(type, N) => aiUVTransform => SoTexture2Transform
                //AI_MATKEY_TEXFLAGS(type, N)

                /*int index;
                if (aiReturn_SUCCESS == material->Get(AI_MATKEY_UVWSRC(*type,i),index)) {
                    std::cout << "I have uvIndex " << index << std::endl;
                } else {
                    std::cout << "I do not have uvIndex" << std::endl;
                }*/

                if (texture) {
                    std::cout << "Found a material with more than one texture. "
                              << "Only the first one will be used." << std::endl;
                } else {
                    ///I don't know what to do with uvIndex or blendFactor
                    ///uvIndex only has a valid value if mapping == aiTextureMapping_UV
                    ///Operation should have something related with SoTextureCombiner
                    ///If mapMode[2] is valid then should I use SoTexture3?
                    ///How I know when I need to use SoTexture3 or Sotexture2?
                    ///I don't know how to check if the image has been loaded correctly
                    ///texture->model is always set to DECAL, is that good?
                    ///It should be related with the loaded info

                    if (mapping != aiTextureMapping_UV) {
                        std::cout << "Invalid texture mapping. Texture will be ignored." << std::endl;
                        continue;
                    }

                    texture = new SoTexture2;

                    std::string filename(scenePath);
                    filename.append(path.C_Str());
                    boost::replace_all(filename,"\\","/");
                    texture->filename.setValue(filename.c_str());
                    texture->setName(filename.substr(filename.find_last_of("/")+1).c_str());

                    texture->model.setValue(SoTexture2::DECAL);

                    switch (mapMode[0]) {
                        case aiTextureMapMode_Wrap:
                            texture->wrapS.setValue(SoTexture2::REPEAT);
                        break;
                        case aiTextureMapMode_Clamp:
                            texture->wrapS.setValue(SoTexture2::CLAMP);
                        break;
                        case aiTextureMapMode_Decal:
                        case aiTextureMapMode_Mirror:
                        default:
                            std::cout << "Wrong S texture mapping mode. "
                                      << "Property will be ignored." << std::endl;
                        break;
                    }

                    switch (mapMode[1]) {
                        case aiTextureMapMode_Wrap:
                            texture->wrapT.setValue(SoTexture2::REPEAT);
                        break;
                        case aiTextureMapMode_Clamp:
                            texture->wrapT.setValue(SoTexture2::CLAMP);
                        break;
                        case aiTextureMapMode_Decal:
                        case aiTextureMapMode_Mirror:
                        default:
                            std::cout << "Wrong T texture mapping mode. "
                                      << "Property will be ignored." << std::endl;
                        break;
                    }
                }
            }
        }
    }

    return texture;
}


SoMaterial *getMaterial(const aiMaterial *const material) {
    SoMaterial *soMat(new SoMaterial);

    aiString name;
    aiColor3D color;
    float value;

    //Add name
    if (aiReturn_SUCCESS == material->Get(AI_MATKEY_NAME,name)) {
        soMat->setName(SbName(name.C_Str()));
    }

    //Add diffuse color
    if (aiReturn_SUCCESS == material->Get(AI_MATKEY_COLOR_DIFFUSE,color)) {
        soMat->diffuseColor.setValue(color.r,
                                     color.g,
                                     color.b);
    }

    //Add specular color
    if (aiReturn_SUCCESS == material->Get(AI_MATKEY_COLOR_SPECULAR,color)) {
        soMat->specularColor.setValue(color.r,
                                      color.g,
                                      color.b);
    }

    //Add ambient color
    if (aiReturn_SUCCESS == material->Get(AI_MATKEY_COLOR_AMBIENT,color)) {
        soMat->ambientColor.setValue(color.r,
                                     color.g,
                                     color.b);
    }

    //Add emissive color
    if (aiReturn_SUCCESS == material->Get(AI_MATKEY_COLOR_EMISSIVE,color)) {
        soMat->emissiveColor.setValue(color.r,
                                      color.g,
                                      color.b);
    }

    //Add transparency
    if (aiReturn_SUCCESS == material->Get(AI_MATKEY_OPACITY,value)) {
        soMat->transparency.setValue(1.0-value);
    }

    //Add shininess
    if (aiReturn_SUCCESS == material->Get(AI_MATKEY_SHININESS_STRENGTH,value)) {
        soMat->shininess.setValue(value);
    }

    return soMat;
}


SoIndexedShape *getShape(const aiMesh *const mesh) {
    if (!mesh->HasPositions() || !mesh->HasFaces()) return NULL; //Mesh is empty

    SoVertexProperty *vertexProperty(new SoVertexProperty);

    //Set vertices
    SbVec3f *vertices = new SbVec3f[mesh->mNumVertices];
    for (std::size_t i(0); i < mesh->mNumVertices; ++i) {
        vertices[i][0] = mesh->mVertices[i].x;
        vertices[i][1] = mesh->mVertices[i].y;
        vertices[i][2] = mesh->mVertices[i].z;
    }
    vertexProperty->vertex.setValues(0,mesh->mNumVertices,vertices);

    if (mesh->HasNormals()) {
        //Set normals
        SbVec3f *normals = new SbVec3f[mesh->mNumVertices];
        for (std::size_t i(0); i < mesh->mNumVertices; ++i) {
            normals[i][0] = mesh->mNormals[i].x;
            normals[i][1] = mesh->mNormals[i].y;
            normals[i][2] = mesh->mNormals[i].z;
        }
        vertexProperty->normal.setValues(0,mesh->mNumVertices,normals);
    }

    if (mesh->GetNumColorChannels() > 0) {
        std::cout << "Mesh has " << mesh->GetNumColorChannels()
                  << " vertex color channels. Property will be ignored." << std::endl;
    }

    if (mesh->GetNumUVChannels() > 0) {
        if (mesh->GetNumUVChannels() > 1) {
                std::cout << "Mesh has " << mesh->GetNumUVChannels()
                          << " UV channels. Only the first one will be used." << std::endl;
                /*How to map UV channels to textures (MATKEY_UVWSRC)

                The MATKEY_UVWSRC property is only present if the source format doesn't
                specify an explicit mapping from textures to UV channels. Many formats
                don't do this and assimp is not aware of a perfect rule either.

                Your handling of UV channels needs to be flexible therefore.
                Our recommendation is to use logic like this to handle most cases properly:

                  have only one uv channel?
                     assign channel 0 to all textures and break

                  for all textures
                     have uvwsrc for this texture?
                        assign channel specified in uvwsrc
                     else
                        assign channels in ascending order for all texture stacks,
                            i.e. diffuse1 gets channel 1, opacity0 gets channel 0.
            */
        }

        //Set texture coordinates
        if (mesh->mNumUVComponents[0] == 2) {
            SbVec2f *texCoords = new SbVec2f[mesh->mNumVertices];
            for (std::size_t i(0); i < mesh->mNumVertices; ++i) {
                texCoords[i][0] = mesh->mTextureCoords[0][i].x;
                texCoords[i][1] = mesh->mTextureCoords[0][i].y;
            }
            vertexProperty->texCoord.setValues(0,mesh->mNumVertices,texCoords);
        } else if (mesh->mNumUVComponents[0] == 3) {
            SbVec3f *texCoords3 = new SbVec3f[mesh->mNumVertices];
            for (std::size_t i(0); i < mesh->mNumVertices; ++i) {
                texCoords3[i][0] = mesh->mTextureCoords[0][i].x;
                texCoords3[i][1] = mesh->mTextureCoords[0][i].y;
                texCoords3[i][2] = mesh->mTextureCoords[0][i].z;
            }
            vertexProperty->texCoord3.setValues(0,mesh->mNumVertices,texCoords3);
        } else {
            std::cout << "Mesh has texture coordinates of " << mesh->mNumUVComponents[0]
                      << " components. Property will be ignored." << std::endl;
        }
    }

    SoIndexedShape *shape;
    std::size_t numIndices;
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
    case aiPrimitiveType_POLYGON:
    default:
        std::cout << "Wrong primitive type. Mesh will be ignored." << std::endl;
        return NULL;
        break;
    }

    //Set faces
    int *indices = new int[mesh->mNumFaces*(numIndices+1)];
    for (std::size_t i(0); i < mesh->mNumFaces; ++i) {
        for (std::size_t j(0); j < numIndices; ++j) {
            indices[i*(numIndices+1)+j] = mesh->mFaces[i].mIndices[j];
        }
        indices[i*(numIndices+1)+numIndices] = -1;
    }
    shape->coordIndex.setValues(0,mesh->mNumFaces*(numIndices+1),indices);
    shape->vertexProperty.setValue(vertexProperty);

    return shape;
}


SoSeparator *getMesh(const aiMesh *const mesh, const aiMaterial *const material,
                     const std::string &path, SoSeparator *meshSep = NULL) {
    SoIndexedShape *shape(getShape(mesh));
    if (shape) {
        if (!meshSep) {
            meshSep = new SoSeparator;
            meshSep->setName(SbName(mesh->mName.C_Str()));
        }

        //Add texture
        SoTexture *texture(getTexture(material,path));
        if (texture) meshSep->addChild(texture);

        //Add material
        meshSep->addChild(getMaterial(material));

        //Add shape
        meshSep->addChild(shape);

        return meshSep;
    } else {
        return NULL;
    }
}


bool hasMesh(const aiNode *node) {
    if (node->mNumMeshes > 0) return true;
    for (std::size_t i(0); i < node->mNumChildren; ++i) {
        if (hasMesh(node->mChildren[i])) return true;
    }
    return false;
}


void addNode(SoSeparator *const parent, const aiNode *const node,
             const aiMaterial *const *const materials, const aiMesh *const *const meshes,
             const aiTexture *const *const textures, const std::string &path) {
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
            if (node->mNumMeshes == 1 && node->mNumChildren == 0) {
                getMesh(meshes[node->mMeshes[0]],
                        materials[meshes[node->mMeshes[0]]->mMaterialIndex],
                        path,nodeSep);
            } else {
                for (std::size_t i(0); i < node->mNumMeshes; ++i) {
                    SoNode *child(getMesh(meshes[node->mMeshes[i]],
                                  materials[meshes[node->mMeshes[i]]->mMaterialIndex],path));
                    if (child) nodeSep->addChild(child);
                }
            }
        }

        //Add children nodes
        for (std::size_t i(0); i < node->mNumChildren; ++i) {
            addNode(nodeSep,node->mChildren[i],materials,meshes,textures,path);
        }
    }
}


SoSeparator *Assimp2Inventor(const aiScene *const scene, const std::string &path) {
    SoSeparator *root(new SoSeparator);
    std::cout << "I imported a scene with " << scene->mNumTextures << " embedded textures, "
              << scene->mNumMaterials << " materials and "
              << scene->mNumMeshes << " meshes." << std::endl;
    if (scene->mNumTextures > 0) {
        std::cout << "Found a scene with embedded textures. They will be ignored." << std::endl;
        ///I don't know how will be referenced inside the scene
    }
    addNode(root,scene->mRootNode,scene->mMaterials,
            scene->mMeshes,scene->mTextures,path);
    return root;
}


SoSeparator *importScene(const std::string &filename, std::string *const error) {
    try {
        Assimp::Importer importer;

        //Set the parts of the data structure to be removed
        importer.SetPropertyInteger(AI_CONFIG_PP_RVC_FLAGS,
                                    aiComponent_TANGENTS_AND_BITANGENTS |
                                    aiComponent_COLORS |
                                    aiComponent_BONEWEIGHTS |
                                    aiComponent_ANIMATIONS |
                                    aiComponent_LIGHTS |
                                    aiComponent_CAMERAS);

        //Set the post processing step to be applied
        unsigned int postProcessSteps
                (//Remove repeated vertices
                 //aiProcess_JoinIdenticalVertices |

                 //Convert polygons to triangles
                 aiProcess_Triangulate |

                 //Remove the previously specified components
                 aiProcess_RemoveComponent |

                 //Generate missing normals
                 aiProcess_GenNormals |

                 //Make sure everything is OK
                 aiProcess_ValidateDataStructure |

                 //Check for redundant materials
                 //aiProcess_RemoveRedundantMaterials |

                 //Invert normals facing inwards
                 //aiProcess_FixInfacingNormals |

                 //Split meshes with more than one primitive
                 //(i.e. points, lines, trinagles, polygons)
                 aiProcess_SortByPType |

                 //Convert degenerated primitives to proper ones
                 aiProcess_FindDegenerates |

                 //Remove/Fix invalid data
                 aiProcess_FindInvalidData |

                 //Convert non-UV mappings to proper texture coordinate channels
                 aiProcess_GenUVCoords |

                 //Apply per-texture UV transformations
                 aiProcess_TransformUVCoords /*|

                 //Reduce the number of meshes
                 aiProcess_OptimizeMeshes |

                 //Optimize the scene hierarchy
                 aiProcess_OptimizeGraph |

                 //Remove bones losslessly
                 aiProcess_Debone*/);

        //Import scene
        const aiScene *const scene(importer.ReadFile(filename,postProcessSteps));

        if (scene) {
            //Convert from Assimp to Inventor
            std::string path(filename.substr(0,filename.find_last_of("/")+1));
            return Assimp2Inventor(scene,path);
        } else {
            if (error) *error = importer.GetErrorString();
            return NULL;
        }
    } catch (std::exception &excp) {
        if (error) {
            *error = "File could not be imported: ";
            error->append(excp.what());
        }
        return NULL;
    }
}


std::vector<std::string> tokenize(const std::string &str, const std::string &token) {
    std::vector<std::string> tokenized;
    size_t from(0), size(str.size());
    for (size_t to(std::min(str.find(token,from),size));
         from < to; to = std::min(str.find(token,from),size)) {
        tokenized.push_back(str.substr(from,to-from));
        from = to + token.size();
    }
    return tokenized;
}


std::vector<std::string> assimpImportedExtensions() {
    aiString tmp;
    Assimp::Importer importer;
    importer.GetExtensionList(tmp);
    std::string extensions(tmp.C_Str());

    return tokenize(extensions.substr(2,std::string::npos),";*.");
}


std::vector<std::pair<std::string,std::vector<std::string> > > assimpImportedFormats() {
    std::vector<std::pair<std::string,std::vector<std::string> > > importedFormats;
    const aiImporterDesc *importerDesc;
    Assimp::Importer importer;
    std::string name;
    std::vector<std::string> extensions;
    std::size_t k, pos;
    for (std::size_t i(0); i < importer.GetImporterCount(); ++i) {
        importerDesc = importer.GetImporterInfo(i);

        name = importerDesc->mName;
        pos = name.find(" Importer");
        if (pos != std::string::npos) name.erase(pos,9);
        pos = name.find(" Reader");
        if (pos != std::string::npos) name.erase(pos,7);
        pos = name.find("\n");
        if (pos != std::string::npos) name.erase(pos,std::string::npos);
        while (name.substr(name.size()-1) == " ") {
            name.erase(name.size()-1,1);
        }
        extensions = tokenize(importerDesc->mFileExtensions," ");

        k = 0;
        while (k < importedFormats.size() &&
               importedFormats.at(k).first != name) {
            k++;
        }
        if (k < importedFormats.size()) {
            for (std::size_t j(0); j < extensions.size(); ++j) {
                importedFormats.at(k).second.push_back(extensions.at(j));
            }
        } else {
            std::pair< std::string,std::vector<std::string> > format;
            format.first = name;
            format.second = extensions;
            importedFormats.push_back(format);
        }
    }

    return importedFormats;
}


#else // KAUTHAM_USE_ASSIMP
SoSeparator *importScene(const std::string filename, std::string *error) {
    if (error) *error = "Assimp not available";
    return NULL;
}


std::vector<std::string> assimpImportedExtensions() {
    std::vector<std::string> importedExtensions;
    return importedExtensions;
}


std::vector<std::pair<std::string,std::vector<std::string> > > assimpImportedFormats() {
    return std::vector<std::pair<std::string,std::vector<std::string> > >();
}
#endif // KAUTHAM_USE_ASSIMP
