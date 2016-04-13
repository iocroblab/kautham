/*********************************************************************
* Rice University Software Distribution License
*
* Copyright (c) 2010, Rice University
* All Rights Reserved.
*
* For a full description see the file named LICENSE.
*
*********************************************************************/

/* Author: Ioan Sucan */

#include "displayOpenDE.h"
#include <iostream>
#ifdef dDOUBLE
#define dsDrawBox dsDrawBoxD
#define dsDrawSphere dsDrawSphereD
#define dsDrawTriangle dsDrawTriangleD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCapsule dsDrawCapsuleD
#define dsDrawLine dsDrawLineD
#define dsDrawConvex dsDrawConvexD
#endif

// copied from an OpenDE demo program
//todo:pass trimesh as argument to this function
void DisplayOpenDESpaces::drawGeom (dGeomID g, const dReal *pos, const dReal *R, int show_aabb, Tmesh tm)
{
    int i;

    if (!g) return;

    if (dGeomIsSpace(g))
    {
        displaySpace((dSpaceID)g);
        return;
    }
    int type = dGeomGetClass (g);
    std::cout<<"Type = "<<type<<std::endl;
    if (type == dBoxClass)
    {
        std::cout<<"Geom is BOX "<<type<<std::endl;
        if (!pos) pos = dGeomGetPosition (g);
        if (!R) R = dGeomGetRotation (g);

        dVector3 sides;
        dGeomBoxGetLengths (g,sides);
        dsDrawBox (pos,R,sides);
    }
    else if (type == dSphereClass)
    {
        std::cout<<"Geom is SPHERE "<<type<<std::endl;

        if (!pos) pos = dGeomGetPosition (g);
        if (!R) R = dGeomGetRotation (g);
        dsDrawSphere (pos,R,dGeomSphereGetRadius (g));
    }
    else if (type == dCapsuleClass)
    {
        std::cout<<"Geom is CAPSULE "<<type<<std::endl;

        if (!pos) pos = dGeomGetPosition (g);
        if (!R) R = dGeomGetRotation (g);
        dReal radius,length;
        dGeomCapsuleGetParams (g,&radius,&length);
        dsDrawCapsule (pos,R,length,radius);
    }
    else if (type == dCylinderClass)
    {
        std::cout<<"Geom is CYLINDER "<<type<<std::endl;

        if (!pos) pos = dGeomGetPosition (g);
        if (!R) R = dGeomGetRotation (g);
        dReal radius,length;
        dGeomCylinderGetParams (g,&radius,&length);
        dsDrawCylinder (pos,R,length,radius);
    }
    else if (type == dGeomTransformClass)
    {
        std::cout<<"Geom is TRANSFORM "<<type<<std::endl;
        if (!pos) pos = dGeomGetPosition (g);
        if (!R) R = dGeomGetRotation (g);

        dGeomID g2 = dGeomTransformGetGeom (g);
        const dReal *pos2 = dGeomGetPosition (g2);
        const dReal *R2 = dGeomGetRotation (g2);
        dVector3 actual_pos;
        dMatrix3 actual_R;
        dMULTIPLY0_331 (actual_pos,R,pos2);
        actual_pos[0] += pos[0];
        actual_pos[1] += pos[1];
        actual_pos[2] += pos[2];
        dMULTIPLY0_333 (actual_R,R,R2);
        drawGeom (g2,actual_pos,actual_R,0);
    }
    else if (type == dTriMeshClass)
    {
        //dTriIndex* Indices = DISP.tmd[i].indices;
        std::cout<<"Geom is MESH "<<type<<std::endl;

        const dReal* Pos = dGeomGetPosition(g);
        const dReal* Rot = dGeomGetRotation(g);
        int i=0;
        for (int ii = 0; ii < (tm.indexSize/3); ii++)
        {

            const dReal v[9] = { // explicit conversion from float to dReal
                                 tm.vertices[tm.indices[ii * 3 + 0] * 3 + 0],
                                 tm.vertices[tm.indices[ii * 3 + 0] * 3 + 1],
                                 tm.vertices[tm.indices[ii * 3 + 0] * 3 + 2],
                                 tm.vertices[tm.indices[ii * 3 + 1] * 3 + 0],
                                 tm.vertices[tm.indices[ii * 3 + 1] * 3 + 1],
                                 tm.vertices[tm.indices[ii * 3 + 1] * 3 + 2],
                                 tm.vertices[tm.indices[ii * 3 + 2] * 3 + 0],
                                 tm.vertices[tm.indices[ii * 3 + 2] * 3 + 1],
                                 tm.vertices[tm.indices[ii * 3 + 2] * 3 + 2]
                               };
            dsDrawTriangle(Pos, Rot, &v[0], &v[3], &v[6], 1);
        }
        //std::cout<<"done once"<<std::endl;
    }
//    else
//        if (type == dRayClass)
//        {
//            std::cout<<"Geom is RAY "<<type<<std::endl;

//            dVector3 Origin, Direction;
//            dGeomRayGet(g, Origin, Direction);

//            dReal Length = dGeomRayGetLength(g);

//            dVector3 End;
//            End[0] = Origin[0] + (Direction[0] * Length);
//            End[1] = Origin[1] + (Direction[1] * Length);
//            End[2] = Origin[2] + (Direction[2] * Length);
//            End[3] = Origin[3] + (Direction[3] * Length);
//            double *ori = new double[3];
//            double *end = new double[4];
//            ori[0]=Origin[0];
//            ori[1]=Origin[1];
//            ori[2]=Origin[2];
//            end[0]=End[0];
//            end[1]=End[1];
//            end[2]=End[2];
//            end[3]=End[3];


//            dsDrawLine(ori, end);
//        }
        else
            show_aabb = 0;

    if (show_aabb)
    {
        // draw the bounding box for this geom
        dReal aabb[6];
        dGeomGetAABB (g,aabb);
        dVector3 bbpos;
        for (i=0; i<3; i++) bbpos[i] = 0.5*(aabb[i*2] + aabb[i*2+1]);
        dVector3 bbsides;
        for (i=0; i<3; i++) bbsides[i] = aabb[i*2+1] - aabb[i*2];
        dMatrix3 RI;
        dRSetIdentity (RI);
        dsSetColorAlpha (1,0,0,0.5);
        dsDrawBox (bbpos,RI,bbsides);
    }
}

void DisplayOpenDESpaces::drawGeom (dGeomID g, const dReal *pos, const dReal *R, int show_aabb)
{
    int i;

    if (!g) return;

    if (dGeomIsSpace(g))
    {
        displaySpace((dSpaceID)g);
        return;
    }
    int type = dGeomGetClass (g);
    std::cout<<"Type = "<<type<<std::endl;
    if (type == dBoxClass)
    {
        std::cout<<"Geom is BOX "<<type<<std::endl;
        if (!pos) pos = dGeomGetPosition (g);
        if (!R) R = dGeomGetRotation (g);

        dVector3 sides;
        dGeomBoxGetLengths (g,sides);
        dsDrawBox (pos,R,sides);
    }
    else if (type == dSphereClass)
    {
        std::cout<<"Geom is SPHERE "<<type<<std::endl;

        if (!pos) pos = dGeomGetPosition (g);
        if (!R) R = dGeomGetRotation (g);
        dsDrawSphere (pos,R,dGeomSphereGetRadius (g));
    }
    else if (type == dCapsuleClass)
    {
        std::cout<<"Geom is CAPSULE "<<type<<std::endl;

        if (!pos) pos = dGeomGetPosition (g);
        if (!R) R = dGeomGetRotation (g);
        dReal radius,length;
        dGeomCapsuleGetParams (g,&radius,&length);
        dsDrawCapsule (pos,R,length,radius);
    }
    else if (type == dCylinderClass)
    {
        std::cout<<"Geom is CYLINDER "<<type<<std::endl;

        if (!pos) pos = dGeomGetPosition (g);
        if (!R) R = dGeomGetRotation (g);
        dReal radius,length;
        dGeomCylinderGetParams (g,&radius,&length);
        dsDrawCylinder (pos,R,length,radius);
    }
    else if (type == dGeomTransformClass)
    {
        std::cout<<"Geom is TRANSFORM "<<type<<std::endl;
        if (!pos) pos = dGeomGetPosition (g);
        if (!R) R = dGeomGetRotation (g);

        dGeomID g2 = dGeomTransformGetGeom (g);
        const dReal *pos2 = dGeomGetPosition (g2);
        const dReal *R2 = dGeomGetRotation (g2);
        dVector3 actual_pos;
        dMatrix3 actual_R;
        dMULTIPLY0_331 (actual_pos,R,pos2);
        actual_pos[0] += pos[0];
        actual_pos[1] += pos[1];
        actual_pos[2] += pos[2];
        dMULTIPLY0_333 (actual_R,R,R2);
        drawGeom (g2,actual_pos,actual_R,0);
    }
        else
            show_aabb = 0;

    if (show_aabb)
    {
        // draw the bounding box for this geom
        dReal aabb[6];
        dGeomGetAABB (g,aabb);
        dVector3 bbpos;
        for (i=0; i<3; i++) bbpos[i] = 0.5*(aabb[i*2] + aabb[i*2+1]);
        dVector3 bbsides;
        for (i=0; i<3; i++) bbsides[i] = aabb[i*2+1] - aabb[i*2];
        dMatrix3 RI;
        dRSetIdentity (RI);
        dsSetColorAlpha (1,0,0,0.5);
        dsDrawBox (bbpos,RI,bbsides);
    }
}


void DisplayOpenDESpaces::displaySpace(dSpaceID space, std::vector<Tmesh> *tm)
{
    int ngeoms;

    ngeoms = dSpaceGetNumGeoms(space);

    for (int i = 0 ; i < ngeoms ; ++i)
    {
        dGeomID geom = dSpaceGetGeom(space, i);
        std::map<dGeomID, Color>::const_iterator it = m_gcolors.find(geom);
        if (it != m_gcolors.end())
            dsSetColor(it->second.r,it->second.g,it->second.b);
        else
            dsSetColor(m_activeColor.r, m_activeColor.g, m_activeColor.b);

        if(tm==NULL)
        drawGeom(geom, NULL, NULL, 0);
        else
        drawGeom(geom, NULL, NULL, 0,tm->at(i));


    }

}
void DisplayOpenDESpaces::displaySpace(dSpaceID space)
{
    int ngeoms = dSpaceGetNumGeoms(space);
    for (int i = 0 ; i < ngeoms ; ++i)
    {
        dGeomID geom = dSpaceGetGeom(space, i);
        std::map<dGeomID, Color>::const_iterator it = m_gcolors.find(geom);
        if (it != m_gcolors.end())
            dsSetColor(it->second.r,it->second.g,it->second.b);
        else
            dsSetColor(m_activeColor.r, m_activeColor.g, m_activeColor.b);
        drawGeom(geom, NULL, NULL, 0);



    }
}

void DisplayOpenDESpaces::displaySpaces(std::vector<Tmesh> *tm)
{


if(tm==NULL)
{
    for (unsigned int i = 0 ; i < m_spaces.size() ; ++i)
    {
        m_activeColor = m_colors[i];
        displaySpace(m_spaces[i],NULL);
    }
}
else
{
    for (unsigned int i = 0 ; i < m_spaces.size() ; ++i)
    {
        m_activeColor = m_colors[i];
        displaySpace(m_spaces[i],tm);
    }
}

}

void DisplayOpenDESpaces::addSpace(dSpaceID space, float r, float g, float b)
{
    Color c = {r, g, b};
    m_colors.push_back(c);
    m_spaces.push_back(space);
}

void DisplayOpenDESpaces::setGeomColor(dGeomID geom, float r, float g, float b)
{
    Color c = {r, g, b};
    m_gcolors[geom] = c;
}
void DisplayOpenDESpaces::setCurrentTransform(dGeomID geom)
{
 const dReal* Pos = dGeomGetPosition(geom);
 const dReal* Rot = dGeomGetRotation(geom);

 const dReal Transform[16] =
 {
   Rot[0], Rot[4], Rot[8],  0,
   Rot[1], Rot[5], Rot[9],  0,
   Rot[2], Rot[6], Rot[10], 0,
   Pos[0], Pos[1], Pos[2],  1
 };

 dGeomTriMeshSetLastTransform( geom, *(dMatrix4*)(&Transform) );

}
void DisplayOpenDESpaces::getTmeshdata(std::vector<Tmesh> tm)
{
    for(int i=0;i<tm.size();i++)
    {
//        Tmesh tmesh;
//         tmesh.indexSize=tm[i].indexSize;
//        tmesh.indices=*tm[i].indices;
//        tmesh.vertices=*tm[i].vertices;
//         tmesh.meshD=tm[i].meshD;
//         tmd.push_back(tmesh);

    }

}

void DisplayOpenDESpaces::clear(void)
{
    m_spaces.clear();
    m_colors.clear();
    m_gcolors.clear();
}
