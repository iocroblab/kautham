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
    if (type == dBoxClass)
    {
        if (!pos) pos = dGeomGetPosition (g);
        if (!R) R = dGeomGetRotation (g);

        dVector3 sides;
        dGeomBoxGetLengths (g,sides);
        dsDrawBox (pos,R,sides);
    }
    else if (type == dSphereClass)
    {
        if (!pos) pos = dGeomGetPosition (g);
        if (!R) R = dGeomGetRotation (g);
        dsDrawSphere (pos,R,dGeomSphereGetRadius (g));
    }
    else if (type == dCapsuleClass)
    {
        if (!pos) pos = dGeomGetPosition (g);
        if (!R) R = dGeomGetRotation (g);
        dReal radius,length;
        dGeomCapsuleGetParams (g,&radius,&length);
        dsDrawCapsule (pos,R,length,radius);
    }
    else if (type == dCylinderClass)
    {
        if (!pos) pos = dGeomGetPosition (g);
        if (!R) R = dGeomGetRotation (g);
        dReal radius,length;
        dGeomCylinderGetParams (g,&radius,&length);
        dsDrawCylinder (pos,R,length,radius);
    }
    else if (type == dGeomTransformClass)
    {
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

void DisplayOpenDESpaces::displaySpaces(void)
{



    for (unsigned int i = 0 ; i < m_spaces.size() ; ++i)
    {
        m_activeColor = m_colors[i];
        displaySpace(m_spaces[i]);
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
