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

// copy from the omplapp and modified for the triangular mesh
#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include <vector>
#include <map>


#include <GL/gl.h>


/// @cond IGNORE
typedef struct
{
    dTriMeshDataID meshD;
    std::vector<float> vertices;
    std::vector<dTriIndex> indices;
    int indexSize;
}Tmesh;
class DisplayOpenDESpaces
{
public:
    static dTriMeshDataID TriData1, TriData2;
    DisplayOpenDESpaces(void)
    {
        m_activeColor.r = m_activeColor.g = m_activeColor.b = 0.5;
    }

    void drawGeom (dGeomID g, const dReal *pos, const dReal *R, int show_aabb, Tmesh tm);
    void drawGeom (dGeomID g, const dReal *pos, const dReal *R, int show_aabb);

    void displaySpace(std::vector<dGeomID> g, std::vector<Tmesh> *tm);
    void displaySpaces(std::vector<Tmesh>  *tm);
    void displaySpace(dSpaceID space);

    void addSpace(dSpaceID space, float r = 0.75, float g = 0.75, float b = 0.75);
    void clear(void);

    void setGeomColor(dGeomID geom, float r, float g, float b);
    void setCurrentTransform(dGeomID geom);

    void addGeoms(std::vector<dGeomID> geom);
    void getTmeshdata(std::vector<Tmesh> tm);
    struct Color
    {
        float r, g, b;
    };

    std::vector<Tmesh> tmd;
    std::vector<dGeomID> geomID;
    std::vector<dSpaceID>     m_spaces;
    std::vector<Color>        m_colors;
    std::map<dGeomID, Color>  m_gcolors;
    Color                     m_activeColor;
};
/// @endcond
