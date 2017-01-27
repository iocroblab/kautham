/***************************************************************************
 *   Copyright (C) 2006 by Adolfo Rodriguez                                *
 *   adolfo.rodriguez@upc.edu                                              *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

/////////////////////////////// PREPROCESSOR DIRECTIVES //////////////////////

// HEADER GUARD
#ifndef MT_RELATION_H
#define MT_RELATION_H

// MT LIBRARY HEADERS

#include <mt/relation/rel_type.h>

#include <mt/relation/rel_vector_vector.h>
#include <mt/relation/rel_unit_unit.h>
#include <mt/relation/rel_point_point.h>
#include <mt/relation/rel_point_line.h>
#include <mt/relation/rel_point_plane.h>
#include <mt/relation/rel_point_sphere.h>
#include <mt/relation/rel_point_cylinder.h>
#include <mt/relation/rel_point_circle.h>
#include <mt/relation/rel_line_line.h>
#include <mt/relation/rel_line_plane.h>
#include <mt/relation/rel_line_sphere.h>
#include <mt/relation/rel_line_cylinder.h>
#include <mt/relation/rel_line_circle.h>
#include <mt/relation/rel_plane_plane.h>
#include <mt/relation/rel_plane_sphere.h>
#include <mt/relation/rel_plane_cylinder.h>
#include <mt/relation/rel_plane_circle.h>
#include <mt/relation/rel_sphere_sphere.h>
#include <mt/relation/rel_sphere_cylinder.h>
#include <mt/relation/rel_sphere_circle.h>
#include <mt/relation/rel_cylinder_cylinder.h>
#include <mt/relation/rel_cylinder_circle.h>
#include <mt/relation/rel_circle_circle.h>

/// \defgroup rel Relation
/// \brief This module contains classes for computing the relation between
/// pairs of geometric elements.
///
/// There is one class for each pair of geometric elements.
/// For example, the class RelVectorVector computes the relation between two
/// \link mt::Vector3 Vector3 \endlink instances.
/// It is possible to query quantitative parameters such
/// as the distance and angle between the vectors; or the relation type
/// qualitative parameter.
///
/// Each class implements the query functions that are compatible with the
/// involved elements, so for example, the class 
/// \link mt::RelPointLine RelPointLine \endlink does not have a getAngle()
/// method since computing the angle between a point and a line
/// makes no sense. On the other hand, the class 
/// \link mt::RelLinePlane RelLinePlane \endlink has the
/// \link mt::RelLinePlane::getIntersectionPoint()
/// getIntersectionPoint() \endlink that will return the intersection of both
/// elements if they intersect, or throw an exception if they do not intersect.
///
/// These are some examples of how to use the geometric relation classes:
///
/// \code
/// using namespace mt;
///
/// // Relation between two vectors
/// Vector3 v1(0.0, -1.0, 0.0);
/// Vector3 v2(1.0, 0.0, 0.0);
/// RelVectorVector rv(v1, v2);
///
/// Scalar dv = rv.getDistance();          // dv = 1.41421
/// Scalar av = radToDeg(rv.getAngle());   // av = 90
///
/// // Relation between two unit vectors
/// Unit3 dir1(1.0, 0.0, 0.0);
/// Unit3 dir2(0.0, 1.0, 0.0);
/// RelUnitUnit ru(dir1, dir2);
///
/// Scalar du = ru.getDistance();          // Compile-time error. getDistance()
///                                        // does not exist
/// Scalar au = radToDeg(ru.getAngle());   // av = 90
///
/// // Relation between two lines
/// Point3 sup1(0.0, 1.0 , 0.0);
/// Point3 sup2(1.0, 0.0 , 0.0);
/// Point3 sup3(1.0, 0.0 , 1.0);
/// Line3 L1(dir1, sup1, UNDIRECTED);
/// Line3 L2(dir2, sup2);
/// Line3 L3(dir2, sup3)
/// RelLineLine rlla(L1, L2);
/// RelLineLine rllb(L1, L3);
///
/// Point3 in(rlla.getIntersectionPoint()); // in = [1 1 0]
/// Point3 in(rllb.getIntersectionPoint()); // Run-time error. Will throw
///                                         // mt::Exception: Lines do not intersect
/// \endcode

/// \file relation.h
/// \ingroup rel
/// This file includes all the headers necessary to compute the geometrical
/// relation between elements of the library; namely vectors, unit vectors,
/// points, lines, planes, spheres, cylinders, and circles.

#endif // MT_RELATION_H
