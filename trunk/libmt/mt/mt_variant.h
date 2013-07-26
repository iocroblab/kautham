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
#ifndef MT_MT_VARIANT_H
#define MT_MT_VARIANT_H

// C++ STANDARD HEADERS
#include <string>
#include <utility>

// BOOST LIBRARY HEADERS
#include <boost/variant.hpp>

// MT LIBRARY HEADERS
#include <mt/empty.h>
#include <mt/vector3.h>
#include <mt/unit3.h>
#include <mt/point3.h>
#include <mt/line3.h>
#include <mt/plane3.h>
#include <mt/sphere3.h>
#include <mt/cylinder3.h>
#include <mt/circle3.h>
#include <mt/ellipse3.h>
#include <mt/transform_utility.h>


/////////////////////////////// NAMESPACE SCOPE //////////////////////////////

namespace mt
{


/////////////////////////////// TYPE DEFINITION //////////////////////////////

/// \brief Universal container of \e mt library geometric features.
///
/// The MtVariant type acts as a \e variant or
/// <em> generalized union </em> capable of holding any of the following 3D
/// geometric elements:
///
/// - Vectors
/// - Unit vectors
/// - Points
/// - Lines
/// - Planes
/// - Spheres
/// - Cylinders
/// - Circles
/// - Ellipses
///
/// The class features additional functionalities such as equality
/// comparisons, output streaming, a string
/// representations of the contained element type (i.e., "vector", "line"),
/// and a function that applies a rigid transform to the contained element.
///
/// The type is implemented using \e boost::variant.
/// For more information on the \e variant data structure refer to
/// the <em> Boost Library </em> documentation
/// http://www.boost.org/doc/html/variant.html


typedef boost::variant<Empty,
                       Vector3,
                       Unit3,
                       Point3,
                       Line3,
                       Plane3,
                       Sphere3,
                       Cylinder3,
                       Circle3,
                       Ellipse3> MtVariant;


/////////////////////////////// HELPER FUNCTIONS /////////////////////////////

/*
bool operator!=(const MtVariant& geom1,
                const MtVariant& geom2);
*/

std::string getTypeName(const MtVariant& geom);

bool isEmpty(const MtVariant& geom);


/// Applies input transform to variant
MtVariant apply(const Transform& tr,
                const MtVariant& geom);


/////////////////////////////// VISITOR CLASSES //////////////////////////////

class NameVisitor : public boost::static_visitor<std::string>
{
public:
  std::string operator()(const Vector3&   v) const {return "vector";}
  std::string operator()(const Unit3&     u) const {return "unit vector";}
  std::string operator()(const Point3&    p) const {return "point";}
  std::string operator()(const Line3&     L) const {return "line";}
  std::string operator()(const Plane3&    P) const {return "plane";}
  std::string operator()(const Sphere3&   s) const {return "sphere";}
  std::string operator()(const Cylinder3& c) const {return "cylinder";}
  std::string operator()(const Circle3&   C) const {return "circle";}
  std::string operator()(const Ellipse3&  E) const {return "ellipse";}
  template<class T>
  std::string operator()(const T&)           const {return "";}
};


class TransformVisitor : public boost::static_visitor<MtVariant>
{
public:
  TransformVisitor(const Transform& tr) : m_tr(tr) {}

  template<class T>
  MtVariant operator()(const T& t) const {return MtVariant(apply(m_tr, t));}

private:
  Transform m_tr;
};


/////////////////////////////// INLINE HELPER FUNCTIONS //////////////////////

/*
inline bool operator!=(const MtVariant& geom1,
                       const MtVariant& geom2)
{
  return std::rel_ops::operator!=(geom1, geom2);
}
*/


inline std::string getTypeName(const MtVariant& geom)
{
  return boost::apply_visitor(NameVisitor(), geom);
}


inline bool isEmpty(const MtVariant& geom)
{
  return boost::apply_visitor(EmptyVisitor(), geom);
}


inline MtVariant apply(const Transform& tr,
                       const MtVariant& geom)
{
  return boost::apply_visitor(TransformVisitor(tr), geom);
}

} // mt

#endif // MT_MT_VARIANT_H


