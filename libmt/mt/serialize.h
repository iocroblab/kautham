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
#ifndef PMF_COM_MT_SERIALIZE_H
#define PMF_COM_MT_SERIALIZE_H

// BOOST LIBRARY HEADERS
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/variant.hpp>
#include <boost/serialization/string.hpp>

// MT LIBRARY HEADERS
#include <mt/basic_scalar.h>
#include <mt/empty.h>
#include <mt/vector3.h>
#include <mt/unit3.h>
#include <mt/interval.h>
#include <mt/matrix3x3.h>
#include <mt/quaternion.h>
#include <mt/rotation.h>
#include <mt/transform.h>

#include <mt/point3.h>
#include <mt/line3.h>
#include <mt/plane3.h>
#include <mt/sphere3.h>
#include <mt/cylinder3.h>
#include <mt/circle3.h>
#include <mt/ellipse3.h>

#include <mt/element.h>
#include <mt/object.h>


/////////////////////////////// NAMESPACE SCOPE //////////////////////////////

namespace boost
{
namespace serialization
{

/////////////////////////////// FUNTION DEFINITIONS //////////////////////////

/// \file serialize.h
/// This file contains functions for serializing all \e mt library classes
/// except for the relation classes.
/// The boost serialization library has been chosen as the serialization
/// framework, since it can serialize standard library containers such as
/// std::vector<T> and std::string<T>, as well as other boost library containers
/// such as boost::variant<T>.
///
/// For more information on the \e serialization library refer to
/// the <em> Boost Library </em> documentation
/// http://www.boost.org/libs/serialization/doc/index.html

// BASIC MATH CLASSES

template<class Archive, class T> inline
void serialize(Archive& ar, mt::BasicScalar<T>& s, const unsigned int version)
{
    ar & s.getRef();
}


template<class Archive> inline
void serialize(Archive& ar, mt::Vector3& v, const unsigned int version)
{
  ar & v[0];
  ar & v[1];
  ar & v[2];
}


template<class Archive> inline
void serialize(Archive& ar, mt::Unit3& u, const unsigned int version)
{
  ar & boost::serialization::base_object<mt::Vector3>(u);
}


template<class Archive> inline
void serialize(Archive& ar, mt::Interval& i, const unsigned int version)
{
  ar & i.getLowerBoundRef();
  ar & i.getUpperBoundRef();
}


template<class Archive> inline
void serialize(Archive& ar, mt::Matrix3x3& M, const unsigned int version)
{
  ar & M[0];
  ar & M[1];
  ar & M[2];
}


template<class Archive> inline
void serialize(Archive& ar, mt::Quaternion& q, const unsigned int version)
{
  ar & q[0];
  ar & q[1];
  ar & q[2];
  ar & q[3];
}


template<class Archive> inline
void serialize(Archive& ar, mt::Rotation& r, const unsigned int version)
{
  ar & boost::serialization::base_object<mt::Quaternion>(r);
}


template<class Archive> inline
void serialize(Archive& ar, mt::Transform& t, const unsigned int version)
{
  ar & t.getRotationRef();
  ar & t.getTranslationRef();
}


// GEOMETRIC ELEMENTS

template<class Archive> inline
void serialize(Archive& ar, mt::Empty& e, const unsigned int version) {}


template<class Archive> inline
void serialize(Archive& ar, mt::Point3& p, const unsigned int version)
{
  ar & boost::serialization::base_object<mt::Vector3>(p);
}


template<class Archive> inline
void serialize(Archive& ar, mt::Line3& L, const unsigned int version)
{
  ar & L.getDirectionRef();
  ar & L.getSupportRef();
  ar & L.getDirectionTypeRef();
}


template<class Archive> inline
void serialize(Archive& ar, mt::Plane3& P, const unsigned int version)
{
  ar & P.getNormalRef();
  ar & P.getDistOrigRef();
  ar & P.getDirectionTypeRef();
}


template<class Archive> inline
void serialize(Archive& ar, mt::Sphere3& S, const unsigned int version)
{
  ar & S.getCenterRef();
  ar & S.getRadiusRef();
}


template<class Archive> inline
void serialize(Archive& ar, mt::Cylinder3& C, const unsigned int version)
{
  ar & C.getAxisRef();
  ar & C.getRadiusRef();
}


template<class Archive> inline
void serialize(Archive& ar, mt::Circle3& C, const unsigned int version)
{
  ar & C.getCenterRef();
  ar & C.getRadiusRef();
  ar & C.getNormalRef();
}


template<class Archive> inline
void serialize(Archive& ar, mt::Ellipse3& E, const unsigned int version)
{
  ar & E.getCenterRef();
  ar & E.getAxisDirRef(1);
  ar & E.getAxisDirRef(2);
  ar & E.getAxisLengthRef(1);
  ar & E.getAxisLengthRef(2);
}

// MT VARIANT DOES NOT NEED SERIALIZE FUNCTION


template<class Archive> inline
void serialize(Archive& ar, mt::Element& el, const unsigned int version)
{
  ar & el.getGeometryRef();
  ar & el.getNameRef();
}


template<class Archive> inline
void serialize(Archive& ar, mt::Object& obj, const unsigned int version)
{
  ar & obj.getOriginRef();
  ar & obj.getNameRef();
}


} // serialization
} // boost

#endif // PMF_COM_MT_SERIALIZE_H
