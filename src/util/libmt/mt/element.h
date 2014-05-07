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
#ifndef MT_ELEMENT_H
#define MT_ELEMENT_H

// C++ STANDARD HEADERS
#include <iostream>
#include <string>

// MT LIBRARY HEADERS
#include <mt/mt_variant.h>


/////////////////////////////// NAMESPACE SCOPE //////////////////////////////

namespace mt
{


/////////////////////////////// CLASS DEFINITION /////////////////////////////

/// \brief Geometric element description class.
///
/// Geometric elements are described by their respective geometric
/// representation and a string with the element name.

class Element
{

public:

// LIFECYCLE

  Element();

  /// Constructor.
  Element(const MtVariant&   geom,
          const std::string& el_name  = "");

  template<class T>
  Element(const T&                 val,
          const std::string&       el_name  = "");

  // Compiler generated copy constructor is being used

  // Compiler generated destructor is being used


// OPERATORS

  // Compiler generated assignment operator is being used

  bool operator==(const Element& el) const;

  bool operator!=(const Element& el) const;

// ACCESS

  /// Gets element geometry.
  const MtVariant getGeometry() const;
  MtVariant& getGeometryRef();
  const MtVariant& getGeometryRef() const;

  /// Gets element name.
  const std::string getName() const;
  std::string& getNameRef();
  const std::string& getNameRef() const;

  /// Sets element geometry.
  void setGeometry(const MtVariant& geom);

  /// Sets element name.
  void setName(const std::string& el_name);


private:

// MEMBERS
  MtVariant   m_geom;
  std::string m_el_name;
};


/////////////////////////////// HELPER FUNCTIONS /////////////////////////////

// OPERATORS

std::ostream& operator<<(std::ostream&  os,
                         const Element& el);


/////////////////////////////// INLINE METHODS ///////////////////////////////

// LIFECYCLE

inline Element::Element() {}

inline Element::Element(const MtVariant&   geom,
                        const std::string& el_name) :

                        m_geom    (geom),
                        m_el_name (el_name) {}

template<class T> inline
Element::Element(const T&           val,
                 const std::string& el_name) :

                 m_geom    (MtVariant(val)),
                 m_el_name (el_name) {}


// OPERATORS

inline bool Element::operator==(const Element& el) const
{
  return (m_geom    == el.getGeometry() &&
          m_el_name == el.getName());
}


inline bool Element::operator!=(const Element& el) const
{
  return !(*this == el);
}


// ACCESS

inline const MtVariant Element::getGeometry() const
{
  return m_geom;
}


inline MtVariant& Element::getGeometryRef()
{
  return m_geom;
}


inline const MtVariant& Element::getGeometryRef() const
{
  return m_geom;
}


inline const std::string Element::getName() const
{
  return m_el_name;
}


inline std::string& Element::getNameRef()
{
  return m_el_name;
}


inline const std::string& Element::getNameRef() const
{
  return m_el_name;
}


inline void Element::setGeometry(const MtVariant& geom)
{
  m_geom = geom;
}


inline void Element::setName(const std::string& el_name)
{
  m_el_name = el_name;
}


/////////////////////////////// INLINE HELPER FUNCTIONS //////////////////////

// OPERATORS

inline std::ostream& operator<<(std::ostream&  os,
                                const Element& el)
{
  return os << el.getName() << ":\n"
            << el.getGeometry();
}


} // mt

#endif // MT_ELEMENT_H
