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
#ifndef MT_OBJECT_H
#define MT_OBJECT_H

// C++ STANDARD HEADERS
#include <iostream>
#include <string>

// MT LIBRARY HEADERS
#include <mt/transform.h>


/////////////////////////////// NAMESPACE SCOPE //////////////////////////////

namespace mt
{


/////////////////////////////// CLASS DEFINITION /////////////////////////////

/// \brief Geometric object description class.
///
/// Geometric objects are described by a transformation that specifies its
/// origin and a string with its name.

class Object
{
public:

// LIFECYCLE

  /// Default constructor.
  Object();

  /// Constructor.
  Object(const Transform&   origin,
         const std::string& name);

  // Compiler generated copy constructor is being used

  // Compiler generated destructor is being used


// OPERATORS

  // Compiler generated assignment operator is being used

  bool operator==(const Object& obj) const;

  bool operator!=(const Object& obj) const;


// ACCESS

  /// Gets transformation representing the object's origin.
  Transform getOrigin() const;
  Transform& getOriginRef();
  const Transform& getOriginRef() const;

  /// Gets object name.
  std::string getName() const;
  std::string& getNameRef();
  const std::string& getNameRef() const;

  /// Sets object origin.
  void setOrigin(const Transform& origin);

  /// Sets object name.
  void setName(const std::string& name);


private:

// MEMBERS

  Transform m_origin;
  std::string m_name;

};

/////////////////////////////// HELPER FUNCTIONS /////////////////////////////

// OPERATORS

std::ostream& operator<<(std::ostream& os,
                         const Object& obj);

/////////////////////////////// INLINE METHODS ///////////////////////////////

// LIFECYCLE

inline Object::Object() {}


inline Object::Object(const Transform&   origin,
                      const std::string& name) :

                      m_origin(origin),
                      m_name  (name) {}


// OPERATORS

inline bool Object::operator==(const Object& obj) const
{
  return(m_origin == obj.getOrigin() &&
         m_name   == obj.getName());
}


inline bool Object::operator!=(const Object& obj) const
{
  return !(*this == obj);
}


// ACCESS

inline Transform Object::getOrigin() const
{
  return m_origin;
}


inline Transform& Object::getOriginRef()
{
  return m_origin;
}


inline const Transform& Object::getOriginRef() const
{
  return m_origin;
}


inline std::string Object::getName() const
{
  return m_name;
}


inline std::string& Object::getNameRef()
{
  return m_name;
}


inline const std::string& Object::getNameRef() const
{
  return m_name;
}


inline void Object::setOrigin(const Transform& origin)
{
  m_origin = origin;
}


inline void Object::setName(const std::string& name)
{
  m_name = name;
}


/////////////////////////////// INLINE HELPER FUNCTIONS //////////////////////

inline std::ostream& operator<<(std::ostream& os,
                                const Object& obj)
{
  return
  os << "Object " << obj.getName() << " - origin:\n" << obj.getOrigin();
}

} // mt

#endif // MT_OBJECT_H
