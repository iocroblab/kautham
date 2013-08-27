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
#ifndef MT_INTERVAL_H
#define MT_INTERVAL_H

// C++ STANDARD HEADERS
#include <iostream>
#include <utility>

// MT LIBRARY HEADERS
#include <mt/scalar.h>


/////////////////////////////// USING DECLARATIONS/DIRECTIVES ////////////////


/////////////////////////////// NAMESPACE SCOPE //////////////////////////////

namespace mt
{


/////////////////////////////// CLASS DEFINITION /////////////////////////////

/// \file
/// \ingroup basic

/// \ingroup basic
/// \brief One-dimensional closed interval class.
///
/// This is an example of how to use the Interval class:
///
/// \code
/// using namespace mt;
///
/// // Some interval declarations
/// Interval in1;              // outputs as [0.0, 0.0]
/// Interval in2(1.0);         // outputs as [1.0, 1.0]
/// Interval in3(0.0, 2.0);    // outputs as [0.0, 2.0]
/// Interval in4(-1.0, 1.0);   // outputs as [-1.0, 1.0]
/// Interval in5(-0.5, -0.25); // outputs as [-0.5, -0.25]
///
/// // Tests and operations using intervals
/// bool test;
/// Scalar s;
///
/// test = isOverlap(in3, in4)   // true
/// test = isOverlap(in3, in5)   // false
/// test = isContained(1.0, in3) // true
/// test = isContained(in5, in4) // true
/// test = isContained(in5, in3) // false
///
/// in   = hull(in3, in5)        // in = [-0.5, 2.0]
/// in   = hull(in2, in5)        // in = [-0.5, 1.0]
///
/// in4.widen(1.0)               // in4 = [-2.0, 2.0]
/// s = in4.center()             // s = 0.0
/// s = in4.width()              // s = 4.0
/// \endcode

class Interval
{
public:

// LIFECYCLE

  /// Default constructor.
  Interval();

  /// Constructor for single point interval (degenerate case [\a s, \a s]).
  Interval(const Scalar& s);

  /// Constructor for lower and upper bounds input (order is irrelevant).
  Interval(const Scalar& bound1,
           const Scalar& bound2);

  // Compiler generated copy constructor is being used

  // Compiler generated destructor is being used


// OPERATORS

  // Compiler generated assignment operator is being used

  bool operator==(const Interval& in) const;
  bool operator!=(const Interval& in) const;

// OPERATIONS

  /// Interval center.
  Scalar center() const;

  /// Interval width.
  Scalar width() const;

  /// Widen interval by 2 \a s (\a s at each endpoint).
  Interval& widen(const Scalar& s);


// ACCESS

  /// Gets interval lower bound.
  Scalar getLowerBound() const;
  Scalar& getLowerBoundRef();
  const Scalar& getLowerBoundRef() const;

  /// Gets interval upper bound.
  Scalar getUpperBound() const;
  Scalar& getUpperBoundRef();
  const Scalar& getUpperBoundRef() const;

  /// Sets interval.
  void setValue(const Scalar& s);

  /// Sets interval.
  void setValue(const Scalar& lb,
                const Scalar& ub);


private:

// MEMBERS

Scalar       m_lb;
Scalar       m_ub;

};


/////////////////////////////// HELPER FUNCTIONS /////////////////////////////

// OPERATORS

Interval operator+(const Scalar&   s,
                   const Interval& in);

Interval operator+(const Interval& in1,
                   const Interval& in2);

Interval operator-(const Scalar&   s,
                   const Interval& in);

Interval operator-(const Interval& in1,
                   const Interval& in2);

std::ostream& operator<<(std::ostream&   os,
                         const Interval& in);


// FUNCTIONS

/// Tests if intervals overlap.
bool isOverlap(const Interval& in1,
               const Interval& in2);

/// Tests if scalar \a s is contained in interval \a in.
bool isContained(const Scalar&   s,
                 const Interval& in);

/// Tests if interval \a in1 is contained in interval \a in2.
bool isContained(const Interval& in1,
                 const Interval& in2);

/// Returns interval \a in widened by 2 \a s (\a s at each endpoint).
Interval widen(const Interval& in,
               const Scalar&   s);

/// Computes the hull of the input intervals.
Interval hull(const Interval& in1,
              const Interval& in2);


/////////////////////////////// INLINE METHODS ///////////////////////////////

// LIFECYCLE

inline Interval::Interval() :

                          m_lb(0.0),
                          m_ub(0.0) {}


inline Interval::Interval(const Scalar& s) :

                          m_lb(s),
                          m_ub(s) {}


inline Interval::Interval(const Scalar& bound1,
                          const Scalar& bound2)
{
  if (bound1 <= bound2)
  {
    m_lb = bound1;
    m_ub = bound2;
  }
  else
  {
    m_lb = bound2;
    m_ub = bound1;
  }
}


// OPERATORS

inline bool Interval::operator==(const Interval& in) const
{
  return (m_lb == in.m_lb && m_ub == in.m_ub);
}


inline bool Interval::operator!=(const Interval& in) const
{
  return std::rel_ops::operator!=(*this, in);
}


// OPERATIONS

inline Scalar Interval::center() const
{
  return (m_lb + m_ub) * Scalar(0.5);
}


inline Scalar Interval::width() const
{
  return m_ub - m_lb;
}


inline Interval& Interval::widen(const Scalar& s)
{
  m_lb -= s;
  m_ub += s;
  return *this;
}


// ACCESS

inline Scalar Interval::getLowerBound() const
{
  return m_lb;
}


inline Scalar& Interval::getLowerBoundRef()
{
  return m_lb;
}


inline const Scalar& Interval::getLowerBoundRef() const
{
  return m_lb;
}


inline Scalar Interval::getUpperBound() const
{
  return m_ub;
}


inline Scalar& Interval::getUpperBoundRef()
{
  return m_ub;
}


inline const Scalar& Interval::getUpperBoundRef() const
{
  return m_ub;
}


inline void Interval::setValue(const Scalar& s)
{
  m_lb = s;
  m_ub = s;
}


inline void Interval::setValue(const Scalar& lb,
                               const Scalar& ub)
{
  m_lb = lb;
  m_ub = ub;
}

/////////////////////////////// INLINE HELPER FUNCTIONS //////////////////////

// OPERATORS

inline Interval operator+(const Scalar&   s,
                          const Interval& in)
{
  return Interval(in.getLowerBound() + s,
                  in.getUpperBound() + s);
}


inline Interval operator+(const Interval& in1,
                          const Interval& in2)
{
  return Interval(in1.getLowerBound() + in2.getLowerBound(),
                  in1.getUpperBound() + in2.getUpperBound());
}


inline Interval operator-(const Scalar&   s,
                          const Interval& in)
{
  return Interval(in.getLowerBound() - s,
                  in.getUpperBound() - s);
}


inline Interval operator-(const Interval& in1,
                          const Interval& in2)
{
  return Interval(in1.getLowerBound() - in2.getLowerBound(),
                  in1.getUpperBound() - in2.getUpperBound());
}


inline std::ostream& operator<<(std::ostream&   os,
                                const Interval& in)
{
  return os << '[' << in.getLowerBound() << ", "
                   << in.getUpperBound() << ']';
}


// FUNCTIONS

inline bool isOverlap(const Interval& in1,
                      const Interval& in2)
{
  return (in1.getLowerBound() <= in2.getUpperBound()) &&
         (in2.getLowerBound() <= in1.getUpperBound());
}


inline bool isContained(const Scalar&   s,
                        const Interval& in)
{
  return s >= in.getLowerBound() &&
         s <= in.getUpperBound();
}


inline bool isContained(const Interval& in1,
                        const Interval& in2)
{
  return in1.getLowerBound() >= in2.getLowerBound() &&
         in1.getUpperBound() <= in2.getUpperBound();
}


inline Interval widen(const Interval& in,
                      const Scalar&   s)
{
  Interval in1(in);
  return in1.widen(s);
}


inline Interval hull(const Interval& in1,
                     const Interval& in2)
{
  return Interval(min(in1.getLowerBound(), in2.getLowerBound()),
                  max(in1.getUpperBound(), in2.getUpperBound()));
}


} // mt

#endif // MT_INTERVAL_H
