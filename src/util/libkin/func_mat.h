
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

/* Author: Alexander Perez, Jan Rosell */
/** \addtogroup libKin
 *  @{
 */

#ifndef FUNC_MATH_H
#define FUNC_MATH_H


#include "boost/numeric/ublas/vector.hpp"
#include "boost/numeric/ublas/matrix.hpp"

using namespace boost::numeric::ublas;

#if defined(KAUTHAM_USE_GSL) && defined(NDEBUG)
// func_gslmat.cpp
matrix<double> inv(const matrix<double>& mat);
matrix<double> pinv(const matrix<double>& A);

/** @}   end of Doxygen module "Util */
#endif  // KAUTHAM_USE_GSL

#endif   // FUNC_MATH_H //
