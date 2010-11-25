#ifndef FUNC_MATH_H
#define FUNC_MATH_H


#include "boost/numeric/ublas/vector.hpp"
#include "boost/numeric/ublas/matrix.hpp"

using namespace boost::numeric::ublas;

#if defined(KAUTHAM_USE_GSL) && defined(NDEBUG)
// func_gslmat.cpp
matrix<double> inv(const matrix<double>& mat);
matrix<double> pinv(const matrix<double>& A);

#endif  // KAUTHAM_USE_GSL

#endif   // FUNC_MATH_H //