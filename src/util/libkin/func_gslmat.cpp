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

/* Author: Josep-Arnau Claret Robert */


#ifndef FUNC_GSLMAT_CPP
#define FUNC_GSLMAT_CPP


#include <cassert>

#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/matrix.hpp>

#if defined(KAUTHAM_USE_GSL) && defined(NDEBUG)
  #include <gsl/gsl_matrix.h>
  #include <gsl/gsl_linalg.h>
  #include <gsl/gsl_blas.h>

  #include "func_mat.h"


  using namespace boost::numeric::ublas;


  matrix<double> inv(const matrix<double>& mat) {
  /*
  *
  * This function calculates and returns the inverse of a real square matrix 'mat'
  * using the LU decomposition through GSL library.
  *
  * As the arguments of the function are written in standard c++, we heve to convert them to GSL variables.
  *
  *
  * Author:   Josep-Arnau Claret Robert
  * Year:		01-2008
  *
  */

	  unsigned n_fil = mat.size1();
	  unsigned n_col = mat.size2();
	  assert(n_fil == n_col);


	  //STANDARD to GSL conversion//
	  //----------------------------------------------------------
	  //Here we have the argument conversions to GSL variables
	  // 'mat' becomes 'm'
	  //----------------------------------------------------------
	  gsl_matrix * m = gsl_matrix_alloc (n_fil, n_fil);
	  unsigned i = 0;
	  unsigned j = 0;
	  for (i = 0; i < n_fil; i++)
		  for (j = 0; j < n_fil; j++)
			  gsl_matrix_set (m, i, j, mat(i,j));
	  //end STANDARD to GSL conversion//
  	
	  gsl_permutation * p = gsl_permutation_alloc (n_fil);
	  int signum;
	  gsl_linalg_LU_decomp (m, p, &signum);	// Computing the LU decomposition

	  gsl_matrix * mc_inv = gsl_matrix_alloc (n_fil, n_fil);
	  gsl_linalg_LU_invert (m, p, mc_inv);	// Computing the inverse through LU decomposition

	  gsl_permutation_free(p);
	  gsl_matrix_free(m);

	  //GSL to STANDARD conversion//
	  //----------------------------------------------------------
	  //Here we have the output conversion to standard c++
	  // 'mc_inv' becomes 'm_inv'
	  //----------------------------------------------------------
	  matrix<double> m_inv (n_fil, n_fil);
	  for (i = 0; i < n_fil; i++)
		  for (j = 0; j < n_fil; j++)
			  m_inv(i,j) = gsl_matrix_get (mc_inv, i, j);
	  //end GSL to STANDARD conversion//


	  gsl_matrix_free(mc_inv);

	  return m_inv;
  }


  matrix<double> pinv(const matrix<double>& A) {

  /*
  *
  * This funtion solves the pseudoinverse of a mxn matrix 'A'
  * If m = n, so 'A' is square the inverse is computed using the LU decomposition
  * In the case where the matrix 'A' has more rows than columns, it computes the SVD decomposition of A
  * and solves the system using the GSL library.
  * When the matrix A has more columns than rows this function finds the pseudoinverse of A using
  * the SVD decomposition of the transpose of A. This is done this way:
  *
  *	Through SVD we know that:	A = U.S.trans(V)			where trans() refers to the transpose
  *	As we know that 		trans(A) = V.S.trans(U)
  *	And as we know also that	pinv(A) = V.inv(S).trans(U)		where pinv() refers to the pseudoinverse
  *																	  inv()  refers to the inverse
  *	So we only have to calculate the SVD of trans(A) to find the matrices U, S and V, and then calculate
  *   pinv(A).
  *
  * As the arguments of the function are written in standard c++, we heve to convert them to GSL variables.
  *
  */

	  unsigned int n_fil = A.size1();
	  unsigned int n_col = A.size2();


	  matrix<double> p_inv(n_col, n_fil);

	  //STANDARD to GSL conversion//
	  //----------------------------------------------------------
	  //Here we have the argument conversions to GSL variables
	  // 'A' becomes 'gA'
	  //----------------------------------------------------------
	  gsl_matrix * gA = gsl_matrix_alloc (n_fil, n_col);
	  for (unsigned int i = 0; i < n_fil; i++)
		  for (unsigned int j = 0; j < n_col; j++)
			  gsl_matrix_set (gA, i, j, A(i,j));
	  //end STANDARD to GSL conversion//

	  if (n_fil >= n_col) {	// If A has more rows than columns

		  gsl_matrix * Up = gsl_matrix_alloc (n_fil, n_col);


		  // Computing the SVD of the transpose of A
		  // The matrix 'gA_t' will contain 'U' after the function is called
		  gsl_matrix * Vp = gsl_matrix_alloc (n_col, n_col);
		  gsl_vector * S = gsl_vector_alloc (n_col);
		  gsl_vector * work = gsl_vector_alloc (n_col);
		  gsl_linalg_SV_decomp (gA, Vp, S, work);
		  gsl_vector_free(work);

		  gsl_matrix_memcpy (Up, gA);


		  //Inverting S//
		  //----------------------------------------------------------
		  // Matrix 'S' is diagonal, so it is contained in a vector.
		  // We operate to convert the vector 'S' into the matrix 'Sp'.
		  //Then we invert 'Sp' to 'Spu'
		  //----------------------------------------------------------
		  gsl_matrix * SIp = gsl_matrix_calloc (n_col, n_col);


		  unsigned int n_si = 0;			// Count number of components of s that are bigger than tolerance
		  for (unsigned int i = 0; i < n_col; i++) {
			  if (gsl_vector_get (S, i) > 0.0000001){
				  n_si++;
				  gsl_matrix_set (SIp, i, i, 1.0 / gsl_vector_get (S, i));
			  }
		  }
		  //end Inverting S//

		  gsl_matrix * SI = gsl_matrix_calloc (n_si, n_si);
		  for (unsigned int i = 0; i < n_si; i++)		gsl_matrix_set (SI, i, i, gsl_matrix_get(SIp, i, i));

		  gsl_matrix * V = gsl_matrix_alloc (n_col, n_si);
		  for (unsigned int i = 0; i < n_col; i++)
			  for (unsigned int j = 0; j < n_si; j++)
				  gsl_matrix_set (V, i, j, gsl_matrix_get(Vp, i, j));

		  gsl_matrix * U = gsl_matrix_alloc (n_fil, n_si);
		  for (unsigned int i = 0; i < n_fil; i++)
			  for (unsigned int j = 0; j < n_si; j++)
				  gsl_matrix_set (U, i, j, gsl_matrix_get(Up, i, j));

		  gsl_matrix * UT = gsl_matrix_alloc (n_si, n_fil);
		  gsl_matrix_transpose_memcpy (UT, U);					// Tranpose of U


		  //THE PSEUDOINVERSE//
		  //----------------------------------------------------------
		  //Computation of the pseudoinverse of trans(A) as pinv(A) = U.inv(S).trans(V)   with trans(A) = U.S.trans(V)
		  //----------------------------------------------------------
		  gsl_matrix * SIpUT = gsl_matrix_alloc (n_si, n_fil);
		  gsl_blas_dgemm (CblasNoTrans, CblasNoTrans,				// Calculating  inv(S).trans(V)
                         1.0, SI, UT,
                         0.0, SIpUT);
		  gsl_matrix_free(UT);
		  gsl_matrix_free(SIp);
		  gsl_matrix_free(SI);

		  gsl_matrix * pinv = gsl_matrix_alloc (n_col, n_fil);	// Calculating  U.inv(S).trans(V)
		  gsl_blas_dgemm (CblasNoTrans, CblasNoTrans,
                         1.0, V, SIpUT,
                         0.0, pinv);

		  gsl_matrix_free(SIpUT);
		  gsl_matrix_free(Up);
		  gsl_matrix_free(U);
		  gsl_matrix_free(Vp);
		  gsl_matrix_free(V);
		  gsl_vector_free(S);
		  /**/

		  //end THE PSEUDOINVERSE//


		  //GSL to STANDARD conversion//
		  //----------------------------------------------------------
		  //Here we have the output conversion to standard c++
		  // 'pinv' becomes 'p_inv'
		  //----------------------------------------------------------
		  //*
		  for (unsigned int i = 0; i < n_col; i++)
			  for (unsigned int j = 0; j < n_fil; j++)
				  p_inv(i,j) = gsl_matrix_get(pinv, i, j);
		  //end GSL to STANDARD conversion//
		  gsl_matrix_free(pinv);
		  /**/

	  } else {		// If A has more columns than rows

		  gsl_matrix * gA_t = gsl_matrix_alloc (n_col, n_fil);
		  gsl_matrix_transpose_memcpy (gA_t, gA);					// Computing the transpose of gA

		  gsl_matrix * Up = gsl_matrix_alloc (n_col, n_fil);


		  // Computing the SVD of the transpose of A
		  // The matrix 'gA_t' will contain 'U' after the function is called
		  gsl_matrix * Vp = gsl_matrix_alloc (n_fil, n_fil);
		  gsl_vector * S = gsl_vector_alloc (n_fil);
		  gsl_vector * work = gsl_vector_alloc (n_fil);
		  gsl_linalg_SV_decomp (gA_t, Vp, S, work);
		  gsl_vector_free(work);

		  gsl_matrix_memcpy (Up, gA_t);


		  //Inverting S//
		  //----------------------------------------------------------
		  // Matrix 'S' is diagonal, so it is contained in a vector.
		  // We operate to convert the vector 'S' into the matrix 'Sp'.
		  //Then we invert 'Sp' to 'Spu'
		  //----------------------------------------------------------
		  gsl_matrix * SIp = gsl_matrix_calloc (n_fil, n_fil);

		  unsigned int n_si = 0;			// Count number of components of s that are bigger than tolerance
		  for (unsigned int i = 0; i < n_fil; i++) {
			  if (gsl_vector_get (S, i) > 0.0000001){
				  n_si++;
				  gsl_matrix_set (SIp, i, i, 1.0 / gsl_vector_get (S, i));
			  }
		  }
		  //end Inverting S//

		  gsl_matrix * SI = gsl_matrix_calloc (n_si, n_si);
		  for (unsigned int i = 0; i < n_si; i++)		gsl_matrix_set (SI, i, i, gsl_matrix_get(SIp, i, i));

		  gsl_matrix * V = gsl_matrix_alloc (n_fil, n_si);
		  for (unsigned int i = 0; i < n_fil; i++)
			  for (unsigned int j = 0; j < n_si; j++)
				  gsl_matrix_set (V, i, j, gsl_matrix_get(Vp, i, j));

		  gsl_matrix * U = gsl_matrix_alloc (n_col, n_si);
		  for (unsigned int i = 0; i < n_col; i++)
			  for (unsigned int j = 0; j < n_si; j++)
				  gsl_matrix_set (U, i, j, gsl_matrix_get(Up, i, j));

		  gsl_matrix * VT = gsl_matrix_alloc (n_si, n_fil);
		  gsl_matrix_transpose_memcpy (VT, V);					// Tranpose of U


		  //THE PSEUDOINVERSE//
		  //----------------------------------------------------------
		  //Computation of the pseudoinverse of trans(A) as pinv(A) = U.inv(S).trans(V)   with trans(A) = U.S.trans(V)
		  //----------------------------------------------------------
		  gsl_matrix * SIpVT = gsl_matrix_alloc (n_si, n_fil);
		  gsl_blas_dgemm (CblasNoTrans, CblasNoTrans,				// Calculating  inv(S).trans(V)
                         1.0, SI, VT,
                         0.0, SIpVT);
		  gsl_matrix_free(VT);
		  gsl_matrix_free(SIp);
		  gsl_matrix_free(SI);

		  gsl_matrix * pinv = gsl_matrix_alloc (n_col, n_fil);	// Calculating  U.inv(S).trans(V)
		  gsl_blas_dgemm (CblasNoTrans, CblasNoTrans,
                         1.0, U, SIpVT,
                         0.0, pinv);
		  gsl_matrix_free(SIpVT);
		  gsl_matrix_free(gA_t);
		  gsl_matrix_free(Up);
		  gsl_matrix_free(U);
		  gsl_matrix_free(Vp);
		  gsl_matrix_free(V);
		  gsl_vector_free(S);


		  //end THE PSEUDOINVERSE//


		  //GSL to STANDARD conversion//
		  //----------------------------------------------------------
		  //Here we have the output conversion to standard c++
		  // 'pinv' becomes 'p_inv'
		  //----------------------------------------------------------
		  for (unsigned int i = 0; i < n_col; i++)
			  for (unsigned int j = 0; j < n_fil; j++)
				  p_inv(i,j) = gsl_matrix_get(pinv, i, j);
		  //end GSL to STANDARD conversion//
		  gsl_matrix_free(pinv);
		  /**/
	  }

	  gsl_matrix_free(gA);

	  return p_inv;
  }
#endif   // KAUTHAM_USE_GSL

#endif   // FUNC_GSLMAT_CPP //

