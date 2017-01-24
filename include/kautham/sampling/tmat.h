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

 
#if !defined(_TMAT_H)
#define _TMAT_H
#include <string>
using namespace std;

namespace Kautham {


/** \addtogroup Sampling
 *  @{
 */

  //! This class is the abstraction of the \f$d x d\f$ binary matrix \f$T_{d}\f$ .
  //! This class implements the square (dxd) matrix \f$T_{d}\f$ that is used to find the
  //! sequence of \f$2^{d}\f$ samples of a d-dimensional space that satisfy that the
  //! mutual distance is maximized, i.e. the minimum distance to all the previous
  //! samples of the sequence is maximized.
 
	class  TMat{
	public:
    //! This is a unique constructor.
        TMat(unsigned d=0);
		~TMat();

    //! This method returns a string that contains the text representation of the matrix.
		string printMatrix();

    //! This method multiply the matrix for a constant k.
		int multiply(const int k);
        void multiply(const int * const w, int * const res ) const ;

    //! This method multiply the matrix for other matrix V that it corresponds to m level and return 
    //! the pointer to the result.
        int **multiply(const int *const *const v, const int m) const ;
	private:
    void prime_factorization(long int x, int *fact, int *numfactors);
    void compose(int *primefactors, int numfactors, int **vC, int dimC, int trunc=0);
    void insert(int **vA, int **vB, unsigned int dimA, unsigned int dimB);

    //! This method create the matrix properly.
    void createTd();

    //! This is the dimension of the matrix.
		int d;

    //! This is the matrix values.
        int** _tMat;

    //! Pointer to the matrix that contains the results of any multiplication operation.
        int** matRes;
	};

    /** @}   end of Doxygen module "Sampling" */
}

#endif  //_TMAT_H

