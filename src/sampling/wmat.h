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


#if !defined(_WMAT_H)
#define _WMAT_H
#include <vector>
#include <string>

using namespace std;

namespace Kautham {


/** \addtogroup Sampling
 *  @{
 */

  //! This class is the abstraction of the \f$d x M\f$ matrix of weights, W.
  //! This class is the matrix that it contains the values of weigths.
  
	class WMat {
	public:
    //! This is the unique constructor provided.
		WMat(int dim, int level);
		~WMat();

    //! This member method returns a string that contains a text representationm of the matrix.
		std::string printMatrix();
    
    //! This method sets the value of the row (index1) and the column (index2) specified.
		inline void setRow(const int index1, const int index2, const long value){
			if( (index1 >= 0 && index1<d )&&(index2 >= 0 && index2 < m)){
				w[index1][index2] = value;
			}
		}

    //! This method returns the value of the row (index1) and the column (index2) specified.
		inline long getRow(const int index1, const int index2){
			if( (index1 >= 0 && index1<d )&&(index2 >= 0 && index2 < m))
				return w[index1][index2];
			else
				return -1;	
		}
		
	private:
    //! This is the dimension of matrix.
		int d;

    //! This is the maximum level of samplig.
		int m;

    //! This is the matrix values.
		long int** w;
	protected:
    //! This is a protected constructor used to restrict the construction way without a correct parameters.
		WMat();
	};

    /** @}   end of Doxygen module "Sampling" */
}

#endif  //_WMAT_H

