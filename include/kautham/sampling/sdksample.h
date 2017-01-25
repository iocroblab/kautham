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

/* Author: Alexander Perez, Jan Rosell, Nestor Garcia Hidalgo */


#if !defined(_SDKSAMPLE_H)
#define _SDKSAMPLE_H


#include <string>
#include <vector>
#include <kautham/sampling/wmat.h>
#include <external/lcprng.h>			//class for linear congruential generators
#include <kautham/sampling/sample.h>

using namespace std;

namespace Kautham {


/** \addtogroup Sampling
 *  @{
 */


  //! This class is the abstraction of a sample entity. This class contains the code, the indexes
  //! and the coordinates of a sample and it provides some methods to extract and to use its 
  //! information in the exploration process.
  
	class SDKSample: public Sample {
    public:
        //! Unique constructor for a class. Indexes parameter is used for neighbours search.
        SDKSample(unsigned d, unsigned long int code, int* indexes, bool random = true);
        //!copy constructor
        SDKSample(SDKSample *s);

        ~SDKSample();

        //! Returns the sample code.
        inline unsigned long int getCode(){return code;}

        //! This method searchs the neighbours of the sample that belong to the vector of candidate
        //! samples provided that the partition level of the sample is over a given threshold.
        void searchNeighs(std::vector<SDKSample*> *candidates, int threshold);

        //! Returns an iterator to point to neighbours vector of the sample smp.
        vector<SDKSample*>::iterator getNeighs(SDKSample &smp);

        //! Returns a string containing the coordinates information and if extend parameter is true,
        //! it contains the code and the coordinates information in a explicit form.
        string	print(bool extend=true);

        //! Returns a string that contais the sample code and the codes of neighbour samples.
        string  printNeighs();

        //! This is the size of an M-Cell.
        static float sizeContainer;

        //! This is the grid partition level.
        static int M;

        static LCPRNG* gen;

    private:
        //! Pointer to the grid indexes.
        int* index;

        //! This is the sample code.
        unsigned long int code;
    };

    /** @}   end of Doxygen module "Sampling" */
}

#endif  //_SDKSAMPLE_H

