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


#if !defined(_SAMPLER_H)
#define _SAMPLER_H
#include <kautham/sampling/sample.h>

//using namespace std;
namespace Kautham{

/** \addtogroup Sampling
 *  @{
 */

  //! This is the abstract sampler used as a template to generate
  //! all the sampler variants. To create a new Sampler you will only need
  //! to derive this class and implement the nextSample() method.
  class Sampler {
  public:
      virtual ~Sampler() {}

    //! This is the most important method of this class. 
    //! All derived classes will have it implemented.
    //! This method makes a new sample following their own
    //! algorithm.
    virtual Sample* nextSample()=0;
          
    //! Returns the last generated sample.  This is an alternative
    //! way to access to the last generated sample.
    inline Sample* getSample(){return _current;}

    //! Set the dimension of the samples that will be generated.
    inline void setDim(char dim){dimension=dim;}

    //! Retrieve the dimension of the samples that will be generated.
    inline char getDim(){return dimension;}
  protected:

    //! This is the dimension.
    unsigned dimension;

    //! This is an internal pointer to the latest generated sample.
    Sample* _current;
  };

  /** @}   end of Doxygen module "Sampling" */
}

#endif  //_SAMPLER_H
