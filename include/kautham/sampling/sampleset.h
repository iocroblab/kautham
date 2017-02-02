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



#if !defined(_SAMPLESET_H)
#define _SAMPLESET_H

#include <vector>

#include <kautham/sampling/sample.h>
#include <kautham/problem/workspace.h>


using namespace std;

namespace Kautham{


/** \addtogroup Sampling
 *  @{
 */

  //! This is the samples container.  It provides some functionalities like 
  //! neighbours searching in several ways.
  class SampleSet{

  public:
    SampleSet();
    ~SampleSet();
	//! writes info of sample is file samples.txt
	bool writeSamples();

    //! This method inserts a sample into the vector. In the case of use SDKSampler
    //! the samples will be inserted in code order if orderIfCode is true.
    bool                    add(Sample* samp, bool orderIfCode = false);

    bool addStart(Sample *smp);

    bool addGoal(Sample *smp);

    inline std::size_t getNumStarts() {return starts.size();}

    inline std::size_t getNumGoals() {return goals.size();}

    inline Sample *getStart(std::size_t index) {return starts.at(index);}

    inline Sample *getGoal(std::size_t index) {return goals.at(index);}

    //! Returns a pointer to the Sample at index pos.
    Sample*                 getSampleAt(unsigned int pos);

    //! Returns an Iterator to the SampleSet begining.
    inline vector<Sample*>::iterator getBeginIterator(){return samples.begin();}

    //! Returns an Iterator to the SampleSet ending.
    inline vector<Sample*>::iterator getEndIterator(){return samples.end();}

    //! Returns the number of contained Samples in the SampleSet.
    inline unsigned          getSize(){return samples.size();}

    //! Returns true if the SampleSet has been changed since the 
    //! latest neighbours searching.
    inline bool             changed(){return hasChanged;}




    //! If the SampleSet contains SDKSamples, it could return the index 
    //! of the sample whose code is the code parameter.
    unsigned long findSDKOrder(unsigned long code);

    //! Returns true if the SampleSet contains a sample whose code
    //! is the code parameter.
    bool existSample(unsigned long code);

    //! Removes the sample at index i.
    bool removeSampleAt(unsigned int i);


    //! Return the index of a known sample smp.
    unsigned int            indexOf(Sample* smp);



  //! Removes and deletes all samples contained in the SampleSet.
  void                    clear();

  //! Only removes the pointer stored into the vector. Take care with the memory.
  inline void             clean(){samples.clear();}


  inline void setWorkspacePtr(WorkSpace* wsptr){ws = wsptr;}



  private:
    //! Vector of pointers to the samples.
    vector<Sample*>         samples;

    std::vector<Sample*> starts;

    std::vector<Sample*> goals;

    //! Indicates if the SampleSet has been changed since the 
    //! latest neighbours searching.
    bool                    hasChanged;

	//!pointer to workSpace
	WorkSpace *ws;
	
  };

  /** @}   end of Doxygen module "Sampling" */
}

#endif  //_SAMPLESET_H
