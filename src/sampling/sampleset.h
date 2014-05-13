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
#include "sample.h"

#include <problem/workspace.h>
#include <external/libann/DNN/ANN.h>
#include <external/libann/DNN/multiann.h>

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

    //! Returns a pointer to the Sample at index pos.
    Sample*                 getSampleAt(unsigned int pos);

    //! Returns an Iterator to the SampleSet begining.
    inline vector<Sample*>::iterator getBeginIterator(){return samples.begin();}

    //! Returns an Iterator to the SampleSet ending.
    inline vector<Sample*>::iterator getEndIterator(){return samples.end();}

    //! Returns the number of contained Samples in the SampleSet.
    inline int              getSize(){return samples.size();}

    //! Returns true if the SampleSet has been changed since the 
    //! latest neighbours searching.
    inline bool             changed(){return hasChanged;}
    //inline void             changed(bool t){hasChanged = t;}

    //! This method finds and returns a set of all neighbours for the 
    //! Sample samp as a pointer to vector.  This is performed using the 
    //! Brute Force algorithm
	vector<unsigned int>*   findBFNeighs(Sample* samp, KthReal thresshold, unsigned int num);

    //! This method finds the neighbours for all contained samples in the SampleSet.
    //! This is performed using the 
    //! Brute Force algorithm
    void findBFNeighs(KthReal thresshold, unsigned int num);
 
	//! This method finds the neighbours for the contained samples in the SampleSet that do not yet have an
	//! assigned connectedcomponent.
    //! This is performed using the 
    //! Brute Force algorithm
	void	updateBFNeighs(KthReal thresshold, unsigned int num);
   
	//! This method finds the neighbours for all contained samples in the SampleSet.
    //! This is performed using the Brute Force algorithm
    void findAnnNeighs(KthReal thresshold);

    //! This method finds and returns a set of all neighbours for the 
    //! Sample samp as a pointer to vector.  This is performed using the  
    //! Approximate Nearest Neighborhood algorithm and its derivatives. 
    vector<unsigned int>*   findAnnNeighs(Sample* samp, KthReal thresshold);

	//! This method finds the neighbours for all contained samples in the SampleSet.
	vector<unsigned int>* findNeighs(Sample* samp, KthReal threshold, unsigned int num);
	//! This method finds and returns a set of all neighbours for the 
    //! Sample samp as a pointer to vector.
	void findNeighs(KthReal threshold, unsigned int num);


    //! If the SampleSet contains SDKSamples, it could return the index 
    //! of the sample whose code is the code parameter.
    long int findSDKOrder(unsigned long code);

    //! Returns true if the SampleSet contains a sample whose code
    //! is the code parameter.
    bool existSample(unsigned long code);

    //! Removes the sample at index i.
    bool removeSampleAt(unsigned int i);


    //! Return the index of a known sample smp.
    unsigned int            indexOf(Sample* smp);

    //! Clears the neighbours of the sample placed at the index i.
    void                    clearNeighs(unsigned int i);

    //! Clears the neigbours of all the samples contained in the SampleSet.
    void                    clearNeighs();

  //! Removes and deletes all samples contained in the SampleSet.
  void                    clear();

  //! Only removes the pointer stored into the vector. Take care with the memory.
  inline void             clean(){samples.clear();}

	//!creates kdtree structures for ANN search
	void setANNdatastructures(int maxneighs=0, int maxSamples=0);
	//!delete ANN data
	void resetANNdatastructures();
	//!loads ANN data from vector of samples
    void loadAnnData();

	inline void setTypeSearch(NEIGHSEARCHTYPE t){typesearch = t;};
	inline NEIGHSEARCHTYPE getTypeSearch(){return typesearch;};
	inline void setWorkspacePtr(WorkSpace* wsptr){ws = wsptr;};
	inline bool isAnnSet(){return setANN;};
  inline void setMaxNeighs(unsigned int mn){maxNeighs = mn;}
  inline unsigned int getMaxNeighs(){return maxNeighs;}

  private:
    //! Vector of pointers to the samples.
    vector<Sample*>         samples;

    //! Indicates if the SampleSet has been changed since the 
    //! latest neighbours searching.
    bool                    hasChanged;

	//!flag to indicate the kind of search: Brute force or ANN
	NEIGHSEARCHTYPE typesearch;
	bool setANN;
	//!maximum number of samples of the cspace (to reserve memory for data_pts)
	unsigned int maxSamples;
	//!maximum number neighs per sample
	unsigned int maxNeighs;
	//!ANN search structure
	MultiANN *MAG; 
	//!array of points for the ANN search for neighs
	ANNpointArray	data_pts;	
	//!pointer to workSpace
	WorkSpace *ws;
	
  };

  /** @}   end of Doxygen module "Sampling" */
}

#endif  //_SAMPLESET_H