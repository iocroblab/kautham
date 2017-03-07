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


#if !defined(_SAMPLE_H)
#define _SAMPLE_H

#include <kautham/util/kthutil/kauthamdefs.h>
#include <kautham/sampling/robweight.h>
#include <kautham/sampling/robconf.h>

#include <boost/function.hpp>

#include <cstdio>
#include <vector>


using namespace std;


namespace Kautham{

/** \addtogroup Sampling
 *  @{
 */

//! Class sample is used to represent a configuration of Cspace.
  class Sample {
    public:
      //! Omission Constructor. This is the simplest way to create a new generic Sample.
      Sample(unsigned d);

      //! Copy constructor
      Sample(Sample *s);

      //! Destructor
      virtual ~Sample();

      //! Sets the coordinates from the array parameter coords. It create the own copy 
      //! of this values.
      bool setCoords(std::vector<KthReal>& coords);

      //! Returns the pointer to the internal coordinates.
      inline std::vector<KthReal>& getCoords() {return _coords;}

      //! Return a true if the sample is free, otherwise returns false. 
      //! If the color is equal to 1 it is free, but the color must be
      //! -1 or 0. If the color is 0 it is meaning than the samples 
      //! was not collision tested. \sa isChecked
      inline bool isFree(){return _color == 1;}

      //! Sets the freeing status. If the sample is free the color is set
      //! to 1 otherwise is set to -1.
      inline void setFree(bool fc){if(fc) _color = 1; else _color = -1;}

      //! Returns the collision check status. If the color is 0 it is meaning
      //! not collision checked.
      inline bool isChecked(){if(_color != 0) return true; return false;}

      //! This methods is provided to clear all neigbours index.
      inline void clearNeighs(){_neighset.clear();neighsetdistances.clear();}

	   //! This methods returns the value of the connected component flag.
	  inline int getConnectedComponent(){return _connectedComponent;}

	   //! This methods sets the value of the connected component flag.
	  inline void setConnectedComponent(int c){_connectedComponent = c;}

      //! Returns the distance to the smp Sample parameter. 
      KthReal getDistance(Sample* smp, Kautham::SPACETYPE spc);
      KthReal getDistance(Sample* smp, std::vector<RobWeight*> &weights, Kautham::SPACETYPE spc);

      //! Returns a pointer to a vector that contains the indexes of all  
      //! sample neighbours in the container of the SampleSet.
      inline vector<unsigned int>* getNeighs(){ return &_neighset;}

      //! Returns a pointer to a vector that contains the distances of all  
      //! sample neighbours.
      inline vector<KthReal>* getNeighDistances(){ return &neighsetdistances; }

      //! Sets the vector that contains the index of all sample neighbours.
      //! This vector in not created here, only is set from the parameter
      //! neighs.
      void setNeighs(vector<unsigned int> neighs);

	  //! Adds a neighbor at the end of vector of neighs
      void addNeigh(unsigned int  newNeigh);
	  //! Adds a neighbor at its place folowwing an incrasing distance
	  //! Also stores the distance at the distance vector
      void addNeighOrdered(unsigned int  newNeigh, KthReal newDistance, unsigned int max, bool force=false);
	  //! Adds a neighbor distance
      void addNeighDistance(KthReal newNeighDistance);

      //! Returns a string that contains the coordinates in the format:
      //! coord[0] = value coord[1] = value ... coord[dimSample-1] = value 
      //! If onlyVal parameter is true, the method returns only a values 
      //! white space separated.
      string	print(bool onlyVal = false);

      //! Returns a string that contains the index of all sample neighbours.
      //! This is an index into the SampleSet group, and it has the format:
      //! Neighbours: # Dir: dir1, dir2, ... dirDim-1
      string printNeighs();

	    //!sets transparency. Returns incremental change in tranparency
        inline KthReal setTransparency(KthReal t){KthReal aux = _transparency; _transparency = t; return (aux-t);}
        //! Returns the dimension of the sample
        inline unsigned int getDim(){return _dim;}
        inline KthReal getTransparency(){return _transparency;}
        inline int getcolor(){return _color;}
        inline void setcolor(int c){_color = c;}

      //! Returns the configuration related to scene.
      inline std::vector<RobConf>& getMappedConf(){return _config;}
      void setMappedConf(vector<RobConf>& _localConf);
      void setMappedConf(vector<RobConf*>& _localConf);

      Sample* interpolate(Sample* smp, KthReal fraction);


      inline void setwithinbounds(bool t){withinbounds=t;}
      inline bool getwithinbounds(){return withinbounds;}

    protected:
      //! Configuration mapping
      vector<RobConf> _config;
      //! This is the dimension of the sample.
      unsigned int _dim;

      //! This is the coordinates array. It must be created and set in the constructor.
      std::vector<KthReal> _coords;

      //! This is the neighbour set.
      std::vector<unsigned int> _neighset;

      //! This is the ditances to the neighbours.
      vector<KthReal> neighsetdistances; 
      //! This is the sample color used in two way, first it shows if the sample has 
      //! been collision checked and second, it shows if the samples is free or not.
      int _color;

	  //!Transparency (mean of color neighbors)
	  KthReal _transparency;

	  //! Label of the connected component for its use in PRM planners
	  int _connectedComponent;
    
      //! This flag is used because the coords of the sample may pose the robot in an out-of-bounds configuration
      bool withinbounds;
  };

  /** @}   end of Doxygen module "Sampling" */
}
#endif  //_SAMPLE_H
