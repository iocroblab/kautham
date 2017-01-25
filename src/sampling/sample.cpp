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


#include <sstream>
#include <iostream>
#include <cmath>
#include <kautham/sampling/sample.h>

using namespace std;

namespace Kautham{

/*! \class Sample
 * This class is the Sample abstraction. It has the dimSample coordinates
 * into a unit cube. If the sample represent an SE3 sample(dimSample=6),
 * it would be mapped to the respective position and orientation
 * representation.
 */

  Sample::Sample(unsigned d){
    _dim = d;
    _coords.resize(_dim);
    _color = 0;
    _connectedComponent = -1;
	_transparency = 0;
  }

  Sample::~Sample(){
    _coords.clear();
    _config.clear();
  }

  bool Sample::setCoords(std::vector<KthReal>& coords){
      if (coords.size() == _dim) {
          for (unsigned int i = 0; i < _dim; ++i)
              _coords[i] = coords[i];
          //if sample has already a config, clear it
          if (_config.size() != 0) _config.clear();
          return true;
      } else {
          return false;
      }
  }

  string Sample::print(bool onlyVal) {
    std::ostringstream s;
    string sal="";
    if(onlyVal){
      for(unsigned i=0; i < _dim; i++)
        s << _coords[i] << " "  ;
      
      sal = s.str();
      sal = sal.substr(0, sal.length() - 1);
    }else{
      for(unsigned i=0; i < _dim; i++){
        s << " coor[" ;
        s << i ;
        s << "]= ";
        s << _coords[i] ;
        s << " "  ;
      }
      s << std::endl;
      sal = s.str();
    }
    return sal;
  }

  string Sample::printNeighs(){
    std::ostringstream s;
    s << "Neighbours: ";
    s << _neighset.size() << " Dir: ";
    for(unsigned int i=0; i < _neighset.size(); i++){
      s << _neighset[i]/*->print()*/ << ", "  ;
    }
    s << std::endl;
    return s.str();
  }

   //!copy constructor
	Sample::Sample(Sample *s)
	{
		_dim = s->getDim();
    _coords.resize(_dim);
        for(unsigned i=0; i < _dim;i++)
      _coords[i] = s->getCoords()[i];
		_color = s->_color;
		_connectedComponent = s->_connectedComponent;
		_transparency = s->_transparency;
		
	
    for(unsigned i=0; i<s->getNeighs()->size();i++)
			_neighset.push_back(s->getNeighs()->at(i));
		
        for(unsigned i=0; i<s->neighsetdistances.size();i++)
      neighsetdistances.push_back(s->neighsetdistances[i]);
		
		_connectedComponent = s->_connectedComponent;
	}

  void Sample::addNeigh(unsigned int newNeigh){
    _neighset.push_back(newNeigh);
  }

  void Sample::addNeighOrdered(unsigned int newNeigh, KthReal newDistance, unsigned int max, bool force){
	 //find where to place the sample in an increasing order of distances
	unsigned int pos=0;
  for(unsigned int i=0; i< neighsetdistances.size();i++)
	{
		if(newDistance > neighsetdistances[i]) pos++;
	}
	//insert the sample
	_neighset.insert(_neighset.begin()+pos, newNeigh);
	neighsetdistances.insert(neighsetdistances.begin()+pos, newDistance);
	//delete the last sample if vector list out of bounds
    if (_neighset.size() > max) {
        if (pos==max && force) {
			//if the new sample has to be included by force (flag force set to true) and it results to
			//be the last one (the farthest) then we should delete not the last of the list but the 
			//the last minus one, in order not to delete the new sample
			_neighset.erase(_neighset.end()-2);
			neighsetdistances.erase(neighsetdistances.end()-2);
		}
		else{
			//delete the one that is farthest, i.e. the last one in the list
			_neighset.erase(_neighset.end()-1);
			neighsetdistances.erase(neighsetdistances.end()-1);
		}
    }
	
  }
  
  void Sample::addNeighDistance(KthReal newNeighDistance){
	  neighsetdistances.push_back(newNeighDistance);
  }


  void Sample::setNeighs(vector<unsigned int> neighs){
    _neighset.clear();
    for(unsigned int i=0;i<neighs.size();i++)
      _neighset.push_back(neighs.at(i));
  }

  //! This method computes the distance to a Sample smp
  //! based on the distance of each configuration who conforms
  //! the sample to configuration mapping.
  //! \$ dist = \left( \sum^{m}_{i=1} \left(dist(RobConf_{1i},
  //! RobConf_{2i}) \right)^{2} \right)^{frac{1}{2}}\$
  KthReal Sample::getDistance(Sample* smp, Kautham::SPACETYPE spc){
    if( smp == NULL ) return -1.0;
    // Each sample has a vector<RobConf> who has a SE3 and a Rn configuration
    // inside.
    KthReal dist=0.0;

    try{
		  if( spc == Kautham::CONFIGSPACE && _config.size() > 0   ){
		    for(unsigned int i = 0; i < _config.size(); i++){
			    SE3Conf& a = _config.at(i).getSE3();
			    dist += a.getDistance2(smp->getMappedConf().at(i).getSE3());

			    RnConf& b = _config.at(i).getRn();
			    if( b.getDim() != 0 )
			      dist += b.getDistance2(smp->getMappedConf().at(i).getRn());
		    }
		  }else{
		    vector<KthReal>& other = smp->getCoords();
		    for(unsigned int i = 0; i < _coords.size(); i++)
			    dist += (_coords.at(i) - other.at(i))*(_coords.at(i) - other.at(i));
		  }

      return sqrt(dist);
    }catch(...){}
    return -1.;
  }

  KthReal Sample::getDistance(Sample* smp, std::vector<RobWeight*> &weights,
                              Kautham::SPACETYPE spc){
    try{
      if( smp == NULL ) return -1.0;
      // Each sample has a vector<RobConf> who has a SE3 and a Rn configuration
      // inside.
      KthReal dist=0.0;
  	
		  if( spc == Kautham::CONFIGSPACE && _config.size() > 0   ){

		    for(unsigned int i = 0; i < _config.size(); i++){
			    SE3Conf& a = ((RobConf&)_config.at(i)).getSE3();
			    dist += a.getDistance2(smp->getMappedConf().at(i).getSE3(),
									    weights.at(i)->getSE3Weight()[0],
                      weights.at(i)->getSE3Weight()[1]);

			    RnConf& b = ((RobConf&)_config.at(i)).getRn();

			    if( b.getDim() != 0 )
			      dist += b.getDistance2(smp->getMappedConf().at(i).getRn(),
									       weights.at(i)->getRnWeights());
		    }
		  }else{
		    vector<KthReal>& other = smp->getCoords();
		    for(unsigned int i = 0; i < _coords.size(); i++)
			  dist += (_coords.at(i) - other.at(i))*(_coords.at(i) - other.at(i));
		  }
      
      return sqrt(dist);
    }catch(...){}
    return -1.;
  }

  void Sample::setMappedConf(vector<RobConf>& _localConf){
    if (_config.size() != _localConf.size()) {
      if (_config.size() != 0) _config.clear();

      for (unsigned int i = 0; i < _localConf.size(); ++i) {
        _config.push_back(_localConf.at(i));
      }
    } else {
      for (unsigned int i = 0; i < _localConf.size(); ++i) {
        _config.at(i) = _localConf.at(i);
      }
    }
  }

  // This methods is used when the _localConf is the WorkSpace
  // arrange of the RobConf of each robot
  void Sample::setMappedConf(vector<RobConf*>& _localConf){
    if( _config.size() != _localConf.size() ){
      if( _config.size() != 0  ) _config.clear();

      for(unsigned i = 0; i < _localConf.size(); i++)
        _config.push_back(*(_localConf.at(i)));

    }else{
      for(unsigned i = 0; i < _localConf.size(); i++)
        _config.at(i) = *(_localConf.at(i));
    }
  }


  //! Retunrs an interpolated sample as a fraction of the distance to smp.
  //! Aditional, if the sample has a mapped configuration, it returns the 
  //! interpolated configuration in the same proportion as the fraction.
  Sample* Sample::interpolate(Sample* smp, KthReal fraction){
    Sample *tmp = new Sample(smp->getDim());
    //Sample tmp(smp->getDim());

	
    // Interpolation in sample space.
    vector<KthReal>& other = smp->getCoords();
    for(unsigned int i = 0; i < _coords.size(); i++)
        tmp->_coords.at(i) = _coords.at(i) + fraction*(other.at(i) - _coords.at(i));


    if(_config.size() > 0){
      // Interpolation in configuration space if it exists.
      //RobConf tmpRobConf;
      vector<RobConf> tmpVec;

	  for(unsigned int i = 0; i < _config.size(); i++){
        //tmpRobConf = _config.at(i).interpolate(smp->getMappedConf().at(i));
        tmpVec.push_back(_config.at(i).interpolate(smp->getMappedConf().at(i), fraction));
	  }

      tmp->setMappedConf(tmpVec);
    }
	
	

	
    return tmp;
  }

}
