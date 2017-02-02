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


#include <stdio.h>
#include <kautham/sampling/sampleset.h>
#include <kautham/sampling/sample.h>
#include <kautham/sampling/sdksample.h>
#include <cmath>
//#include <crtdbg.h>



namespace Kautham{


  SampleSet::SampleSet(){
    samples.clear();
      ws = NULL;
  }





  SampleSet::~SampleSet(){
    clear();
  }

  bool SampleSet::existSample(unsigned long code){
    try{
        unsigned long pos = findSDKOrder(code);
	    if(samples.size() == 0 || samples.size() == pos)
		    return false;
	    else
		    return (((SDKSample*) samples[pos])->getCode() == code);
    }catch(...){
      return false;
    }
  }

  bool SampleSet::add(Sample* samp, bool orderIfCode){
    hasChanged = true;

    if( orderIfCode == true ){
      try{ //if the sample has code is a SDKSample
        unsigned long pos = findSDKOrder(((SDKSample*) samp)->getCode());
		    samples.insert(samples.begin()+pos, samp);
		    return true;
      }catch(...){
        samples.push_back(samp);
        return false;
      }
    }else{
      samples.push_back(samp);
      return true;
    }
    hasChanged = true;
  }

  bool SampleSet::addStart(Sample *smp) {
      if (add(smp)) {
          starts.push_back(smp);
          return true;
      } else {
          return false;
      }
  }


  bool SampleSet::addGoal(Sample *smp) {
      if (add(smp)) {
          goals.push_back(smp);
          return true;
      } else {
          return false;
      }
  }

  unsigned int SampleSet::indexOf(Sample* smp){
    for(unsigned int i=0; i < samples.size(); i++){
      if( samples.at(i) == smp)
        return i;
    }
    return samples.size();
  }

  Sample* SampleSet::getSampleAt(unsigned int pos){
    if(pos < samples.size())
      return samples.at(pos);
    return NULL;
  }

  bool SampleSet::removeSampleAt(unsigned int i){
    hasChanged = true;
    try{
      Sample* tmp = samples.at(i);
      samples.erase(samples.begin() + i);
      delete tmp;
	  

      hasChanged = true;
      return true;
    }catch(...){
      return false;
    }
  }





  void SampleSet::clear(){
    for (vector<Sample*>::iterator smp(samples.begin()); smp != samples.end(); ++smp) {
      delete (*smp);
    }
    samples.clear();

    hasChanged = true;
  }


  unsigned long SampleSet::findSDKOrder(unsigned long code) {
      if (samples.size() == 0) {
          return 0;
      } else {
          if (samples.size() == 1) {
              if (code > ((SDKSample*)samples[0])->getCode()) {
                  return 1;
              } else {
                  return 0;
              }
          } else {
              unsigned long inf, sup, mitad;
              inf = 0;
              sup = samples.size()-1;

              if (code > ((SDKSample*)samples[sup])->getCode()) {
                  return sup+1;
              } else {
                  while (inf != (sup-1)) {
                      mitad = (inf+sup)/2;
                      if (code > ((SDKSample*)samples[mitad])->getCode()) {
                          inf = mitad;
                      } else {
                          sup = mitad;
                      }
                  }
                  if (code <= ((SDKSample*)samples[inf])->getCode()) {
                      return inf;
                  } else {
                      return sup;
                  }
              }
          }
      }
  }
  




  
  bool SampleSet::writeSamples()
	{
		FILE *fp;
		fp = fopen("samples.txt","wt");
		if(fp==NULL) return false;

        for(unsigned i=0; i<samples.size(); i++ ){
			fprintf(fp,"%d: ", indexOf(samples.at(i)));
            fprintf(fp,"numNeighs %d: ", (int)samples.at(i)->getNeighs()->size());
            for(unsigned j=0;j<samples.at(i)->getNeighs()->size(); j++)
				fprintf(fp,"%d ", samples.at(i)->getNeighs()->at(j));
				
            fprintf(fp," distsize %d: ", (int)samples.at(i)->getNeighDistances()->size());
            for(unsigned j=0;j<samples.at(i)->getNeighDistances()->size(); j++)
				fprintf(fp,"%.1f ", samples.at(i)->getNeighDistances()->at(j));

			fprintf(fp,"\n");
		}
		fclose(fp);
		return true;
	}

}

