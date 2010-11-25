#include "robweight.h"

namespace libSampling{
  RobWeight::RobWeight(unsigned int dim) {
    setRnDim(dim);
    for(unsigned int i =0; i < dim; i++)
      _rn.at(i) = (KthReal) 1.0;
    _se3[0] = (KthReal) 1.0;
    _se3[1] = (KthReal) 1.0;
  }

  void RobWeight::setRnWeigh(unsigned int i, KthReal w){
    if( i < _rn.size())
      _rn.at(i) = w;
  }
}
