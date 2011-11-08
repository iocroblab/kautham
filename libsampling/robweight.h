#ifndef ROBWEIGHT_H
#define ROBWEIGHT_H

#include <libutil/kauthamdefs.h>

namespace libSampling{
  class RobWeight{
  public:
      RobWeight(unsigned int dim);
      inline void                   setRnDim(unsigned int d){_rn.resize(d);}
      inline void                   setSE3Weight(KthReal t, KthReal r){_se3[0] = t;_se3[1] = r;}
      inline KthReal*               getSE3Weight(){return _se3;}
      inline std::vector<KthReal>&  getRnWeights(){return _rn;}
      void                          setRnWeigh(unsigned int i, KthReal w);
  private:
      //! This _se3 attribute is an array where the 
      //! first component is translational weight and the
      //! second one is the rotational weight.
      KthReal                       _se3[2];

      //! This attribute is a vector that contains
      //! the respective weight for each joint.
      std::vector<KthReal>          _rn;
  };
}

#endif // ROBWEIGHT_H
