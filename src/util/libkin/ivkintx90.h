
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

/* Author: Alexander Perez, Jan Rosell and Nestor Garcia Hidalgo */
 
 

#if !defined(_IVKINTX90_H)
#define _IVKINTX90_H

#include "inversekinematic.h"
#include "txrobot.h"


/** \addtogroup libKin
 *  @{
 */
  class IvKinTx90:public Kautham::InverseKinematic{
  public:
    IvKinTx90(Robot* const rob);
    ~IvKinTx90();
    INVKINECLASS    type() {return TX90;}
    string          name() {return "TX90";}
    bool            solve();
    bool            setParameters();
    RobLayout&      getRobLayout(vector<KthReal> &target);
    void            setTarget(vector<KthReal> &target, vector<KthReal> masterconf, bool maintainSameWrist);
    bool            solve(mt::Transform& tcp, const TXrobot::config& conf = TXrobot::config(), TXrobot::config* solution = NULL, Vect6* qNear = NULL );
    bool            solve(mt::Transform& tcp, const Vect6 &current);
    inline void     setConfiguration(const TXrobot::config& conf = TXrobot::config());
  private:
    IvKinTx90();
    TXrobot::TXRobot* _tx;
    TXrobot::TXerror  _error;
    TXrobot::config   _txConf;
    Vect6*            _result;
    vector<KthReal>   _eulPos;
    mt::Transform     _rHomeOffset; //!< This is an offset from Modeled Home to Real Home
  };

  /** @}   end of Doxygen module "Util */
#endif  //_IVKINTX90_H
