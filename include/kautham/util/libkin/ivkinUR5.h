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

/* Author: Nestor Garcia Hidalgo */


#if !defined(_IVKINUR5_H)
#define _IVKINUR5_H

#include "inversekinematic.h"


/** \addtogroup libKin
 *  @{
 */

enum UR5shoulder{shoulder_right=0, shoulder_left, shoulder_free};
enum UR5elbow{elbow_up=0, elbow_down, elbow_free};
enum UR5wrist{wrist_in=0, wrist_out , wrist_free};

struct UR5config {
    UR5config() : sh(shoulder_left), el(elbow_up), wr(wrist_in){}
    bool matchMask(UR5config mask){
        if((mask.sh == shoulder_free || mask.sh == sh) &&
                (mask.el == elbow_free || mask.el == el) &&
                (mask.wr == wrist_free || mask.wr == wr))
            return true;
        else
            return false;
    }

    void set(UR5shoulder s, UR5elbow e, UR5wrist w){
        sh = s; el = e; wr = w;
    }

    UR5shoulder sh;
    UR5elbow el;
    UR5wrist wr;
};

class IvKinUR5:public Kautham::InverseKinematic{
public:
    IvKinUR5(Robot* const rob);
    ~IvKinUR5();
    INVKINECLASS type() {return UR5;}
    string       name() {return "UR5";}
    bool solve();
    bool setParameters();
    void setTarget(vector<KthReal> &target, vector<KthReal> masterconf, bool maintainSameWrist);
private:
    IvKinUR5();
    vector<KthReal> _eulPos;
    UR5config          _UR5Conf;
    double          _result[6];
};

/** @}   end of Doxygen module "Util */
#endif  //_IVKINUR5_H

