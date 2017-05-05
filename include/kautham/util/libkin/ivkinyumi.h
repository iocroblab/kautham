/*************************************************************************\
   Copyright 2017 Institute of Industrial and Control Engineering (IOC)
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

/* Author: Josep-Arnau Claret Robert */



#if !defined(_IVKINYUMI_H)
#define _IVKINYUMI_H

#include <kautham/util/libkin/inversekinematic.h>


/** \addtogroup IK
 *  @{
 */

class IvKinYumi:public Kautham::InverseKinematic{
public:
    IvKinYumi(Robot* const rob);
    ~IvKinYumi();
    INVKINECLASS type() {return YUMI;}
    string       name() {return "YUMI";}
    bool solve();
    bool setParameters();
    void setTarget(vector<KthReal> &target, vector<KthReal> masterconf, bool maintainSameWrist);
private:
    IvKinYumi();
    vector<KthReal> _eulPos;
    double          _redundantJoint;
    double          _result[6];
};

/** @}   end of Doxygen module */

#endif // _IVKINYUMI_H
