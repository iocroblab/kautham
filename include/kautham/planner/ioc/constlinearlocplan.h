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



#if !defined(_CONSTLINEARLOCALPLANNER_H)
#define _CONSTLINEARLOCALPLANNER_H

#include <kautham/problem/workspace.h>
#include <kautham/sampling/sampling.h>
#include <kautham/planner/ioc/localplanner.h>
#include <kautham/planner/ioc/linearlocplan.h>

namespace Kautham {
    /** \defgroup Planner  Planning module
    *  \brief contains planners to solve the path planning problems
    *
    *   \todo  Add detailed description of Planning module
    *
    *  @{
    *
    */

    namespace IOC {
        /**
        *   \brief This is a brief description of class constLinearLocalPlanner
        *
        *    This is  a long descrition of class constLinearLocal planner
        *
        */

        class ConstLinearLocalPlanner:public LinearLocalPlanner {
        public:
            /** descrition of constructor */
            ConstLinearLocalPlanner(SPACETYPE stype, Sample *init, Sample *goal, WorkSpace *ws, KthReal st );
            void setMethod(bool vandercorput = true);
            bool canConect();
            KthReal distance(Sample* from, Sample* to);
            inline void setStaringPoint(KthReal x, KthReal y, KthReal z){ ox=x; oy=y; oz=z;}
            inline KthReal getOx(){return ox;}/**< description of getOx()*/
            inline KthReal getOy(){return oy;}
            inline KthReal getOz(){return oz;}
            inline void setTypeConstraint(int t){type = t;}
            inline int getTypeConstraint(){return type;}
            KthReal computeorientation(KthReal px, KthReal py, KthReal pz, KthReal* X1, KthReal* X2, KthReal* X3, KthReal theta=1000);
            /**
            * Function getTheta retrives the theta angle of a sample
            *
            * \param c: the configuration, either with 6 or 7 values
            * \return The theta angle
            */
            KthReal getTheta(vector<KthReal> c);
            inline void setConstrained(bool f){constrained=f;}
            inline bool getConstrained(){return constrained;}
            bool ArmInverseKinematics(vector<KthReal> &carm, Sample *smp, bool maintainSameWrist );

            bool correctorientation(Sample* tmpSample, KthReal theta, Sample *smp);

            void setCameraTransform(mt::Transform t){_cameraTransform = t;}

        private:
            // ConstLinearLocalPlanner();
            bool vanderMethod;
            KthReal ox,oy,oz; //point towards where the palm is staring
            int type;
            bool constrained;
            LCPRNG* _gen;
            mt::Transform _cameraTransform;
        };
    }
    /** @}   end of Doxygen module "libPlanner" */
}

#endif  //_CONSTLINEARLOCALPLANNER_H
