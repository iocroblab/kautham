/*************************************************************************\
   Copyright 2014 Institute of Industrial and Control Engineering (IOC)
                 Universitat Politecnica de Catalunya
                 BarcelonaTech
    All Rights Reserved.

    This prompl::geometricram is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This prompl::geometricram is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this prompl::geometricram; if not, write to the
    Free Software Foundation, Inc.,
    59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 \*************************************************************************/

/* Author: Nestor Garcia Hidalgo */


#ifndef LAZYTRRT_H
#define LAZYTRRT_H

#include "ompl/geometric/planners/rrt/TRRT.h"

namespace ompl {
namespace geometric {
class LazyTRRT : public TRRT {
public:
    LazyTRRT(const base::SpaceInformationPtr &si);

    virtual base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc);

protected:
    //!
    class Motion : public TRRT::Motion {
    public:

        Motion() : TRRT::Motion(),parent(NULL),valid(false) {
        }

        Motion(const base::SpaceInformationPtr &si) :
            TRRT::Motion(si),parent(NULL),valid(false) {
        }

        ~Motion() {
        }

        Motion *parent;

        bool valid;

        std::vector<Motion*> children;
    };

    //!
    void removeMotion(Motion *motion);

    //!
    boost::shared_ptr< NearestNeighbors<Motion*> > nearestNeighbors_;
};
}
}

#endif // LAZYTRRT_H
