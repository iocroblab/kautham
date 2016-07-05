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


#ifndef PLANNERDATAIO_H
#define PLANNERDATAIO_H

#ifdef KAUTHAM_USE_OMPL


#include <sstream>
#include <ompl/control/PlannerData.h>


bool savePlannerData(ompl::base::PlannerData *pdata, const std::string &path,
                     const ompl::base::OptimizationObjective *opt = nullptr);

bool savePlannerData(ompl::base::PlannerData *pdata, std::ostream &stream,
                     const ompl::base::OptimizationObjective *opt = nullptr);

ompl::base::PlannerData *loadPlannerData(const ompl::base::SpaceInformationPtr &si,
                                         const std::string &path);

ompl::base::PlannerData *loadPlannerData(const ompl::base::SpaceInformationPtr &si,
                                         std::istream &stream);


#endif //KAUTHAM_USE:OMPL

#endif // PLANNERDATAIO_H
