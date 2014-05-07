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


#if !defined(_iocPLANNER_H)
#define _iocPLANNER_H

#include <libproblem/workspace.h>
#include <libsampling/sampling.h>
#include <cmath>
#include <string>
#include "localplanner.h"
#include <mt/transform.h>
#include "kthquery.h"
#include "planner.h"

using namespace std;

namespace Kautham {
/** \addtogroup libPlanner
 *  @{
 */

 namespace IOC{

  class iocPlanner: public Planner{
    public:
    iocPlanner(SPACETYPE stype, Sample *init, Sample *goal, SampleSet *samples, Sampler *sampler, WorkSpace *ws);

    ~iocPlanner();

    //! This is a simple save method to store the samples set. It should be overload in the
    //! derived class in order to save the connectivity and the solutions if exists. The
    //! data is saved in a KPS (Kautham Planner Solution)file using xml format. For an
    //! example see /libplanner/output_file_model.kps
    virtual bool                  saveData(string path);

    //! This functions provides the user to save any interested data in a convenient way. 
    //! Data saved using this method can not be loaded by the loadData method. This function
    //! is called when the standard saveData method is finishing.
    virtual void                  saveData();

    //! This method is a simple load method to restore the samples set. It should be
    //! overload in the derived class in order to load the connectivity and the solutions
    //! if exists.  The
    //! data is saved in a KPS (Kautham Planner Solution)file using xml format. For an
    //! example see /libplanner/output_file_model.kps
    virtual bool                  loadData(string path);

    inline void                   setSampler(Sampler* smp){_sampler = smp;}
    inline Sampler*               getSampler(){return _sampler;}


    inline vector<KthQuery>&       getQueries(){return _queries;}
    bool                           addQuery(int init, int goal );
    int                            findQuery(int init, int goal, int from = 0);


    //!  This method must be used to add samples that corresponds with the 
    //! zero crossing point in case the planner will provide a path to be 
    //! teleoperated and it must have the possibility to crossing singular points.
    void                           addZeroCrossingToPath();


    bool solveAndInherit();
    inline LocalPlanner* getLocalPlanner(){return _locPlanner;};

	protected:
    iocPlanner();
    Sampler*                      _sampler;
    LocalPlanner* _locPlanner;


    vector<KthQuery>              _queries;
    // Stats
    unsigned int                  _collChecks;
    unsigned int                  _worldcollChecks;
    unsigned int                  _triedSamples;
    unsigned int                  _generatedEdges;
    KthReal                       _totalTime;
    KthReal                       _smoothTime;


	};
 }
  /** @}   end of Doxygen module "libPlanner */
}

#endif  //_PLANNER_H

