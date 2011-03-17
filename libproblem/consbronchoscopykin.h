#if !defined(_ConsBronchoscopyKin_H)
#define _ConsBronchoscopyKin_H

#include "constrainedkinematic.h"
#include <hardlab/robot/txrobot.h>

using namespace Kautham;
using namespace robot;

namespace libProblem {
	class ConsBronchoscopyKin :	public ConstrainedKinematic{
	public:
		ConsBronchoscopyKin(Robot* const rob);
		~ConsBronchoscopyKin();
    bool            solve();
		
		bool		        setParameters();
		void		        setvalues(KthReal currentvalues, int i){ _currentvalues[i]=currentvalues;}
		KthReal		      getvalues(int i){return _currentvalues[i];}

    ///XXXXXXXXXXXXXXXXXXXXXX
    vector<KthReal> constrainedparameter2Pose(vector<KthReal> &values);
    vector<float>   constrainedinterpolate(vector<float> coords, vector<KthReal> &values);
    void            SolveConstrainedKinematics(vector<KthReal> &values);

	private:
    RobConf		      solve(vector<KthReal> &values);
		vector<KthReal>	_indexi;
		KthReal			    _currentvalues[3];
    mt::Transform   Tbase;
	};
}

#endif  //_ConsBronchoscopyKin_H