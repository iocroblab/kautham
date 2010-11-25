#if !defined(_CONSBRONCOSCOPYKIN_H)
#define _CONSBRONCOSCOPYKIN_H

#include "constrainedkinematic.h"
#include <hardlab/robot/txrobot.h>

using namespace Kautham;
using namespace robot;

namespace libProblem {
	class ConsBroncoscopyKin :	public ConstrainedKinematic{
	public:
		ConsBroncoscopyKin(Robot* const rob);
		~ConsBroncoscopyKin();
		RobConf		solve(vector<KthReal> &values);
		bool		setParameters();
		void		setvalues(KthReal currentvalues, int i){ _currentvalues[i]=currentvalues;}
		KthReal		getvalues(int i){return _currentvalues[i];}
	private:
		vector<KthReal>	_indexi;
		KthReal			_currentvalues[3];
	};
}

#endif  //_CONSBRONCOSCOPYKIN_H