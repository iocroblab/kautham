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
		
		bool		    setParameters();
		void		    setvalues(KthReal currentvalues, int i){ _currentvalues[i]=currentvalues;}
		KthReal			getvalues(int i){return _currentvalues[i];}
		void		    registerValues();

    ///XXXXXXXXXXXXXXXXXXXXXX
	inline void  	setMaxBending(KthReal max){_maxbending=max;};
	inline KthReal 	getMaxBending(){return _maxbending;};
	inline void  	setMaxAlpha(KthReal max){_maxalpha=max;};
	inline KthReal 	getMaxAlpha(){return _maxalpha;};

	private:
		RobConf		solve(vector<KthReal> &values);
		KthReal		_currentvalues[3];
		mt::Transform   Tbase;
		KthReal		_maxbending;
		KthReal		_maxalpha;
	};
}

#endif  //_ConsBronchoscopyKin_H