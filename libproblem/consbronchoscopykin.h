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
	inline void  	setMinBending(KthReal min){_minbending=min;};
	inline KthReal 	getMaxBending(){return _maxbending;};
	inline KthReal 	getMinBending(){return _minbending;};
	inline void  	setMaxAlpha(KthReal max){_maxalpha=max;};
	inline void  	setMinAlpha(KthReal min){_minalpha=min;};
	inline KthReal 	getMaxAlpha(){return _maxalpha;};
	inline KthReal 	getMinAlpha(){return _minalpha;};

	private:
		RobConf		solve(vector<KthReal> &values);
		KthReal		_currentvalues[3];
		mt::Transform   Tbase;
		KthReal		_maxbending;
		KthReal		_minbending;
		KthReal		_maxalpha;
		KthReal		_minalpha;
		KthReal		_rangealpha;
		KthReal		_rangebending;
	};
}

#endif  //_ConsBronchoscopyKin_H