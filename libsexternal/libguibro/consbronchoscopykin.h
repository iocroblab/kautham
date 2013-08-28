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
		void setAngleLimits(KthReal mb, KthReal Mb, KthReal ma, KthReal Ma);

    ///XXXXXXXXXXXXXXXXXXXXXX
	inline void  	setMaxBending(KthReal max){_maxbending_RAD=max;};
	inline void  	setMinBending(KthReal min){_minbending_RAD=min;};
	inline KthReal 	getMaxBending(){return _maxbending_RAD;};
	inline KthReal 	getMinBending(){return _minbending_RAD;};
	inline void  	setMaxAlpha(KthReal max){_maxalpha_RAD=max;};
	inline void  	setMinAlpha(KthReal min){_minalpha_RAD=min;};
	inline KthReal 	getMaxAlpha(){return _maxalpha_RAD;};
	inline KthReal 	getMinAlpha(){return _minalpha_RAD;};
	bool iJ(int k, int row, int column, KthReal alpha, KthReal xi, KthReal *coef);
	bool ApplyInverseJ(int k, KthReal alpha, KthReal xi, KthReal vx, KthReal vy, 
		KthReal vz, KthReal *Dalpha, KthReal *Dxi, KthReal *Dz);

	private:
		RobConf		solve(vector<KthReal> &values);
		KthReal		_currentvalues[3];
		mt::Transform   Tbase;
		KthReal		_initialmaxbending_RAD;
		KthReal		_initialminbending_RAD;
		KthReal		_initialmaxalpha_RAD;
		KthReal		_initialminalpha_RAD;
		KthReal		_maxbending_RAD;
		KthReal		_minbending_RAD;
		KthReal		_maxalpha_RAD;
		KthReal		_minalpha_RAD;
		KthReal		_rangealpha_RAD;
		KthReal		_rangebending_RAD;
	};
}

#endif  //_ConsBronchoscopyKin_H