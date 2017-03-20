#ifndef __LCPRNG_H__
#define __LCPRNG_H__

#include <time.h>
#include <math.h>
//#include <iostream.h>

//define to be 1/(max unsigned int value)
#define _CI_2b ((1.0)/(double)(0xffffffff));

class LCPRNG;

struct lcprng_struct
{
	unsigned int _a;
	unsigned int _c;
	unsigned int _m;
	unsigned int _x0;
};

class LCPRNG
{
private:
	// Linear Congruential Variables
	unsigned int _seed_;
	unsigned int _next_;
	const unsigned int _multiplier;
	const unsigned int _increment;
	const unsigned int _modulus;

	//Conversion Variables
	double _CI_M;

public:
 /* Default Constructor */
	LCPRNG(unsigned int _a=16807,		//default _a=16807
		 unsigned int _c=0,				//default _c=0, multiplicative
		 unsigned int _m=0x7fffffff,	//default _m=2147483647
		 unsigned int _x0 = (unsigned int)time(NULL)): //seed with time
	_multiplier(_a % (_m - 1)),			//init _multiplier in range
	_increment (_c % (_m - 1)),			//init _increment in range
	_modulus   (_m)						//init _modulus
	{
		//Seed the generator.
		rand_init (_x0 % (_m - 1));		

		//X * _CI = [0,1] if X = [0,M-1]
		if (_modulus > 1)
			_CI_M = 1.0 / ((double)(_modulus - 1));
		else
			_CI_M = _CI_2b;
	}

 /* LCPRNG Constructor */
	LCPRNG(lcprng_struct* lcprng):
	_multiplier(lcprng->_a),
	_increment(lcprng->_c),
	_modulus(lcprng->_m)
	{
		//Seed the generator
		rand_init (lcprng->_x0 % (_modulus-1));

		//X * _CI = [0,1] if X = [0,M-1]
		if (_modulus > 1)
			_CI_M = 1.0 / ((double)(_modulus - 1));
		else
			_CI_M = _CI_2b;
	}
			
 /*===========================================================*\
< *	               LINEAR CONGRUENTIAL FUNCTIONS              * >
 \*===========================================================*/
		
	 /*-------------------------------*\
	| * LCPRNG Reseed/Reset Functions * |
	 \*-------------------------------*/
	void rand_init (unsigned int _x0 = (unsigned int)time(NULL))
	{
		_seed_ = _x0;		//set _seed_ to _x0
		_next_ = _x0;		//sed _next_ to _x0
	}

	void rand_reset ()
	{
		_next_ = _seed_;
	}

	 /*-------------------------------*\
	| *  LCPRNG Generator Functions   * |
	 \*-------------------------------*/
	unsigned int i_rand()
	{
		if (_modulus > 0)
			_next_ = (_multiplier*_next_+_increment) % _modulus;
		else
			_next_ = (_multiplier*_next_+_increment);

		return _next_;
	}

	double		 d_rand()
	{
		return i_rand() * _CI_M;
	}

	 /*-------------------------------*\
	| *  LCPRNG Data Read Functions   * |
	 \*-------------------------------*/
	unsigned int get_seed ()
	{
		return _seed_;
	}

    unsigned int get_mult()
	{
		return _multiplier;
	}

    unsigned int get_mod()
	{
		return _modulus;
	}

    unsigned int get_inc()
	{
		return _increment;
	}

	double get_CI_M()
	{
		return _CI_M;
	}
};

#endif

