//Based on code by John Burkardt distributed under the GNU LGPL license
//http://people.sc.fsu.edu/~jburkardt/cpp_src/halton/halton.html

/*************************************************************************\
   Copyright 2014-2024  Institute of Industrial and Control Engineering (IOC)
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

/* Author: Alexander Perez, Jan Rosell */

 
#include <kautham/sampling/haltonsampler.h>
#include <kautham/sampling/sample.h>
# include <ctime>
# include <cmath>
# include <cstring>
# include <iostream> 
# include <iomanip>

namespace Kautham{


  
//! This class is the sampler implementation based on the Halton 
//! This wraps the C functions implemented by John Burkardt 


using namespace std;

//
//  These variables are accessible to the user via calls to routines.
//
static int *halton_BASE = NULL;
static int *halton_LEAP = NULL;
static int  halton_DIM_NUM = -1;
static int *halton_SEED = NULL;
static int  halton_STEP = -1;

HaltonSampler::HaltonSampler(char dim)
{
    setDim(dim);
}

HaltonSampler::~HaltonSampler(void)
{

}

Sample* HaltonSampler::nextSample()
{
    _current= new Sample(dimension);
	halton_DIM_NUM = (int)dimension;
	double *c = new double[dimension];
	halton(c);
	vector<KthReal> cf(dimension);
    for(unsigned int i=0;i<dimension;i++) cf[i] = (KthReal)c[i];
	_current->setCoords(cf);
	/*
	cout<<"smp[]=";
	for(int i=0;i<dimension;i++)cout<<_current->getCoords()[i]<<", ";
	cout<<endl;
	*/
	return _current;
}




//What follows are C functions implemented by John Burkardt
//Code is distributed under the GNU LGPL license
//http://people.sc.fsu.edu/~jburkardt/cpp_src/halton/halton.html
//****************************************************************************80

double arc_cosine ( double c )

//****************************************************************************80
//
//  Purpose:
//
//    ARC_COSINE computes the arc cosine function, with argument truncation.
//
//  Discussion:
//
//    If you call your system ACOS routine with an input argument that is
//    outside the range [-1.0, 1.0 ], you may get an unpleasant surprise.
//    This routine truncates arguments outside the range.
//
//  Modified:
//
//    28 February 2003
//
//  Author:
//
//    John Burkardt
//
//  Parameters:
//
//    Input, double C, the argument, which is usually between -1 and 1.
//
//    Output, double ARC_COSINE, an angle whose cosine is C.
//
{
  double pi = 3.141592653589793;

  if ( c < -1.0 )
  {
    return -pi;
  }
  else if ( 1.0 < c )
  {
    return 0.0;
  }
  else
  {
    return acos ( c );
  }
}
//****************************************************************************80

double atan4 ( double y, double x )

//****************************************************************************80
//
//  Purpose:
//
//    ATAN4 computes the inverse tangent of the ratio Y / X.
//
//  Discussion:
//
//    ATAN4 returns an angle whose tangent is ( Y / X ), a job which
//    the built in functions ATAN and ATAN2 already do.  
//
//    However:
//
//    * ATAN4 always returns a positive angle, between 0 and 2 PI, 
//      while ATAN and ATAN2 return angles in the interval [-PI/2,+PI/2]
//      and [-PI,+PI] respectively;
//
//    * ATAN4 accounts for the signs of X and Y, (as does ATAN2).  The ATAN 
//     function by contrast always returns an angle in the first or fourth
//     quadrants.
//
//  Modified:
//
//    18 April 1999
//
//  Author:
//
//    John Burkardt
//
//  Parameters:
//
//    Input, double Y, X, two quantities which represent the tangent of
//    an angle.  If Y is not zero, then the tangent is (Y/X).
//
//    Output, double ATAN4, a positive angle whose tangent is (Y/X), and
//    which lies in the appropriate quadrant so that the signs of its
//    cosine and sine match those of X and Y.
//
{
  double abs_x;
  double abs_y;
  double pi = 3.141592653589793;
  double theta = 0.;
  double theta_0 = 0.;
//
//  Special cases:
//
  if ( x == 0.0 )
  {
    if ( 0.0 < y )
    {
      theta = pi / 2.0;
    }
    else if ( y < 0.0 )
    {
      theta = 3.0 * pi / 2.0;
    }
    else
    {
      theta = 0.0;
    }

  }
  else if ( y == 0.0 )
  {
    if ( 0.0 < x )
    {
      theta = 0.0;
    }
    else if ( x < 0.0 )
    {
      theta = pi;
    }

  }
//
//  We assume that ATAN2 is correct when both arguments are positive.
//
  else
  {
    abs_y = fabs ( y );
    abs_x = fabs ( x );

    theta_0 = atan2 ( abs_y, abs_x );

    if ( 0.0 < x && 0.0 < y )
    {
      theta = theta_0;
    }
    else if ( x < 0.0 && 0.0 < y )
    {
      theta = pi - theta_0;
    }
    else if ( x < 0.0 && y < 0.0 )
    {
      theta = pi + theta_0;
    }
    else if ( 0.0 < x && y < 0.0 )
    {
      theta = 2.0 * pi - theta_0;
    }
  }

  return theta;
}
//****************************************************************************80

char digit_to_ch ( int i )

//****************************************************************************80
//
//  Purpose:
//
//    DIGIT_TO_CH returns the base 10 digit character corresponding to a digit.
//
//  Example:
//
//     I     C
//   -----  ---
//     0    '0'
//     1    '1'
//   ...    ...
//     9    '9'  
//    10    '*'
//   -83    '*'
//
//  Modified:
//
//    16 June 2003
//
//  Author:
//
//    John Burkardt
//
//  Parameters:
//
//    Input, int I, the digit, which should be between 0 and 9.
//
//    Output, char DIGIT_TO_CH, the appropriate character '0' through '9' or '*'.
//
{
  char c;

  if ( 0 <= i && i <= 9 )
    c = '0' + i;
  else
    c = '*';
  

  return c;
}
//****************************************************************************80

int get_seed ( void )

//****************************************************************************80
//
//  Purpose:
//
//    GET_SEED returns a random seed for the random number generator.
//
//  Modified:
//
//    15 September 2003
//
//  Author:
//
//    John Burkardt
//
//  Parameters:
//
//    Output, int GET_SEED, a random seed value.
//
{
# define I4_MAX 2147483647
  time_t clock;
  int ihour;
  int imin;
  int isec;
  int seed;
  struct tm *lt;
  time_t tloc;
//
//  If the internal seed is 0, generate a value based on the time.
//
  clock = time ( &tloc );
  lt = localtime ( &clock );
//
//  Hours is 1, 2, ..., 12.
//
  ihour = lt->tm_hour;

  if ( 12 < ihour )
  {
    ihour = ihour - 12;
  }
//
//  Move Hours to 0, 1, ..., 11
//
  ihour = ihour - 1;

  imin = lt->tm_min;

  isec = lt->tm_sec;

  seed = isec + 60 * ( imin + 60 * ihour );
//
//  We want values in [1,43200], not [0,43199].
//
  seed = seed + 1;
//
//  Remap ISEED from [1,43200] to [1,IMAX].
//
  seed = ( int ) 
    ( ( ( double ) seed )
    * ( ( double ) I4_MAX ) / ( 60.0 * 60.0 * 12.0 ) );
//
//  Never use a seed of 0.
//
  if ( seed == 0 )
  {
    seed = 1;
  }

  return seed;
# undef I4_MAX
}
//****************************************************************************80

bool halham_dim_num_check ( int dim_num )

//****************************************************************************80
//
//  Purpose:
//
//    HALHAM_DIM_NUM_CHECK checks DIM_NUM for a Halton or Hammersley sequence.
//
//  Modified:
//
//    16 September 2004
//
//  Author:
//
//    John Burkardt
//
//  Parameters:
//
//    Input, int DIM_NUM, the spatial dimension.
//    DIM_NUM must be positive.
//
//    Output, bool HALHAM_DIM_NUM_CHECK, is true if DIM_NUM is legal.
//
{
  bool value;

  if ( dim_num < 1 ) 
  {
    cout << "\n";
    cout << "HALHAM_DIM_NUM_CHECK - Fatal error!\n";
    cout << "  DIM_NUM < 0.";
    cout << "  DIM_NUM = " << dim_num << "\n";
    value = false;
  }
  else
  {
    value = true;
  }

  return value;
}
//****************************************************************************80

bool halham_leap_check ( int dim_num, int leap[] )

//****************************************************************************80
//
//  Purpose:
//
//    HALHAM_LEAP_CHECK checks LEAP for a Halton or Hammersley sequence.
//
//  Modified:
//
//    16 September 2004
//
//  Author:
//
//    John Burkardt
//
//  Parameters:
//
//    Input, int DIM_NUM, the spatial dimension.
//
//    Input, int LEAP[DIM_NUM], the successive jumps in the sequence.
//    Each entry must be greater than 0.
//
//    Output, bool HALHAM_LEAP_CHECK, is true if LEAP is legal.
//
{
  int i;
  bool value;

  value = true;

  for ( i = 0; i < dim_num; i++ )
  {
    if ( leap[i] < 1 ) 
    {
      cout << "\n";
      cout << "HALHAM_LEAP_CHECK - Fatal error!\n";
      cout << "  Leap entries must be greater than 0.\n";
      cout << "  leap[" << i << "] = " << leap[i] << "\n";
      value = false;
      break;
    }
  }

  return value;
}
//****************************************************************************80

bool halham_n_check ( int n )

//****************************************************************************80
//
//  Purpose:
//
//    HALHAM_N_CHECK checks N for a Halton or Hammersley sequence.
//
//  Modified:
//
//    16 September 2004
//
//  Author:
//
//    John Burkardt
//
//  Parameters:
//
//    Input, int N, the number of elements of the subsequence.
//    N must be positive.
//
//    Output, bool HALHAM_N_CHECK, is true if N is legal.
//
{
  bool value;

  if ( n < 1 ) 
  {
    cout << "\n";
    cout << "HALHAM_N_CHECK - Fatal error!\n";
    cout << "  N < 0.";
    cout << "  N = " << n << "\n";
    value = false;
  }
  else
  {
    value = true;
  }

  return value;
}
//****************************************************************************80

bool halham_seed_check ( int dim_num, int seed[] )

//****************************************************************************80
//
//  Purpose:
//
//    HALHAM_SEED_CHECK checks SEED for a Halton or Hammersley sequence.
//
//  Modified:
//
//    16 September 2004
//
//  Author:
//
//    John Burkardt
//
//  Parameters:
//
//    Input, int DIM_NUM, the spatial dimension.
//
//    Input, int SEED[DIM_NUM], the sequence index
//    corresponding to STEP = 0.  Each entry must be 0 or greater.
//
//    Output, bool HALHAM_SEED_CHECK, is true if SEED is legal.
//
{
  int i;
  bool value;

  value = true;

  for ( i = 0; i < dim_num; i++ )
  {
    if ( seed[i] < 0 ) 
    {
      cout << "\n";
      cout << "HALHAM_SEED_CHECK - Fatal error!\n";
      cout << "  SEED entries must be nonnegative.\n";
      cout << "  seed[" << i << "] = " << seed[i] << "\n";
      value = false;
      break;
    }
  }

  return value;
}
//****************************************************************************80

bool halham_step_check ( int step )

//****************************************************************************80
//
//  Purpose:
//
//    HALHAM_STEP_CHECK checks STEP for a Halton or Hammersley sequence.
//
//  Modified:
//
//    16 September 2004
//
//  Author:
//
//    John Burkardt
//
//  Parameters:
//
//    Input, int STEP, the index of the subsequence element.
//    STEP must be 1 or greater.
//
//    Output, bool HALHAM_STEP_CHECK, is true if STEP is legal.
//
{
  bool value;

  if ( step < 0 ) 
  {
    cout << "\n";
    cout << "HALHAM_STEP_CHECK - Fatal error!\n";
    cout << "  STEP < 0.";
    cout << "  STEP = " << step << "\n";
    value = false;
  }
  else
  {
    value = true;
  }

  return value;
}
//****************************************************************************80

void halham_write ( int dim_num, int n, int step, int seed[], int leap[], 
  int base[], double r[], char *file_out_name )

//****************************************************************************80
//
//  Purpose:
//
//    HALHAM_WRITE writes a Halton or Hammersley dataset to a file.
//
//  Discussion:
//
//    The initial lines of the file are comments, which begin with a
//    "#" character.
//
//    Thereafter, each line of the file contains the DIM_NUM-dimensional
//    components of the next entry of the dataset.
//
//  Modified:
//
//    16 September 2004
//
//  Author:
//
//    John Burkardt
//
//  Parameters:
//
//    Input, int DIM_NUM, the spatial dimension.
//
//    Input, int N, the number of elements in the subsequence.
//
//    Input, int STEP, the index of the subsequence element.
//    0 <= STEP is required.
//
//    Input, int SEED[DIM_NUM], the sequence index for STEP = 0.
//
//    Input, int LEAP[DIM_NUM], the successive jumps in the sequence.
//
//    Input, int BASE[DIM_NUM], the bases.
//
//    Input, double R[DIM_NUM*N], the points.
//
//    Input, char *FILE_OUT_NAME, the name of the output file.
//
{
  FILE *fp;

  int i;
  int j;
  char *s;

  (void)step; //unused now
  (void)seed; //unused now
  (void)leap; //unused now
  (void)base; //unused now

 fp=fopen(file_out_name,"wt");
	if(fp==NULL){
		printf("Cannot open %s\n",file_out_name);
//		exit(0);
                return ;
	}

  //file_out.open ( file_out_name );

  /*if ( !file_out )
  {
    cout << "\n";
    cout << "HALHAM_WRITE - Fatal error!\n";
    cout << "  Could not open the output file.\n";
    exit ( 1 );
  }*/

  s = timestring ( );

  /*fprintf(fp,"%s",file_out_name);
  fprintf(fp,"created by routine HALHAM_WRITE.CC\n");
  fprintf(fp,"at %s \n",s);
  fprintf(fp,"\n");
  fprintf(fp,"DIM_NUM = %d",dim_num);
  fprintf(fp,"DIM_NUM = %d",dim_num);

  file_out << "#  " << file_out_name << "\n";
  file_out << "#  created by routine HALHAM_WRITE.CC" << "\n";
  file_out << "#  at " << s << "\n";
  file_out << "#\n";
  file_out << "#  DIM_NUM = " << setw(12) << dim_num << "\n";
  file_out << "#  N =    " << setw(12) << n    << "\n";
  file_out << "#  STEP = " << setw(12) << step << "\n";*/
  //for ( mlo = 1; mlo <= dim_num; mlo = mlo + 5 )
  //{
  //  mhi = i4_min ( mlo + 5 - 1, dim_num );
  //  if ( mlo == 1 )
  //  {
  //   // file_out << "#  SEED = ";
  //  }
  //  else
  //  {
  //    //file_out << "#         ";
  //  }
  //  for ( i = mlo; i <= mhi; i++ )
  //  {
  //    //file_out << setw(12) << seed[i-1];
  //  }
  //  //file_out << "\n";
  //}
  //for ( mlo = 1; mlo <= dim_num; mlo = mlo + 5 )
  //{
  //  mhi = i4_min ( mlo + 5 - 1, dim_num );
  //  if ( mlo == 1 )
  //  {
  //   // file_out << "#  LEAP = ";
  //  }
  //  else
  //  {
  //   // file_out << "#         ";
  //  }
  //  for ( i = mlo; i <= mhi; i++ )
  //  {
  //    file_out << setw(12) << leap[i-1];
  //  }
  // // file_out << "\n";
  //}
  //for ( mlo = 1; mlo <= dim_num; mlo = mlo + 5 )
  //{
  //  mhi = i4_min ( mlo + 5 - 1, dim_num );
  //  if ( mlo == 1 )
  //  {
  //    //file_out << "#  BASE = ";
  //  }
  //  else
  //  {
  //    //file_out << "#         ";
  //  }
  //  for ( i = mlo; i <= mhi; i++ )
  //  {
  //    //file_out << setw(12) << base[i-1];
  //  }
  //  file_out << "\n";
  //}
  //file_out << "#  EPSILON (unit roundoff) = " << r8_epsilon ( ) << "\n";
  //file_out << "#\n";

  for ( j = 0; j < n; j++ )
  {
    for ( i = 0; i < dim_num; i++ )
    {
     
	  fprintf(fp,"%f  ",r[i+j*dim_num]);
    }
    fprintf(fp,"\n");
  }

 fclose(fp);

  delete [] s;

  return;
}
//****************************************************************************80

void halton ( double r[] )

//****************************************************************************80
//
//  Purpose:
//
//    HALTON computes the next element in a leaped Halton subsequence.
//
//  Discussion:
//
//    The DIM_NUM-dimensional Halton sequence is really DIM_NUM separate
//    sequences, each generated by a particular base.
//
//    This routine selects elements of a "leaped" subsequence of the 
//    Halton sequence.  The subsequence elements are indexed by a
//    quantity called STEP, which starts at 0.  The STEP-th subsequence 
//    element is simply element 
//
//      SEED(1:DIM_NUM) + STEP * LEAP(1:DIM_NUM) 
//
//    of the original Halton sequence.
//
//
//    This routine "hides" a number of input arguments.  To specify these
//    arguments explicitly, use I4_TO_HALTON instead.
//
//    All the arguments have default values.  However, if you want to
//    examine or change them, you may call the appropriate routine
//    before calling HALTON.
//
//    The arguments that the user may set include:
//
//    * DIM_NUM, the spatial dimension, 
//      Default: DIM_NUM = 1;
//      Required: 1 <= DIM_NUM is required.
//
//    * STEP, the subsequence index.
//      Default: STEP = 0.
//      Required: 0 <= STEP.
//
//    * SEED(1:DIM_NUM), the Halton sequence element corresponding to STEP = 0.
//      Default SEED = (0, 0, ... 0).  
//      Required: 0 <= SEED(1:DIM_NUM).
//
//    * LEAP(1:DIM_NUM), the succesive jumps in the Halton sequence.
//      Default: LEAP = (1, 1, ..., 1). 
//      Required: 1 <= LEAP(1:DIM_NUM).
//
//    * BASE(1:DIM_NUM), the Halton bases.
//      Default: BASE = (2, 3, 5, 7, 11, ... ). 
//      Required: 1 < BASE(1:DIM_NUM).
//
//  Modified:
//
//    08 July 2004
//
//  Author:
//
//    John Burkardt
//
//  Reference:
//
//    J H Halton,
//    On the efficiency of certain quasi-random sequences of points
//    in evaluating multi-dimensional integrals,
//    Numerische Mathematik,
//    Volume 2, 1960, pages 84-90.
//
//    J H Halton and G B Smith,
//    Algorithm 247: Radical-Inverse Quasi-Random Point Sequence,
//    Communications of the ACM,
//    Volume 7, 1964, pages 701-702.
//
//    Ladislav Kocis and William Whiten,
//    Computational Investigations of Low-Discrepancy Sequences,
//    ACM Transactions on Mathematical Software,
//    Volume 23, Number 2, 1997, pages 266-294.
//
//  Parameters:
//
//    Output, double R[DIM_NUM], the next element of the leaped Halton
//    subsequence.
//
{
  int *base;
  int i;
  int dim_num;
  int *leap;
  int *seed;
  int step;

  if ( halton_DIM_NUM < 1 )
  { 
    halton_DIM_NUM = 1;
  }

  if ( halton_STEP < 0 )
  { 
    halton_STEP = 0;
  }

  if ( !halton_SEED )
  {
    halton_SEED = new int[halton_DIM_NUM];
    for ( i = 0; i < halton_DIM_NUM; i++ )
    {
      halton_SEED[i] = 0;
    }
  }

  if ( !halton_LEAP )
  {
    halton_LEAP = new int[halton_DIM_NUM];
    for ( i = 0; i < halton_DIM_NUM; i++ )
    {
      halton_LEAP[i] = 1;
    }
  }

  if ( !halton_BASE )
  {
    halton_BASE = new int[halton_DIM_NUM];
    for ( i = 0; i < halton_DIM_NUM; i++ )
    {
      halton_BASE[i] = prime ( i + 1 );
    }
  }

  dim_num = halton_DIM_NUM;
  step = halton_STEP;
  seed = halton_SEED;
  leap = halton_LEAP;
  base = halton_BASE;

  i4_to_halton ( dim_num, step, seed, leap, base, r );

  halton_STEP = halton_STEP + 1;

  return;
}
//****************************************************************************80

bool halton_base_check ( int dim_num, int base[] )

//****************************************************************************80
//
//  Purpose:
//
//    HALTON_BASE_CHECK checks BASE for a Halton sequence.
//
//  Modified:
//
//    16 July 2004
//
//  Author:
//
//    John Burkardt
//
//  Parameters:
//
//    Input, int DIM_NUM, the spatial dimension.
//
//    Input, int BASE[DIM_NUM], the bases.
//
//    Output, bool HALTON_BASE_CHECK, is true if BASE is legal.
//
{
  int i;
  bool value;

  value = true;

  for ( i = 0; i < dim_num; i++ )
  {
    if ( base[i] <= 1 ) 
    {
      cout << "\n";
      cout << "HALTON_BASE_CHECK - Fatal error!\n";
      cout << "  Bases must be greater than 1.\n";
      cout << "  base[" << i << "] = " << base[i] << "\n";
      value = false;
      break;
    }
  }

  return value;
}
//****************************************************************************80

int *halton_base_get ( void )

//****************************************************************************80
//
//  Purpose:
//
//    HALTON_BASE_GET gets the base vector for a leaped Halton subsequence.
//
//  Modified:
//
//    28 February 2003
//
//  Author:
//
//    John Burkardt
//
//  Parameters:
//
//    Output, int *HALTON_BASE_GET, a pointer to the Halton bases.
//
{
  return halton_BASE;
}
//****************************************************************************80

void halton_base_set ( int base[] )

//****************************************************************************80
//
//  Purpose:
//
//    HALTON_BASE_SET sets the base vector for a leaped Halton subsequence.
//
//  Modified:
//
//    28 February 2003
//
//  Author:
//
//    John Burkardt
//
//  Parameters:
//
//    Input, int BASE[HALTON_DIM_NUM], the Halton bases.
//    Each base must be greater than 1.
//
{
  int i;

  if ( !halton_base_check ( halton_DIM_NUM, base ) )
  {
    exit ( 1 );
  }

  for ( i = 0; i < halton_DIM_NUM; i++ )
  {
    halton_BASE[i] = base[i];
  }

  return;
}
//****************************************************************************80

int *halton_leap_get ( void )

//****************************************************************************80
//
//  Purpose:
//
//    HALTON_LEAP_GET gets the leap vector for a leaped Halton subsequence.
//
//  Modified:
//
//    06 July 2004
//
//  Author:
//
//    John Burkardt
//
//  Parameters:
//
//    Output, int *HALTON_LEAP_GET, a pointer to the successive jumps in
//    the Halton sequence.
//
{
  return halton_LEAP;
}
//****************************************************************************80

void halton_leap_set ( int leap[] )

//****************************************************************************80
//
//  Purpose:
//
//    HALTON_LEAP_SET sets the leap vector for a leaped Halton subsequence.
//
//  Modified:
//
//    16 July 2004
//
//  Author:
//
//    John Burkardt
//
//  Parameters:
//
//    Input, int LEAP[HALTON_DIM_NUM], the successive jumps in the Halton sequence.
//    Each entry must be greater than 0.
//
{
  int i;

  if ( !halham_leap_check ( halton_DIM_NUM, leap ) )
  {
    exit ( 1 );
  }

  for ( i = 0; i < halton_DIM_NUM; i++ )
  {
    halton_LEAP[i] = leap[i];
  }

  return;
}
//****************************************************************************80

int halton_dim_num_get ( void )

//****************************************************************************80
//
//  Purpose:
//
//    HALTON_DIM_NUM_GET gets the spatial dimension for a leaped Halton subsequence.
//
//  Modified:
//
//    28 February 2003
//
//  Author:
//
//    John Burkardt
//
//  Parameters:
//
//    Output, int HALTON_DIM_NUM_GET, the spatial dimension.
//
{
  return halton_DIM_NUM;
}
//****************************************************************************80

void halton_dim_num_set ( int dim_num )

//****************************************************************************80
//
//  Purpose:
//
//    HALTON_DIM_NUM_SET sets the spatial dimension for a leaped Halton subsequence.
//
//  Modified:
//
//    28 February 2003
//
//  Author:
//
//    John Burkardt
//
//  Parameters:
//
//    Input, int DIM_NUM, the spatial dimension.
//    DIM_NUM must be positive.
//
{
  int i;

  if ( !halham_dim_num_check ( dim_num ) ) 
  {
    exit ( 1 );
  }

  if ( halton_DIM_NUM != dim_num && 0 < halton_DIM_NUM )
  {
    delete [] halton_BASE;
    delete [] halton_LEAP;
    delete [] halton_SEED;
  }

  if ( halton_DIM_NUM != dim_num )
  {
    halton_DIM_NUM = dim_num;
    halton_SEED = new int[halton_DIM_NUM];
    for ( i = 0; i < halton_DIM_NUM; i++ )
    {
      halton_SEED[i] = 0;
    }
    halton_LEAP = new int[halton_DIM_NUM];
    for ( i = 0; i < halton_DIM_NUM; i++ )
    {
      halton_LEAP[i] = 1;
    }
    halton_BASE = new int[halton_DIM_NUM];
    for ( i = 0; i < halton_DIM_NUM; i++ )
    {
      halton_BASE[i] = prime ( i + 1 );
    }
    halton_STEP = 0;
  }

  return;
}
//****************************************************************************80

int *halton_seed_get ( void )

//****************************************************************************80
//
//  Purpose:
//
//    HALTON_SEED_GET gets the seed vector for a leaped Halton subsequence.
//
//  Modified:
//
//    06 July 2004
//
//  Author:
//
//    John Burkardt
//
//  Parameters:
//
//    Output, int *HALTON_SEED_GET, a pointer to the Halton sequence index
//    corresponding to STEP = 0.
//
{
  return halton_SEED;
}
//****************************************************************************80

void halton_seed_set ( int seed[] )

//****************************************************************************80
//
//  Purpose:
//
//    HALTON_SEED_SET sets the seed vector for a leaped Halton subsequence.
//
//  Modified:
//
//    06 July 2004
//
//  Author:
//
//    John Burkardt
//
//  Parameters:
//
//    Input, int SEED[HALTON_DIM_NUM], the Halton sequence index
//    corresponding to STEP = 0.  Each entry must be 0 or greater.
//
{
  int i;

  if ( !halham_seed_check ( halton_DIM_NUM, seed ) )
  {
    exit ( 1 );
  }

  for ( i = 0; i < halton_DIM_NUM; i++ )
  {
    halton_SEED[i] = seed[i];
  }

  return;
}
//****************************************************************************80

void halton_sequence ( int n, double r[] )

//****************************************************************************80
//
//  Purpose:
//
//    HALTON_SEQUENCE computes N elements in an DIM_NUM-dimensional Halton sequence.
//
//  Discussion:
//
//    The DIM_NUM-dimensional Halton sequence is really DIM_NUM separate
//    sequences, each generated by a particular base.
//
//    This routine selects elements of a "leaped" subsequence of the 
//    Halton sequence.  The subsequence elements are indexed by a
//    quantity called STEP, which starts at 0.  The STEP-th subsequence 
//    element is simply element 
//
//      SEED(1:DIM_NUM) + STEP * LEAP(1:DIM_NUM) 
//
//    of the original Halton sequence.
//
//
//    This routine "hides" a number of input arguments.  To specify these
//    arguments explicitly, use the routine I4_TO_HALTON_SEQUENCE instead.
//
//    All the arguments have default values.  However, if you want to
//    examine or change them, you may call the appropriate routine first.
//
//    The arguments that the user may set include:
//
//    * DIM_NUM, the spatial dimension, 
//      Default: DIM_NUM = 1;
//      Required: 1 <= DIM_NUM is required.
//
//    * STEP, the subsequence index.
//      Default: STEP = 0.
//      Required: 0 <= STEP.
//
//    * SEED(1:DIM_NUM), the Halton sequence element corresponding to STEP = 0.
//      Default SEED = (0, 0, ... 0).  
//      Required: 0 <= SEED(1:DIM_NUM).
//
//    * LEAP(1:DIM_NUM), the succesive jumps in the Halton sequence.
//      Default: LEAP = (1, 1, ..., 1). 
//      Required: 1 <= LEAP(1:DIM_NUM).
//
//    * BASE(1:DIM_NUM), the Halton bases.
//      Default: BASE = (2, 3, 5, 7, 11, ... ). 
//      Required: 1 < BASE(1:DIM_NUM).
//
//
//    The data to be computed has two dimensions.
//
//    The number of data items is DIM_NUM * N, where DIM_NUM is the spatial dimension
//    of each element of the sequence, and N is the number of elements of the sequence.
//
//    The data is stored in a one dimensional array R.  The first element of the
//    sequence is stored in the first DIM_NUM entries of R, followed by the DIM_NUM entries
//    of the second element, and so on.
//
//    In particular, the J-th element of the sequence is stored in entries
//    0+(J-1)*DIM_NUM through (DIM_NUM-1) + (J-1)*DIM_NUM.
//
//  Modified:
//
//    08 July 2004
//
//  Author:
//
//    John Burkardt
//
//  Reference:
//
//    J H Halton,
//    On the efficiency of certain quasi-random sequences of points
//    in evaluating multi-dimensional integrals,
//    Numerische Mathematik,
//    Volume 2, 1960, pages 84-90.
//
//    J H Halton and G B Smith,
//    Algorithm 247: Radical-Inverse Quasi-Random Point Sequence,
//    Communications of the ACM,
//    Volume 7, 1964, pages 701-702.
//
//    Ladislav Kocis and William Whiten,
//    Computational Investigations of Low-Discrepancy Sequences,
//    ACM Transactions on Mathematical Software,
//    Volume 23, Number 2, 1997, pages 266-294.
//
//  Parameters:
//
//    Input, int N, the number of elements desired.
//
//    Output, double R[DIM_NUM*N], the next N elements of the Halton sequence.
//
{
  int *base;
  int i;
  int *leap;
  int dim_num;
  int *seed;
  int step;

  if ( halton_DIM_NUM < 1 )
  { 
    halton_DIM_NUM = 1;
  }

  if ( halton_STEP < 0 )
  { 
    halton_STEP = 0;
  }

  if ( !halton_SEED )
  {
    halton_SEED = new int[halton_DIM_NUM];
    for ( i = 0; i < halton_DIM_NUM; i++ )
    {
      halton_SEED[i] = 0;
    }
  }

  if ( !halton_LEAP )
  {
    halton_LEAP = new int[halton_DIM_NUM];
    for ( i = 0; i < halton_DIM_NUM; i++ )
    {
      halton_LEAP[i] = 1;
    }
  }

  if ( !halton_BASE )
  {
    halton_BASE = new int[halton_DIM_NUM];
    for ( i = 0; i < halton_DIM_NUM; i++ )
    {
      halton_BASE[i] = prime ( i + 1 );
    }
  }

  dim_num = halton_DIM_NUM;
  step = halton_STEP;
  seed = halton_SEED;
  leap = halton_LEAP;
  base = halton_BASE;

  i4_to_halton_sequence ( dim_num, n, step, seed, leap, base, r );

  halton_STEP = halton_STEP + n;

  return;
}
//****************************************************************************80

int halton_step_get ( void )

//****************************************************************************80
//
//  Purpose:
//
//    HALTON_STEP_GET gets the step for the leaped Halton subsequence.
//
//  Modified:
//
//    06 July 2004
//
//  Author:
//
//    John Burkardt
//
//  Parameters:
//
//    Output, int HALTON_STEP_GET, the index of the subsequence element.
//
{
  return halton_STEP;
}
//****************************************************************************80

void halton_step_set ( int step )

//****************************************************************************80
//
//  Purpose:
//
//    HALTON_STEP_SET sets the step for a leaped Halton subsequence.
//
//  Modified:
//
//    06 July 2004
//
//  Author:
//
//    John Burkardt
//
//  Parameters:
//
//    Input, int STEP, the index of the subsequence element.
//    STEP must be 1 or greater.
//
{

  if ( !halham_step_check ( step ) ) 
  {
    exit ( 1 );
  }

  halton_STEP = step;

  return;
}
//****************************************************************************80

int i4_log_10 ( int i )

//****************************************************************************80
//
//  Purpose:
//
//    I4_LOG_10 returns the whole part of the logarithm base 10 of an I4.
//
//  Discussion:
//
//    It should be the case that 10^I4_LOG_10(I) <= |I| < 10^(I4_LOG_10(I)+1).
//    (except for I = 0).
//
//    The number of decimal digits in I is I4_LOG_10(I) + 1.
//
//  Examples:
//
//        I    I4_LOG_10(I)
//
//        0     0
//        1     0
//        2     0
//
//        9     0
//       10     1
//       11     1
//
//       99     1
//      100     2
//      101     2
//
//      999     2
//     1000     3
//     1001     3
//
//     9999     3
//    10000     4
//    10001     4
//
//  Modified:
//
//    17 June 2003
//
//  Author:
//
//    John Burkardt
//
//  Parameters:
//
//    Input, int I, the integer.
//
//    Output, int I4_LOG_10, the whole part of the logarithm of abs ( I ).
//
{
  int ten_pow;
  int value;

  i = abs ( i );

  ten_pow = 10;
  value = 0;

  while ( ten_pow <= i )
  {
    ten_pow = ten_pow * 10;
    value = value + 1;
  }

  return value;
}
//****************************************************************************80

int i4_min ( int i1, int i2 )

//****************************************************************************80
//
//  Purpose:
//
//    I4_MIN returns the smaller of two I4's.
//
//  Modified:
//
//    13 October 1998
//
//  Author:
//
//    John Burkardt
//
//  Parameters:
//
//    Input, int I1, I2, two integers to be compared.
//
//    Output, int I4_MIN, the smaller of I1 and I2.
//
//
{
  if ( i1 < i2 )
  {
    return i1;
  }
  else
  {
    return i2;
  }

}
//****************************************************************************80

void i4_to_halton ( int dim_num, int step, int seed[], int leap[], int base[], 
  double r[] )

//****************************************************************************80
//
//  Purpose:
//
//    I4_TO_HALTON computes one element of a leaped Halton subsequence.
//
//  Modified:
//
//    16 July 2004
//
//  Author:
//
//    John Burkardt
//
//  Reference:
//
//    J H Halton,
//    On the efficiency of certain quasi-random sequences of points
//    in evaluating multi-dimensional integrals,
//    Numerische Mathematik,
//    Volume 2, 1960, pages 84-90.
//
//    J H Halton and G B Smith,
//    Algorithm 247: Radical-Inverse Quasi-Random Point Sequence,
//    Communications of the ACM,
//    Volume 7, 1964, pages 701-702.
//
//    Ladislav Kocis and William Whiten,
//    Computational Investigations of Low-Discrepancy Sequences,
//    ACM Transactions on Mathematical Software,
//    Volume 23, Number 2, 1997, pages 266-294.
//
//  Parameters:
//
//    Input, int DIM_NUM, the spatial dimension.
//    1 <= DIM_NUM is required.
//
//    Input, int STEP, the index of the subsequence element.
//    0 <= STEP is required.
//
//    Input, int SEED[DIM_NUM], the Halton sequence index corresponding 
//    to STEP = 0.
//    0 <= SEED(1:DIM_NUM) is required.
//
//    Input, int LEAP[DIM_NUM], the successive jumps in the Halton sequence.
//    1 <= LEAP(1:DIM_NUM) is required.
//
//    Input, int BASE[DIM_NUM], the Halton bases.
//    1 < BASE(1:DIM_NUM) is required.
//
//    Output, double R[DIM_NUM], the STEP-th element of the leaped 
//    Halton subsequence.
//
{
  double base_inv;
  int digit;
  int i;
  int seed2;
//
//  Check the input.
//
  if ( !halham_dim_num_check ( dim_num ) )
  {
    exit ( 1 );
  }

  if ( !halham_step_check ( step ) )
  {
    exit ( 1 );
  }

  if ( !halham_seed_check ( dim_num, seed ) )
  {
    exit ( 1 );
  }

  if ( !halham_leap_check ( dim_num, leap ) )
  {
    exit ( 1 );
  }

  if ( !halton_base_check ( dim_num, base ) )
  {
    exit ( 1 );
  }
//
//  Calculate the data.
//
  for ( i = 0; i < dim_num; i++ )
  {
    seed2 = seed[i] + step * leap[i];

    r[i] = 0.0;

    base_inv = 1.0 / ( ( double ) base[i] );

    while ( seed2 != 0 )
    {
      digit = seed2 % base[i];
      r[i] = r[i] + ( ( double ) digit ) * base_inv;
      base_inv = base_inv / ( ( double ) base[i] );
      seed2 = seed2 / base[i];
    }
  }

  return;
}
//****************************************************************************80

void i4_to_halton_sequence ( int dim_num, int n, int step, int seed[], 
  int leap[], int base[], double r[] )

//****************************************************************************80
//
//  Purpose:
//
//    I4_TO_HALTON_SEQUENCE computes N elements of a leaped Halton subsequence.
//
//  Discussion:
//
//    The DIM_NUM-dimensional Halton sequence is really DIM_NUM separate
//    sequences, each generated by a particular base.
//
//    This routine selects elements of a "leaped" subsequence of the
//    Halton sequence.  The subsequence elements are indexed by a
//    quantity called STEP, which starts at 0.  The STEP-th subsequence
//    element is simply element
//
//      SEED(1:DIM_NUM) + STEP * LEAP(1:DIM_NUM)
//
//    of the original Halton sequence.
//
//
//    The data to be computed has two dimensions.
//
//    The number of data items is DIM_NUM * N, where DIM_NUM is the spatial dimension
//    of each element of the sequence, and N is the number of elements of the sequence.
//
//    The data is stored in a one dimensional array R.  The first element of the
//    sequence is stored in the first DIM_NUM entries of R, followed by the DIM_NUM entries
//    of the second element, and so on.
//
//    In particular, the J-th element of the sequence is stored in entries
//    0+(J-1)*DIM_NUM through (DIM_NUM-1) + (J-1)*DIM_NUM.
//
//  Modified:
//
//    16 July 2004
//
//  Author:
//
//    John Burkardt
//
//  Reference:
//
//    J H Halton,
//    On the efficiency of certain quasi-random sequences of points
//    in evaluating multi-dimensional integrals,
//    Numerische Mathematik,
//    Volume 2, 1960, pages 84-90.
//
//    J H Halton and G B Smith,
//    Algorithm 247: Radical-Inverse Quasi-Random Point Sequence,
//    Communications of the ACM,
//    Volume 7, 1964, pages 701-702.
//
//    Ladislav Kocis and William Whiten,
//    Computational Investigations of Low-Discrepancy Sequences,
//    ACM Transactions on Mathematical Software,
//    Volume 23, Number 2, 1997, pages 266-294.
//
//  Parameters:
//
//    Input, int DIM_NUM, the spatial dimension.
//
//    Input, int N, the number of elements of the sequence.
//
//    Input, int STEP, the index of the subsequence element.
//    0 <= STEP is required
//
//    Input, int SEED[DIM_NUM], the Halton sequence index corresponding
//    to STEP = 0. 
//
//    Input, int LEAP[DIM_NUM], the succesive jumps in the Halton sequence.
//
//    Input, int BASE[DIM_NUM], the Halton bases.
//
//    Output, double R[DIM_NUM*N], the next N elements of the
//    leaped Halton subsequence, beginning with element STEP.
//
{
  double base_inv;
  int digit;
  int i;
  int j;
  int *seed2;
//
//  Check the input.
//
  if ( !halham_dim_num_check ( dim_num ) )
  {
    exit ( 1 );
  }

  if ( !halham_n_check ( n ) )
  {
    exit ( 1 );
  }

  if ( !halham_step_check ( step ) )
  {
    exit ( 1 );
  }

  if ( !halham_seed_check ( dim_num, seed ) )
  {
    exit ( 1 );
  }

  if ( !halham_leap_check ( dim_num, leap ) )
  {
    exit ( 1 );
  }

  if ( !halton_base_check ( dim_num, base ) )
  {
    exit ( 1 );
  }
//
//  Calculate the data.
//
  seed2 = new int[n];

  for ( i = 0; i < dim_num; i++ )
  {
    for ( j = 0; j < n; j++ )
    {
      seed2[j] = seed[i] + ( step + j ) * leap[i];
    }

    for ( j = 0; j < n; j++ )
    {
      r[i+j*dim_num] = 0.0;
    }

    for ( j = 0; j < n; j++ )
    {
      base_inv = 1.0 / ( ( double ) base[i] );

      while ( seed2[j] != 0 )
      {
        digit = seed2[j] % base[i];
        r[i+j*dim_num] = r[i+j*dim_num] + ( ( double ) digit ) * base_inv;
        base_inv = base_inv / ( ( double ) base[i] );
        seed2[j] = seed2[j] / base[i];
      }
    }
  }

  delete [] seed2;

  return;
}
//****************************************************************************80

char *i4_to_s ( int i )

//****************************************************************************80
//
//  Purpose:
//
//    I4_TO_S converts an integer to a string.
//
//  Examples:
//
//    INTVAL  S
//
//         1  1
//        -1  -1
//         0  0
//      1952  1952
//    123456  123456
//   1234567  1234567
//
//  Modified:
//
//    13 March 2004
//
//  Author:
//
//    John Burkardt
//
//  Parameters:
//
//    Input, int I, an integer to be converted.
//
//    Output, char *I4_TO_S, the representation of the integer.
//
{
  int digit;
  int j;
  int length;
  int ten_power;
  char *s;

  length = i4_log_10 ( i );

  ten_power = ( int ) pow ( ( double ) 10, ( double ) length );

  if ( i < 0 )
  {
    length = length + 1;
  }
//
//  Add one position for the trailing null.
//
  length = length + 1;

  s = new char[length];

  if ( i == 0 )
  {
    s[0] = '0';
    s[1] = '\0';
    return s;
  }
//
//  Now take care of the sign.
//
  j = 0;
  if ( i < 0 )
  {
    s[j] = '-';
    j = j + 1;
    i = abs ( i );
  }
//
//  Find the leading digit of I, strip it off, and stick it into the string.
//
  while ( 0 < ten_power )
  {
    digit = i / ten_power;
    s[j] = digit_to_ch ( digit );
    j = j + 1;
    i = i - digit * ten_power;
    ten_power = ten_power / 10;
  }
//
//  Tack on the trailing NULL.
//
  s[j] = '\0';
  j = j + 1;

  return s;
}
//****************************************************************************80

void i4vec_transpose_print ( int n, int a[], char *title )

//****************************************************************************80
//
//  Purpose:
//
//    I4VEC_TRANSPOSE_PRINT prints an I4VEC "transposed".
//
//  Example:
//
//    A = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 }
//    TITLE = "My vector:  "
//
//    My vector:      1    2    3    4    5
//                    6    7    8    9   10
//                   11
//
//  Modified:
//
//    03 July 2004
//
//  Author:
//
//    John Burkardt
//
//  Parameters:
//
//    Input, int N, the number of components of the vector.
//
//    Input, int A[N], the vector to be printed.
//
//    Input, char *TITLE, a title to be printed first.
//    TITLE may be blank or NULL.
//
{
  int i;
  int ihi;
  int ilo;
  int title_len;

  if ( 0 < s_len_trim ( title ) )
  {
    title_len = strlen ( title );

    for ( ilo = 1; ilo <= n; ilo = ilo + 5 )
    {
      ihi = i4_min ( ilo + 5 - 1, n );
      if ( ilo == 1 )
      {
        cout << title;
      }
      else
      {
        for ( i = 1; i <= title_len; i++ )
        {
          cout << " ";
        }
      }
      for ( i = ilo; i <= ihi; i++ )
      {
        cout << setw(12) << a[i-1];
      }
      cout << "\n";
    }
  }
  else
  {
    for ( ilo = 1; ilo <= n; ilo = ilo + 5 )
    {
      ihi = i4_min ( ilo + 5 - 1, n );
      for ( i = ilo; i <= ihi; i++ )
      {
        cout << setw(12) << a[i-1];
      }
      cout << "\n";
    }
  }

  return;
}
//****************************************************************************80

int prime ( int n )

//****************************************************************************80
//
//  Purpose:
//
//    PRIME returns any of the first PRIME_MAX prime numbers.
//
//  Discussion:
//
//    PRIME_MAX is 1600, and the largest prime stored is 13499.
//
//    Thanks to Bart Vandewoestyne for pointing out a typo, 18 February 2005.
//
//  Modified:
//
//    18 February 2005
//
//  Author:
//
//    John Burkardt
//
//  Reference:
//
//    Milton Abramowitz and Irene Stegun,
//    Handbook of Mathematical Functions,
//    US Department of Commerce, 1964, pages 870-873.
//
//    Daniel Zwillinger,
//    CRC Standard Mathematical Tables and Formulae,
//    30th Edition,
//    CRC Press, 1996, pages 95-98.
//
//  Parameters:
//
//    Input, int N, the index of the desired prime number.
//    In general, is should be true that 0 <= N <= PRIME_MAX.
//    N = -1 returns PRIME_MAX, the index of the largest prime available.
//    N = 0 is legal, returning PRIME = 1.
//
//    Output, int PRIME, the N-th prime.  If N is out of range, PRIME
//    is returned as -1.
//
{
# define PRIME_MAX 1600

  int npvec[PRIME_MAX] = {
        2,    3,    5,    7,   11,   13,   17,   19,   23,   29,
       31,   37,   41,   43,   47,   53,   59,   61,   67,   71,
       73,   79,   83,   89,   97,  101,  103,  107,  109,  113,
      127,  131,  137,  139,  149,  151,  157,  163,  167,  173,
      179,  181,  191,  193,  197,  199,  211,  223,  227,  229,
      233,  239,  241,  251,  257,  263,  269,  271,  277,  281,
      283,  293,  307,  311,  313,  317,  331,  337,  347,  349,
      353,  359,  367,  373,  379,  383,  389,  397,  401,  409,
      419,  421,  431,  433,  439,  443,  449,  457,  461,  463,
      467,  479,  487,  491,  499,  503,  509,  521,  523,  541,
      547,  557,  563,  569,  571,  577,  587,  593,  599,  601,
      607,  613,  617,  619,  631,  641,  643,  647,  653,  659,
      661,  673,  677,  683,  691,  701,  709,  719,  727,  733,
      739,  743,  751,  757,  761,  769,  773,  787,  797,  809,
      811,  821,  823,  827,  829,  839,  853,  857,  859,  863,
      877,  881,  883,  887,  907,  911,  919,  929,  937,  941,
      947,  953,  967,  971,  977,  983,  991,  997, 1009, 1013,
     1019, 1021, 1031, 1033, 1039, 1049, 1051, 1061, 1063, 1069,
     1087, 1091, 1093, 1097, 1103, 1109, 1117, 1123, 1129, 1151,
     1153, 1163, 1171, 1181, 1187, 1193, 1201, 1213, 1217, 1223,
     1229, 1231, 1237, 1249, 1259, 1277, 1279, 1283, 1289, 1291, 
     1297, 1301, 1303, 1307, 1319, 1321, 1327, 1361, 1367, 1373, 
     1381, 1399, 1409, 1423, 1427, 1429, 1433, 1439, 1447, 1451, 
     1453, 1459, 1471, 1481, 1483, 1487, 1489, 1493, 1499, 1511, 
     1523, 1531, 1543, 1549, 1553, 1559, 1567, 1571, 1579, 1583, 
     1597, 1601, 1607, 1609, 1613, 1619, 1621, 1627, 1637, 1657, 
     1663, 1667, 1669, 1693, 1697, 1699, 1709, 1721, 1723, 1733, 
     1741, 1747, 1753, 1759, 1777, 1783, 1787, 1789, 1801, 1811, 
     1823, 1831, 1847, 1861, 1867, 1871, 1873, 1877, 1879, 1889,
     1901, 1907, 1913, 1931, 1933, 1949, 1951, 1973, 1979, 1987,
     1993, 1997, 1999, 2003, 2011, 2017, 2027, 2029, 2039, 2053,
     2063, 2069, 2081, 2083, 2087, 2089, 2099, 2111, 2113, 2129,
     2131, 2137, 2141, 2143, 2153, 2161, 2179, 2203, 2207, 2213,
     2221, 2237, 2239, 2243, 2251, 2267, 2269, 2273, 2281, 2287,
     2293, 2297, 2309, 2311, 2333, 2339, 2341, 2347, 2351, 2357, 
     2371, 2377, 2381, 2383, 2389, 2393, 2399, 2411, 2417, 2423, 
     2437, 2441, 2447, 2459, 2467, 2473, 2477, 2503, 2521, 2531, 
     2539, 2543, 2549, 2551, 2557, 2579, 2591, 2593, 2609, 2617, 
     2621, 2633, 2647, 2657, 2659, 2663, 2671, 2677, 2683, 2687, 
     2689, 2693, 2699, 2707, 2711, 2713, 2719, 2729, 2731, 2741,
     2749, 2753, 2767, 2777, 2789, 2791, 2797, 2801, 2803, 2819, 
     2833, 2837, 2843, 2851, 2857, 2861, 2879, 2887, 2897, 2903, 
     2909, 2917, 2927, 2939, 2953, 2957, 2963, 2969, 2971, 2999, 
     3001, 3011, 3019, 3023, 3037, 3041, 3049, 3061, 3067, 3079, 
     3083, 3089, 3109, 3119, 3121, 3137, 3163, 3167, 3169, 3181, 
     3187, 3191, 3203, 3209, 3217, 3221, 3229, 3251, 3253, 3257, 
     3259, 3271, 3299, 3301, 3307, 3313, 3319, 3323, 3329, 3331, 
     3343, 3347, 3359, 3361, 3371, 3373, 3389, 3391, 3407, 3413, 
     3433, 3449, 3457, 3461, 3463, 3467, 3469, 3491, 3499, 3511, 
     3517, 3527, 3529, 3533, 3539, 3541, 3547, 3557, 3559, 3571,
     3581, 3583, 3593, 3607, 3613, 3617, 3623, 3631, 3637, 3643,
     3659, 3671, 3673, 3677, 3691, 3697, 3701, 3709, 3719, 3727,
     3733, 3739, 3761, 3767, 3769, 3779, 3793, 3797, 3803, 3821,
     3823, 3833, 3847, 3851, 3853, 3863, 3877, 3881, 3889, 3907,
     3911, 3917, 3919, 3923, 3929, 3931, 3943, 3947, 3967, 3989,
     4001, 4003, 4007, 4013, 4019, 4021, 4027, 4049, 4051, 4057,
     4073, 4079, 4091, 4093, 4099, 4111, 4127, 4129, 4133, 4139,
     4153, 4157, 4159, 4177, 4201, 4211, 4217, 4219, 4229, 4231,
     4241, 4243, 4253, 4259, 4261, 4271, 4273, 4283, 4289, 4297,
     4327, 4337, 4339, 4349, 4357, 4363, 4373, 4391, 4397, 4409,
     4421, 4423, 4441, 4447, 4451, 4457, 4463, 4481, 4483, 4493, 
     4507, 4513, 4517, 4519, 4523, 4547, 4549, 4561, 4567, 4583, 
     4591, 4597, 4603, 4621, 4637, 4639, 4643, 4649, 4651, 4657, 
     4663, 4673, 4679, 4691, 4703, 4721, 4723, 4729, 4733, 4751, 
     4759, 4783, 4787, 4789, 4793, 4799, 4801, 4813, 4817, 4831, 
     4861, 4871, 4877, 4889, 4903, 4909, 4919, 4931, 4933, 4937, 
     4943, 4951, 4957, 4967, 4969, 4973, 4987, 4993, 4999, 5003, 
     5009, 5011, 5021, 5023, 5039, 5051, 5059, 5077, 5081, 5087, 
     5099, 5101, 5107, 5113, 5119, 5147, 5153, 5167, 5171, 5179, 
     5189, 5197, 5209, 5227, 5231, 5233, 5237, 5261, 5273, 5279,
     5281, 5297, 5303, 5309, 5323, 5333, 5347, 5351, 5381, 5387,
     5393, 5399, 5407, 5413, 5417, 5419, 5431, 5437, 5441, 5443,
     5449, 5471, 5477, 5479, 5483, 5501, 5503, 5507, 5519, 5521,
     5527, 5531, 5557, 5563, 5569, 5573, 5581, 5591, 5623, 5639,
     5641, 5647, 5651, 5653, 5657, 5659, 5669, 5683, 5689, 5693,
     5701, 5711, 5717, 5737, 5741, 5743, 5749, 5779, 5783, 5791,
     5801, 5807, 5813, 5821, 5827, 5839, 5843, 5849, 5851, 5857,
     5861, 5867, 5869, 5879, 5881, 5897, 5903, 5923, 5927, 5939,
     5953, 5981, 5987, 6007, 6011, 6029, 6037, 6043, 6047, 6053,
     6067, 6073, 6079, 6089, 6091, 6101, 6113, 6121, 6131, 6133,
     6143, 6151, 6163, 6173, 6197, 6199, 6203, 6211, 6217, 6221,
     6229, 6247, 6257, 6263, 6269, 6271, 6277, 6287, 6299, 6301,
     6311, 6317, 6323, 6329, 6337, 6343, 6353, 6359, 6361, 6367, 
     6373, 6379, 6389, 6397, 6421, 6427, 6449, 6451, 6469, 6473, 
     6481, 6491, 6521, 6529, 6547, 6551, 6553, 6563, 6569, 6571, 
     6577, 6581, 6599, 6607, 6619, 6637, 6653, 6659, 6661, 6673, 
     6679, 6689, 6691, 6701, 6703, 6709, 6719, 6733, 6737, 6761, 
     6763, 6779, 6781, 6791, 6793, 6803, 6823, 6827, 6829, 6833, 
     6841, 6857, 6863, 6869, 6871, 6883, 6899, 6907, 6911, 6917, 
     6947, 6949, 6959, 6961, 6967, 6971, 6977, 6983, 6991, 6997,
     7001, 7013, 7019, 7027, 7039, 7043, 7057, 7069, 7079, 7103, 
     7109, 7121, 7127, 7129, 7151, 7159, 7177, 7187, 7193, 7207, 
     7211, 7213, 7219, 7229, 7237, 7243, 7247, 7253, 7283, 7297, 
     7307, 7309, 7321, 7331, 7333, 7349, 7351, 7369, 7393, 7411, 
     7417, 7433, 7451, 7457, 7459, 7477, 7481, 7487, 7489, 7499, 
     7507, 7517, 7523, 7529, 7537, 7541, 7547, 7549, 7559, 7561, 
     7573, 7577, 7583, 7589, 7591, 7603, 7607, 7621, 7639, 7643, 
     7649, 7669, 7673, 7681, 7687, 7691, 7699, 7703, 7717, 7723, 
     7727, 7741, 7753, 7757, 7759, 7789, 7793, 7817, 7823, 7829, 
     7841, 7853, 7867, 7873, 7877, 7879, 7883, 7901, 7907, 7919,
     7927, 7933, 7937, 7949, 7951, 7963, 7993, 8009, 8011, 8017,
     8039, 8053, 8059, 8069, 8081, 8087, 8089, 8093, 8101, 8111,
     8117, 8123, 8147, 8161, 8167, 8171, 8179, 8191, 8209, 8219,
     8221, 8231, 8233, 8237, 8243, 8263, 8269, 8273, 8287, 8291,
     8293, 8297, 8311, 8317, 8329, 8353, 8363, 8369, 8377, 8387,
     8389, 8419, 8423, 8429, 8431, 8443, 8447, 8461, 8467, 8501,
     8513, 8521, 8527, 8537, 8539, 8543, 8563, 8573, 8581, 8597,
     8599, 8609, 8623, 8627, 8629, 8641, 8647, 8663, 8669, 8677,
     8681, 8689, 8693, 8699, 8707, 8713, 8719, 8731, 8737, 8741, 
     8747, 8753, 8761, 8779, 8783, 8803, 8807, 8819, 8821, 8831,
     8837, 8839, 8849, 8861, 8863, 8867, 8887, 8893, 8923, 8929,
     8933, 8941, 8951, 8963, 8969, 8971, 8999, 9001, 9007, 9011,
     9013, 9029, 9041, 9043, 9049, 9059, 9067, 9091, 9103, 9109,
     9127, 9133, 9137, 9151, 9157, 9161, 9173, 9181, 9187, 9199,
     9203, 9209, 9221, 9227, 9239, 9241, 9257, 9277, 9281, 9283,
     9293, 9311, 9319, 9323, 9337, 9341, 9343, 9349, 9371, 9377,
     9391, 9397, 9403, 9413, 9419, 9421, 9431, 9433, 9437, 9439,
     9461, 9463, 9467, 9473, 9479, 9491, 9497, 9511, 9521, 9533,
     9539, 9547, 9551, 9587, 9601, 9613, 9619, 9623, 9629, 9631,
     9643, 9649, 9661, 9677, 9679, 9689, 9697, 9719, 9721, 9733,
     9739, 9743, 9749, 9767, 9769, 9781, 9787, 9791, 9803, 9811,
     9817, 9829, 9833, 9839, 9851, 9857, 9859, 9871, 9883, 9887,
     9901, 9907, 9923, 9929, 9931, 9941, 9949, 9967, 9973,10007,
    10009,10037,10039,10061,10067,10069,10079,10091,10093,10099,
    10103,10111,10133,10139,10141,10151,10159,10163,10169,10177,
    10181,10193,10211,10223,10243,10247,10253,10259,10267,10271,
    10273,10289,10301,10303,10313,10321,10331,10333,10337,10343,
    10357,10369,10391,10399,10427,10429,10433,10453,10457,10459,
    10463,10477,10487,10499,10501,10513,10529,10531,10559,10567,
    10589,10597,10601,10607,10613,10627,10631,10639,10651,10657,
    10663,10667,10687,10691,10709,10711,10723,10729,10733,10739,
    10753,10771,10781,10789,10799,10831,10837,10847,10853,10859,
    10861,10867,10883,10889,10891,10903,10909,10937,10939,10949,
    10957,10973,10979,10987,10993,11003,11027,11047,11057,11059,
    11069,11071,11083,11087,11093,11113,11117,11119,11131,11149,
    11159,11161,11171,11173,11177,11197,11213,11239,11243,11251,
    11257,11261,11273,11279,11287,11299,11311,11317,11321,11329,
    11351,11353,11369,11383,11393,11399,11411,11423,11437,11443,
    11447,11467,11471,11483,11489,11491,11497,11503,11519,11527,
    11549,11551,11579,11587,11593,11597,11617,11621,11633,11657,
    11677,11681,11689,11699,11701,11717,11719,11731,11743,11777,
    11779,11783,11789,11801,11807,11813,11821,11827,11831,11833,
    11839,11863,11867,11887,11897,11903,11909,11923,11927,11933,
    11939,11941,11953,11959,11969,11971,11981,11987,12007,12011,
    12037,12041,12043,12049,12071,12073,12097,12101,12107,12109,
    12113,12119,12143,12149,12157,12161,12163,12197,12203,12211,
    12227,12239,12241,12251,12253,12263,12269,12277,12281,12289,
    12301,12323,12329,12343,12347,12373,12377,12379,12391,12401,
    12409,12413,12421,12433,12437,12451,12457,12473,12479,12487,
    12491,12497,12503,12511,12517,12527,12539,12541,12547,12553,
    12569,12577,12583,12589,12601,12611,12613,12619,12637,12641, 
    12647,12653,12659,12671,12689,12697,12703,12713,12721,12739, 
    12743,12757,12763,12781,12791,12799,12809,12821,12823,12829, 
    12841,12853,12889,12893,12899,12907,12911,12917,12919,12923, 
    12941,12953,12959,12967,12973,12979,12983,13001,13003,13007, 
    13009,13033,13037,13043,13049,13063,13093,13099,13103,13109, 
    13121,13127,13147,13151,13159,13163,13171,13177,13183,13187, 
    13217,13219,13229,13241,13249,13259,13267,13291,13297,13309, 
    13313,13327,13331,13337,13339,13367,13381,13397,13399,13411, 
    13417,13421,13441,13451,13457,13463,13469,13477,13487,13499 };

  if ( n == -1 )
  {
    return PRIME_MAX;
  }
  else if ( n == 0 )
  {
    return 1;
  }
  else if ( n <= PRIME_MAX )
  {
    return npvec[n-1];
  }
  else
  {
    cout << "\n";
    cout << "PRIME - Fatal error!\n";
    cout << "  Unexpected input value of n = " << n << "\n";
    exit ( 1 );
  }

  return 0;
# undef PRIME_MAX
}
//****************************************************************************80

double r8_epsilon ( void )

//****************************************************************************80
//
//  Purpose:
//
//    R8_EPSILON returns the R8 roundoff unit.
//
//  Discussion:
//
//    The roundoff unit is a number R which is a power of 2 with the property
//    that, to the precision of the computer's arithmetic,
//      1 < 1 + R
//    but 
//      1 = ( 1 + R / 2 )
//
//  Modified:
//
//    01 July 2004
//
//  Author:
//
//    John Burkardt
//
//  Parameters:
//
//    Output, double R8_EPSILON, the round-off unit.
//
{
  double r;

  r = 1.0;

  while ( 1.0 < ( double ) ( 1.0 + r )  )
  {
    r = r / 2.0;
  }

  return ( 2.0 * r );
}
//****************************************************************************80

double r8vec_dot_product ( int n, double *r1, double *r2 )

//****************************************************************************80
//
//  Purpose:
//
//    R8VEC_DOT_PRODUCT returns the dot product of two R8VEC's.
//
//  Modified:
//
//    01 March 2003
//
//  Author:
//
//    John Burkardt
//
//  Parameters:
//
//    Input, int N, the number of entries in the arrays.
//
//    Input, double *R1, R2, pointers to the first entries of the arrays.
//
//    Output, double R8VEC_DOT_PRODUCT, the dot product of the two arrays.
//
{
  int i;
  double dot;

  dot = 0.0;

  for ( i = 0; i < n; i++ ) 
  {
    dot = dot + (*r1) * (*r2);
    r1 = r1 + 1;
    r2 = r2 + 1;
  }

  return dot;
  
}
//****************************************************************************80

double r8vec_norm_l2 ( int n, double a[] )

//****************************************************************************80
//
//  Purpose:
//
//    R8VEC_NORM_L2 returns the L2 norm of an R8VEC.
//
//  Definition:
//
//    The vector L2 norm is defined as:
//
//      sqrt ( sum ( 1 <= I <= N ) A(I)**2 ).
//
//  Modified:
//
//    01 March 2003
//
//  Author:
//
//    John Burkardt
//
//  Parameters:
//
//    Input, int N, the number of entries in A.
//
//    Input, double A[N], the vector whose L2 norm is desired.
//
//    Output, double R8VEC_NORM_L2, the L2 norm of A.
//
{
  int i;
  double v;

  v = 0.0;

  for ( i = 0; i < n; i++ )
  {
    v = v + a[i] * a[i];
  }
  v = sqrt ( v );

  return v;
}
//****************************************************************************80

int s_len_trim ( char *s )

//****************************************************************************80
//
//  Purpose:
//
//    S_LEN_TRIM returns the length of a string to the last nonblank.
//
//  Modified:
//
//    26 April 2003
//
//  Author:
//
//    John Burkardt
//
//  Parameters:
//
//    Input, char *S, a pointer to a string.
//
//    Output, int S_LEN_TRIM, the length of the string to the last nonblank.
//    If S_LEN_TRIM is 0, then the string is entirely blank.
//
{
  int n;
  char* t;

  n = strlen ( s );
  t = s + strlen ( s ) - 1;

  while ( 0 < n ) 
  {
    if ( *t != ' ' )
    {
      return n;
    }
    t--;
    n--;
  }

  return n;
}
//****************************************************************************80

void timestamp ( void )

//****************************************************************************80
//
//  Purpose:
//
//    TIMESTAMP prints the current YMDHMS date as a time stamp.
//
//  Example:
//
//    May 31 2001 09:45:54 AM
//
//  Modified:
//
//    02 October 2003
//
//  Author:
//
//    John Burkardt
//
//  Parameters:
//
//    None
//
{
# define TIME_SIZE 40

  static char time_buffer[TIME_SIZE];

  cout << time_buffer << "\n";

  return;
# undef TIME_SIZE
}
//****************************************************************************80

char *timestring ( void )

//****************************************************************************80
//
//  Purpose:
//
//    TIMESTRING returns the current YMDHMS date as a string.
//
//  Example:
//
//    May 31 2001 09:45:54 AM
//
//  Modified:
//
//    24 September 2003
//
//  Author:
//
//    John Burkardt
//
//  Parameters:
//
//    Output, char *TIMESTRING, a string containing the current YMDHMS date.
//
{
# define TIME_SIZE 40

  char *s = new char[TIME_SIZE];

  return s;
# undef TIME_SIZE
}
//****************************************************************************80

void u1_to_sphere_unit_2d ( double u[], double x[2] )

//****************************************************************************80
//
//  Purpose:
//
//    U1_TO_SPHERE_UNIT_2D maps a point in the unit interval onto the circle in 2D.
//
//  Modified:
//
//    06 July 2004
//
//  Author:
//
//    John Burkardt
//
//  Parameters:
//
//    Input, double U[1], a point in the unit interval.
//
//    Output, double X[2], the corresponding point on the unit circle.
//
{
  double angle;
  double pi = 3.141592653589793;

  angle = 2.0 * pi * u[0];

  x[0] = cos ( angle );
  x[1] = sin ( angle );

  return;
}
//****************************************************************************80

void u2_to_ball_unit_2d ( double u[2], double x[2] )

//****************************************************************************80
//
//  Purpose:
//
//    U2_TO_BALL_UNIT_2D maps points from the unit box to the unit ball in 2D.
//
//  Modified:
//
//    06 July 2004
//
//  Author:
//
//    John Burkardt
//
//  Parameters:
//
//    Input, double U[2], a point in the unit box.
//
//    Output, double X[2], the corresponding point in the unit ball.
//
{
  double pi = 3.141592653589793;
  double r;
  double theta;

  r = sqrt ( u[0] );
  theta = 2.0 * pi * u[1];

  x[0] = r * cos ( theta );
  x[1] = r * sin ( theta );

  return;
}
//****************************************************************************80

void u2_to_sphere_unit_3d ( double u[2], double x[3] )

//****************************************************************************80
//
//  Purpose:
//
//    U2_TO_SPHERE_UNIT_3D maps a point in the unit box to the unit sphere in 3D.
//
//  Modified:
//
//    06 July 2004
//
//  Author:
//
//    John Burkardt
//
//  Parameters:
//
//    Input, double U[2], a point in the unit box.
//
//    Output, double X[3], the corresponding point on the unit sphere.
//
{
  double phi;
  double pi = 3.141592653589793;
  double theta;
  double vdot;
//
//  Pick a uniformly random VDOT, which must be between -1 and 1.
//  This represents the dot product of the random vector with the Z unit vector.
//
//  Note: this works because the surface area of the sphere between
//  Z and Z + dZ is independent of Z.  So choosing Z uniformly chooses
//  a patch of area uniformly.
//
  vdot = 2.0 * u[0] - 1.0;

  phi = arc_cosine ( vdot );
//
//  Pick a uniformly random rotation between 0 and 2 Pi around the
//  axis of the Z vector.
//
  theta = 2.0 * pi * u[1];

  x[0] = cos ( theta ) * sin ( phi );
  x[1] = sin ( theta ) * sin ( phi );
  x[2] = cos ( phi );

  return;
}
//****************************************************************************80

void u3_to_ball_unit_3d ( double u[3], double x[3] )

//****************************************************************************80
//
//  Purpose:
//
//    U3_TO_BALL_UNIT_3D maps points from the unit box to the unit ball in 3D.
//
//  Modified:
//
//    06 July 2004
//
//  Author:
//
//    John Burkardt
//
//  Parameters:
//
//    Input, double U[3], a point in the unit box.
//
//    Output, double X[3], the corresponding point in the unit ball.
//
{
  double phi;
  double pi = 3.141592653589793;
  double r;
  double theta;
  double vdot;
//
//  Pick a uniformly random VDOT, which must be between -1 and 1.
//  This represents the dot product of the random vector with the Z unit vector.
//
//  Note: this works because the surface area of the sphere between
//  Z and Z + dZ is independent of Z.  So choosing Z uniformly chooses
//  a patch of area uniformly.
//
  vdot = 2.0 * u[0] - 1.0;

  phi = arc_cosine ( vdot );
//
//  Pick a uniformly random rotation between 0 and 2 Pi around the
//  axis of the Z vector.
//
  theta = 2.0 * pi * u[1];
//
//  Pick a random radius R.
//
  r = pow ( ( double ) u[2], ( double ) ( 1.0 / 3.0 ) );

  x[0] = r * cos ( theta ) * sin ( phi );
  x[1] = r * sin ( theta ) * sin ( phi );
  x[2] = r * cos ( phi );

  return;
}


}



