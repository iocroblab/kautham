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

//! This class is the sampler implementation based on the Halton 
//! This wraps the C functions implemented by John Burkardt

#if !defined(_HALTONSAMPLER_H)
#define _HALTONSAMPLER_H

#include <kautham/sampling/sampler.h>
#include <vector>
#include <string>

using namespace std;

namespace Kautham{


/** \addtogroup Sampling
 *  @{
 */

class HaltonSampler: public Sampler 
{
public:
    HaltonSampler(char dim);
    ~HaltonSampler(void);
    Sample* nextSample();
};


//What follows are C functions implemented by John Burkardt
double arc_cosine ( double c );
double atan4 ( double y, double x );
char digit_to_ch ( int i );
int get_seed ( void );
bool halham_leap_check ( int dim_num, int leap[] );
bool halham_n_check ( int n );
bool halham_dim_num_check ( int dim_num );
bool halham_seed_check ( int dim_num, int seed[] );
bool halham_step_check ( int step );
void halham_write ( int dim_num, int n, int step, int seed[], int leap[], int base[], 
double r[], char *file_out_name );
void halton ( double r[] );
bool halton_base_check ( int dim_num, int base[] );
int *halton_base_get ( void );
void halton_base_set ( int base[] );
int *halton_leap_get ( void );
void halton_leap_set ( int leap[] );
int halton_dim_num_get ( void );
void halton_dim_num_set ( int dim_num );
int *halton_seed_get ( void );
void halton_seed_set ( int seed[] );
void halton_sequence ( int n, double r[] );
int halton_step_get ( void );
void halton_step_set ( int step );
int i4_log_10 ( int i );
int i4_min ( int i1, int i2 );
void i4_to_halton ( int dim_num, int step, int seed[], int leap[], int base[], 
double r[] );
void i4_to_halton_sequence ( int dim_num, int n, int step, int seed[], int leap[],
int base[], double r[] );
char *i4_to_s ( int i );
void i4vec_transpose_print ( int n, int a[], char *title );
int prime ( int n );
double r8_epsilon ( void );
double r8vec_dot_product ( int n, double *r1, double *r2 );
double r8vec_norm_l2 ( int n, double a[] );
int s_len_trim ( char *s );
void timestamp ( void );
char *timestring ( void );
void u1_to_sphere_unit_2d ( double u[1], double x[2] );
void u2_to_ball_unit_2d ( double u[2], double x[2] );
void u2_to_sphere_unit_3d ( double u[2], double x[3] );
void u3_to_ball_unit_3d ( double u[3], double x[3] );




/** @}   end of Doxygen module "Sampling" */

}

#endif  //_HALTONSAMPLER_H

