/***************************************************************************
 *   Copyright (C) 2014 by Nestor Garcia Hidalgo                           *
 *   nestor.garcia.hidalgo@upc.edu                                         *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/


#include <mt/transform.h>
#include <math.h>
#include <utility>

//modified DH alpha values
#define alpha0 0.0
#define alpha1 PI/2.0
#define alpha2 0.0
#define alpha3 0.0
#define alpha4 PI/2.0
#define alpha5 -PI/2.0

//modified DH a values
#define a0 0.0
#define a1 0.0
#define a2 425.0
#define a3 392.25
#define a4 0.0
#define a5 0.0

//modified DH d values
#define d1 89.159
#define d2 0.0
#define d3 0.0
#define d4 109.15
#define d5 -94.65
#define d6 82.3

//joint offset values
#define offset1 0.0
#define offset2 -PI/2.0
#define offset3 0.0
#define offset4 -PI/2.0
#define offset5 0.0
#define offset6 0.0

//joint lower limit values
#define low1 -2*PI
#define low2 -2*PI
#define low3 -2*PI
#define low4 -PI
#define low5 -PI
#define low6 -PI

//joint higher limit values
#define high1 2*PI
#define high2 2*PI
#define high3 2*PI
#define high4 PI/4.0
#define high5 PI
#define high6 PI

//robot configuration flags
//shoulder valid values
#define right true
#define left !right
//elbow valid values
#define up true
#define down !up
//wrist valid values
#define in true
#define out !in

//default absolute tolerance for interval inclusion
#define TOL 0.01

//other constant values
#define MAX_DIST 24 // MAX_DIST > distance(high,low)
#define INF 1.0/0.0


//error values for inv_kin
enum {UR5_NO_ERROR = 1,//succeed
      UR5_JOINT_1,     //1st joint failed
      UR5_JOINT_2,     //2nd joint failed
      UR5_JOINT_3,     //3rd joint failed
      UR5_JOINT_4,     //4th joint failed
      UR5_JOINT_5,     //5th joint failed
      UR5_JOINT_6};    //6th joint failed


using namespace std;
using namespace mt;


//class containing a possible solution for the UR5 inverse kinematics
class Solution {
public:
    double theta[6];//joint values
    bool valid;//true if solution is valid

    Solution();//class constructor
};

//class containing a set of possible solution for the UR5 inverse kinematics
class Solution_set {
public:
    int num_solutions;//number of valid solutions
    Solution *solution;//pointer to solutions

    Solution_set();//class constructor
};

//returns the modified DH transform for the specified modified DH parameters
Transform DH_transform(double alpha, double a, double theta, double d);

//solves the UR5 direct kinematics for the specified joint values
//and returns the TCP transform
Transform UR5_dir_kin(double *theta);

//solves the UR5 inverse kinematics for the specified configuration,
//puts the result in theta, and returns true if a solution was found
//configuration is definied as:
//shoulder=right/left, elbow=up/down and wrist=in/out
//joint values will be the nearest ones next to the reference
//if no reference is specified, offset values will be used as reference
int UR5_inv_kin(Transform transform, bool shoulder, bool elbow, bool wrist,
                 double *theta, double *theta_ref = NULL);

//solves the UR5 inverse kinematics for all the 8 possible configurations,
//puts the result in solution_set, and returns true if a solution was found
//joint values will be the nearest ones next to the reference
//if no reference is specified, offset values will be used as reference
bool UR5_inv_kin(Transform transform, Solution_set *solution_set,
                 double *theta_ref = NULL);

//solves the UR5 inverse kinematics for the nearest configuration next to the
//specified joint values, puts the result in theta and returns true if at
//least a solution was found
//joint values will be the nearest ones next to the reference
bool UR5_inv_kin(Transform transform, double *theta,
                 double *theta_ref);

//finds the control values for the specified joint values
//joint values must be physically achievable for control values are between
//0 and 1
void UR5_controls(double *control, double *theta);

//finds the joint values for the specified control values
//the control values must be between 0 and 1 for the joint values are
//physically achievable
void UR5_thetas(double *control, double *theta);

//solves the UR5 inverse kinematics for the two 1st joint possible values
//and calls the function that calculates the 5th joint value
//joint value will be the nearest one next to the reference
void find_theta1(Transform transform, Solution *solution,
                 double *theta_ref);

//solves the UR5 inverse kinematics for the two 2nd joint possible values
//and calls the functions that calculates the 3rd joint value
//joint value will be the nearest one next to the reference
void find_theta2(Transform transform, Solution *solution,
                 double *theta_ref);

//solves the UR5 inverse kinematics for the 3rd joint possible value
//and calls the function that calculates the 4th joint value
//joint value will be the nearest one next to the reference
void find_theta3(Transform transform, Solution *solution,
                 double *theta_ref);

//solves the UR5 inverse kinematics for the 4th joint possible value
//and decides if the solution is valid
//joint value will be the nearest one next to the reference
void find_theta4(Transform transform, Solution *solution,
                 double *theta_ref);

//solves the UR5 inverse kinematics for the two 5th joint possible values
//and calls the function that calculates the 6th joint value
//joint value will be the nearest one next to the reference
void find_theta5(Transform transform, Solution *solution,
                 double *theta_ref);

//solves the UR5 inverse kinematics for the 6th joint and calls the function
//that calculates the 2nd joint value
//joint value will be the nearest one next to the reference
void find_theta6(Transform transform, Solution *solution,
                 double *theta_ref);

//adjusts the specified i-th joint value to be the nearest one next to the
//reference, in the physically achievable interval
void adjust(double *theta, double theta_ref, int i);

//puts in theta the nearest solution next to the reference
void nearest(Solution_set solution_set, double *theta, double *theta_ref);

//returns the distance between two configurations
double distance(double *theta_a, double *theta_b);

//returns true if x is the interval [xmin, xmax] with a tol absolute tolerance
//in case x is in the interval, it will be adjusted to be in the interval
//without needing a tolerance anymore
bool in_interval(double *x, double xmin, double xmax, double tol = TOL);

//returns sign of x
double sign(double x);

//returns theta in the [-pi, pi] interval
double saturate(double theta);

//prints the specified transform
void print_transform(Transform transform);
