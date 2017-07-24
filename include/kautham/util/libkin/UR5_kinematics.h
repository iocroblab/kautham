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

/* Author: Nestor Garcia Hidalgo */


#include <mt/transform.h>


//modified DH alpha values
#define alpha0 0.0
#define alpha1 PI/2.0
#define alpha2 0.0
#define alpha3 0.0
#define alpha4 -PI/2.0
#define alpha5 -PI/2.0

//modified DH a values
#define a0 0.0
#define a1 0.0
#define a2 0.4250   // 425.0
#define a3 0.39225  // 392.25
#define a4 0.0
#define a5 0.0

//modified DH d values
#define d1 0.089159 // 89.159
#define d2 0.0
#define d3 0.0
#define d4 0.10915  // 109.15
#define d5 0.09465  // 94.65
#define d6 0.0823   // 82.3

//joint offset values
#define offset1 PI
#define offset2 PI
#define offset3 0.0
#define offset4 0.0
#define offset5 PI
#define offset6 0.0

//joint lower limit values
#define low1 -2.0*PI
#define low2 -2.0*PI
#define low3 -2.0*PI
#define low4 -2.0*PI
#define low5 -2.0*PI
#define low6 -2.0*PI

//joint higher limit values
#define high1 2.0*PI
#define high2 2.0*PI
#define high3 2.0*PI
#define high4 2.0*PI
#define high5 2.0*PI
#define high6 2.0*PI

//default absolute tolerance for interval inclusion
#define TOL 0.01

//other constant values
#define MAX_DIST 30.8 // MAX_DIST > distance(high,low) = 4*sqrt(6)*PI
#define INF 1.0/0.0


/** \addtogroup IK
 *  @{
 */


//! error values for inv_kin
enum {UR5_NO_ERROR = 1,//succeed
      UR5_JOINT_1,     //1st joint failed
      UR5_JOINT_2,     //2nd joint failed
      UR5_JOINT_3,     //3rd joint failed
      UR5_JOINT_4,     //4th joint failed
      UR5_JOINT_5,     //5th joint failed
      UR5_JOINT_6};    //6th joint failed


/*!
 * \brief The Solution class contains a possible solution for the UR5 inverse kinematics
 */
class Solution {
public:
    /*!
     * \brief theta contains the joint values
     */
    double theta[6];

    /*!
     * \brief valid is true if solution is valid
     */
    bool valid;


    /*!
     * \brief Solution constructs the class
     */
    Solution();
};


/*!
 * \brief The Solution_set class contains a set of possible solution for the
 *UR5 inverse kinematics
 */
class Solution_set {
public:
    /*!
     * \brief num_solutions is the number of valid solutions
     */
    int num_solutions;


    /*!
     * \brief solution is a pointer to solutions
     */
    Solution *solution;

    /*!
     * \brief Solution_set constructs the class
     */
    Solution_set();
};


/*!
 * \brief DH_transform returns the modified DH transform for the specified modified DH parameters
 */
mt::Transform DH_transform(double alpha, double a, double theta, double d);

/*!
 * \brief UR5_dir_kin solves the UR5 direct kinematics for the specified joint values and
 *returns the TCP transform
 */
mt::Transform UR5_dir_kin(double *theta);

/*!
 * \brief UR5_dir_kin solves the UR5 direct kinematics for the specified joint values and
 *returns the TCP transform of the i-th frame
 */
mt::Transform UR5_dir_kin(double *theta, const unsigned int joint);

/*!
 * \brief UR5_inv_kin solves the UR5 inverse kinematics for the specified configuration,
 *and only if a solutyion was found, puts the result in theta and returns joint values
 *will be the nearest ones next to the reference if no reference is specified,
 *null values will be used as reference
 */
int UR5_inv_kin(mt::Transform transform, bool shoulder_positive,
                bool wrist_positive, bool elbow_positive, double *theta,
                double *theta_ref = NULL);

/*!
 * \brief UR5_inv_kin solves the UR5 inverse kinematics for all the 8 possible configurations,
 *puts the result in solution_set, and returns true if at leat a solution was found joint
 *values will be the nearest ones next to the reference if no reference is specified,
 *null values will be used as reference
 */
bool UR5_inv_kin(mt::Transform transform, Solution_set *solution_set,
                 double *theta_ref = NULL);

/*!
 * \brief UR5_inv_kin solves the UR5 inverse kinematics for the nearest configuration next
 *to the specified joint values, puts the result in theta and returns true if at least a
 *solution was found joint values will be the nearest ones next to the reference
 */
bool UR5_inv_kin(mt::Transform transform, double *theta,
                 double *theta_ref);

/*!
 * \brief UR5_controls finds the control values for the specified joint values joint values
 *must be physically achievable for control values are between 0 and 1
 */
void UR5_controls(double *control, double *theta);

/*!
 * \brief UR5_thetas finds the joint values for the specified control values the control
 *values must be between 0 and 1 for the joint values are physically achievable
 */
void UR5_thetas(double *control, double *theta);

/*!
 * \brief find_theta1 solves the UR5 inverse kinematics for the two 1st joint possible
 *values and calls the function that calculates the 5th joint value joint value will be
 *the nearest one next to the reference
 */
void find_theta1(mt::Transform transform, Solution *solution,
                 double *theta_ref);

/*!
 * \brief find_theta2 solves the UR5 inverse kinematics for the two 2nd joint possible
 *values and calls the functions that calculates the 3rd joint value joint value will be
 *the nearest one next to the reference
 */
void find_theta2(mt::Transform transform, Solution *solution,
                 double *theta_ref);

/*!
 * \brief find_theta3 solves the UR5 inverse kinematics for the 3rd joint possible
 *value and calls the function that calculates the 4th joint value joint value will be
 *the nearest one next to the reference
 */
void find_theta3(mt::Transform transform, Solution *solution,
                 double *theta_ref);

/*!
 * \brief find_theta4 solves the UR5 inverse kinematics for the 4th joint possible
 *value and decides if the solution is valid joint value will be
 *the nearest one next to the reference
 */
void find_theta4(mt::Transform transform, Solution *solution,
                 double *theta_ref);

/*!
 * \brief find_theta5 solves the UR5 inverse kinematics for the two 5th joint possible
 *values and calls the function that calculates the 6th joint value joint value will be
 *the nearest one next to the reference
 */
void find_theta5(mt::Transform transform, Solution *solution,
                 double *theta_ref);

/*!
 * \brief find_theta6 solves the UR5 inverse kinematics for the 6th joint and calls
 *the function that calculates the 2nd joint value joint value will be
 *the nearest one next to the reference
 */
void find_theta6(mt::Transform transform, Solution *solution,
                 double *theta_ref);

/*!
 * \brief adjust adjusts the specified joint value to be the nearest one next to the
 *reference, in the physically achievable interval
 */
void adjust(double *theta, double theta_ref);

/*!
 * \brief nearest puts in theta the nearest solution in solution_set next to the reference
 */
void nearest(Solution_set solution_set, double *theta, double *theta_ref);

/*!
 * \brief distance returns the distance between two configurations
 */
double distance(double *theta_a, double *theta_b);

/*!
 * \brief in_interval returns true if x is in the interval [xmin, xmax] with a tol
 *absolute tolerance in case x is in the interval, it will be adjusted to be in
 *the interval without needing a tolerance anymore
 */
bool in_interval(double *x, double xmin, double xmax, double tol = TOL);

/*!
 * \brief sign returns sign of x
 */
double sign(double x);

/*!
 * \brief saturate returns theta in the [-pi, pi] interval
 */
double saturate(double theta);

/*!
 * \brief print_transform prints the specified transform
 */
void print_transform(mt::Transform transform);

/** @}   end of Doxygen module */
