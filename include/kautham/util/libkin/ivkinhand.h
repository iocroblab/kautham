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

/* Author: Josep-Arnau Claret Robert and Nestor Garcia Hidalgo */
 

#if !defined(_IVKINHAND_H)
#define _IVKINHAND_H

#include "inversekinematic.h"

#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/matrix.hpp>


/** \addtogroup libKin
 *  @{
 */
#if defined(KAUTHAM_USE_GSL) && defined(NDEBUG)
  struct ikSahandResults {
	boost::numeric::ublas::vector<double> t_wrist;
	boost::numeric::ublas::vector<double> configuration;

	boost::numeric::ublas::matrix<double> t_wrist_list;
	boost::numeric::ublas::matrix<double> configuration_list;

	bool ik_solved;

	boost::numeric::ublas::matrix<double> it_data;
	// Each row corresponds to a lambda iteration:
	//	1) # iterations
	//	2) {0,1,2} = {hand not stopped (maximum iterations reached), fingers at position, hand stopped (slope near zero)}
	//	3) {0,1,2} = {last configuration in corresponding it_configs matrix has no collision, it has collisions, not calculated}
	//	4) Error of last configuration
	//	5) Maximum error of all finger/fingertips for last configuration

	double time;	// Time in milliseconds
  };

#endif  // KAUTHAM_USE_GSL 

  class IvKinHand:public Kautham::InverseKinematic{
  public:
    IvKinHand(Robot* const rob);
    ~IvKinHand();
    INVKINECLASS type() {return HAND;}
    string       name() {return "Hand";}
    bool solve();
    bool setParameters();

#if defined(KAUTHAM_USE_GSL) && defined(NDEBUG)
        // Constructors
	void SahandIkSolver(const unsigned int max_jacobian_iterations_in, const double max_error_dist_in, 
			const double error_precision_in, const unsigned int max_slope_samples_in, const double slope_precision_in);

	// Functions
	void SolveIK(const boost::numeric::ublas::vector<double> &fing_pos_or, ikSahandResults &ik_results_out);
	void SolveIkLambda(const boost::numeric::ublas::vector<double> &fingertip_positions_orientations,
						const boost::numeric::ublas::vector<double> &lambda,
						ikSahandResults &ik_results_out);
	void SolveIkConfig(const boost::numeric::ublas::vector<double> &fingertip_positions, 
						const boost::numeric::ublas::vector<double> &initial_configuration,
						ikSahandResults &ik_results_out);
	boost::numeric::ublas::vector<double> GenerateRandFingPosOr(boost::numeric::ublas::vector<double> &twrist,
						boost::numeric::ublas::vector<double> &wconfig);
	boost::numeric::ublas::matrix<double> FingPosOrToThumb(boost::numeric::ublas::vector<double> &fingertip_positions_orientations);

	boost::numeric::ublas::vector<double> LambdaInitialConfiguration(
					const boost::numeric::ublas::vector<double> &fingertip_positions_orientations,
					const boost::numeric::ublas::vector<double> &lambda);

	boost::numeric::ublas::matrix<double> error_increment_list;		// List with errors and slopes

#endif    // KAUTHAM_USE_GSL

  private:
    IvKinHand();
	  vector<KthReal> _indexi; //fingertip (xyz)
	  vector<KthReal> _indexg; //point at fingertip+delta*n
	  vector<KthReal> _middlei; //fingertip (xyz)
	  vector<KthReal> _middleg; //point at fingertip+delta*n
	  vector<KthReal> _ringi; //fingertip (xyz)
	  vector<KthReal> _ringg; //point at fingertip+delta*n
	  vector<KthReal> _thumbi; //fingertip (xyz)
	  vector<KthReal> _thumbg; //point at fingertip+delta*n

#if defined(KAUTHAM_USE_GSL) && defined(NDEBUG)

// 	std::vector<KthReal> _indexi; //fingertip (xyz)
// 	KthReal _middlei[3];
// 	KthReal _ringi[3];
// 	KthReal _thumbi[3];
// 	KthReal _indexg[3];//point at fingertip+delta*n
// 	KthReal _middleg[3];
// 	KthReal _ringg[3];
// 	KthReal _thumbg[3];


	void Kt2IKSInData(void);						// Set values in vector des_fing_posor so it can be passed to sahand_iksolver
	void KSI2KtOutData(void);						// Get data from sahand_iksolver and convert the data to kautham format
	bool KtCheckCollision(boost::numeric::ublas::vector<float> &sah_conf);	// Function used by sahand_iksolver to check autocollisions in robot

	
	bool sahand_iksolved;

	boost::numeric::ublas::vector<double> des_fing_posor;	// desired finger positions and orientations

	// Functions
	inline void InitIKSolver(void);
	inline void SetLambdaSeq(const boost::numeric::ublas::vector<double> &fingertip_positions_orientations);
	inline void PosErrorAdjust(boost::numeric::ublas::vector<double> &position_error);
	inline boost::numeric::ublas::vector<double> ObtainJointIncrement(const boost::numeric::ublas::vector<double> 						&configuration,	const boost::numeric::ublas::matrix<double> &jacobian,
						const boost::numeric::ublas::vector<double> &position_error);
	inline void AdjustJointLimits(boost::numeric::ublas::vector<double> &configuration);
	inline unsigned int FingerDistEvolution(const boost::numeric::ublas::vector<double> &position_error, double 						*max_finger_error);
	inline unsigned int CollisionCheck(const boost::numeric::ublas::vector<double> &configuration);
	inline void LoadJointLimits(void);
	inline void LoadLambdas(void);
	inline void LoadEigenData(void);
	inline boost::numeric::ublas::matrix<double> LoadLogitCoef(void);
	inline void InitCollisions(void);
	inline void ConfigThumbToWrist(const boost::numeric::ublas::matrix<double> &t_thumb);

	//  Direct kinematics function
	inline boost::numeric::ublas::matrix<double> DirKinThumbToWrist(const boost::numeric::ublas::vector<double> 									&thumb_configuration);
	inline boost::numeric::ublas::vector<double> ThumbDirKinSAHand_Index_Ring(const boost::numeric::ublas::vector<double> &config);
	inline boost::numeric::ublas::matrix<double> WristDirKin(const boost::numeric::ublas::matrix<double> &wrist_base,
						const boost::numeric::ublas::vector<double> &wrist_config);
	inline boost::numeric::ublas::matrix<double> ThumbDirKin(const boost::numeric::ublas::vector<double> &thumb_config);
	inline void JacobianAndThumbDirKin(const boost::numeric::ublas::vector<double> &configuration);

	//  Math functions
	inline boost::numeric::ublas::vector<double> cross_p(const boost::numeric::ublas::vector<double>& p1,
								const boost::numeric::ublas::vector<double>& p2);
	inline double angle_2_vec_F(const boost::numeric::ublas::vector<double>& vec1, const boost::numeric::ublas::vector<double>& vec2);
	inline double prod_esc(const boost::numeric::ublas::vector<double>& vec1, const boost::numeric::ublas::vector<double>& vec2);
	inline double sqrmod(const boost::numeric::ublas::vector<double>& vec);


	// Variables
	unsigned int it_lambda;
	unsigned int max_jacobian_iterations;	// Maximum iterations for the jacobian 
	double max_error_dist;			// Maximum distance error allowed when calculating new joint increment
	double error_precision;			// Position error precision to assume that a finger/fingertip is in its desired position
	unsigned int max_slope_samples;		// Maximum iterations used to calculate the slope error
	double slope_precision;			// Slope precision to assume iterations are stopped
		
	boost::numeric::ublas::matrix<double> joint_limits;	// Lower and upper joint limits
	unsigned int n_lambdas;					// Number of lambdas
	boost::numeric::ublas::matrix<double> lambda_list;	// Lambda values
	boost::numeric::ublas::vector<double> eigenmean;	// principal component analysis's mean
	boost::numeric::ublas::matrix<double> eigenvectors;	// Eigen vectors
	boost::numeric::ublas::vector<double> eigenvalues;	// Eigen values
	double n_sigma;						// Number of standard deviation in eigen vectors

	boost::numeric::ublas::matrix<double> jacobian;			// Jacobian matrix
	boost::numeric::ublas::matrix<double> thumb_dir_kin;		// Thumb direct kinematics
	boost::numeric::ublas::vector<double> effective_position;	// End effector and fingertip position

	unsigned int n_logitprobs;		// # of logit probabilities
	unsigned int n_logitparam;		// # of characteristics for logit

	ikSahandResults ik_results;

// 	std::vector<geomobject*> objects;



	// Fileplot
	int vari1, vari2, vari3, vari4;
	double vard1, vard2, vard3, vard4;
	inline void DataPlot(void);
#endif  // KAUTHAM_USE_GSL
  };
  /** @}   end of Doxygen module "Util */

#endif  //_IVKINHAND_H
