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

/* Author: Alexander Perez, Jan Rosell */

 
#include "ivkinhand.h"
#include <problem/robot.h>

#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/matrix.hpp>

#include <iostream>
#include <iomanip>
#include <fstream>
#include <cstdlib>
#include <cmath>
#include <cassert>
#include <time.h>

#include "func_mat.h"


#define N_VDC_CONFIG 25         // Decoupled joints and virtual fingertips
#define N_TA_VDC_CONFIG 26      // 3rd Thumb angle, decoupled joints and virtual fingertips
#define N_POS_FING_THUMB 18	// Output dimension
#define FT_RAD 10		// Fingertip radius


  IvKinHand::IvKinHand(Robot* const rob) : Kautham::InverseKinematic(rob){

	  _robot = rob;
	_robConf.setRn(rob->getNumJoints());

	_indexi.resize(3);
	_indexg.resize(3);
	_middlei.resize(3);
	_middleg.resize(3);
	_ringi.resize(3);
	_ringg.resize(3);
	_thumbi.resize(3);
	_thumbg.resize(3);

	  _indexi[0] = 0.0;
	  _indexi[1] = 0.0;
	  _indexi[2] = 0.0;
	  _indexg[0] = 0.0;
	  _indexg[1] = 0.0;
	  _indexg[2] = 0.0;
	  _middlei[0] = 0.0;
	  _middlei[1] = 0.0;
	  _middlei[2] = 0.0;
	  _middleg[0] = 0.0;
	  _middleg[1] = 0.0;
	  _middleg[2] = 0.0;
	  _ringi[0] = 0.0;
	  _ringi[1] = 0.0;
	  _ringi[2] = 0.0;
	  _ringg[0] = 0.0;
	  _ringg[1] = 0.0;
	  _ringg[2] = 0.0;
	  _thumbi[0] = 0.0;
	  _thumbi[1] = 0.0;
	  _thumbi[2] = 0.0;
	  _thumbg[0] = 0.0;
	  _thumbg[1] = 0.0;
	  _thumbg[2] = 0.0;

      addParameter("Index xi", _indexi[0]);
      addParameter("Index yi", _indexi[1]);
      addParameter("Index zi", _indexi[2]);
      addParameter("Index xg", _indexg[0]);
      addParameter("Index yg", _indexg[1]);
      addParameter("Index zg", _indexg[2]);
      addParameter("Middle xi", _middlei[0]);
      addParameter("Middle yi", _middlei[1]);
      addParameter("Middle zi", _middlei[2]);
      addParameter("Middle xg", _middleg[0]);
      addParameter("Middle yg", _middleg[1]);
      addParameter("Middle zg", _middleg[2]);
      addParameter("Ring xi", _ringi[0]);
      addParameter("Ring yi", _ringi[1]);
      addParameter("Ring zi", _ringi[2]);
      addParameter("Ring xg", _ringg[0]);
      addParameter("Ring yg", _ringg[1]);
      addParameter("Ring zg", _ringg[2]);
      addParameter("Thumb xi", _thumbi[0]);
      addParameter("Thumb yi", _thumbi[1]);
      addParameter("Thumb zi", _thumbi[2]);
      addParameter("Thumb xg", _thumbg[0]);
      addParameter("Thumb yg", _thumbg[1]);
      addParameter("Thumb zg", _thumbg[2]);
#if defined(KAUTHAM_USE_GSL) && defined(NDEBUG)
	sahand_iksolved = false;
	des_fing_posor.resize(24,false);


	// Sahand_iksolver algorithm specifications
	max_error_dist = 100;
	max_jacobian_iterations = 100;
	error_precision = 1.5;	
	max_slope_samples = 15;
	slope_precision = 1;

	InitIKSolver();
#endif    // KAUTHAM_USE_GSL
  }

  IvKinHand::~IvKinHand(){

  }

  bool IvKinHand::solve(){
#if defined(KAUTHAM_USE_GSL) && defined(NDEBUG)
	cout << "GOAL HAND CONFIGURATION:" <<endl;
	cout << "Index i: ("<< _indexi[0] << ", " << _indexi[1] << ", " <<  _indexi[2] << ")" << endl;
	cout << "Index g: ("<< _indexg[0] << ", " << _indexg[1] << ", " <<  _indexg[2] << ")" << endl;
	cout << "Middle i: ("<< _middlei[0] << ", " << _middlei[1] << ", " <<  _middlei[2] << ")" << endl;
	cout << "Middle g: ("<< _middleg[0] << ", " << _middleg[1] << ", " <<  _middleg[2] << ")" << endl;
	cout << "Ring i: ("<< _ringi[0] << ", " << _ringi[1] << ", " <<  _ringi[2] << ")" << endl;
	cout << "Ring g: ("<< _ringg[0] << ", " << _ringg[1] << ", " <<  _ringg[2] << ")" << endl;
	cout << "Thumb i: ("<< _thumbi[0] << ", " << _thumbi[1] << ", " <<  _thumbi[2] << ")" << endl;
	cout << "Thumb g: ("<< _thumbg[0] << ", " << _thumbg[1] << ", " <<  _thumbg[2] << ")" << endl;

	// Check if contact points values are ok
	bool cp_ok = true;
	if ( (_indexi[0] == _indexg[0]) && (_indexi[1] == _indexg[1]) && (_indexi[2] == _indexg[2]) ){
		cout << "Error in Contact Points values:  indexi == indexg" <<endl;
		cp_ok = false;
	}
	if ( (_middlei[0] == _middleg[0]) && (_middlei[1] == _middleg[1]) && (_middlei[2] == _middleg[2]) ){
		cout << "Error in Contact Points values:  middlei == middleg" <<endl;
		cp_ok = false;
	}
	if ( (_ringi[0] == _ringg[0]) && (_ringi[1] == _ringg[1]) && (_ringi[2] == _ringg[2]) ){
		cout << "Error in Contact Points values:  ringi == ringg" <<endl;
		cp_ok = false;
	}
	if ( (_thumbi[0] == _thumbg[0]) && (_thumbi[1] == _thumbg[1]) && (_thumbi[2] == _thumbg[2]) ){
		cout << "Error in Contact Points values:  thumbi == thumbg" <<endl;
		cp_ok = false;
	}

	if ( (_indexi[0] == _ringi[0]) && (_indexi[1] == _ringi[1]) && (_indexi[2] == _ringi[2]) ){
		cout << "Error in Contact Points values:  indexi == ringi" <<endl;
		cp_ok = false;
	}

	// AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA
// 	// Get Random Contact Points
// 	boost::numeric::ublas::vector<double> rand_cpo, rand_conf, rand_twrist;
// 	rand_cpo = GenerateRandFingPosOr(rand_twrist, rand_conf);
// 	std::vector<KthReal> coords(17);
// 	// Thumb
// 	coords[0] = -(float)rand_conf(0);
// 	coords[1] = (float)rand_conf(1);
// 	coords[2] = -(float)rand_conf(2);
// 	coords[3] = -(float)rand_conf(3);
// 	coords[4] = -(float)rand_conf(4);
// 	//  Index
// 	coords[5] = (float)rand_conf(7);
// 	coords[6] = -(float)rand_conf(8);
// 	coords[7] = -(float)rand_conf(9);
// 	coords[8] = -(float)rand_conf(10);
// 	//  Middle
// 	coords[9] = (float)rand_conf(13);
// 	coords[10] = -(float)rand_conf(14);
// 	coords[11] = -(float)rand_conf(15);
// 	coords[12] = -(float)rand_conf(16);
// 	//  Ring
// 	coords[13] = (float)rand_conf(19);
// 	coords[14] = -(float)rand_conf(20);
// 	coords[15] = -(float)rand_conf(21);
// 	coords[16] = -(float)rand_conf(22);
//  	for (unsigned int i=0; i<17; i++)	cout << coords[i]*(180/PI) << " ";	cout << endl;
// 
// 	// Set wrist base
// 	//  Translation
// // 	std::vector<KthReal> wrist_position(3);
// // 	wrist_position[0] = 500;
// // 	wrist_position[1] = 500;
// // 	wrist_position[2] = 500;
// // 	_robConf.getSE3().setPos(wrist_position);
// 
// 	_robConf.setRn(coords);
// 	_robot->Kinematics(_robConf.getSE3());
// 	_robot->Kinematics(_robConf.getRn());
// 	if(_robot->autocollision())
// 	{
// 		cout << "Non valid solution - hand in autocolision" << endl;
// 	}
// 	else
// 	{
// 		cout << "Valid solution - hand is free from autocollisions" << endl;
// 	}
// 
// 	cp_ok = true;
// 	des_fing_posor = rand_cpo;
	// AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA

	// Ik solved 1 ini conf 33 it
// 	  _indexi[0] = 80.0;
// 	  _indexi[1] = 0.0;
// 	  _indexi[2] = 20.0;
// 	  _indexg[0] = 60.0;
// 	  _indexg[1] = 0.0;
// 	  _indexg[2] = 20.0;
// 	  _middlei[0] = 80.0;
// 	  _middlei[1] = 0.0;
// 	  _middlei[2] = 60.0;
// 	  _middleg[0] = 60.0;
// 	  _middleg[1] = 0.0;
// 	  _middleg[2] = 60.0;
// 	  _ringi[0] = 80.0;
// 	  _ringi[1] = 0.0;
// 	  _ringi[2] = 100.0;
// 	  _ringg[0] = 60.0;
// 	  _ringg[1] = 0.0;
// 	  _ringg[2] = 100.0;
// 	  _thumbi[0] = 0.0;
// 	  _thumbi[1] = 0.0;
// 	  _thumbi[2] = 60.0;
// 	  _thumbg[0] = 20.0;
// 	  _thumbg[1] = 0.0;
// 	  _thumbg[2] = 60.0;

	// Ik NOT solved
// 	  _indexi[0] = 120.0;
// 	  _indexi[1] = 0.0;
// 	  _indexi[2] = 20.0;
// 	  _indexg[0] = 100.0;
// 	  _indexg[1] = 0.0;
// 	  _indexg[2] = 20.0;
// 	  _middlei[0] = 120.0;
// 	  _middlei[1] = 0.0;
// 	  _middlei[2] = 60.0;
// 	  _middleg[0] = 100.0;
// 	  _middleg[1] = 0.0;
// 	  _middleg[2] = 60.0;
// 	  _ringi[0] = 120.0;
// 	  _ringi[1] = 0.0;
// 	  _ringi[2] = 100.0;
// 	  _ringg[0] = 100.0;
// 	  _ringg[1] = 0.0;
// 	  _ringg[2] = 100.0;
// 	  _thumbi[0] = 0.0;
// 	  _thumbi[1] = 0.0;
// 	  _thumbi[2] = 60.0;
// 	  _thumbg[0] = 20.0;
// 	  _thumbg[1] = 0.0;
// 	  _thumbg[2] = 60.0;
// 	cp_ok = true;



	sahand_iksolved = false;
	if (cp_ok == true){
		//setParameters();
  		cout << "Kt2IKSInData... ";
		Kt2IKSInData();
  		cout << "DONE!" << endl;
	
//  		cout << "SolveIK... " << endl;
		SolveIK(des_fing_posor, ik_results);

//  		cout << "KSI2KtOutData... ";	
		 KSI2KtOutData();
 		cout << " IK DONE!" << endl;
	}
	else{
		cout << "OPERATION CANCELLED!" <<endl;
		sahand_iksolved = false;
	}

// 	cout << "sahand_iksolved = " << sahand_iksolved << endl;

	return sahand_iksolved;
#else
  return true;
#endif  // KAUTHAM_USE_GSL


    //////////////////////////////////////////////////////
	//here goes the code to compute the inverse kinematics
	//must load configurations _se3 and _rn


    //as an example set the xyz position of the palm at the values of _indexi
//   _robConf.getSE3().setPos(_indexi);
//   std::vector<KthReal> coords(_robot->getNumJoints());
// 
//   for(int i=0; i<_robot->getNumJoints(); i++)
//     coords[i]=-45*(PI/180);
// 
// 	coords[0] = _ringi[0]*PI/180;//-PI/4.0;
// 	cout<<"coords[0] set to "<<coords[0]<<endl;
//   _robConf.setRn(coords);
// 
//   cout<<_robConf.getRn().print();
//   cout<<_robConf.getSE3().print();
// 
// 	//here ends the code to compute the inverse kinematics
//     //////////////////////////////////////////////////////
// 
// 
// 	//put the hand at the found configuration and test for autocollision
//   _robot->Kinematics(_robConf.getSE3());
//   _robot->Kinematics(_robConf.getRn());
// 
// 	if(_robot->autocollision())
// 	{
// 		cout << "Non valid solution - hand in autocolision" << endl;
// 	}
// 	else
// 	{
// 		cout << "Valid solution - hand is free from autocollisions" << endl;
// 	}
// 
//     return true;
  }

  bool IvKinHand::setParameters(){
    try{
		//update index data
        HASH_S_K::iterator it = _parameters.find("Index xi");
        if(it != _parameters.end())
          _indexi[0] = it->second;
        else
          return false;

        it = _parameters.find("Index yi");
        if(it != _parameters.end())
          _indexi[1] = it->second;
        else
          return false;

        it = _parameters.find("Index zi");
        if(it != _parameters.end())
          _indexi[2] = it->second;
        else
          return false;

		it = _parameters.find("Index xg");
        if(it != _parameters.end())
          _indexg[0] = it->second;
        else
          return false;

        it = _parameters.find("Index yg");
        if(it != _parameters.end())
          _indexg[1] = it->second;
        else
          return false;

        it = _parameters.find("Index zg");
        if(it != _parameters.end())
          _indexg[2] = it->second;
        else
          return false;

		//update middle data
        it = _parameters.find("Middle xi");
        if(it != _parameters.end())
          _middlei[0] = it->second;
        else
          return false;

        it = _parameters.find("Middle yi");
        if(it != _parameters.end())
          _middlei[1] = it->second;
        else
          return false;

        it = _parameters.find("Middle zi");
        if(it != _parameters.end())
          _middlei[2] = it->second;
        else
          return false;

		it = _parameters.find("Middle xg");
        if(it != _parameters.end())
          _middleg[0] = it->second;
        else
          return false;

        it = _parameters.find("Middle yg");
        if(it != _parameters.end())
          _middleg[1] = it->second;
        else
          return false;

        it = _parameters.find("Middle zg");
        if(it != _parameters.end())
          _middleg[2] = it->second;
        else
          return false;
		
		//update ring data
        it = _parameters.find("Ring xi");
        if(it != _parameters.end())
          _ringi[0] = it->second;
        else
          return false;

        it = _parameters.find("Ring yi");
        if(it != _parameters.end())
          _ringi[1] = it->second;
        else
          return false;

        it = _parameters.find("Ring zi");
        if(it != _parameters.end())
          _ringi[2] = it->second;
        else
          return false;

		it = _parameters.find("Ring xg");
        if(it != _parameters.end())
          _ringg[0] = it->second;
        else
          return false;

        it = _parameters.find("Ring yg");
        if(it != _parameters.end())
          _ringg[1] = it->second;
        else
          return false;

        it = _parameters.find("Ring zg");
        if(it != _parameters.end())
          _ringg[2] = it->second;
        else
          return false;

		//update thumb data
        it = _parameters.find("Thumb xi");
        if(it != _parameters.end())
          _thumbi[0] = it->second;
        else
          return false;

        it = _parameters.find("Thumb yi");
        if(it != _parameters.end())
          _thumbi[1] = it->second;
        else
          return false;

        it = _parameters.find("Thumb zi");
        if(it != _parameters.end())
          _thumbi[2] = it->second;
        else
          return false;

		it = _parameters.find("Thumb xg");
        if(it != _parameters.end())
          _thumbg[0] = it->second;
        else
          return false;

        it = _parameters.find("Thumb yg");
        if(it != _parameters.end())
          _thumbg[1] = it->second;
        else
          return false;

        it = _parameters.find("Thumb zg");
        if(it != _parameters.end())
			_thumbg[2] = it->second;
        else
          return false;

      }catch(...){
        return false;
      }
      return true;
  }

#if defined(KAUTHAM_USE_GSL) && defined(NDEBUG)
	void IvKinHand::Kt2IKSInData(void){
		// Function that fills vector des_fing_posor with parameters values as:
		//	fingertip_positions_orientations(0:2) = Thumb desired position
		//	fingertip_positions_orientations(3:5) = Thumb desired orientation
		//	fingertip_positions_orientations(6:8) = Index desired positio
		//	fingertip_positions_orientations(9:11) = Index desired orientation
		//	fingertip_positions_orientations(12:14) = Middle desired position
		//	fingertip_positions_orientations(15:17) = Middle desired orientatio
		//	fingertip_positions_orientations(18:20) = Ring desired position	
		//	fingertip_positions_orientations(21:23) = Ring desired orientation
		//  Each orientation corresponds to the external normal to the object surface

		// Thumb
		des_fing_posor(0) = _thumbi[0];
		des_fing_posor(1) = _thumbi[1];
		des_fing_posor(2) = _thumbi[2];
		float pnorm = sqrt((_thumbg[0]-_thumbi[0])*(_thumbg[0]-_thumbi[0]) +
					(_thumbg[1]-_thumbi[1])*(_thumbg[1]-_thumbi[1]) +
					(_thumbg[2]-_thumbi[2])*(_thumbg[2]-_thumbi[2]));
		des_fing_posor(3) = (_thumbg[0]-_thumbi[0])/pnorm;
		des_fing_posor(4) = (_thumbg[1]-_thumbi[1])/pnorm;
		des_fing_posor(5) = (_thumbg[2]-_thumbi[2])/pnorm;

		// Index
		des_fing_posor(6) = _indexi[0];
		des_fing_posor(7) = _indexi[1];
		des_fing_posor(8) = _indexi[2];
		pnorm = sqrt((_indexg[0]-_indexi[0])*(_indexg[0]-_indexi[0]) +
				(_indexg[1]-_indexi[1])*(_indexg[1]-_indexi[1]) +
				(_indexg[2]-_indexi[2])*(_indexg[2]-_indexi[2]));
		des_fing_posor(9) = (_indexg[0]-_indexi[0])/pnorm;
		des_fing_posor(10) = (_indexg[1]-_indexi[1])/pnorm;
		des_fing_posor(11) = (_indexg[2]-_indexi[2])/pnorm;

		// Middle
		des_fing_posor(12) = _middlei[0];
		des_fing_posor(13) = _middlei[1];
		des_fing_posor(14) = _middlei[2];
		pnorm = sqrt((_middleg[0]-_middlei[0])*(_middleg[0]-_middlei[0]) +
				(_middleg[1]-_middlei[1])*(_middleg[1]-_middlei[1]) +
				(_middleg[2]-_middlei[2])*(_middleg[2]-_middlei[2]));
		des_fing_posor(15) = (_middleg[0]-_middlei[0])/pnorm;
		des_fing_posor(16) = (_middleg[1]-_middlei[1])/pnorm;
		des_fing_posor(17) = (_middleg[2]-_middlei[2])/pnorm;

		// Ring
		des_fing_posor(18) = _ringi[0];
		des_fing_posor(19) = _ringi[1];
		des_fing_posor(20) = _ringi[2];
		pnorm = sqrt((_ringg[0]-_ringi[0])*(_ringg[0]-_ringi[0]) +
				(_ringg[1]-_ringi[1])*(_ringg[1]-_ringi[1]) +
				(_ringg[2]-_ringi[2])*(_ringg[2]-_ringi[2]));
		des_fing_posor(21) = (_ringg[0]-_ringi[0])/pnorm;
		des_fing_posor(22) = (_ringg[1]-_ringi[1])/pnorm;
		des_fing_posor(23) = (_ringg[2]-_ringi[2])/pnorm;

		return;
	}



	void IvKinHand::KSI2KtOutData(void){

		// Set if solved or not
		if(ik_results.ik_solved == true)
			sahand_iksolved = true;
		else
			sahand_iksolved = false;

		// Set configuration
		//  Thumb
		std::vector<KthReal> coords(17);
		coords[0] = -(float)ik_results.configuration(0);
		coords[1] = (float)ik_results.configuration(1);
		coords[2] = -(float)ik_results.configuration(2);
		coords[3] = -(float)ik_results.configuration(3);
		coords[4] = -(float)ik_results.configuration(4);
		//  Index
		coords[5] = (float)ik_results.configuration(7);
		coords[6] = -(float)ik_results.configuration(8);
		coords[7] = -(float)ik_results.configuration(9);
		coords[8] = -(float)ik_results.configuration(10);
		//  Middle
		coords[9] = (float)ik_results.configuration(13);
		coords[10] = -(float)ik_results.configuration(14);
		coords[11] = -(float)ik_results.configuration(15);
		coords[12] = -(float)ik_results.configuration(16);
		//  Ring
		coords[13] = (float)ik_results.configuration(19);
		coords[14] = -(float)ik_results.configuration(20);
		coords[15] = -(float)ik_results.configuration(21);
		coords[16] = -(float)ik_results.configuration(22);
		_robConf.setRn(coords);

		// Set wrist base
		//  Translation
		std::vector<KthReal> wrist_position(3);
		wrist_position[0] = (float)ik_results.t_wrist(12);
		wrist_position[1] = (float)ik_results.t_wrist(13);
		wrist_position[2] = (float)ik_results.t_wrist(14);
		_robConf.getSE3().setPos(wrist_position);

		//  Orientation
		Matrix3x3 MatRWrist((float)ik_results.t_wrist(0), (float)ik_results.t_wrist(4), (float)ik_results.t_wrist(8),
        			    (float)ik_results.t_wrist(1), (float)ik_results.t_wrist(5), (float)ik_results.t_wrist(9),
				    (float)ik_results.t_wrist(2), (float)ik_results.t_wrist(6), (float)ik_results.t_wrist(10));
		Rotation rWrist(MatRWrist);
		std::vector<KthReal> wrist_quaternion(4);
		wrist_quaternion[0] = rWrist.at(0);
		wrist_quaternion[1] = rWrist.at(1);
		wrist_quaternion[2] = rWrist.at(2);
		wrist_quaternion[3] = rWrist.at(3);
		_robConf.getSE3().setOrient(wrist_quaternion);


		// Ploting results
		cout << "Inverse Kinematics ";
		if (ik_results.ik_solved == false)	cout<< "NOT ";
		cout << "solved. ";

		unsigned int n_init_conf = ik_results.it_data.size1();
		if (ik_results.ik_solved == true){
			cout << n_init_conf << " initial configurations and " 
				<< ik_results.it_data(n_init_conf-1,0)  << " jacobian iterations.";
		}
		else{
			cout << n_init_conf << " initial configurations.";
		}

		return;
	}


	bool IvKinHand::KtCheckCollision(boost::numeric::ublas::vector<float> &sah_conf){
		// Function used by sahand_iksolver to check autocollisions in robot
		// This function gets as input a 17 dimensional vector, corresponding to the non
		//  coupled joint values of the SAH as:
		//   sah_conf(0) = thumb roll
		//   sah_conf(1) = thumb proximal abduction
		//   sah_conf(2) = thumb proximal flexion
		//   sah_conf(3) = thumb intermediate flexion
		//   sah_conf(4) = thumb proximal flexion
		//   sah_conf(5) = index proximal abduction
		//   sah_conf(6) = index proximal flexion
		//   sah_conf(7) = index intermediate flexion
		//   sah_conf(8) = index proximal flexion
		//   sah_conf(9) = middle proximal abduction
		//   sah_conf(10) = middle proximal flexion
		//   sah_conf(11) = middle intermediate flexion
		//   sah_conf(12) = middle proximal flexion
		//   sah_conf(13) = ring proximal abduction
		//   sah_conf(14) = ring proximal flexion
		//   sah_conf(15) = ring intermediate flexion
		//   sah_conf(16) = ring proximal flexion
		//  If there are autocollisions, function outputs TRUE; otherwise, FALSE.


		unsigned int sah_conf_s = sah_conf.size();
		unsigned int n_sah_joints = _robot->getNumJoints();

		assert(n_sah_joints == sah_conf_s);
		assert(n_sah_joints == 17);

		std::vector<KthReal> coords(n_sah_joints);
		for(unsigned int i=0; i<n_sah_joints; i++)
			coords[i]=sah_conf(i);
		_robConf.setRn(coords);

		_robot->Kinematics(_robConf.getSE3());
		_robot->Kinematics(_robConf.getRn());
		if(_robot->autocollision())
		{
			cout << "AUTOCOLLISION!" << endl;	// TTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT
			return true;
		}
		else
		{
			cout << "NO AUTOCOLLISION!" << endl;	// TTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT
			return false;
		}
	}

	void IvKinHand::SahandIkSolver(const unsigned int max_jacobian_iterations_in, const double max_error_dist_in, 
								const double error_precision_in, const unsigned int max_slope_samples_in, const double slope_precision_in){
		// algorithm specifications
		max_error_dist = max_error_dist_in;
		max_jacobian_iterations = max_jacobian_iterations_in;
		error_precision = error_precision_in;	
		max_slope_samples = max_slope_samples_in;
		slope_precision = slope_precision_in;
	
		InitIKSolver();
	}
	
	inline void IvKinHand::InitIKSolver(void){
	
		// Variable to iterate for one lambda or for more
		it_lambda = 0;
	
		// Load robot joint limits
		LoadJointLimits();
	
		// Load lambda values
		LoadLambdas();
	
		// Load eigen vectors and eigen values
		LoadEigenData();
	
		// Initiallize collision object
		InitCollisions();
	
		ik_results.configuration.resize(26,false);	ik_results.configuration.clear();
		ik_results.it_data.resize(1,5,false);		ik_results.it_data.clear();
	
		jacobian.resize(N_POS_FING_THUMB+4,N_TA_VDC_CONFIG,false);
		effective_position.resize(N_POS_FING_THUMB,false);
	}
	
	
	void IvKinHand::SolveIK(const boost::numeric::ublas::vector<double> &fing_pos_or, ikSahandResults &ik_results_out){
		// ***************************************************************
		//
		//  Function that solves the inverse kinematics of Sahand that places finger and fingertips by 'fingertip_positions_orientations':
		//		fingertip_positions_orientations(0:2) = Thumb desired position		fingertip_positions_orientations(3:5) = Thumb desired orientation
		//		fingertip_positions_orientations(6:8) = Index desired position		fingertip_positions_orientations(9:11) = Index desired orientation
		//		fingertip_positions_orientations(12:14) = Middle desired position	fingertip_positions_orientations(15:17) = Middle desired orientatio
		//		fingertip_positions_orientations(18:20) = Ring desired position		fingertip_positions_orientations(21:23) = Ring desired orientation
		//
		//  As an output, the function returns
		//	  ik_results_out.t_wrists: 16 dimension vector of the transformation matrix related to the wrist base as:
		//		t_wrists(i,:) = [oxx oxy oxz 0 oyx oyy oyz 0 ozx ozy ozz 0 px py pz 1]
		//
		//	  ik_results_out.configuration_list: matrix with a 13 dimension vector with a sahand configuration. 
		//		Each row has the same data structure as thumb_results.configuration
		//
		//	  ik_results_out.t_wrist_list: matrix with a 16 dimension vector in each row of the transformation matrix
		//      related to the wrist base. Each row has the same data structure as thumb_results.t_wrists
		//
		//	  ik_results_out.ik_solved: variable containing 'true' if inverse kinematics has been solved. Otherwise, 'false'.
		//
		// ***************************************************************
	
		// Assertions
		assert(fing_pos_or.size() == 24);
		// End assertions
	
		boost::numeric::ublas::vector<double> fingertip_positions_orientations;
		fingertip_positions_orientations = fing_pos_or;
	
		// Clock
		double t_1, t_2;
		t_1 = clock();
	
		it_lambda = 1;
	
		ik_results.it_data.resize(1,5,false);
	
		boost::numeric::ublas::vector<double> vlambda(2);
	
		//std::cout << "--> " << ik_results.configuration_list.size1() << std::endl;	// TTTTTTTTTTTTTTTTTTTTTT
	
		//SetLambdaSeq(fingertip_positions_orientations);
	
		// Pass finger positions and orientations to thumb point of view
		boost::numeric::ublas::matrix<double> t_thumb(4,4);
		t_thumb = FingPosOrToThumb(fingertip_positions_orientations);
	
		ik_results.ik_solved = false;
		for (unsigned int nl=0; (nl<n_lambdas) && (!ik_results.ik_solved); nl++){
	
			// Get lambda
			vlambda(0) = lambda_list(0,nl); 
			vlambda(1) = lambda_list(1,nl);
	
			SolveIkLambda(fingertip_positions_orientations, vlambda, ik_results);
		}
	
		// Transform obtained configurations to wrist reference
		ConfigThumbToWrist(t_thumb);
	
		// Time calculation
		t_2 = clock() - t_1;
		double elapSeconds = t_2/CLOCKS_PER_SEC;
		double elapMilli = elapSeconds*1000;
		ik_results.time = elapMilli;
		// End Time calculation
	
		ik_results_out = ik_results;
	
		it_lambda = 0;
	
		return;
	}
	
	void IvKinHand::SetLambdaSeq(const boost::numeric::ublas::vector<double> &fpo){
	
		// Assertions
		assert(fpo.size() == 24);
		// End assertions
	
		n_logitprobs = 4;
		n_logitparam = 12;
	
		boost::numeric::ublas::vector<double> zprob(n_logitprobs), zcharac(n_logitparam);
		boost::numeric::ublas::matrix<double> coef;
	
		// Extract characteristic parameter
		//  Distances
		zcharac(0) = pow(fpo(0)-fpo(6),2) + pow(fpo(1)-fpo(7),2) + pow(fpo(2)-fpo(8),2);	// Distance index-thumb
		zcharac(1) = pow(fpo(0)-fpo(12),2) + pow(fpo(1)-fpo(13),2) + pow(fpo(2)-fpo(14),2);	// Distance middle-thumb
		zcharac(2) = pow(fpo(0)-fpo(18),2) + pow(fpo(1)-fpo(19),2) + pow(fpo(2)-fpo(20),2);	// Distance ring-thumb
		zcharac(3) = pow(fpo(6)-fpo(12),2) + pow(fpo(7)-fpo(13),2) + pow(fpo(8)-fpo(14),2);	// Distance index-middle 
		zcharac(4) = pow(fpo(6)-fpo(18),2) + pow(fpo(7)-fpo(19),2) + pow(fpo(8)-fpo(20),2);	// Distance index-ring
		zcharac(5) = pow(fpo(12)-fpo(18),2) + pow(fpo(13)-fpo(19),2) + pow(fpo(14)-fpo(20),2);	// Distance middle-ring
	
		// 6 Euler angles for the orientations
		//  Constructing the rotation matrix
		boost::numeric::ublas::vector<double> vx(3), vy(3), vz(3), tr(3);
		for (unsigned int i=0; i<3; i++){
			tr(i) = fpo(i);			// Origin
			vx(i) = fpo(i+12)-fpo(i);	// Provisional X Axis - From thumb to middle
			vz(i) = (fpo(i+3)-fpo(i))/FT_RAD;	// Z Axis
		}
		vx = vx/sqrt(vx(0)*vx(0)+vx(1)*vx(1)+vx(2)*vx(2));
		vz = vz/sqrt(vz(0)*vz(0)+vz(1)*vz(1)+vz(2)*vz(2));
	
		vy = cross_p(vz,vx);			// Y Axis
		vy = vy/sqrt(vy(0)*vy(0)+vy(1)*vy(1)+vy(2)*vy(2));
		vx = cross_p(vy,vz);			// X Axis
		vx = vx/sqrt(vx(0)*vx(0)+vx(1)*vx(1)+vx(2)*vx(2));
	
		boost::numeric::ublas::matrix<double> mrot(4,4);	mrot.clear();
		for (unsigned int i=0; i<3; i++){
			mrot(i,0) = vx(i);		// X Axis
			mrot(i,1) = vy(i);		// Y Axis
			mrot(i,2) = vz(i);		// Z Axis
			mrot(i,3) = tr(i);		// Origin
		}
		mrot(3,3) = 1;
	
		boost::numeric::ublas::matrix<double> mirot(4,4);
		mirot = inv(mrot);
	
		//  Calculating Euler Angles
		boost::numeric::ublas::vector<double> vec(4);
		//   Index
		for (unsigned int i=0; i<3; i++)	vec(i) = fpo(i+9);	vec(3)=0;
		vec = prod(mirot,vec);
        if (vec(1) < -1. || vec(1) > 1.) throw std::invalid_argument("Called asin(x) with |x| > 1");
		zcharac(6) = asin(vec(1));							// Index Beta 
		zcharac(7) = atan2(-(vec(0)/cos(zcharac(6))),-(vec(2)/cos(zcharac(6))));		// Index Alpha 
		//   Middle
		for (unsigned int i=0; i<3; i++)	vec(i) = fpo(i+15);	vec(3)=0;
		vec = prod(mirot,vec);
        if (vec(1) < -1. || vec(1) > 1.) throw std::invalid_argument("Called asin(x) with |x| > 1");
		zcharac(8) = asin(vec(1));							// Index Beta 
		zcharac(9) = atan2(-(vec(0)/cos(zcharac(8))),-(vec(2)/cos(zcharac(8))));		// Index Alpha 
		//   Ring
		for (unsigned int i=0; i<3; i++)	vec(i) = fpo(i+21);	vec(3)=0;
		vec = prod(mirot,vec);
        if (vec(1) < -1. || vec(1) > 1.) throw std::invalid_argument("Called asin(x) with |x| > 1");
		zcharac(10) = asin(vec(1));							// Index Beta 
		zcharac(11) = atan2(-(vec(0)/cos(zcharac(10))),-(vec(2)/cos(zcharac(10))));	// Index Alpha 
	
	
		// Calculate probabilities
		//   Load coefficients
		coef = LoadLogitCoef();
		zprob.clear();
		for (unsigned int nlg=0; nlg<n_logitprobs; nlg++){
			zprob(nlg) = coef(nlg,0);
			for (unsigned int nc=1; nc<n_logitparam+1; nc++)		zprob(nlg) += coef(nlg,nc)*zcharac(nc);
			zprob(nlg) = (exp(zprob(nlg)))/(1+exp(zprob(nlg)));
		}
	
		// Sort lambdas
		unsigned int ip_max = 0;
		unsigned int p_max = 0;
		unsigned int lv1, lv2;
		for (unsigned int i=0; i<n_logitprobs; i++){
			ip_max = i;
			p_max = zprob(i);
			for (unsigned int j=i; j<n_logitprobs; j++){
				if (zprob(j) > p_max){
					p_max = zprob(j);
					ip_max = j;
				}
			}
			// Switching positions
			lv1 = lambda_list(0,ip_max);
			lv2 = lambda_list(1,ip_max);
	
			zprob(ip_max) = zprob(i);
			lambda_list(0,ip_max) = lambda_list(0,i);
			lambda_list(1,ip_max) = lambda_list(1,i);
	
			zprob(i) = p_max;
			lambda_list(0,i) = lv1;
			lambda_list(1,i) = lv2;
		}
	
	
		return;
	}
	
	void IvKinHand::SolveIkLambda(const boost::numeric::ublas::vector<double> &fingertip_positions_orientations,
										const boost::numeric::ublas::vector<double> &lambda,
										ikSahandResults &ik_results_out){
	
		// Generate initial configuration
		boost::numeric::ublas::vector<double> initial_thumb_configuration;
		initial_thumb_configuration = LambdaInitialConfiguration(fingertip_positions_orientations, lambda);
	
		if (it_lambda == 0){
			ik_results.it_data.resize(1,5,false);
		}
	
		// Execute jacobian iteration
		ik_results.ik_solved = false;
		error_increment_list.resize(2,max_jacobian_iterations+1,false);
		SolveIkConfig(fingertip_positions_orientations, initial_thumb_configuration, ik_results);
	
		if (it_lambda == 1){
			if (ik_results.ik_solved == false)		
				ik_results.it_data.resize(ik_results.it_data.size1()+1,5,true);
		}
	
		ik_results_out = ik_results;
	
		return;
	}
	
	inline boost::numeric::ublas::vector<double> IvKinHand::LambdaInitialConfiguration(
							const boost::numeric::ublas::vector<double> &fingertip_positions_orientations,
							const boost::numeric::ublas::vector<double> &lambda){
	
		// *******************************************************************************************************************
		// Function that generates a initial configuration given a certain 'lambda'
		//
		//	Input variables:
		//   fingertip_positions_orientations: vector of dimension 18 with the information related to desired finger
		//	  positions and orientations referenced from the thumb base as
		//		finger_positions(0:2) = Index desired position		finger_positions(3:5) = Index desired orientatio
		//		finger_positions(6:8) = Middle desired position		finger_positions(9:11) = Middle desired orientatio
		//		finger_positions(12:14) = Ring desired position		finger_positions(15:17) = Ring desired orientatio
		//
		//	 lambda: vector that contains lambda values
		//
		//	Output variables:
		//      config(0) = hand rotation around thumb axis
		//      config(1) = thumbrot
		//	config(2) = thumb abduction		config(3) = first thumb flexion		config(4) = second thumb flexion
		//	config(5) = third thumb flexion		config(6) = first thumb fingertip	config(7) = second thumb fingertip
		//	config(8) = index abduction		config(9) = first index flexion		config(10) = second index flexion
		//	config(11) = third index flexion	config(12) = first index fingertip	config(13) = second index fingertip
		//	config(14) = middle abduction		config(15) = first middle flexion	config(16) = second middle flexion
		//	config(17) = third middle flexion	config(18) = first middle fingertip	config(19) = second middle fingertip
		//	config(20) = ring abduction		config(21) = first ring flexion		config(22) = second ring flexion
		//	config(23) = third ring flexion		config(24) = first ring fingertip	config(25) = second ring fingertip
		//
		// *******************************************************************************************************************
	
	
		// ----- Asserting input variables -----
		assert(lambda.size() > 0);
		// ----- End Asserting input variables -----
	
	
		boost::numeric::ublas::vector<double> initial_thumb_configuration(N_TA_VDC_CONFIG);
	
		// Eigen ----------------------------------------------------------------------------
		// Using first and second eigen vectors to compute the initial configuration
		boost::numeric::ublas::vector<double> eig_one(25), eig_two(25);
		for (unsigned int i=0; i<25; i++){
			eig_one(i) = eigenvectors(i,0);
			eig_two(i) = eigenvectors(i,1);
		}
		
		boost::numeric::ublas::vector<double> vec1(25), vec2(25);
		vec1 = n_sigma*sqrt(eigenvalues(0))*eig_one;		vec2 = n_sigma*sqrt(eigenvalues(1))*eig_two;
	
		// Values of finger angles are set depending on the 'lambda' value
		for (unsigned int i=1; i<N_TA_VDC_CONFIG; i++)
			initial_thumb_configuration(i) = eigenmean(i-1) + (2*lambda(0)-1)*vec1(i-1) + (2*lambda(1)-1)*vec2(i-1);
	
		for (unsigned int i=0; i<4; i++) {
			// Angle 2 is set to its middle range value
			initial_thumb_configuration(6*i+2) = (joint_limits(6*i+1,0) + joint_limits(6*i+1,1))/2;	
			// Angle 6 is set to its middle range value
			initial_thumb_configuration(6*i+6) = (joint_limits(6*i+5,0) + joint_limits(6*i+5,1))/2;
			// Angle 7 is set to its middle range value
			initial_thumb_configuration(6*i+7) = (joint_limits(6*i+6,0) + joint_limits(6*i+6,1))/2;
		}
		// End Eigen -----------------------------------------------------------------------
	
	
		// Obtaining Alpha angle by iteratively searching the best Alpha that minimizes 'angle_2_vec_F(vdfp, vdrp)'
		// ------------------------------------------------------------------------------------------------------------
		boost::numeric::ublas::vector<double> pdi(3), pdr(3);
		for (unsigned int i=0; i<3; i++) {
			pdi(i) = fingertip_positions_orientations(i);       // index desired fingertip position
			pdr(i) = fingertip_positions_orientations(i+12);    // ring desired fingertip position
		}
		boost::numeric::ublas::vector<double> vdr(3), vdrp(3);
		vdr = pdr - pdi;
	
		boost::numeric::ublas::vector<double> pfr(3), pfi(3), vdf(3), vdfp(3), v_pfi_pfr(6);
		boost::numeric::ublas::vector<double> ex(2), sl(4), val(4);
		double val_m, ang_min, val_min;
	
		// ALPHA Optimization
		ex(0) = 0;
		ex(1) = 2*PI;
		val_min = 10000;
		for (unsigned int i=0; i<4; i++) {
			sl(0) = ex(0);
			sl(3) = ex(1);
			sl(1) = (2*ex(0) + ex(1))/3;
			sl(2) = (ex(0) + 2*ex(1))/3;
	
			for (unsigned int j=0; j<4; j++) {
				initial_thumb_configuration(0) = sl(j);
	
				// Ring
				v_pfi_pfr = ThumbDirKinSAHand_Index_Ring(initial_thumb_configuration);
				for (unsigned int k=0; k<3; k++){
					pfi(k) = v_pfi_pfr(k);
					pfr(k) = v_pfi_pfr(k+3);
				}
				vdf = pfr - pfi;
	
				// vdf projection to plane z=0
				vdfp(0) = vdf(0);   vdfp(1) = vdf(1);   vdfp(2) = 0;
	
				// vdr projection to plane z=0
				vdrp(0) = vdr(0);   vdrp(1) = vdr(1);   vdrp(2) = 0;
	
				val(j) = angle_2_vec_F(vdfp, vdrp);
                double tmp = prod_esc(vdfp,vdrp)/( norm_2(vdfp)*norm_2(vdrp) );
                if (tmp < -1. || tmp > 1.) throw std::invalid_argument("Called acos(x) with |x| > 1");
                val(j) = acos( tmp );
			}
			
			for (unsigned int j=0; j<3; j++) {
				val_m = val(j) + val(j+1);
		
				if (val_m < val_min) {
					ex(0) = sl(j);
					ex(1) = sl(j+1);
					ang_min = (ex(0) + ex(1))/2;
					val_min = val_m;
				}
			}
	
			initial_thumb_configuration(0) = ang_min;		// Alpha
		}
	
		return initial_thumb_configuration;
	}
	
	void IvKinHand::SolveIkConfig(const boost::numeric::ublas::vector<double> &fingertip_positions, 
					const boost::numeric::ublas::vector<double> &initial_configuration,
					ikSahandResults &ik_results_out){
		// *******************************************************************************************************************
		// Function that, starting by an initial configuration of the hand, iterates using the jacobian in order to find a
		//  configuration of the hand that satisfies the constraints in fingertip_positions_orientations. If this configuration
		//  is found, data is stored in ikSahandResults
		//
		//	Input variables:
		//   fingertip_positions: vector of dimension 18 with the information of the desired finger positions and orientations 
		//	 referenced from the thumb base as
		//		fingertip_positions(0:2) = Index desired position		fingertip_positions(3:5) = Index desired inside fingertip position
		//		fingertip_positions(6:8) = Middle desired position		fingertip_positions(9:11) = Middle desired inside fingertip position
		//		fingertip_positions(12:14) = Ring desired position		fingertip_positions(15:17) = Ring desired inside fingertip position
		//
		//	 initial_configuration:
		//      config(0) = hand rotation around thumb axis
		//      config(1) = thumbrot
		//		config(2) = thumb abduction		config(3) = first thumb flexion			config(4) = second thumb flexion
		//		config(5) = third thumb flexion		config(6) = first thumb fingertip		config(7) = second thumb fingertip
		//		config(8) = index abduction		config(9) = first index flexion			config(10) = second index flexion
		//		config(11) = third index flexion	config(12) = first index fingertip		config(13) = second index fingertip
		//		config(14) = middle abduction		config(15) = first middle flexion		config(16) = second middle flexion
		//		config(17) = third middle flexion	config(18) = first middle fingertip		config(19) = second middle fingertip
		//		config(20) = ring abduction		config(21) = first ring flexion			config(22) = second ring flexion
		//		config(23) = third ring flexion		config(24) = first ring fingertip		config(25) = second ring fingertip
		//
		//   max_jacobian_iterations: maximum number of jacobian iterations allowed per each initial configuration
		//
		//	output parameters:
		//    thumb_results.configuration: 13 dimension sahand configuration vector as
		//      config(0) = thumbrot
		//		config(1) = thumb abduction		config(2) = first thumb flexion		config(3) = second thumb flexion	
		//		config(4) = index abduction		config(5) = first index flexion		config(6) = second index flexion
		//		config(7) = middle abduction	config(8) = first middle flexion	config(9) = second middle flexion
		//		config(10) = ring abduction		config(11) = first ring flexion		config(12) = second ring flexion
		//
		//	  thumb_results.configuration_list: matrix with a 13 dimension vector with a sahand configuration. 
		//		Each row has the same data structure as thumb_results.configuration
		//
		//	  thumb_results.ik_solved: variable containing 'true' if inverse kinematics has been solved. Otherwise, 'false'.
		//
		// ******************************************************************************************************************
	
	
		// ----- Asserting input variables -----
		//		finger_positions
		assert(fingertip_positions.size() == 18);	// Checking size
		//		initial_configuration
		assert(initial_configuration.size() == N_TA_VDC_CONFIG);	
		// ----- End Asserting input variables -----
	
		// Configuration and configuration increment during iterations
		boost::numeric::ublas::vector<double> jac_configuration(N_TA_VDC_CONFIG), inc_jac_conf(jac_configuration.size());
	
		// Desired finger position and finger position error
		boost::numeric::ublas::vector<double> desired_position, position_error;
		
		// ----- Starting jacobian pseudoinverse algorithm -----
		// Clearing variables where to store data
		ik_results_out.configuration.clear();
	
		ik_results.configuration_list.resize(max_jacobian_iterations+1,N_TA_VDC_CONFIG,false);
		ik_results.configuration_list.clear();
		
		error_increment_list.clear();
	
		// First configuration: initial configuration
		jac_configuration = initial_configuration;
		desired_position = fingertip_positions;
	
		unsigned int fingers_stop = 0;
		unsigned int n_jac_it = 0;
	
		// Storing first configuration to configuration list
		for (unsigned int i=0; i<N_TA_VDC_CONFIG; i++)	ik_results.configuration_list(n_jac_it,i) = jac_configuration(i);
	
		// Jacobian and end effector position calculation
		JacobianAndThumbDirKin(jac_configuration);
	
		// Position error
		position_error = desired_position - effective_position;
	
		double max_finger_dist;
	
		ik_results.ik_solved = false;
		do{
			PosErrorAdjust(position_error);
		
			inc_jac_conf = ObtainJointIncrement(jac_configuration, jacobian, position_error);
	
			// Calculating new configuration
			jac_configuration += inc_jac_conf;
	
			// Adjusting joint limits and storing configuration
			AdjustJointLimits(jac_configuration);
	
			// Storing configuration to configuration list
			for (unsigned int i=0; i<N_TA_VDC_CONFIG; i++)	ik_results.configuration_list(n_jac_it+1,i) = jac_configuration(i);
	
			// Jacobian and end effector position calculation
			JacobianAndThumbDirKin(jac_configuration);
	
			// Position error
			position_error = desired_position - effective_position;
	
			// Adjusting error and checking if fingers are at pose or stopped
			fingers_stop = FingerDistEvolution(position_error, &max_finger_dist);
			
			n_jac_it++;
		} while ( (!fingers_stop) && (n_jac_it<max_jacobian_iterations) );
		ik_results.configuration_list.resize(n_jac_it+1,N_TA_VDC_CONFIG,true);
		ik_results.configuration = jac_configuration;
	
		// Storing last configuration as the good one if it satisfies the constraints of the problem
		unsigned int collision = 2;
		if (fingers_stop == 1){
			collision = 1;
			if (!CollisionCheck(jac_configuration)){
				ik_results.ik_solved = true;
				collision = 0;
			}
		}
	
	
		// Iteration data
		unsigned int nl = ik_results.it_data.size1()-1;
		ik_results.it_data(nl,0) = n_jac_it;
		ik_results.it_data(nl,1) = fingers_stop;
		ik_results.it_data(nl,2) = collision;
		unsigned int done = 0;
		for (unsigned int i=0; (i<error_increment_list.size2()) && (!done); i++){
			if (error_increment_list(0,i) == 0){
				ik_results.it_data(nl,3) = error_increment_list(0,i-1);
				done = 1;
			}
		}
	
		if (ik_results.it_data(nl,3) == 0)
			ik_results.it_data(nl,3) = error_increment_list(0,error_increment_list.size2()-1);
		ik_results.it_data(nl,4) = max_finger_dist;
	
		ik_results_out = ik_results;
	
	
		return;
	}
	
	inline void IvKinHand::PosErrorAdjust(boost::numeric::ublas::vector<double> &position_error){
		// *************************************************************************************************************************
		// Function that checks that the position error's lenght of position_error is valid for each finger. Otherwise it adjusts
		//	it in its valid length.
		//
		//   position_error: vector with the position error of end of finger and inside fingertips
		//
		// *************************************************************************************************************************
	
	
		// --- CHECKING AND FIXING ERRORS POSITION LENGHT ---
		double finger_error_1, finger_error_2;
	
		// Test distance
		for (unsigned int i=0; i<N_POS_FING_THUMB/6; i++){
	
			//   Finger error norm - end of finger
			finger_error_1 = sqrt(pow(position_error(6*i),2) + pow(position_error(6*i+1),2) + pow(position_error(6*i+2),2));
			if (finger_error_1 > max_error_dist)
				for (unsigned int j=0; j<3; j++)
					position_error(6*i+j) = max_error_dist*(position_error(6*i+j)/finger_error_1);
	
			//   Inside of fingertip
			finger_error_2 = sqrt(pow(position_error(6*i+3),2) + pow(position_error(6*i+4),2) + pow(position_error(6*i+5),2));
			if (finger_error_2 > max_error_dist)
				for (unsigned int j=0; j<3; j++)
					position_error(6*i+j+3) = max_error_dist*(position_error(6*i+j+3)/finger_error_2);
		}
	
		return;
	}
	
	inline boost::numeric::ublas::vector<double> IvKinHand::ObtainJointIncrement(const boost::numeric::ublas::vector<double> &configuration,
									const boost::numeric::ublas::matrix<double> &jacobian,
									const boost::numeric::ublas::vector<double> &position_error){
		// *************************************************************************************************************************
		// Function that calculates the direct kinematics and the jacobian for a certain thumb configuration
		//
		//	Input variables:
		//      configuration(0) = hand rotation around thumb axis
		//      configuration(1) = thumbrot
		//		configuration(2) = thumb abduction			configuration(3) = first thumb flexion			configuration(4) = second thumb flexion
		//		configuration(5) = third thumb flexion		configuration(6) = first thumb fingertip		configuration(7) = second thumb fingertip
		//		configuration(8) = index abduction			configuration(9) = first index flexion			configuration(10) = second index flexion
		//		configuration(11) = third index flexion		configuration(12) = first index fingertip		configuration(13) = second index fingertip
		//		configuration(14) = middle abduction		configuration(15) = first middle flexion		configuration(16) = second middle flexion
		//		configuration(17) = third middle flexion	configuration(18) = first middle fingertip		configuration(19) = second middle fingertip
		//		configuration(20) = ring abduction			configuration(21) = first ring flexion			configuration(22) = second ring flexion
		//		configuration(23) = third ring flexion		configuration(24) = first ring fingertip		configuration(25) = second ring fingertip
		//
		//	  jacobian: a 22x26 matrix. First 18x26 submatrix is the jacobian matrix. Second 4x26 matrix relates to the 
		//	   coupling of angles of the sahand.
		//
		//    position_error:
		//	   vector of dimension 18 with the information of the finger and fingertip positions 
		//	    referenced from the thumb base as
		//		effective_position(0:2) = Index desired position		effective_position(3:5) = Index desired inside fingertip position
		//		effective_position(6:8) = Middle desired position		effective_position(9:11) = Middle desired inside fingertip position
		//		effective_position(12:14) = Ring desired position		effective_position(15:17) = Ring desired inside fingertip position
		//
		//	Output variables:
		//
		//    inc_ang:
		//	   vector of dimension 26 with the information of the increment in joint angles, as in configuration. 
		//
		// *************************************************************************************************************************
	
	
		// ----- Asserting input variables -----
		assert(configuration.size() == N_TA_VDC_CONFIG);
		assert((jacobian.size1() == N_POS_FING_THUMB+4) && (jacobian.size2() == N_TA_VDC_CONFIG));
		assert(position_error.size() == N_POS_FING_THUMB);
		// ----- End Asserting input variables -----
	
	
		// Calculating jacobian pseudoinverse
		boost::numeric::ublas::matrix<double> jacobian_pseudoinv;
		jacobian_pseudoinv = pinv(jacobian);
	
		// Extending position error to multiply it with pseudoinverse
		boost::numeric::ublas::vector<double> position_error_ext(N_POS_FING_THUMB+4);		position_error_ext.clear();
		for (unsigned int i=0; i<position_error.size(); i++)		position_error_ext(i) = position_error(i);
	
		// Calculating joint angle increments
		boost::numeric::ublas::vector<double> inc_ang(jacobian.size2());
		inc_ang = prod(jacobian_pseudoinv, position_error_ext);
	
		return inc_ang;
	}
	
	inline void IvKinHand::AdjustJointLimits(boost::numeric::ublas::vector<double> &configuration){
		// *************************************************************************************************************************
		// Function that checks if a joint is inside its valid range; otherwise it sets is value to its nearest joint limit. 
		//	Function also checks if coupled joints have the same value; otherwise, second coupled joint is set to its first coupled
		//	joint.
		//
		//	Input variables:
		//      configuration(0) = hand rotation around thumb axis
		//      configuration(1) = thumbrot
		//		configuration(2) = thumb abduction			configuration(3) = first thumb flexion			configuration(4) = second thumb flexion
		//		configuration(5) = third thumb flexion		configuration(6) = first thumb fingertip		configuration(7) = second thumb fingertip
		//		configuration(8) = index abduction			configuration(9) = first index flexion			configuration(10) = second index flexion
		//		configuration(11) = third index flexion		configuration(12) = first index fingertip		configuration(13) = second index fingertip
		//		configuration(14) = middle abduction		configuration(15) = first middle flexion		configuration(16) = second middle flexion
		//		configuration(17) = third middle flexion	configuration(18) = first middle fingertip		configuration(19) = second middle fingertip
		//		configuration(20) = ring abduction			configuration(21) = first ring flexion			configuration(22) = second ring flexion
		//		configuration(23) = third ring flexion		configuration(24) = first ring fingertip		configuration(25) = second ring fingertip
		//
		// *************************************************************************************************************************
	
	
		// ----- Asserting input variables -----
		assert((configuration.size() == N_TA_VDC_CONFIG) || (configuration.size() == N_VDC_CONFIG));	// Checking size
		// ----- End Asserting input variables -----
	
	
		double joint_value, min_joint_value, max_joint_value;
	
		if (configuration.size() == N_VDC_CONFIG){
			for(unsigned int i=0; i<N_VDC_CONFIG; i++){
				joint_value = configuration(i);
				min_joint_value = joint_limits(i,0);
				max_joint_value = joint_limits(i,1);
	
				if (joint_value<min_joint_value)
					configuration(i) = min_joint_value;
				else if (joint_value>max_joint_value)
					configuration(i) = max_joint_value;
			}
	
			// Coupling
			configuration(4) = configuration(3);
			configuration(10) = configuration(9);
			configuration(16) = configuration(15);
			configuration(22) = configuration(21);
		}
		else if (configuration.size() == N_TA_VDC_CONFIG){
			for(unsigned int i=0; i<N_TA_VDC_CONFIG-1; i++){
				joint_value = configuration(i+1);
				min_joint_value = joint_limits(i,0);
				max_joint_value = joint_limits(i,1);
	
				if (joint_value<min_joint_value)
					configuration(i+1) = min_joint_value;
				else if (joint_value>max_joint_value)
					configuration(i+1) = max_joint_value;
			}
	
			// Coupling
			configuration(5) = configuration(4);
			configuration(11) = configuration(10);
			configuration(17) = configuration(16);
			configuration(23) = configuration(22);
		}
	
		return;
	}
	
	inline unsigned int IvKinHand::FingerDistEvolution(const boost::numeric::ublas::vector<double> &position_error, double *max_finger_error){
		// *************************************************************************************************************************
		// Function checks, using the error, if the jacobian iteration is over. This is done by checking that each finger is on
		//  desired position under a certain tolerance especified in error_precision; in this case a valid configuration of the hand
		//  that satisfies the constraints of the problem is found. Otherwise the function checks the slope of the error; if its 
		//	value is under the value in error_precision, the hand is assumed to be stopped.
		//
		//	Input variables:
		//   position_error: vector of dimension 18 with the information related to desired finger
		//	  positions and orientations referenced from the thumb base as
		//		finger_positions(0:2) = Index desired position		finger_positions(3:5) = Index desired orientatio
		//		finger_positions(6:8) = Middle desired position		finger_positions(9:11) = Middle desired orientatio
		//		finger_positions(12:14) = Ring desired position		finger_positions(15:17) = Ring desired orientatio
		//
		//	Output variables:
		//	 hand_at_pos: if 0, hand is not at pose. 1, it is at pose. 2, it is not at pose but it is stopped.
		//
		// *************************************************************************************************************************
	
	
		// ----- Asserting input variables -----
		//		position_error
		assert(position_error.size() == N_POS_FING_THUMB);	// Checking size
		// ----- End Asserting input variables -----
	
	
		// --- CHECKING AND FIXING ERRORS POSITION LENGHT ---
		double error = 0;
		double finger_error_1, finger_error_2, max_finger_dist;
		unsigned int hand_at_pos = 1;			// Suposition: Hand is at position
	
		// Test distance
		max_finger_dist = 0;
		for (unsigned int i=0; i<N_POS_FING_THUMB/6; i++){
	
			// -- Adjusting finger and fingertip length
			//   Finger error norm - end of finger
			finger_error_1 = sqrt(pow(position_error(6*i),2) + pow(position_error(6*i+1),2) + pow(position_error(6*i+2),2));
			
			//   Inside of fingertip
			finger_error_2 = sqrt(pow(position_error(6*i+3),2) + pow(position_error(6*i+4),2) + pow(position_error(6*i+5),2));
	
			if (finger_error_1 > max_finger_dist)		max_finger_dist = finger_error_1;
			if (finger_error_2 > max_finger_dist)		max_finger_dist = finger_error_2;
	
			// -- Checking hand at pose
			if ( (finger_error_1 > error_precision) || (finger_error_2 > error_precision) )     hand_at_pos = 0;
	
			error += finger_error_1 + finger_error_2;
		}
		*max_finger_error = max_finger_dist;
	
		// -- Checking slope
		if (!hand_at_pos){
			hand_at_pos = 2;				// Suposition: Hand is not at position but is stopped
	
			// Search for first component in error_increment_list that its value is 0, so the new error can be stored
			unsigned int n_it = 0;
			while ((n_it < error_increment_list.size2()) && (error_increment_list(0,n_it) != 0))	n_it++;
			//  If all components in slope list are nonzero, the new error is stored in last list position 
			if (n_it == error_increment_list.size2())	n_it--;
	
			// Storing error
			error_increment_list(0,n_it) = error;
	
			// Storing slope
			if (n_it == 0)  error_increment_list(1,0) = 100;		// First slope is 100
			else            error_increment_list(1,n_it) = fabs(error_increment_list(0,n_it-1)-error);
	
			// -- Checking if hand is stopped
			if (n_it < max_slope_samples-1){
				hand_at_pos = 0;	// Hand is not at position nor stopped
			}
			else {
				for (unsigned int i=(n_it-max_slope_samples+1); (i<n_it) && (hand_at_pos == 2); i++){
					if (error_increment_list(1,i) > slope_precision ){
						hand_at_pos = 0;	// Hand is not at position nor stopped
					}
				}
			}
		}
	
		return hand_at_pos;
	}
	
	
	boost::numeric::ublas::vector<double> IvKinHand::GenerateRandFingPosOr(boost::numeric::ublas::vector<double> &twrist,
										boost::numeric::ublas::vector<double> &wconfig){
	
		// Generating random configuration
		boost::numeric::ublas::vector<double> config(N_VDC_CONFIG), config_a(N_TA_VDC_CONFIG);
		do{
			for (unsigned int i=0; i<N_VDC_CONFIG; i++)
				config(i) = joint_limits(i,0) + (joint_limits(i,1) - joint_limits(i,0))*((double)rand()/(double)RAND_MAX);
	
			config(4) = config(3);
			config(10) = config(9);
			config(16) = config(15);
			config(22) = config(21);
	
			config_a(0) = 0;
			for (unsigned int i=0; i<N_VDC_CONFIG; i++)		config_a(i+1) = config(i);
			
		} while (CollisionCheck(config_a));
	
		wconfig.resize(config.size(),false);
		wconfig = config;
	
		//finger_positions = ConfigToPosOr(config);
		boost::numeric::ublas::vector<double> desired_pos_or(24);
		boost::numeric::ublas::matrix<double> wdk, wrist_base(4,4);
	
		wrist_base.clear();
		for (unsigned int i=0; i<4; i++)	wrist_base(i,i) = 1;
	
		wdk = WristDirKin(wrist_base, config);
	
		// Storing positions and orientations
		for (unsigned int i=0; i<4; i++){
			for (unsigned int j=0; j<3; j++){
				desired_pos_or(6*i+j) = wdk(j,8+7*i);		// Positions
				desired_pos_or(6*i+j+3) = wdk(j+3,8+7*i);	// Orientations - X axis
			}
		}
		
		twrist.resize(16, false);	twrist.clear();
		for (unsigned int i=0; i<4; i++)	twrist(5*i) = 1;
		
	
		return desired_pos_or;
	}
	
	inline unsigned int IvKinHand::CollisionCheck(const boost::numeric::ublas::vector<double> &config_in){
		// *************************************************************************************************************************
		// Function that checks which if a certain configuration of the hand has collision between its components
		//
		//	Input variables:
		//	  configuration:
		//		configuration(0) = hand rotation around thumb axis
		//      configuration(1) = thumbrot
		//		configuration(2) = thumb abduction			configuration(3) = first thumb flexion			configuration(4) = second thumb flexion
		//		configuration(5) = third thumb flexion		configuration(6) = first thumb fingertip		configuration(7) = second thumb fingertip
		//		configuration(8) = index abduction			configuration(9) = first index flexion			configuration(10) = second index flexion
		//		configuration(11) = third index flexion		configuration(12) = first index fingertip		configuration(13) = second index fingertip
		//		configuration(14) = middle abduction		configuration(15) = first middle flexion		configuration(16) = second middle flexion
		//		configuration(17) = third middle flexion	configuration(18) = first middle fingertip		configuration(19) = second middle fingertip
		//		configuration(20) = ring abduction			configuration(21) = first ring flexion			configuration(22) = second ring flexion
		//		configuration(23) = third ring flexion		configuration(24) = first ring fingertip		configuration(25) = second ring fingertip
		//
		// *************************************************************************************************************************
	
	
		// ----- Assertions -----
		assert(config_in.size() == N_TA_VDC_CONFIG);
		// ----- End Assertions -----
	
		boost::numeric::ublas::vector<double> configw(N_VDC_CONFIG);
		for (unsigned int i=0; i<configw.size(); i++)	configw(i) = config_in(i+1);
	
		// *************************************************************
		// Set config objects
		// *************************************************************	
	
		// Direct kinematics
	// 	boost::numeric::ublas::matrix<double> wrist_base(4,4);	wrist_base.clear();
	// 	for (unsigned int i = 0; i < 4; i++)	wrist_base(i,i) = 1;
	// 	boost::numeric::ublas::matrix<double> tdk;
	// 	tdk = WristDirKin(wrist_base, configw);
	// 
	// 	std::vector<configuration> config;
	// 	configuration cf;
	// 
	// 	boost::numeric::ublas::vector<double> po(3), pf(3), vn(3), ey(3), eix_gir(3), v_x(3), v_y(3), v_z(3);
	// 	boost::numeric::ublas::matrix<double> base(3,3);
	// 	double ang_gir;
	// 	mt::Scalar angle;		mt::Unit3 axis(1,0,0);	
	// 
	// 	// Thumb 
	// 	//  Rotational
	// 	//for (unsigned int k = 0; k < 3; k++)	po(k) = tdk(k,1);
	// 	//for (unsigned int k = 0; k < 3; k++){	v_x(k) = tdk(k+3,1);	v_z(k) = tdk(k+6,1);	}	v_y = cross_p(v_z, v_x);
	// 	//for (unsigned int k = 0; k < 3; k++){	base(k,0) = v_x(k);		base(k,1) = v_y(k);		base(k,2) = v_z(k);	}
	// 	//mt::Matrix3x3 mtrot(base(0,0), base(0,1), base(0,2), base(1,0), base(1,1), base(1,2), base(2,0), base(2,1), base(2,2));
	// 	//mt::Rotation rtrot(mtrot);	rtrot.getAxisAngle(axis,angle);
	// 	//for (unsigned int k = 0; k < 3; k++)	eix_gir(k) = axis[k];	ang_gir = angle;
	// 	//for (unsigned int k = 0; k < 3; k++){	cf.c[k] = po(k);	cf.c[k+3] = eix_gir(k);	}	cf.c[6] = ang_gir;
	// 	//config.push_back(cf);
	// 
	// 	//  Base
	// 	for (unsigned int k = 0; k < 3; k++)	po(k) = tdk(k,2);
	// 	for (unsigned int k = 0; k < 3; k++){	v_x(k) = tdk(k+3,2);	v_z(k) = tdk(k+6,2);	}	v_y = cross_p(v_z, v_x);
	// 	for (unsigned int k = 0; k < 3; k++){	base(k,0) = v_x(k);		base(k,1) = v_y(k);		base(k,2) = v_z(k);	}
	// 	mt::Matrix3x3 mtb(base(0,0), base(0,1), base(0,2), base(1,0), base(1,1), base(1,2), base(2,0), base(2,1), base(2,2));
	// 	mt::Rotation rtb(mtb);	rtb.getAxisAngle(axis,angle);
	// 	for (unsigned int k = 0; k < 3; k++)	eix_gir(k) = axis[k];	ang_gir = angle;
	// 	for (unsigned int k = 0; k < 3; k++){	cf.c[k] = po(k);	cf.c[k+3] = eix_gir(k);	}	cf.c[6] = ang_gir;
	// 	config.push_back(cf);
	// 
	// 	//  Proximal
	// 	for (unsigned int k = 0; k < 3; k++)	po(k) = tdk(k,3);
	// 	for (unsigned int k = 0; k < 3; k++){	v_x(k) = tdk(k+3,3);	v_z(k) = -tdk(k+6,3);	}	v_y = cross_p(v_z, v_x);
	// 	for (unsigned int k = 0; k < 3; k++){	base(k,0) = v_x(k);		base(k,1) = v_y(k);		base(k,2) = v_z(k);	}
	// 	mt::Matrix3x3 mtp(base(0,0), base(0,1), base(0,2), base(1,0), base(1,1), base(1,2), base(2,0), base(2,1), base(2,2));
	// 	mt::Rotation rtp(mtp);	rtp.getAxisAngle(axis,angle);
	// 	for (unsigned int k = 0; k < 3; k++)	eix_gir(k) = axis[k];	ang_gir = angle;
	// 	for (unsigned int k = 0; k < 3; k++){	cf.c[k] = po(k);	cf.c[k+3] = eix_gir(k);	}	cf.c[6] = ang_gir;
	// 	config.push_back(cf);
	// 
	// 	//  Intermediate
	// 	for (unsigned int k = 0; k < 3; k++)	po(k) = tdk(k,4);
	// 	for (unsigned int k = 0; k < 3; k++){	v_x(k) = tdk(k+3,4);	v_z(k) = -tdk(k+6,4);	}	v_y = cross_p(v_z, v_x);
	// 	for (unsigned int k = 0; k < 3; k++){	base(k,0) = v_x(k);		base(k,1) = v_y(k);		base(k,2) = v_z(k);	}
	// 	mt::Matrix3x3 mtm(base(0,0), base(0,1), base(0,2), base(1,0), base(1,1), base(1,2), base(2,0), base(2,1), base(2,2));
	// 	mt::Rotation rtm(mtm);	rtm.getAxisAngle(axis,angle);
	// 	for (unsigned int k = 0; k < 3; k++)	eix_gir(k) = axis[k];	ang_gir = angle;
	// 	for (unsigned int k = 0; k < 3; k++){	cf.c[k] = po(k);	cf.c[k+3] = eix_gir(k);	}	cf.c[6] = ang_gir;
	// 	config.push_back(cf);
	// 
	// 	//  Distal
	// 	for (unsigned int k = 0; k < 3; k++)	po(k) = tdk(k,5);
	// 	for (unsigned int k = 0; k < 3; k++){	v_x(k) = tdk(k+3,5);	v_z(k) = -tdk(k+6,5);	}	v_y = cross_p(v_z, v_x);
	// 	v_y = v_x;	v_x = cross_p(v_y, v_z);
	// 	for (unsigned int k = 0; k < 3; k++){	base(k,0) = v_x(k);		base(k,1) = v_y(k);		base(k,2) = v_z(k);	}
	// 	mt::Matrix3x3 mtd(base(0,0), base(0,1), base(0,2), base(1,0), base(1,1), base(1,2), base(2,0), base(2,1), base(2,2));
	// 	mt::Rotation rtd(mtd);	rtd.getAxisAngle(axis,angle);
	// 	for (unsigned int k = 0; k < 3; k++)	eix_gir(k) = axis[k];	ang_gir = angle;
	// 	for (unsigned int k = 0; k < 3; k++){	cf.c[k] = po(k);	cf.c[k+3] = eix_gir(k);	}	cf.c[6] = ang_gir;
	// 	config.push_back(cf);
	// 
	// 	// Index 
	// 	//  Base
	// 	for (unsigned int k = 0; k < 3; k++)	po(k) = tdk(k,9);
	// 	for (unsigned int k = 0; k < 3; k++){	v_x(k) = tdk(k+3,9);	v_z(k) = tdk(k+6,9);	}	v_y = cross_p(v_z, v_x);
	// 	for (unsigned int k = 0; k < 3; k++){	base(k,0) = v_x(k);		base(k,1) = v_y(k);		base(k,2) = v_z(k);	}
	// 	mt::Matrix3x3 mib(base(0,0), base(0,1), base(0,2), base(1,0), base(1,1), base(1,2), base(2,0), base(2,1), base(2,2));
	// 	mt::Rotation rib(mib);	rib.getAxisAngle(axis,angle);
	// 	for (unsigned int k = 0; k < 3; k++)	eix_gir(k) = axis[k];	ang_gir = angle;
	// 	for (unsigned int k = 0; k < 3; k++){	cf.c[k] = po(k);	cf.c[k+3] = eix_gir(k);	}	cf.c[6] = ang_gir;
	// 	config.push_back(cf);
	// 
	// 	//  Proximal
	// 	for (unsigned int k = 0; k < 3; k++)	po(k) = tdk(k,10);
	// 	for (unsigned int k = 0; k < 3; k++){	v_x(k) = tdk(k+3,10);	v_z(k) = -tdk(k+6,10);	}	v_y = cross_p(v_z, v_x);
	// 	for (unsigned int k = 0; k < 3; k++){	base(k,0) = v_x(k);		base(k,1) = v_y(k);		base(k,2) = v_z(k);	}
	// 	mt::Matrix3x3 mip(base(0,0), base(0,1), base(0,2), base(1,0), base(1,1), base(1,2), base(2,0), base(2,1), base(2,2));
	// 	mt::Rotation rip(mip);	rip.getAxisAngle(axis,angle);
	// 	for (unsigned int k = 0; k < 3; k++)	eix_gir(k) = axis[k];	ang_gir = angle;
	// 	for (unsigned int k = 0; k < 3; k++){	cf.c[k] = po(k);	cf.c[k+3] = eix_gir(k);	}	cf.c[6] = ang_gir;
	// 	config.push_back(cf);
	// 
	// 	//  Intermediate
	// 	for (unsigned int k = 0; k < 3; k++)	po(k) = tdk(k,11);
	// 	for (unsigned int k = 0; k < 3; k++){	v_x(k) = tdk(k+3,11);	v_z(k) = -tdk(k+6,11);	}	v_y = cross_p(v_z, v_x);
	// 	for (unsigned int k = 0; k < 3; k++){	base(k,0) = v_x(k);		base(k,1) = v_y(k);		base(k,2) = v_z(k);	}
	// 	mt::Matrix3x3 mim(base(0,0), base(0,1), base(0,2), base(1,0), base(1,1), base(1,2), base(2,0), base(2,1), base(2,2));
	// 	mt::Rotation rim(mim);	rim.getAxisAngle(axis,angle);
	// 	for (unsigned int k = 0; k < 3; k++)	eix_gir(k) = axis[k];	ang_gir = angle;
	// 	for (unsigned int k = 0; k < 3; k++){	cf.c[k] = po(k);	cf.c[k+3] = eix_gir(k);	}	cf.c[6] = ang_gir;
	// 	config.push_back(cf);
	// 
	// 	//  Distal
	// 	for (unsigned int k = 0; k < 3; k++)	po(k) = tdk(k,12);
	// 	for (unsigned int k = 0; k < 3; k++){	v_x(k) = tdk(k+3,12);	v_z(k) = -tdk(k+6,12);	}	v_y = cross_p(v_z, v_x);
	// 	v_y = v_x;	v_x = cross_p(v_y, v_z);
	// 	for (unsigned int k = 0; k < 3; k++){	base(k,0) = v_x(k);		base(k,1) = v_y(k);		base(k,2) = v_z(k);	}
	// 	mt::Matrix3x3 mid(base(0,0), base(0,1), base(0,2), base(1,0), base(1,1), base(1,2), base(2,0), base(2,1), base(2,2));
	// 	mt::Rotation rid(mid);	rid.getAxisAngle(axis,angle);
	// 	for (unsigned int k = 0; k < 3; k++)	eix_gir(k) = axis[k];	ang_gir = angle;
	// 	for (unsigned int k = 0; k < 3; k++){	cf.c[k] = po(k);	cf.c[k+3] = eix_gir(k);	}	cf.c[6] = ang_gir;
	// 	config.push_back(cf);
	// 
	// 
	// 	// Middle 
	// 	//  Base
	// 	for (unsigned int k = 0; k < 3; k++)	po(k) = tdk(k,16);
	// 	for (unsigned int k = 0; k < 3; k++){	v_x(k) = tdk(k+3,16);	v_z(k) = tdk(k+6,16);	}	v_y = cross_p(v_z, v_x);
	// 	for (unsigned int k = 0; k < 3; k++){	base(k,0) = v_x(k);		base(k,1) = v_y(k);		base(k,2) = v_z(k);	}
	// 	mt::Matrix3x3 mmb(base(0,0), base(0,1), base(0,2), base(1,0), base(1,1), base(1,2), base(2,0), base(2,1), base(2,2));
	// 	mt::Rotation rmb(mmb);	rmb.getAxisAngle(axis,angle);
	// 	for (unsigned int k = 0; k < 3; k++)	eix_gir(k) = axis[k];	ang_gir = angle;
	// 	for (unsigned int k = 0; k < 3; k++){	cf.c[k] = po(k);	cf.c[k+3] = eix_gir(k);	}	cf.c[6] = ang_gir;
	// 	config.push_back(cf);
	// 
	// 	//  Proximal
	// 	for (unsigned int k = 0; k < 3; k++)	po(k) = tdk(k,17);
	// 	for (unsigned int k = 0; k < 3; k++){	v_x(k) = tdk(k+3,17);	v_z(k) = -tdk(k+6,17);	}	v_y = cross_p(v_z, v_x);
	// 	for (unsigned int k = 0; k < 3; k++){	base(k,0) = v_x(k);		base(k,1) = v_y(k);		base(k,2) = v_z(k);	}
	// 	mt::Matrix3x3 mmp(base(0,0), base(0,1), base(0,2), base(1,0), base(1,1), base(1,2), base(2,0), base(2,1), base(2,2));
	// 	mt::Rotation rmp(mmp);	rmp.getAxisAngle(axis,angle);
	// 	for (unsigned int k = 0; k < 3; k++)	eix_gir(k) = axis[k];	ang_gir = angle;
	// 	for (unsigned int k = 0; k < 3; k++){	cf.c[k] = po(k);	cf.c[k+3] = eix_gir(k);	}	cf.c[6] = ang_gir;
	// 	config.push_back(cf);
	// 
	// 	//  Intermediate
	// 	for (unsigned int k = 0; k < 3; k++)	po(k) = tdk(k,18);
	// 	for (unsigned int k = 0; k < 3; k++){	v_x(k) = tdk(k+3,18);	v_z(k) = -tdk(k+6,18);	}	v_y = cross_p(v_z, v_x);
	// 	for (unsigned int k = 0; k < 3; k++){	base(k,0) = v_x(k);		base(k,1) = v_y(k);		base(k,2) = v_z(k);	}
	// 	mt::Matrix3x3 mmm(base(0,0), base(0,1), base(0,2), base(1,0), base(1,1), base(1,2), base(2,0), base(2,1), base(2,2));
	// 	mt::Rotation rmm(mmm);	rmm.getAxisAngle(axis,angle);
	// 	for (unsigned int k = 0; k < 3; k++)	eix_gir(k) = axis[k];	ang_gir = angle;
	// 	for (unsigned int k = 0; k < 3; k++){	cf.c[k] = po(k);	cf.c[k+3] = eix_gir(k);	}	cf.c[6] = ang_gir;
	// 	config.push_back(cf);
	// 
	// 	//  Distal
	// 	for (unsigned int k = 0; k < 3; k++)	po(k) = tdk(k,19);
	// 	for (unsigned int k = 0; k < 3; k++){	v_x(k) = tdk(k+3,19);	v_z(k) = -tdk(k+6,19);	}	v_y = cross_p(v_z, v_x);
	// 	v_y = v_x;	v_x = cross_p(v_y, v_z);
	// 	for (unsigned int k = 0; k < 3; k++){	base(k,0) = v_x(k);		base(k,1) = v_y(k);		base(k,2) = v_z(k);	}
	// 	mt::Matrix3x3 mmd(base(0,0), base(0,1), base(0,2), base(1,0), base(1,1), base(1,2), base(2,0), base(2,1), base(2,2));
	// 	mt::Rotation rmd(mmd);	rmd.getAxisAngle(axis,angle);
	// 	for (unsigned int k = 0; k < 3; k++)	eix_gir(k) = axis[k];	ang_gir = angle;
	// 	for (unsigned int k = 0; k < 3; k++){	cf.c[k] = po(k);	cf.c[k+3] = eix_gir(k);	}	cf.c[6] = ang_gir;
	// 	config.push_back(cf);
	// 
	// 	// Ring 
	// 	//  Base
	// 	for (unsigned int k = 0; k < 3; k++)	po(k) = tdk(k,23);
	// 	for (unsigned int k = 0; k < 3; k++){	v_x(k) = tdk(k+3,23);	v_z(k) = tdk(k+6,23);	}	v_y = cross_p(v_z, v_x);
	// 	for (unsigned int k = 0; k < 3; k++){	base(k,0) = v_x(k);		base(k,1) = v_y(k);		base(k,2) = v_z(k);	}
	// 	mt::Matrix3x3 mrb(base(0,0), base(0,1), base(0,2), base(1,0), base(1,1), base(1,2), base(2,0), base(2,1), base(2,2));
	// 	mt::Rotation rrb(mrb);	rrb.getAxisAngle(axis,angle);
	// 	for (unsigned int k = 0; k < 3; k++)	eix_gir(k) = axis[k];	ang_gir = angle;
	// 	for (unsigned int k = 0; k < 3; k++){	cf.c[k] = po(k);	cf.c[k+3] = eix_gir(k);	}	cf.c[6] = ang_gir;
	// 	config.push_back(cf);
	// 
	// 	//  Proximal
	// 	for (unsigned int k = 0; k < 3; k++)	po(k) = tdk(k,24);
	// 	for (unsigned int k = 0; k < 3; k++){	v_x(k) = tdk(k+3,24);	v_z(k) = -tdk(k+6,24);	}	v_y = cross_p(v_z, v_x);
	// 	for (unsigned int k = 0; k < 3; k++){	base(k,0) = v_x(k);		base(k,1) = v_y(k);		base(k,2) = v_z(k);	}
	// 	mt::Matrix3x3 mrp(base(0,0), base(0,1), base(0,2), base(1,0), base(1,1), base(1,2), base(2,0), base(2,1), base(2,2));
	// 	mt::Rotation rrp(mrp);	rrp.getAxisAngle(axis,angle);
	// 	for (unsigned int k = 0; k < 3; k++)	eix_gir(k) = axis[k];	ang_gir = angle;
	// 	for (unsigned int k = 0; k < 3; k++){	cf.c[k] = po(k);	cf.c[k+3] = eix_gir(k);	}	cf.c[6] = ang_gir;
	// 	config.push_back(cf);
	// 
	// 	//  Intermediate
	// 	for (unsigned int k = 0; k < 3; k++)	po(k) = tdk(k,25);
	// 	for (unsigned int k = 0; k < 3; k++){	v_x(k) = tdk(k+3,25);	v_z(k) = -tdk(k+6,25);	}	v_y = cross_p(v_z, v_x);
	// 	for (unsigned int k = 0; k < 3; k++){	base(k,0) = v_x(k);		base(k,1) = v_y(k);		base(k,2) = v_z(k);	}
	// 	mt::Matrix3x3 mrm(base(0,0), base(0,1), base(0,2), base(1,0), base(1,1), base(1,2), base(2,0), base(2,1), base(2,2));
	// 	mt::Rotation rrm(mrm);	rrm.getAxisAngle(axis,angle);
	// 	for (unsigned int k = 0; k < 3; k++)	eix_gir(k) = axis[k];	ang_gir = angle;
	// 	for (unsigned int k = 0; k < 3; k++){	cf.c[k] = po(k);	cf.c[k+3] = eix_gir(k);	}	cf.c[6] = ang_gir;
	// 	config.push_back(cf);
	// 
	// 	//  Distal
	// 	for (unsigned int k = 0; k < 3; k++)	po(k) = tdk(k,26);
	// 	for (unsigned int k = 0; k < 3; k++){	v_x(k) = tdk(k+3,26);	v_z(k) = -tdk(k+6,26);	}	v_y = cross_p(v_z, v_x);
	// 	v_y = v_x;	v_x = cross_p(v_y, v_z);
	// 	for (unsigned int k = 0; k < 3; k++){	base(k,0) = v_x(k);		base(k,1) = v_y(k);		base(k,2) = v_z(k);	}
	// 	mt::Matrix3x3 mrd(base(0,0), base(0,1), base(0,2), base(1,0), base(1,1), base(1,2), base(2,0), base(2,1), base(2,2));
	// 	mt::Rotation rrd(mrd);	rrd.getAxisAngle(axis,angle);
	// 	for (unsigned int k = 0; k < 3; k++)	eix_gir(k) = axis[k];	ang_gir = angle;
	// 	for (unsigned int k = 0; k < 3; k++){	cf.c[k] = po(k);	cf.c[k+3] = eix_gir(k);	}	cf.c[6] = ang_gir;
	// 	config.push_back(cf);
	// 
	// 
	// 	// Palm ----------------------------------------
	// 	for (unsigned int k = 0; k < 3; k++)	po(k) = tdk(k,0);
	// 	for (unsigned int k = 0; k < 3; k++){	v_x(k) = tdk(k+3,0);	v_z(k) = tdk(k+6,0);	}	v_y = cross_p(v_z, v_x);
	// 	for (unsigned int k = 0; k < 3; k++){	base(k,0) = v_x(k);		base(k,1) = v_y(k);		base(k,2) = v_z(k);	}
	// 	mt::Matrix3x3 mp(base(0,0), base(0,1), base(0,2), base(1,0), base(1,1), base(1,2), base(2,0), base(2,1), base(2,2));
	// 	mt::Rotation rp(mp);	rp.getAxisAngle(axis,angle);
	// 	for (unsigned int k = 0; k < 3; k++)	eix_gir(k) = axis[k];	ang_gir = angle;
	// 	for (unsigned int k = 0; k < 3; k++){	cf.c[k] = po(k); 	cf.c[k+3] = eix_gir(k);	}	cf.c[6] = ang_gir;
	// 	config.push_back(cf);
	// 	// Palm End ----------------------------------------
	// 
	// 
	// 	// Storing 'config' in 'objects'
	// 	for(unsigned int i=0; i<(unsigned int)objects.size(); i++)
	// 	{
	// 		mt::Point3 p(config[i].c[0],config[i].c[1],config[i].c[2]);
	// 		objects[i]->setTranslation(p);
	// 		mt::Unit3 axis(config[i].c[3],config[i].c[4],config[i].c[5]);
	// 		mt::Scalar angle = config[i].c[6];
	// 		mt::Rotation r(axis,angle);
	// 		objects[i]->setRotation(r);
	// 	}
	// 
	// 
		// *************************************************************
		// Check collisions
		// *************************************************************
	
		boost::numeric::ublas::vector<float> configwf(17);
		// Thumb
		configwf(0) = -(float)configw(0);
		configwf(1) = (float)configw(1);
		configwf(2) = -(float)configw(2);
		configwf(3) = -(float)configw(3);
		configwf(4) = -(float)configw(4);
		// Index
		configwf(5) = (float)configw(7);
		configwf(6) = -(float)configw(8);
		configwf(7) = -(float)configw(9);
		configwf(8) = -(float)configw(10);
		// Middle
		configwf(9) = (float)configw(13);
		configwf(10) = -(float)configw(14);
		configwf(11) = -(float)configw(15);
		configwf(12) = -(float)configw(16);
		// Ring
		configwf(13) = (float)configw(19);
		configwf(14) = -(float)configw(20);
		configwf(15) = -(float)configw(21);
		configwf(16) = -(float)configw(22);
	
// 		cout << "About to check collisions...";	// TTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT
		bool boolcollision = KtCheckCollision(configwf);
// 		cout << "END"<< endl;	// TTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT
	
		unsigned int collision = 0;
		if (boolcollision == true)	collision = 1;
	
	//  	for(unsigned int i=0; (i<objects.size()) && (!collision); i++)
	//  		if (objects[i]->collidemodel->getAllCollisions() > 0)		collision = 1;
	
		return collision;
	}
	
	
	inline void IvKinHand::LoadJointLimits(void){
	
		joint_limits.resize(25,2,false);
	
	joint_limits(0,0) = 0;			// 0�
	joint_limits(1,0) = -0.2617993;   // -15�
	joint_limits(2,0) = -0.0698131;   // -4�
	joint_limits(3,0) = 4*(3.141592/180);    // 4�
	joint_limits(4,0) = 4*(3.141592/180);    // 4�
	joint_limits(5,0) = -0.6108652;	// -35�
	joint_limits(6,0) = -0.2617993;	// -15�
	// Index
	joint_limits(7,0) = -0.2617993;   // -15�
	joint_limits(8,0) = -0.0698131;   // -4�
	joint_limits(9,0) = 4*(3.141592/180);     // 4�
	joint_limits(10,0) = 4*(3.141592/180);    // 4�
	joint_limits(11,0) = -0.6108652;	// -35�
	joint_limits(12,0) = -0.2617993;  // -15�
	// Middle
	joint_limits(13,0) = -0.2617993;  // -15�
	joint_limits(14,0) = -0.0698131;  // -4�
	joint_limits(15,0) = 4*(3.141592/180);    // 4�
	joint_limits(16,0) = 4*(3.141592/180);    // 4�
	joint_limits(17,0) = -0.6108652;	// -35�
	joint_limits(18,0) = -0.2617993;  // -15�
	// Ring
	joint_limits(19,0) = -0.2617993;  // -15�
	joint_limits(20,0) = -0.0698131;  // -4�
	joint_limits(21,0) = 4*(3.141592/180);    // 4�
	joint_limits(22,0) = 4*(3.141592/180);    // 4
	joint_limits(23,0) = -0.6108652;	// -35�
	joint_limits(24,0) = -0.2617993;  // -15�
	
	// Maximum Joint Values
	// Thumb
	joint_limits(0,1) = 1.570796;			// 90�
	joint_limits(1,1) = 0.2617993;		// 15�
	joint_limits(2,1) = 75*(3.141592/180);    // 75�
	joint_limits(3,1) = 75*(3.141592/180);    // 75�
	joint_limits(4,1) = 75*(3.141592/180);    // 75�
	joint_limits(5,1) = 0.6108652;		// 35�
	joint_limits(6,1) = 1.396263;			// 80�
	// Index
	joint_limits(7,1) = 0.2617993;		// 15�
	joint_limits(8,1) = 75*(3.141592/180);    // 75�
	joint_limits(9,1) = 75*(3.141592/180);    // 75�
	joint_limits(10,1) = 75*(3.141592/180);    // 75�
	joint_limits(11,1) = 0.6108652;		// 35�
	joint_limits(12,1) = 1.396263;		// 80�
	// Middle
	joint_limits(13,1) = 0.2617993;		// 15�
	joint_limits(14,1) = 75*(3.141592/180);    // 75�
	joint_limits(15,1) = 75*(3.141592/180);    // 75�
	joint_limits(16,1) = 75*(3.141592/180);    // 75�
	joint_limits(17,1) = 0.6108652;		// 35�
	joint_limits(18,1) = 1.396263;		// 80�
	// Ring
	joint_limits(19,1) = 0.2617993;		// 15�
	joint_limits(20,1) = 75*(3.141592/180);    // 75�
	joint_limits(21,1) = 75*(3.141592/180);    // 75�
	joint_limits(22,1) = 75*(3.141592/180);    // 75�
	joint_limits(23,1) = 0.6108652;		// 35�
	joint_limits(24,1) = 1.396263;		// 80�
	
		return;
	}
	
	inline void IvKinHand::LoadLambdas(void){
	
		n_lambdas = 13;
	
		lambda_list.resize(2,n_lambdas,false);
	
		lambda_list(0,0) = 0.517241;	lambda_list(1,0) = 0.482759;
		lambda_list(0,1) = 0.793103 ;	lambda_list(1,1) = 0.551724;
		lambda_list(0,2) = 0.310345;	lambda_list(1,2) = 0.689655;
		lambda_list(0,3) = 0;		lambda_list(1,3) = 0.517241;
		lambda_list(0,4) = 0.689655;	lambda_list(1,4) = 0.448276;
		lambda_list(0,5) = 0.137931;	lambda_list(1,5) = 0.241379;
		lambda_list(0,6) = 0.0344828;	lambda_list(1,6) = 0.827586;
		lambda_list(0,7) = 0.655172;	lambda_list(1,7) = 0.241379;
		lambda_list(0,8) = 0.689655;	lambda_list(1,8) = 0;
		lambda_list(0,9) = 0.586207;	lambda_list(1,9) = 0.482759;
		lambda_list(0,10) = 0.724138;	lambda_list(1,10) = 0.275862;
		lambda_list(0,11) = 0.275862;	lambda_list(1,11) = 0.827586;
	
		lambda_list(0,12) = 0.0344828;	lambda_list(1,12) = 0.482759;
	
	
	// 	lambda_list(0,13) = 0;		lambda_list(1,13) = 0.689655;
	// 	lambda_list(0,14) = 0.655172;	lambda_list(1,14) = 0.241379;
	// 	lambda_list(0,15) = 0.137931;	lambda_list(1,15) = 0.206897;
	// 	lambda_list(0,16) = 0.827586;	lambda_list(1,16) = 0.931034;
	
	
		return;
	}
	
	inline void IvKinHand::LoadEigenData(void){
	
		eigenmean.resize(25,false);			eigenmean.clear();
		eigenvectors.resize(25,25,false);	eigenvectors.clear();
		eigenvalues.resize(25,false);		eigenvalues.clear();
	
		// Eigenmean
		eigenmean(0) = 0.6610;
		eigenmean(1) = 0.0143;		eigenmean(2) = 0.3096;		eigenmean(3) = 0.5125;		eigenmean(4) = 0.5125;
		eigenmean(7) = -0.0195;		eigenmean(8) = 0.5195;		eigenmean(9) = 0.5571;		eigenmean(10) = 0.5571;
		eigenmean(13) = 0.0039;		eigenmean(14) = 0.5464;		eigenmean(15) = 0.5672;		eigenmean(16) = 0.5672;
		eigenmean(19) = 0.0329;		eigenmean(20) = 0.5470;		eigenmean(21) = 0.5674;		eigenmean(22) = 0.5674;
	
		// 1st Eigen Vector - Eigen value = 7.2143
		eigenvectors(0,0) = -0.8800;
		eigenvectors(1,0) = -0.0010;	eigenvectors(2,0) = 0.1417;		eigenvectors(3,0) = -0.0337;	eigenvectors(4,0) = -0.0337;
		eigenvectors(7,0) = 0.0056;		eigenvectors(8,0) = 0.4478;		eigenvectors(9,0) = 0.0489;		eigenvectors(10,0) = 0.0489;
		eigenvectors(13,0) = -0.0107;	eigenvectors(14,0) = -0.0371;	eigenvectors(15,0) = 0.0004;	eigenvectors(16,0) = 0.0004;
		eigenvectors(19,0) = -0.0025;	eigenvectors(20,0) = 0.0024;	eigenvectors(21,0) = 0.0001;	eigenvectors(22,0) = 0.0001;
	
		// 2nd Eigen Vector - Eigen value = 5.8188
		eigenvectors(0,1) = 0.1303;
		eigenvectors(1,1) = 0.0049;		eigenvectors(2,1) = -0.0129;	eigenvectors(3,1) = -0.0126;	eigenvectors(4,1) = -0.0126;
		eigenvectors(7,1) = -0.0018;	eigenvectors(8,1) = 0.1919;		eigenvectors(9,1) = -0.0007;	eigenvectors(10,1) = -0.0007;
		eigenvectors(13,1) = 0.0069;	eigenvectors(14,1) = -0.7772;	eigenvectors(15,1) = -0.0110;	eigenvectors(16,1) = -0.0110;
		eigenvectors(19,1) = -0.0063;	eigenvectors(20,1) = 0.5844;	eigenvectors(21,1) = 0.0079;	eigenvectors(22,1) = 0.0079;
	
		
		// Eigenvalues
		eigenvalues(0) = 7.2143;
		eigenvalues(1) = 5.8188;
	
	
		// Standard deviation for eigen vectors
		n_sigma = 3;
	
		return;
	}
	
	inline boost::numeric::ublas::matrix<double> IvKinHand::LoadLogitCoef(void){
	
		boost::numeric::ublas::matrix<double> coefs(n_logitprobs,n_logitparam);
	
		// 1rst initial configuration
		coefs(0,0) = 0.0396338;		coefs(0,1) = -0.0001312;	coefs(0,2) = -0.0000239;
		coefs(0,3) = 0.0000899;		coefs(0,4) = 0.0000010;		coefs(0,5) = -0.0000081;
		coefs(0,6) = -0.0000455;	coefs(0,7) = -0.109282;		coefs(0,8) = -0.0206739;
		coefs(0,9) = -0.0969307;	coefs(0,10) = 0.0191692;	coefs(0,11) = -0.258509;
		coefs(0,12) = 0.0576181;
	
		// 2nd initial configuration
		coefs(1,0) = -0.469679;		coefs(1,1) = 0.0000250;		coefs(1,2) = -0.0000198;
		coefs(1,3) = 0.0000378;		coefs(1,4) = -0.0000315;	coefs(1,5) = -0.0000163;
		coefs(1,6) = -0.0000068;	coefs(1,7) = 0.0618887;		coefs(1,8) = 0.0251680;
		coefs(1,9) = -0.108688;		coefs(1,10) = -0.0236212;	coefs(1,11) = 0.0335184;
		coefs(1,12) = 0.0169883;
	
		// 3rd initial configuration
		coefs(2,0) = -0.629716;		coefs(2,1) = -0.0000853;	coefs(2,2) = -0.0000188;
		coefs(2,3) = 0.0000685;		coefs(2,4) = 0.0000112;		coefs(2,5) = -0.0000072;
		coefs(2,6) = -0.0000175;	coefs(2,7) = -0.212162;		coefs(2,8) = -0.0083337;
		coefs(2,9) = -0.168104;		coefs(2,10) = -0.0038531;	coefs(2,11) = -0.116706;
		coefs(2,12) = 0.0883323;
	
		// 4rth initial configuration
		coefs(3,0) = 0.0997549;		coefs(3,1) = -0.0000866;	coefs(3,2) = -0.0000136;
		coefs(3,3) = 0.0000495;		coefs(3,4) = 0.0000048;		coefs(3,5) = 0.0000145;
		coefs(3,6) = -0.0000353;	coefs(3,7) = -0.0103096;	coefs(3,8) = -0.0118926;
		coefs(3,9) = -0.0252717;	coefs(3,10) = 0.0102670;	coefs(3,11) = -0.0658594;
		coefs(3,12) = 0.0345584;
	
		return coefs;
	}
	
	inline void IvKinHand::InitCollisions(void){
	
	// 	char *filename;
	// 	//filename = "ivfiles/sahand.xml";
	// 	filename = "/home/users/josep.a.claret/Desktop/master/ikjac/trunk/newpfmaster/ivfiles/sahand.xml";
	// 	QString path(filename);
	// 
	// 	//ini_obj(path, objects);
	// 	StrucParse parser;
	// 	QFile file(path);
	// 	QXmlInputSource xmlfile(&file);
	// 	QXmlSimpleReader reader;
	// 	reader.setContentHandler(&parser);
	// 	reader.setErrorHandler(&parser);
	// 
	// 	QFileInfo qfile(file);
	// 	parser.setDirBase(qfile.absoluteDir().absolutePath().toUtf8().constData());
	// 
	// 	//Parse the xml file 
	// 	reader.parse(xmlfile);
	// 	
	// 	for(int i=0; i<parser.numobjects; i++)
	// 	{
	// 		mesh *MyMesh = new mesh(parser.file[i], parser.scale[i], 
	// 								parser.x[i], parser.y[i], parser.z[i],
	// 			                    parser.rx[i], parser.ry[i], parser.rz[i], parser.th[i]);
	// 		pqpobject *pqpobj = new pqpobject(MyMesh);
	// 		MyMesh->setCollideModel(pqpobj);
	// 		objects.push_back(MyMesh);
	// 	}
	// 
	// 	//set_colliders(objects);
	// 	// ----- THUMB -----
	// 	//  Finger Base
	// 	objects[0]->collidemodel->colliders.push_back(objects[7]->collidemodel);	// Index distal phalange
	// 
	// 	//  Phalanges
	// 	for(unsigned int i=1; i<4; i++){
	// 		for(unsigned int j=4; j<12; j++)
	// 			objects[i]->collidemodel->colliders.push_back(objects[j]->collidemodel);	// Index & Middle phalanges
	// 		objects[i]->collidemodel->colliders.push_back(objects[16]->collidemodel);		// Palm
	// 	}
	// 
	// 	// ----- INDEX -----
	// 	for(unsigned int i=5; i<8; i++)
	// 		for(unsigned int j=9; j<12; j++)
	// 			objects[i]->collidemodel->colliders.push_back(objects[j]->collidemodel);	// Middle phalanges
	// 	objects[7]->collidemodel->colliders.push_back(objects[15]->collidemodel);			// Ring distal phalanges
	// 
	// 	// ----- MIDDLE -----
	// 	for(unsigned int i=9; i<12; i++)
	// 		for(unsigned int j=13; j<16; j++)
	// 			objects[i]->collidemodel->colliders.push_back(objects[j]->collidemodel);	// Ring phalanges
	
	
		return;
	}
	
	inline boost::numeric::ublas::matrix<double> IvKinHand::FingPosOrToThumb(boost::numeric::ublas::vector<double> &fingertip_positions_orientations){
		// *************************************************************************************************************************
		// Function that calculates the homogeneous transformation matrix for the base located at the thumb and with the Z axis
		//	as the desired inside fingertip finger position for thumb finger.
		//
		//	Input variables:
		//   fingertip_positions_orientations: vector of dimension 24 with the information related to desired finger
		//	  positions and orientations as
		//		finger_positions(0:2) = Thumb desired position		finger_positions(3:5) = Thumb desired orientation
		//		finger_positions(6:8) = Index desired position		finger_positions(9:11) = Index desired orientatio
		//		finger_positions(12:14) = Middle desired position	finger_positions(15:17) = Middle desired orientatio
		//		finger_positions(18:20) = Ring desired position		finger_positions(21:23) = Ring desired orientatio
		//
		//	Output variables:
		//   fingertip_positions_orientations: vector of dimension 18 with the information related to desired finger
		//	  positions and orientations referenced from the thumb base as
		//		finger_positions(0:2) = Index desired position		finger_positions(3:5) = Index desired fingertip position
		//		finger_positions(6:8) = Middle desired position		finger_positions(9:11) = Middle desired fingertip position
		//		finger_positions(12:14) = Ring desired position		finger_positions(15:17) = Ring desired fingertip position
		//
		//   t_thumb: 4x4 matrix with the transformation matrix related to thumb base
		//
		// *************************************************************************************************************************
	
	
		// ----- Asserting input variables -----
		//		finger_positions
		double v = 0;
		boost::numeric::ublas::vector<double> v1(3), v2(3);
		assert(fingertip_positions_orientations.size() == 24);	// Checking size
		for (unsigned int i=0; i<4; i++){
			for (unsigned int j=0; j<3; j++)	v1(j) = fingertip_positions_orientations(6*i+j+3);
			v = sqrmod(v1);
			// Checking orientation vector module		
			assert((v > 0.9999) && (v < 1.0001));
		}
		// ----- End Asserting input variables -----
	
	
		boost::numeric::ublas::matrix<double> t_thumb(4,4);
		t_thumb.clear();
		t_thumb(3,3) = 1;
	
		// Thumb position
		for (unsigned int i=0; i<3; i++)    t_thumb(i,3) = fingertip_positions_orientations(i);
	
		// Z axis
		double vnorm = 0;
		for (unsigned int i=0; i<3; i++)
			t_thumb(i,2) = -fingertip_positions_orientations(i+3);
	
		// X axis
		if ((t_thumb(1,2) != 0) || (t_thumb(2,2) != 0)) {
			vnorm = sqrt(t_thumb(1,2)*t_thumb(1,2) + t_thumb(2,2)*t_thumb(2,2));
			t_thumb(0,0) = 0;
			t_thumb(1,0) = -t_thumb(2,2)/vnorm;
			t_thumb(2,0) = t_thumb(1,2)/vnorm;
		}
		else {
			vnorm = sqrt(t_thumb(0,2)*t_thumb(0,2) + t_thumb(2,2)*t_thumb(2,2));
			t_thumb(0,0) = -t_thumb(2,2)/vnorm;
			t_thumb(1,0) = 0;
			t_thumb(2,0) = t_thumb(0,2)/vnorm;
		}
	
		// Y axis
		boost::numeric::ublas::vector<double> vx(3), vy(3), vz(3);
		for (unsigned int i=0; i<3; i++){
			vx(i) = t_thumb(i,0);
			vz(i) = t_thumb(i,2);
		}
		vy = cross_p(vz, vx);
		for (unsigned int i=0; i<3; i++)    t_thumb(i,1) = vy(i);
	
	
		// Change finger and fingertip positions to thumb base
		boost::numeric::ublas::vector<double> fing_pos_or(N_POS_FING_THUMB);
		boost::numeric::ublas::vector<double> vec(4);
		//  Positions
		vec(3) = 1;
		for (unsigned int i=0; i<3; i++){
			for (unsigned int j=0; j<3; j++)    vec(j) = fingertip_positions_orientations(6*i+6+j);
		
				vec = prod(inv(t_thumb),vec);
		
				for (unsigned int j=0; j<3; j++)    fing_pos_or(6*i+j) = vec(j);
		}
		//  Inside fingertip positions
		vec(3) = 0;
		for (unsigned int i=0; i<3; i++){
			for (unsigned int j=0; j<3; j++)    vec(j) = fingertip_positions_orientations(6*i+9+j);
		
				vec = prod(inv(t_thumb),vec);
		
				for (unsigned int j=0; j<3; j++)    fing_pos_or(6*i+3+j) = fing_pos_or(6*i+j)-FT_RAD*vec(j);
		}
		fingertip_positions_orientations.resize(N_POS_FING_THUMB,false);
		for (unsigned int i=0; i<fingertip_positions_orientations.size(); i++)
			fingertip_positions_orientations(i) = fing_pos_or(i);
	
		return t_thumb;
	}
	
	inline void IvKinHand::ConfigThumbToWrist(const boost::numeric::ublas::matrix<double> &t_thumb){
	// *************************************************************************************************************************
		// Function that calculates the wrist base given a sah configuration and a thumb base and stores it into 
		//	
		//	input parameters:
		//	thumb_results
		//    thumb_results.configuration: 13 dimension sahand configuration vector as
		//      config(0) = thumbrot
		//		config(1) = thumb abduction		config(2) = first thumb flexion		config(3) = second thumb flexion	
		//		config(4) = index abduction		config(5) = first index flexion		config(6) = second index flexion
		//		config(7) = middle abduction	config(8) = first middle flexion	config(9) = second middle flexion
		//		config(10) = ring abduction		config(11) = first ring flexion		config(12) = second ring flexion
		//
		//	  thumb_results.configuration_list: matrix with a 13 dimension vector with a sahand configuration. 
		//		Each row has the same data structure as thumb_results.configuration
		//
		//   t_thumb: the thumb base, a 4x4 matrix which is an homogeneous transformation matrix.
		//
		//	output parameters:
		//	  thumb_results.t_wrists: 16 dimension vector of the transformation matrix related to the wrist base as:
		//		t_wrists(i,:) = [oxx oxy oxz 0 oyx oyy oyz 0 ozx ozy ozz 0 px py pz 1]
		//
		//	  thumb_results.t_wrist_list: matrix with a 16 dimension vector in each row of the transformation matrix
		//      related to the wrist base. Each row has the same data structure as thumb_results.t_wrists
		//
		// *************************************************************************************************************************
	
	
		// ----- Asserting input variables -----
		double v = 0;
		boost::numeric::ublas::vector<double> v1(3), v2(3);
	
		//  t_thumb
		assert((t_thumb.size1() == 4) && (t_thumb.size2() == 4));	// checking size
		for (unsigned int i=0; i<3; i++){
			// Checking ox, oy, oz have module 1
			for (unsigned int j=0; j<3; j++)	v1(j) = t_thumb(j,i);
			v = sqrmod(v1);		assert((v > 0.9999) && (v < 1.0001));
		}
		// Checking orthonormality between ox, oy and oz
		for (unsigned int i=0; i<3; i++) {	v1(i) = t_thumb(i,0);		v2(i) = t_thumb(i,1);	}	v = prod_esc(v1,v2);	assert((v > -0.0001) && (v < 0.0001));
		for (unsigned int i=0; i<3; i++) {	v1(i) = t_thumb(i,0);		v2(i) = t_thumb(i,2);	}	v = prod_esc(v1,v2);	assert((v > -0.0001) && (v < 0.0001));
		for (unsigned int i=0; i<3; i++) {	v1(i) = t_thumb(i,1);		v2(i) = t_thumb(i,2);	}	v = prod_esc(v1,v2);	assert((v > -0.0001) && (v < 0.0001));
		assert((t_thumb(3,0) == 0) && (t_thumb(3,1) == 0) && (t_thumb(3,2) == 0) && (t_thumb(3,3) == 1));
	
		// ----- End Asserting input variables -----
	
	
		boost::numeric::ublas::matrix<double> wrist_base(4,4);
		boost::numeric::ublas::vector<double> config_t(N_TA_VDC_CONFIG);
	
		// thumb_results.configuration & thumb_results.t_wrist
		//		Obtain wrist base
		wrist_base = DirKinThumbToWrist(ik_results.configuration);
		wrist_base = prod(t_thumb,wrist_base);
	
		ik_results.t_wrist.resize(16,false);
		ik_results.t_wrist.clear();
		for (unsigned int i=0; i<4; i++)
			for (unsigned int j=0; j<3; j++)
				ik_results.t_wrist(4*i+j) = wrist_base(j,i);
		ik_results.t_wrist(15) = 1;
		//		Pass config thumb to config wrist
		for (unsigned int i=0; i<N_VDC_CONFIG; i++)    ik_results.configuration(i) = ik_results.configuration(i+1);
		ik_results.configuration.resize(N_VDC_CONFIG,true);
	
	
		// thumb_results.configuration_list & thumb_results.t_wrist_list
		unsigned int n_confs = ik_results.configuration_list.size1();
		ik_results.t_wrist_list.resize(n_confs,16,false);
		ik_results.t_wrist_list.clear();
		for (unsigned int nc=0; nc<n_confs; nc++){
			
			//	Obtain wrist base
			for (unsigned int i=0; i<N_TA_VDC_CONFIG; i++)
				config_t(i) = ik_results.configuration_list(nc,i);
	
			wrist_base = DirKinThumbToWrist(config_t);
			wrist_base = prod(t_thumb,wrist_base);
	
			for (unsigned int i=0; i<4; i++)
				for (unsigned int j=0; j<3; j++)
					ik_results.t_wrist_list(nc,4*i+j) = wrist_base(j,i);
			ik_results.t_wrist_list(nc,15) = 1;
	
			//	Pass config thumb to config wrist
			for (unsigned int i=0; i<N_VDC_CONFIG; i++)
				ik_results.configuration_list(nc,i) = ik_results.configuration_list(nc,i+1);
		}
		ik_results.configuration_list.resize(n_confs,N_VDC_CONFIG,true);
	
		return;
	}
	
	
	/*************************************************************
	Cinem�tica directa
	************************************************************/
	
	inline boost::numeric::ublas::matrix<double> IvKinHand::DirKinThumbToWrist(const boost::numeric::ublas::vector<double> &thumb_configuration){
		boost::numeric::ublas::matrix<double> t_wrist(4,4);
	
		boost::numeric::ublas::matrix<double> tdk;
		tdk = ThumbDirKin(thumb_configuration);
	
		t_wrist.clear();
		for (unsigned int i=0; i<3; i++){
			t_wrist(i,0) = tdk(i+3,0);	// X Axis
			t_wrist(i,2) = tdk(i+6,0);	// Z Axis
			t_wrist(i,3) = tdk(i,0);	// Origin
		}
		t_wrist(3,3) = 1; 
	
		// Y Axis
		boost::numeric::ublas::vector<double> vx(3), vy(3), vz(3);
		for (unsigned int i=0; i<3; i++){
			vx(i) = tdk(i+3,0);	// X Axis
			vz(i) = tdk(i+6,0);	// Z Axis
		}
		vy =  cross_p(vz, vx);
		for (unsigned int i=0; i<3; i++)	t_wrist(i,1) = vy(i);	// X Axis
	
		return t_wrist;
	}
	
	inline boost::numeric::ublas::vector<double> IvKinHand::ThumbDirKinSAHand_Index_Ring(const boost::numeric::ublas::vector<double> &config){
	
		// **********************************************************************************
		// **** Calculating Direct Kynematics of Index and Ring referenced to Thumb base ****
	
		boost::numeric::ublas::vector<double> v_pfi_pfr(6);
	
		boost::numeric::ublas::matrix<double> tdk;
		tdk = ThumbDirKin(config);
	
		for (unsigned int i=0; i<3; i++){
			v_pfi_pfr(i) = tdk(i,15);
			v_pfi_pfr(i+3) = tdk(i,29);
		}
	
		return v_pfi_pfr;
	}
	
	inline boost::numeric::ublas::matrix<double> IvKinHand::WristDirKin(const boost::numeric::ublas::matrix<double> &wrist_base,
									const boost::numeric::ublas::vector<double> &wrist_config){
	
	// *************************************************************************************************************************
		// Function that calculates the position and axis of each joint given a wrist base and a configuration base
		//
		//	Input variables:
		//
		//	 wrist_base: a 4x4 homogeneous transformation matrix
		//
		//   configs: a 25 dimension sahand configuration vector as
		//      config(0) = thumbrot
		//		config(1) = thumb abduction			config(2) = first thumb flexion			config(3) = second thumb flexion
		//		config(4) = third thumb flexion		config(5) = first thumb fingertip		config(6) = second thumb fingertip
		//		config(7) = index abduction			config(8) = first index flexion			config(9) = second index flexion
		//		config(10) = third index flexion	config(11) = first index fingertip		config(12) = second index fingertip
		//		config(13) = middle abduction		config(14) = first middle flexion		config(15) = second middle flexion
		//		config(16) = third middle flexion	config(17) = first middle fingertip		config(18) = second middle fingertip
		//		config(19) = ring abduction			config(20) = first ring flexion			config(21) = second ring flexion
		//		config(22) = third ring flexion		config(23) = first ring fingertip		config(24) = second ring fingertip
		//
		// *************************************************************************************************************************
	
	
		// ----- Asserting input variables -----
		// wrist_Base
		assert((wrist_base.size1() == 4) && (wrist_base.size2() == 4));
		double v;
		boost::numeric::ublas::vector<double> v1(3), v2(3);
		for (unsigned int i=0; i<3; i++){
			// Checking ox, oy, oz have module 1
			for (unsigned int j=0; j<3; j++)	v1(j) = wrist_base(j,i);
			v = sqrmod(v1);		assert((v > 0.9999) && (v < 1.0001));
		}
		//   Checking orthonormality between ox, oy and oz
		for (unsigned int i=0; i<3; i++) {	v1(i) = wrist_base(i,0);		v2(i) = wrist_base(i,1);	}	v = prod_esc(v1,v2);	assert((v > -0.0001) && (v < 0.0001));
		for (unsigned int i=0; i<3; i++) {	v1(i) = wrist_base(i,0);		v2(i) = wrist_base(i,2);	}	v = prod_esc(v1,v2);	assert((v > -0.0001) && (v < 0.0001));
		for (unsigned int i=0; i<3; i++) {	v1(i) = wrist_base(i,1);		v2(i) = wrist_base(i,2);	}	v = prod_esc(v1,v2);	assert((v > -0.0001) && (v < 0.0001));
		assert((wrist_base(3,0) == 0) && (wrist_base(3,1) == 0) && (wrist_base(3,2) == 0) && (wrist_base(3,3) == 1));
	
		// wrist_config
		assert(wrist_config.size() == N_VDC_CONFIG);
		// ----- End Asserting input variables -----
	
	
		unsigned int nj = 0;
		boost::numeric::ublas::matrix<double> pos_axis(9,30);	pos_axis.clear();
		boost::numeric::ublas::matrix<double> tr(3,4), tr2(3,4), m1(4,4), m2(4,4);;    // Transformation Matrix
	
	
		boost::numeric::ublas::vector<double> config(25);	config = wrist_config;
	
	
		// ---- WRIST ----
		for (unsigned int i=0; i<3; i++){
			pos_axis(i,nj) = wrist_base(i,3);
			pos_axis(i+3,nj) = wrist_base(i,0);
			pos_axis(i+6,nj) = wrist_base(i,2);
		}
		nj++;
	
		// ---- THUMB ----
		m1.clear();	for (unsigned int i=0; i<4; i++)	m1(i,i) = 1;
		m1(0,3) = -3;	m1(1,3) = 27.1;		m1(2,3) = 0;
		m1 = prod(wrist_base,m1);
	
		for (unsigned int i=0; i<3; i++){
			pos_axis(i,nj) = m1(i,3);
			pos_axis(i+3,nj) = m1(i,0);
			pos_axis(i+6,nj) = m1(i,2);
		}
		nj++;
	
		// ThumbRotTF
		double sT0 = sin(config(0));	double cT0 = cos(config(0));
		tr(0,0) = 0.57358*sT0;
		tr(0,1) = -cT0;
		tr(0,2) = -0.81915*sT0;
		tr(0,3) = -6*cT0 + 86.9*sT0;
		tr(1,0) = 0.57358*cT0;
		tr(1,1) = sT0;
		tr(1,2) = -0.81915*cT0;
		tr(1,3) = 6*sT0 + 86.9*cT0;
		tr(2,0) = 0.81915;
		tr(2,1) = 0;
		tr(2,2) = 0.57358;
		tr(2,3) = 97;
		m2.clear();
		for (unsigned int i=0; i<3; i++)
			for (unsigned int j=0; j<4; j++)
				m2(i,j) = tr(i,j);
		m2(3,3) = 1;
		m2 = prod(m1,m2);
		for (unsigned int i=0; i<3; i++)
			for (unsigned int j=0; j<4; j++)
				tr(i,j) = m2(i,j);
	
		// 1st Finger angle
		double sT1 = sin(config(1));	double cT1 = cos(config(1));
		tr2(0,0) = tr(0,0)*cT1 + tr(0,1)*sT1;
		tr2(0,1) = -tr(0,0)*sT1 + tr(0,1)*cT1;
		tr2(0,2) = tr(0,2);
		tr2(1,0) = tr(1,0)*cT1 + tr(1,1)*sT1;
		tr2(1,1) = -tr(1,0)*sT1 + tr(1,1)*cT1;
		tr2(1,2) = tr(1,2);
		tr2(2,0) = tr(2,0)*cT1 + tr(2,1)*sT1;
		tr2(2,1) = -tr(2,0)*sT1 + tr(2,1)*cT1;
		tr2(2,2) = tr(2,2);
		tr2(0,3) = tr(0,3);
		tr2(1,3) = tr(1,3);
		tr2(2,3) = tr(2,3);
		tr = tr2;
	
		for (unsigned int i=0; i<3; i++){
			pos_axis(i,nj) = tr(i,3);
			pos_axis(i+3,nj) = tr(i,0);
			pos_axis(i+6,nj) = tr(i,2);
		}
		nj++;
	
		// 2nd Finger angle
		double sT2 = sin(config(2));               double cT2 = cos(config(2));
		tr2(0,0) = tr(0,0)*cT2 + tr(0,2)*sT2;
		tr2(0,1) = -tr(0,0)*sT2 + tr(0,2)*cT2;
		tr2(0,2) = -tr(0,1);
		tr2(1,0) = tr(1,0)*cT2 + tr(1,2)*sT2;
		tr2(1,1) = -tr(1,0)*sT2 + tr(1,2)*cT2;
		tr2(1,2) = -tr(1,1);
		tr2(2,0) = tr(2,0)*cT2 + tr(2,2)*sT2;
		tr2(2,1) = -tr(2,0)*sT2 + tr(2,2)*cT2;
		tr2(2,2) = -tr(2,1);
		tr2(0,3) = tr(0,3);
		tr2(1,3) = tr(1,3);
		tr2(2,3) = tr(2,3);
		tr = tr2;
	
		for (unsigned int i=0; i<3; i++){
			pos_axis(i,nj) = tr(i,3);
			pos_axis(i+3,nj) = tr(i,0);
			pos_axis(i+6,nj) = tr(i,2);
		}
		nj++;
	
		// 3rd Finger angle
		double sT3 = sin(config(3));	            double cT3 = cos(config(3));
		tr2(0,0) = tr(0,0)*cT3 + tr(0,1)*sT3;
		tr2(0,1) = -tr(0,0)*sT3 + tr(0,1)*cT3;
		tr2(0,2) = tr(0,2);
		tr2(1,0) = tr(1,0)*cT3 + tr(1,1)*sT3;
		tr2(1,1) = -tr(1,0)*sT3 + tr(1,1)*cT3;
		tr2(1,2) = tr(1,2);
		tr2(2,0) = tr(2,0)*cT3 + tr(2,1)*sT3;
		tr2(2,1) = -tr(2,0)*sT3 + tr(2,1)*cT3;
		tr2(2,2) = tr(2,2);
		tr2(0,3) = tr(0,0)*67.8 + tr(0,3);
		tr2(1,3) = tr(1,0)*67.8 + tr(1,3);
		tr2(2,3) = tr(2,0)*67.8 + tr(2,3);
		tr = tr2;
	
		for (unsigned int i=0; i<3; i++){
			pos_axis(i,nj) = tr(i,3);
			pos_axis(i+3,nj) = tr(i,0);
			pos_axis(i+6,nj) = tr(i,2);
		}
		nj++;;
	
		// 4th Finger angle
		double sT4 = sin(config(4));        double cT4 = cos(config(4));
		tr2(0,0) = tr(0,0)*cT4 + tr(0,1)*sT4;
		tr2(0,1) = -tr(0,0)*sT4 + tr(0,1)*cT4;
		tr2(0,2) = tr(0,2);
		tr2(1,0) = tr(1,0)*cT4 + tr(1,1)*sT4;
		tr2(1,1) = -tr(1,0)*sT4 + tr(1,1)*cT4;
		tr2(1,2) = tr(1,2);
		tr2(2,0) = tr(2,0)*cT4 + tr(2,1)*sT4;
		tr2(2,1) = -tr(2,0)*sT4 + tr(2,1)*cT4;
		tr2(2,2) = tr(2,2);
		tr2(0,3) = tr(0,0)*30 + tr(0,3);
		tr2(1,3) = tr(1,0)*30 + tr(1,3);
		tr2(2,3) = tr(2,0)*30 + tr(2,3);
		tr = tr2;
	
		for (unsigned int i=0; i<3; i++){
			pos_axis(i,nj) = tr(i,3);
			pos_axis(i+3,nj) = tr(i,0);
			pos_axis(i+6,nj) = tr(i,2);
		}
		nj++;
	
		// 1rt Tip angle
		double sT5 = sin(config(5));          double cT5 = cos(config(5));
		tr2(0,0) = tr(0,0)*cT5 - tr(0,2)*sT5;
		tr2(0,1) = -tr(0,0)*sT5 - tr(0,2)*cT5;
		tr2(0,2) = tr(0,1);
		tr2(1,0) = tr(1,0)*cT5 - tr(1,2)*sT5;
		tr2(1,1) = -tr(1,0)*sT5 - tr(1,2)*cT5;
		tr2(1,2) = tr(1,1);
		tr2(2,0) = tr(2,0)*cT5 - tr(2,2)*sT5;
		tr2(2,1) = -tr(2,0)*sT5 - tr(2,2)*cT5;
		tr2(2,2) = tr(2,1);
		tr2(0,3) = tr(0,0)*29.5 + tr(0,3);
		tr2(1,3) = tr(1,0)*29.5 + tr(1,3);
		tr2(2,3) = tr(2,0)*29.5 + tr(2,3);
		tr = tr2;
	
		for (unsigned int i=0; i<3; i++){
			pos_axis(i,nj) = tr(i,3);
			pos_axis(i+3,nj) = tr(i,0);
			pos_axis(i+6,nj) = tr(i,2);
		}
		nj++;
	
		// 2nd Tip angle
		double sT6 = sin(config(6));	        double cT6 = cos(config(6));
		tr2(0,0) = tr(0,0)*cT6 + tr(0,2)*sT6;
		tr2(0,1) = -tr(0,0)*sT6 + tr(0,2)*cT6;
		tr2(0,2) = -tr(0,1);
		tr2(1,0) = tr(1,0)*cT6 + tr(1,2)*sT6;
		tr2(1,1) = -tr(1,0)*sT6 + tr(1,2)*cT6;
		tr2(1,2) = -tr(1,1);
		tr2(2,0) = tr(2,0)*cT6 + tr(2,2)*sT6;
		tr2(2,1) = -tr(2,0)*sT6 + tr(2,2)*cT6;
		tr2(2,2) = -tr(2,1);
		tr2(0,3) = tr(0,3);
		tr2(1,3) = tr(1,3);
		tr2(2,3) = tr(2,3);
		tr = tr2;
	
		for (unsigned int i=0; i<3; i++){
			pos_axis(i,nj) = tr(i,3);
			pos_axis(i+3,nj) = tr(i,0);
			pos_axis(i+6,nj) = tr(i,2);
		}
		nj++;
	
		// Storing finger position and orientation
		for (unsigned int i=0; i<3; i++){
			pos_axis(i,nj) = tr(i,3)+FT_RAD*tr(i,0);
			pos_axis(i+3,nj) = tr(i,0);
			pos_axis(i+6,nj) = tr(i,2);
		}
		nj++;
	
	
		// ---- INDEX ----
		// 1st Finger angle
		sT1 = sin(config(7));               cT1 = cos(config(7));
		tr(0,0) = 0;
		tr(0,1) = 0;
		tr(0,2) = 1;
		tr(1,0) = 0.035485*cT1 - 0.99937*sT1;
		tr(1,1) = -0.035485*sT1 - 0.99937*cT1;
		tr(1,2) = 0;
		tr(2,0) = 0.99937*cT1 + 0.035485*sT1;
		tr(2,1) = -0.99937*sT1 + 0.035485*cT1;
		tr(2,2) = 0;
		tr(0,3) = -4.3;
		tr(1,3) = 40.165;
		tr(2,3) = 145.43;
	
		m2.clear();
		for (unsigned int i=0; i<3; i++)
			for (unsigned int j=0; j<4; j++)
				m2(i,j) = tr(i,j);
		m2(3,3) = 1;
		m2 = prod(wrist_base,m2);
		for (unsigned int i=0; i<3; i++)
			for (unsigned int j=0; j<4; j++)
				tr(i,j) = m2(i,j);
	
		for (unsigned int i=0; i<3; i++){
			pos_axis(i,nj) = tr(i,3);
			pos_axis(i+3,nj) = tr(i,0);
			pos_axis(i+6,nj) = tr(i,2);
		}
		nj++;
	
		// 2nd Finger angle
		sT2 = sin(config(8));               cT2 = cos(config(8));
		tr2(0,0) = tr(0,0)*cT2 + tr(0,2)*sT2;
		tr2(0,1) = -tr(0,0)*sT2 + tr(0,2)*cT2;
		tr2(0,2) = -tr(0,1);
		tr2(1,0) = tr(1,0)*cT2 + tr(1,2)*sT2;
		tr2(1,1) = -tr(1,0)*sT2 + tr(1,2)*cT2;
		tr2(1,2) = -tr(1,1);
		tr2(2,0) = tr(2,0)*cT2 + tr(2,2)*sT2;
		tr2(2,1) = -tr(2,0)*sT2 + tr(2,2)*cT2;
		tr2(2,2) = -tr(2,1);
		tr2(0,3) = tr(0,3);
		tr2(1,3) = tr(1,3);
		tr2(2,3) = tr(2,3);
		tr = tr2;
	
		for (unsigned int i=0; i<3; i++){
			pos_axis(i,nj) = tr(i,3);
			pos_axis(i+3,nj) = tr(i,0);
			pos_axis(i+6,nj) = tr(i,2);
		}
		nj++;
	
		// 3rd Finger angle
		sT3 = sin(config(9));	            cT3 = cos(config(9));
		tr2(0,0) = tr(0,0)*cT3 + tr(0,1)*sT3;
		tr2(0,1) = -tr(0,0)*sT3 + tr(0,1)*cT3;
		tr2(0,2) = tr(0,2);
		tr2(1,0) = tr(1,0)*cT3 + tr(1,1)*sT3;
		tr2(1,1) = -tr(1,0)*sT3 + tr(1,1)*cT3;
		tr2(1,2) = tr(1,2);
		tr2(2,0) = tr(2,0)*cT3 + tr(2,1)*sT3;
		tr2(2,1) = -tr(2,0)*sT3 + tr(2,1)*cT3;
		tr2(2,2) = tr(2,2);
		tr2(0,3) = tr(0,0)*67.8 + tr(0,3);
		tr2(1,3) = tr(1,0)*67.8 + tr(1,3);
		tr2(2,3) = tr(2,0)*67.8 + tr(2,3);
		tr = tr2;
	
		for (unsigned int i=0; i<3; i++){
			pos_axis(i,nj) = tr(i,3);
			pos_axis(i+3,nj) = tr(i,0);
			pos_axis(i+6,nj) = tr(i,2);
		}
		nj++;
	
		// 4th Finger angle
		sT4 = sin(config(10));        cT4 = cos(config(10));
		tr2(0,0) = tr(0,0)*cT4 + tr(0,1)*sT4;
		tr2(0,1) = -tr(0,0)*sT4 + tr(0,1)*cT4;
		tr2(0,2) = tr(0,2);
		tr2(1,0) = tr(1,0)*cT4 + tr(1,1)*sT4;
		tr2(1,1) = -tr(1,0)*sT4 + tr(1,1)*cT4;
		tr2(1,2) = tr(1,2);
		tr2(2,0) = tr(2,0)*cT4 + tr(2,1)*sT4;
		tr2(2,1) = -tr(2,0)*sT4 + tr(2,1)*cT4;
		tr2(2,2) = tr(2,2);
		tr2(0,3) = tr(0,0)*30 + tr(0,3);
		tr2(1,3) = tr(1,0)*30 + tr(1,3);
		tr2(2,3) = tr(2,0)*30 + tr(2,3);
		tr = tr2;
	
		for (unsigned int i=0; i<3; i++){
			pos_axis(i,nj) = tr(i,3);
			pos_axis(i+3,nj) = tr(i,0);
			pos_axis(i+6,nj) = tr(i,2);
		}
		nj++;
	
		// 1rt Tip angle
		sT5 = sin(config(11));          cT5 = cos(config(11));
		tr2(0,0) = tr(0,0)*cT5 - tr(0,2)*sT5;
		tr2(0,1) = -tr(0,0)*sT5 - tr(0,2)*cT5;
		tr2(0,2) = tr(0,1);
		tr2(1,0) = tr(1,0)*cT5 - tr(1,2)*sT5;
		tr2(1,1) = -tr(1,0)*sT5 - tr(1,2)*cT5;
		tr2(1,2) = tr(1,1);
		tr2(2,0) = tr(2,0)*cT5 - tr(2,2)*sT5;
		tr2(2,1) = -tr(2,0)*sT5 - tr(2,2)*cT5;
		tr2(2,2) = tr(2,1);
		tr2(0,3) = tr(0,0)*29.5 + tr(0,3);
		tr2(1,3) = tr(1,0)*29.5 + tr(1,3);
		tr2(2,3) = tr(2,0)*29.5 + tr(2,3);
		tr = tr2;
	
		for (unsigned int i=0; i<3; i++){
			pos_axis(i,nj) = tr(i,3);
			pos_axis(i+3,nj) = tr(i,0);
			pos_axis(i+6,nj) = tr(i,2);
		}
		nj++;
	
		// 2nd Tip angle
		sT6 = sin(config(12));	        cT6 = cos(config(12));
		tr2(0,0) = tr(0,0)*cT6 + tr(0,2)*sT6;
		tr2(0,1) = -tr(0,0)*sT6 + tr(0,2)*cT6;
		tr2(0,2) = -tr(0,1);
		tr2(1,0) = tr(1,0)*cT6 + tr(1,2)*sT6;
		tr2(1,1) = -tr(1,0)*sT6 + tr(1,2)*cT6;
		tr2(1,2) = -tr(1,1);
		tr2(2,0) = tr(2,0)*cT6 + tr(2,2)*sT6;
		tr2(2,1) = -tr(2,0)*sT6 + tr(2,2)*cT6;
		tr2(2,2) = -tr(2,1);
		tr2(0,3) = tr(0,3);
		tr2(1,3) = tr(1,3);
		tr2(2,3) = tr(2,3);
		tr = tr2;
	
		for (unsigned int i=0; i<3; i++){
			pos_axis(i,nj) = tr(i,3);
			pos_axis(i+3,nj) = tr(i,0);
			pos_axis(i+6,nj) = tr(i,2);
		}
		nj++;
	
		// Storing finger position and orientation
		for (unsigned int i=0; i<3; i++){
			pos_axis(i,nj) = tr(i,3)+FT_RAD*tr(i,0);
			pos_axis(i+3,nj) = tr(i,0);
			pos_axis(i+6,nj) = tr(i,2);
		}
		nj++;
	
	
		// ---- MIDDLE ----
		// 1st Finger angle
		sT1 = sin(config(13));               cT1 = cos(config(13));
		tr(0,0) = 0;
		tr(0,1) = 0;
		tr(0,2) = 1;
		tr(1,0) = -sT1;
		tr(1,1) = -cT1;
		tr(1,2) = 0;
		tr(2,0) = cT1;
		tr(2,1) = -sT1;
		tr(2,2) = 0;
		tr(0,3) = -4.3;
		tr(1,3) = 0;
		tr(2,3) = 150.15;
	
		m2.clear();
		for (unsigned int i=0; i<3; i++)
			for (unsigned int j=0; j<4; j++)
				m2(i,j) = tr(i,j);
		m2(3,3) = 1;
		m2 = prod(wrist_base,m2);
		for (unsigned int i=0; i<3; i++)
			for (unsigned int j=0; j<4; j++)
				tr(i,j) = m2(i,j);
	
		for (unsigned int i=0; i<3; i++){
			pos_axis(i,nj) = tr(i,3);
			pos_axis(i+3,nj) = tr(i,0);
			pos_axis(i+6,nj) = tr(i,2);
		}
		nj++;
	
		// 2nd Finger angle
		sT2 = sin(config(14));               cT2 = cos(config(14));
		tr2(0,0) = tr(0,0)*cT2 + tr(0,2)*sT2;
		tr2(0,1) = -tr(0,0)*sT2 + tr(0,2)*cT2;
		tr2(0,2) = -tr(0,1);
		tr2(1,0) = tr(1,0)*cT2 + tr(1,2)*sT2;
		tr2(1,1) = -tr(1,0)*sT2 + tr(1,2)*cT2;
		tr2(1,2) = -tr(1,1);
		tr2(2,0) = tr(2,0)*cT2 + tr(2,2)*sT2;
		tr2(2,1) = -tr(2,0)*sT2 + tr(2,2)*cT2;
		tr2(2,2) = -tr(2,1);
		tr2(0,3) = tr(0,3);
		tr2(1,3) = tr(1,3);
		tr2(2,3) = tr(2,3);
		tr = tr2;
	
		for (unsigned int i=0; i<3; i++){
			pos_axis(i,nj) = tr(i,3);
			pos_axis(i+3,nj) = tr(i,0);
			pos_axis(i+6,nj) = tr(i,2);
		}
		nj++;
	
		// 3rd Finger angle
		sT3 = sin(config(15));	            cT3 = cos(config(15));
		tr2(0,0) = tr(0,0)*cT3 + tr(0,1)*sT3;
		tr2(0,1) = -tr(0,0)*sT3 + tr(0,1)*cT3;
		tr2(0,2) = tr(0,2);
		tr2(1,0) = tr(1,0)*cT3 + tr(1,1)*sT3;
		tr2(1,1) = -tr(1,0)*sT3 + tr(1,1)*cT3;
		tr2(1,2) = tr(1,2);
		tr2(2,0) = tr(2,0)*cT3 + tr(2,1)*sT3;
		tr2(2,1) = -tr(2,0)*sT3 + tr(2,1)*cT3;
		tr2(2,2) = tr(2,2);
		tr2(0,3) = tr(0,0)*67.8 + tr(0,3);
		tr2(1,3) = tr(1,0)*67.8 + tr(1,3);
		tr2(2,3) = tr(2,0)*67.8 + tr(2,3);
		tr = tr2;
	
		for (unsigned int i=0; i<3; i++){
			pos_axis(i,nj) = tr(i,3);
			pos_axis(i+3,nj) = tr(i,0);
			pos_axis(i+6,nj) = tr(i,2);
		}
		nj++;
	
		// 4th Finger angle
		sT4 = sin(config(16));        cT4 = cos(config(16));
		tr2(0,0) = tr(0,0)*cT4 + tr(0,1)*sT4;
		tr2(0,1) = -tr(0,0)*sT4 + tr(0,1)*cT4;
		tr2(0,2) = tr(0,2);
		tr2(1,0) = tr(1,0)*cT4 + tr(1,1)*sT4;
		tr2(1,1) = -tr(1,0)*sT4 + tr(1,1)*cT4;
		tr2(1,2) = tr(1,2);
		tr2(2,0) = tr(2,0)*cT4 + tr(2,1)*sT4;
		tr2(2,1) = -tr(2,0)*sT4 + tr(2,1)*cT4;
		tr2(2,2) = tr(2,2);
		tr2(0,3) = tr(0,0)*30 + tr(0,3);
		tr2(1,3) = tr(1,0)*30 + tr(1,3);
		tr2(2,3) = tr(2,0)*30 + tr(2,3);
		tr = tr2;
	
		for (unsigned int i=0; i<3; i++){
			pos_axis(i,nj) = tr(i,3);
			pos_axis(i+3,nj) = tr(i,0);
			pos_axis(i+6,nj) = tr(i,2);
		}
		nj++;
	
		// 1rt Tip angle
		sT5 = sin(config(17));          cT5 = cos(config(17));
		tr2(0,0) = tr(0,0)*cT5 - tr(0,2)*sT5;
		tr2(0,1) = -tr(0,0)*sT5 - tr(0,2)*cT5;
		tr2(0,2) = tr(0,1);
		tr2(1,0) = tr(1,0)*cT5 - tr(1,2)*sT5;
		tr2(1,1) = -tr(1,0)*sT5 - tr(1,2)*cT5;
		tr2(1,2) = tr(1,1);
		tr2(2,0) = tr(2,0)*cT5 - tr(2,2)*sT5;
		tr2(2,1) = -tr(2,0)*sT5 - tr(2,2)*cT5;
		tr2(2,2) = tr(2,1);
		tr2(0,3) = tr(0,0)*29.5 + tr(0,3);
		tr2(1,3) = tr(1,0)*29.5 + tr(1,3);
		tr2(2,3) = tr(2,0)*29.5 + tr(2,3);
		tr = tr2;
	
		for (unsigned int i=0; i<3; i++){
			pos_axis(i,nj) = tr(i,3);
			pos_axis(i+3,nj) = tr(i,0);
			pos_axis(i+6,nj) = tr(i,2);
		}
		nj++;
	
		// 2nd Tip angle
		sT6 = sin(config(18));	        cT6 = cos(config(18));
		tr2(0,0) = tr(0,0)*cT6 + tr(0,2)*sT6;
		tr2(0,1) = -tr(0,0)*sT6 + tr(0,2)*cT6;
		tr2(0,2) = -tr(0,1);
		tr2(1,0) = tr(1,0)*cT6 + tr(1,2)*sT6;
		tr2(1,1) = -tr(1,0)*sT6 + tr(1,2)*cT6;
		tr2(1,2) = -tr(1,1);
		tr2(2,0) = tr(2,0)*cT6 + tr(2,2)*sT6;
		tr2(2,1) = -tr(2,0)*sT6 + tr(2,2)*cT6;
		tr2(2,2) = -tr(2,1);
		tr2(0,3) = tr(0,3);
		tr2(1,3) = tr(1,3);
		tr2(2,3) = tr(2,3);
		tr = tr2;
	
		for (unsigned int i=0; i<3; i++){
			pos_axis(i,nj) = tr(i,3);
			pos_axis(i+3,nj) = tr(i,0);
			pos_axis(i+6,nj) = tr(i,2);
		}
		nj++;
	
		// Storing finger position and orientation
		for (unsigned int i=0; i<3; i++){
			pos_axis(i,nj) = tr(i,3)+FT_RAD*tr(i,0);
			pos_axis(i+3,nj) = tr(i,0);
			pos_axis(i+6,nj) = tr(i,2);
		}
		nj++;
	
	
	
		// ---- RING ----
		// 1st Finger angle
		sT1 = sin(config(19));               cT1 = cos(config(19));
		tr(0,0) = 0;
		tr(0,1) = 0;
		tr(0,2) = 1;
		tr(1,0) = -0.034301*cT1 - 0.99941*sT1;
		tr(1,1) = 0.034301*sT1 - 0.99941*cT1;
		tr(1,2) = 0;
		tr(2,0) = 0.99941*cT1 - 0.034301*sT1;
		tr(2,1) = -0.99941*sT1 - 0.034301*cT1;
		tr(2,2) = 0;
		tr(0,3) = -4.3;
		tr(1,3) = -40.165;
		tr(2,3) = 145.43;
	
		m2.clear();
		for (unsigned int i=0; i<3; i++)
			for (unsigned int j=0; j<4; j++)
				m2(i,j) = tr(i,j);
		m2(3,3) = 1;
		m2 = prod(wrist_base,m2);
		for (unsigned int i=0; i<3; i++)
			for (unsigned int j=0; j<4; j++)
				tr(i,j) = m2(i,j);
	
		for (unsigned int i=0; i<3; i++){
			pos_axis(i,nj) = tr(i,3);
			pos_axis(i+3,nj) = tr(i,0);
			pos_axis(i+6,nj) = tr(i,2);
		}
		nj++;
	
		// 2nd Finger angle
		sT2 = sin(config(20));               cT2 = cos(config(20));
		tr2(0,0) = tr(0,0)*cT2 + tr(0,2)*sT2;
		tr2(0,1) = -tr(0,0)*sT2 + tr(0,2)*cT2;
		tr2(0,2) = -tr(0,1);
		tr2(1,0) = tr(1,0)*cT2 + tr(1,2)*sT2;
		tr2(1,1) = -tr(1,0)*sT2 + tr(1,2)*cT2;
		tr2(1,2) = -tr(1,1);
		tr2(2,0) = tr(2,0)*cT2 + tr(2,2)*sT2;
		tr2(2,1) = -tr(2,0)*sT2 + tr(2,2)*cT2;
		tr2(2,2) = -tr(2,1);
		tr2(0,3) = tr(0,3);
		tr2(1,3) = tr(1,3);
		tr2(2,3) = tr(2,3);
		tr = tr2;
	
		for (unsigned int i=0; i<3; i++){
			pos_axis(i,nj) = tr(i,3);
			pos_axis(i+3,nj) = tr(i,0);
			pos_axis(i+6,nj) = tr(i,2);
		}
		nj++;
	
		// 3rd Finger angle
		sT3 = sin(config(21));	            cT3 = cos(config(21));
		tr2(0,0) = tr(0,0)*cT3 + tr(0,1)*sT3;
		tr2(0,1) = -tr(0,0)*sT3 + tr(0,1)*cT3;
		tr2(0,2) = tr(0,2);
		tr2(1,0) = tr(1,0)*cT3 + tr(1,1)*sT3;
		tr2(1,1) = -tr(1,0)*sT3 + tr(1,1)*cT3;
		tr2(1,2) = tr(1,2);
		tr2(2,0) = tr(2,0)*cT3 + tr(2,1)*sT3;
		tr2(2,1) = -tr(2,0)*sT3 + tr(2,1)*cT3;
		tr2(2,2) = tr(2,2);
		tr2(0,3) = tr(0,0)*67.8 + tr(0,3);
		tr2(1,3) = tr(1,0)*67.8 + tr(1,3);
		tr2(2,3) = tr(2,0)*67.8 + tr(2,3);
		tr = tr2;
	
		for (unsigned int i=0; i<3; i++){
			pos_axis(i,nj) = tr(i,3);
			pos_axis(i+3,nj) = tr(i,0);
			pos_axis(i+6,nj) = tr(i,2);
		}
		nj++;
	
		// 4th Finger angle
		sT4 = sin(config(22));        cT4 = cos(config(22));
		tr2(0,0) = tr(0,0)*cT4 + tr(0,1)*sT4;
		tr2(0,1) = -tr(0,0)*sT4 + tr(0,1)*cT4;
		tr2(0,2) = tr(0,2);
		tr2(1,0) = tr(1,0)*cT4 + tr(1,1)*sT4;
		tr2(1,1) = -tr(1,0)*sT4 + tr(1,1)*cT4;
		tr2(1,2) = tr(1,2);
		tr2(2,0) = tr(2,0)*cT4 + tr(2,1)*sT4;
		tr2(2,1) = -tr(2,0)*sT4 + tr(2,1)*cT4;
		tr2(2,2) = tr(2,2);
		tr2(0,3) = tr(0,0)*30 + tr(0,3);
		tr2(1,3) = tr(1,0)*30 + tr(1,3);
		tr2(2,3) = tr(2,0)*30 + tr(2,3);
		tr = tr2;
	
		for (unsigned int i=0; i<3; i++){
			pos_axis(i,nj) = tr(i,3);
			pos_axis(i+3,nj) = tr(i,0);
			pos_axis(i+6,nj) = tr(i,2);
		}
		nj++;
	
		// 1rt Tip angle
		sT5 = sin(config(23));          cT5 = cos(config(23));
		tr2(0,0) = tr(0,0)*cT5 - tr(0,2)*sT5;
		tr2(0,1) = -tr(0,0)*sT5 - tr(0,2)*cT5;
		tr2(0,2) = tr(0,1);
		tr2(1,0) = tr(1,0)*cT5 - tr(1,2)*sT5;
		tr2(1,1) = -tr(1,0)*sT5 - tr(1,2)*cT5;
		tr2(1,2) = tr(1,1);
		tr2(2,0) = tr(2,0)*cT5 - tr(2,2)*sT5;
		tr2(2,1) = -tr(2,0)*sT5 - tr(2,2)*cT5;
		tr2(2,2) = tr(2,1);
		tr2(0,3) = tr(0,0)*29.5 + tr(0,3);
		tr2(1,3) = tr(1,0)*29.5 + tr(1,3);
		tr2(2,3) = tr(2,0)*29.5 + tr(2,3);
		tr = tr2;
	
		for (unsigned int i=0; i<3; i++){
			pos_axis(i,nj) = tr(i,3);
			pos_axis(i+3,nj) = tr(i,0);
			pos_axis(i+6,nj) = tr(i,2);
		}
		nj++;
	
		// 2nd Tip angle
		sT6 = sin(config(24));	        cT6 = cos(config(24));
		tr2(0,0) = tr(0,0)*cT6 + tr(0,2)*sT6;
		tr2(0,1) = -tr(0,0)*sT6 + tr(0,2)*cT6;
		tr2(0,2) = -tr(0,1);
		tr2(1,0) = tr(1,0)*cT6 + tr(1,2)*sT6;
		tr2(1,1) = -tr(1,0)*sT6 + tr(1,2)*cT6;
		tr2(1,2) = -tr(1,1);
		tr2(2,0) = tr(2,0)*cT6 + tr(2,2)*sT6;
		tr2(2,1) = -tr(2,0)*sT6 + tr(2,2)*cT6;
		tr2(2,2) = -tr(2,1);
		tr2(0,3) = tr(0,3);
		tr2(1,3) = tr(1,3);
		tr2(2,3) = tr(2,3);
		tr = tr2;
	
		for (unsigned int i=0; i<3; i++){
			pos_axis(i,nj) = tr(i,3);
			pos_axis(i+3,nj) = tr(i,0);
			pos_axis(i+6,nj) = tr(i,2);
		}
		nj++;
	
		// Storing finger position and orientation
		for (unsigned int i=0; i<3; i++){
			pos_axis(i,nj) = tr(i,3)+FT_RAD*tr(i,0);
			pos_axis(i+3,nj) = tr(i,0);
			pos_axis(i+6,nj) = tr(i,2);
		}
		nj++;
	
	
		return pos_axis;
	}
	
	inline boost::numeric::ublas::matrix<double> IvKinHand::ThumbDirKin(const boost::numeric::ublas::vector<double> &thumb_config){
	
		// *************************************************************************************************************************
		// Function that calculates the position and axis of each joint given a wrist base and a configuration base
		//
		//	Input variables:
		//
		//	 wrist_base: a 4x4 homogeneous transformation matrix
		//
		//   configs: a 26 dimension sahand configuration vector as
		//      config(0) = rotation around thumb X axis
		//      config(1) = thumbrot
		//		config(2) = thumb abduction			config(3) = first thumb flexion			config(4) = second thumb flexion
		//		config(5) = third thumb flexion		config(6) = first thumb fingertip		config(7) = second thumb fingertip
		//		config(8) = index abduction			config(9) = first index flexion			config(10) = second index flexion
		//		config(11) = third index flexion	config(12) = first index fingertip		config(13) = second index fingertip
		//		config(14) = middle abduction		config(15) = first middle flexion		config(16) = second middle flexion
		//		config(17) = third middle flexion	config(18) = first middle fingertip		config(19) = second middle fingertip
		//		config(20) = ring abduction			config(21) = first ring flexion			config(22) = second ring flexion
		//		config(23) = third ring flexion		config(24) = first ring fingertip		config(25) = second ring fingertip
		//
		// *************************************************************************************************************************
	
	
		// ----- Asserting input variables -----
		assert(thumb_config.size() == N_TA_VDC_CONFIG);
		// ----- End Asserting input variables -----
	
	
		boost::numeric::ublas::matrix<double> pos_axis(9,30);	pos_axis.clear();
		boost::numeric::ublas::matrix<double> tr(3,4), tr2(3,4), trp(3,4);    // Transformation Matrix
		tr.clear();		tr2.clear();	trp.clear();
		unsigned int nj = 0;
	
		double ftr = FT_RAD;   //Fingertip Radius
	
	
		boost::numeric::ublas::vector<double> config(26);	config = thumb_config;
	
	
		// ---- THUMB ----
	// Alpha
		double sTa = sin(config(0));	double cTa = cos(config(0));
		tr(0,0) = cTa;
		tr(0,1) = -sTa;
		tr(1,0) = sTa;
		tr(1,1) = cTa;
		tr(2,2) = 1;
		nj = 8;
		for (unsigned int i=0; i<3; i++){
			pos_axis(i,nj) = 0;
			pos_axis(i+3,nj) = tr(i,0);
			pos_axis(i+6,nj) = tr(i,2);
		}
		nj--;
	
		// 2nd Tip angle
		double sT6 = sin(config(7)-(PI/2));	double cT6 = cos(config(7)-(PI/2));
		tr2(0,0) = tr(0,0)*cT6;
		tr2(0,1) = -tr(0,0)*sT6;
		tr2(0,2) = tr(0,1);
		tr2(1,0) = tr(1,0)*cT6;
		tr2(1,1) = -tr(1,0)*sT6;
		tr2(1,2) = tr(1,1);
		tr2(2,0) = -sT6;
		tr2(2,1) = -cT6;
		tr2(0,3) = 0;
		tr2(1,3) = 0;
		tr2(2,3) = ftr;
		tr = tr2;
		for (unsigned int i=0; i<3; i++){
			pos_axis(i,nj) = tr(i,3);
			pos_axis(i+3,nj) = tr(i,0);
			pos_axis(i+6,nj) = tr(i,2);
		}
		nj--;
	
		// 1rt Tip angle
		double sT5 = sin(config(6));	double cT5 = cos(config(6));
		tr2(0,0) = tr(0,0)*cT5 + tr(0,2)*sT5;
		tr2(0,1) = tr(0,0)*sT5 - tr(0,2)*cT5;
		tr2(0,2) = tr(0,1);
		tr2(1,0) = tr(1,0)*cT5 + tr(1,2)*sT5;
		tr2(1,1) = tr(1,0)*sT5 - tr(1,2)*cT5;
		tr2(1,2) = tr(1,1);
		tr2(2,0) = tr(2,0)*cT5 + tr(2,2)*sT5;
		tr2(2,1) = tr(2,0)*sT5 - tr(2,2)*cT5;
		tr2(2,2) = tr(2,1);
		tr2(0,3) = tr(0,3);
		tr2(1,3) = tr(1,3);
		tr2(2,3) = tr(2,3);
		tr = tr2;
		for (unsigned int i=0; i<3; i++){
			pos_axis(i,nj) = tr(i,3);
			pos_axis(i+3,nj) = tr(i,0);
			pos_axis(i+6,nj) = tr(i,2);
		}
		nj--;
	
		// 4th Finger angle
		double sT4 = sin(config(5));	double cT4 = cos(config(5));
		tr2(0,0) = tr(0,0)*cT4 + tr(0,2)*sT4;
		tr2(0,1) = -tr(0,0)*sT4 + tr(0,2)*cT4;
		tr2(0,2) = -tr(0,1);
		tr2(1,0) = tr(1,0)*cT4 + tr(1,2)*sT4;
		tr2(1,1) = -tr(1,0)*sT4 + tr(1,2)*cT4;
		tr2(1,2) = -tr(1,1);
		tr2(2,0) = tr(2,0)*cT4 + tr(2,2)*sT4;
		tr2(2,1) = -tr(2,0)*sT4 + tr(2,2)*cT4;
		tr2(2,2) = -tr(2,1);
		tr2(0,3) = tr(0,0)*29.5 + tr(0,3);
		tr2(1,3) = tr(1,0)*29.5 + tr(1,3);
		tr2(2,3) = tr(2,0)*29.5 + tr(2,3);
		tr = tr2;
		for (unsigned int i=0; i<3; i++){
			pos_axis(i,nj) = tr(i,3);
			pos_axis(i+3,nj) = tr(i,0);
			pos_axis(i+6,nj) = tr(i,2);
		}
		nj--;
	
		// 3rd Finger angle
		double sT3 = sin(config(4));	double cT3 = cos(config(4));
		tr2(0,0) = tr(0,0)*cT3 + tr(0,1)*sT3;
		tr2(0,1) = -tr(0,0)*sT3 + tr(0,1)*cT3;
		tr2(0,2) = tr(0,2);
		tr2(1,0) = tr(1,0)*cT3 + tr(1,1)*sT3;
		tr2(1,1) = -tr(1,0)*sT3 + tr(1,1)*cT3;
		tr2(1,2) = tr(1,2);
		tr2(2,0) = tr(2,0)*cT3 + tr(2,1)*sT3;
		tr2(2,1) = -tr(2,0)*sT3 + tr(2,1)*cT3;
		tr2(2,2) = tr(2,2);
		tr2(0,3) = tr(0,0)*30 + tr(0,3);
		tr2(1,3) = tr(1,0)*30 + tr(1,3);
		tr2(2,3) = tr(2,0)*30 + tr(2,3);
		tr = tr2;
		for (unsigned int i=0; i<3; i++){
			pos_axis(i,nj) = tr(i,3);
			pos_axis(i+3,nj) = tr(i,0);
			pos_axis(i+6,nj) = tr(i,2);
		}
		nj--;
	
		// 2nd Finger angle
		double sT2 = sin(config(3));	double cT2 = cos(config(3));
		tr2(0,0) = tr(0,0)*cT2 + tr(0,1)*sT2;
		tr2(0,1) = -tr(0,0)*sT2 + tr(0,1)*cT2;
		tr2(0,2) = tr(0,2);
		tr2(1,0) = tr(1,0)*cT2 + tr(1,1)*sT2;
		tr2(1,1) = -tr(1,0)*sT2 + tr(1,1)*cT2;
		tr2(1,2) = tr(1,2);
		tr2(2,0) = tr(2,0)*cT2 + tr(2,1)*sT2;
		tr2(2,1) = -tr(2,0)*sT2 + tr(2,1)*cT2;
		tr2(2,2) = tr(2,2);
		tr2(0,3) = tr(0,0)*67.8 + tr(0,3);
		tr2(1,3) = tr(1,0)*67.8 + tr(1,3);
		tr2(2,3) = tr(2,0)*67.8 + tr(2,3);
		tr = tr2;
		for (unsigned int i=0; i<3; i++){
			pos_axis(i,nj) = tr(i,3);
			pos_axis(i+3,nj) = tr(i,0);
			pos_axis(i+6,nj) = tr(i,2);
		}
		nj--;
	
		// 1st Finger angle
		double sT1 = sin(config(2));	double cT1 = cos(config(2));
		tr2(0,0) = tr(0,0)*cT1 + tr(0,2)*sT1;
		tr2(0,1) = tr(0,0)*sT1 - tr(0,2)*cT1;
		tr2(0,2) = tr(0,1);
		tr2(1,0) = tr(1,0)*cT1 + tr(1,2)*sT1;
		tr2(1,1) = tr(1,0)*sT1 - tr(1,2)*cT1;
		tr2(1,2) = tr(1,1);
		tr2(2,0) = tr(2,0)*cT1 + tr(2,2)*sT1;
		tr2(2,1) = tr(2,0)*sT1 - tr(2,2)*cT1;
		tr2(2,2) = tr(2,1);
		tr2(0,3) = tr(0,3);
		tr2(1,3) = tr(1,3);
		tr2(2,3) = tr(2,3);
		tr = tr2;
		for (unsigned int i=0; i<3; i++){
			pos_axis(i,nj) = tr(i,3);
			pos_axis(i+3,nj) = tr(i,0);
			pos_axis(i+6,nj) = tr(i,2);
		}
		nj--;
	
		// ThumbRotTF
		double sT0 = sin(config(1));	double cT0 = cos(config(1));
		tr2(0,0) = -0.57358*sT0*tr(0,0) + cT0*tr(0,1) - 0.81915*sT0*tr(0,2);
		tr2(0,1) = -0.57358*cT0*tr(0,0) - sT0*tr(0,1) - 0.81915*cT0*tr(0,2);
		tr2(0,2) = -0.81915*tr(0,0) + 0.57358*tr(0,2);
		tr2(1,0) = -0.57358*sT0*tr(1,0) + cT0*tr(1,1) - 0.81915*sT0*tr(1,2);
		tr2(1,1) = -0.57358*cT0*tr(1,0) - sT0*tr(1,1) - 0.81915*cT0*tr(1,2);
		tr2(1,2) = -0.81915*tr(1,0) + 0.57358*tr(1,2);
		tr2(2,0) = -0.57358*sT0*tr(2,0) + cT0*tr(2,1) - 0.81915*sT0*tr(2,2);
		tr2(2,1) = -0.57358*cT0*tr(2,0) - sT0*tr(2,1) - 0.81915*cT0*tr(2,2);
		tr2(2,2) = -0.81915*tr(2,0) + 0.57358*tr(2,2);
		tr2(0,3) = tr(0,0)*129.3016 + tr(0,1)*6 + tr(0,2)*15.5469 + tr(0,3);
		tr2(1,3) = tr(1,0)*129.3016 + tr(1,1)*6 + tr(1,2)*15.5469 + tr(1,3);
		tr2(2,3) = tr(2,0)*129.3016 + tr(2,1)*6 + tr(2,2)*15.5469 + tr(2,3);
		tr = tr2;
		for (unsigned int i=0; i<3; i++){
			pos_axis(i,nj) = tr(i,3);
			pos_axis(i+3,nj) = tr(i,0);
			pos_axis(i+6,nj) = tr(i,2);
		}
		nj = 9;
	
		// Wrist
		tr2(0,3) = 3*tr(0,0) - 27.1*tr(0,1) + tr(0,3);
		tr2(1,3) = 3*tr(1,0) - 27.1*tr(1,1) + tr(1,3);
		tr2(2,3) = 3*tr(2,0) - 27.1*tr(2,1) + tr(2,3);
		tr = tr2;
		trp = tr;
		for (unsigned int i=0; i<3; i++){
			pos_axis(i,0) = tr(i,3);
			pos_axis(i+3,0) = tr(i,0);
			pos_axis(i+6,0) = tr(i,2);
		}
	
	
		// ---- INDEX ----
		// 1st Finger angle
		sT1 = sin(config(8));               cT1 = cos(config(8));
		tr2(0,0) = (0.035485*trp(0,1) + 0.99937*trp(0,2))*cT1 + (-0.99937*trp(0,1) + 0.035485*trp(0,2))*sT1;
		tr2(0,1) = (-0.035485*trp(0,1) - 0.99937*trp(0,2))*sT1 + (-0.99937*trp(0,1) + 0.035485*trp(0,2))*cT1;
		tr2(0,2) = trp(0,0);
		tr2(1,0) = (0.035485*trp(1,1) + 0.99937*trp(1,2))*cT1 + (-0.99937*trp(1,1) + 0.035485*trp(1,2))*sT1;
		tr2(1,1) = (-0.035485*trp(1,1) - 0.99937*trp(1,2))*sT1 + (-0.99937*trp(1,1) + 0.035485*trp(1,2))*cT1;
		tr2(1,2) = trp(1,0);
		tr2(2,0) = (0.035485*trp(2,1) + 0.99937*trp(2,2))*cT1 + (-0.99937*trp(2,1) + 0.035485*trp(2,2))*sT1;
		tr2(2,1) = (-0.035485*trp(2,1) - 0.99937*trp(2,2))*sT1 + (-0.99937*trp(2,1) + 0.035485*trp(2,2))*cT1;
		tr2(2,2) = trp(2,0);
		tr2(0,3) = -4.3*trp(0,0) + 40.165*trp(0,1) + 145.43*trp(0,2) + trp(0,3);
		tr2(1,3) = -4.3*trp(1,0) + 40.165*trp(1,1) + 145.43*trp(1,2) + trp(1,3);
		tr2(2,3) = -4.3*trp(2,0) + 40.165*trp(2,1) + 145.43*trp(2,2) + trp(2,3);
		tr = tr2;
		for (unsigned int i=0; i<3; i++){
			pos_axis(i,nj) = tr(i,3);
			pos_axis(i+3,nj) = tr(i,0);
			pos_axis(i+6,nj) = tr(i,2);
		}
		nj++;
	
		// 2nd Finger angle
		sT2 = sin(config(9));               cT2 = cos(config(9));
		tr2(0,0) = tr(0,0)*cT2 + tr(0,2)*sT2;
		tr2(0,1) = -tr(0,0)*sT2 + tr(0,2)*cT2;
		tr2(0,2) = -tr(0,1);
		tr2(1,0) = tr(1,0)*cT2 + tr(1,2)*sT2;
		tr2(1,1) = -tr(1,0)*sT2 + tr(1,2)*cT2;
		tr2(1,2) = -tr(1,1);
		tr2(2,0) = tr(2,0)*cT2 + tr(2,2)*sT2;
		tr2(2,1) = -tr(2,0)*sT2 + tr(2,2)*cT2;
		tr2(2,2) = -tr(2,1);
		tr2(0,3) = tr(0,3);
		tr2(1,3) = tr(1,3);
		tr2(2,3) = tr(2,3);
		tr = tr2;
		for (unsigned int i=0; i<3; i++){
			pos_axis(i,nj) = tr(i,3);
			pos_axis(i+3,nj) = tr(i,0);
			pos_axis(i+6,nj) = tr(i,2);
		}
		nj++;
	
		// 3rd Finger angle
		sT3 = sin(config(10));	            cT3 = cos(config(10));
		tr2(0,0) = tr(0,0)*cT3 + tr(0,1)*sT3;
		tr2(0,1) = -tr(0,0)*sT3 + tr(0,1)*cT3;
		tr2(0,2) = tr(0,2);
		tr2(1,0) = tr(1,0)*cT3 + tr(1,1)*sT3;
		tr2(1,1) = -tr(1,0)*sT3 + tr(1,1)*cT3;
		tr2(1,2) = tr(1,2);
		tr2(2,0) = tr(2,0)*cT3 + tr(2,1)*sT3;
		tr2(2,1) = -tr(2,0)*sT3 + tr(2,1)*cT3;
		tr2(2,2) = tr(2,2);
		tr2(0,3) = tr(0,0)*67.8 + tr(0,3);
		tr2(1,3) = tr(1,0)*67.8 + tr(1,3);
		tr2(2,3) = tr(2,0)*67.8 + tr(2,3);
		tr = tr2;
		for (unsigned int i=0; i<3; i++){
			pos_axis(i,nj) = tr(i,3);
			pos_axis(i+3,nj) = tr(i,0);
			pos_axis(i+6,nj) = tr(i,2);
		}
		nj++;
	
		// 4th Finger angle
		sT4 = sin(config(11));        cT4 = cos(config(11));
		tr2(0,0) = tr(0,0)*cT4 + tr(0,1)*sT4;
		tr2(0,1) = -tr(0,0)*sT4 + tr(0,1)*cT4;
		tr2(0,2) = tr(0,2);
		tr2(1,0) = tr(1,0)*cT4 + tr(1,1)*sT4;
		tr2(1,1) = -tr(1,0)*sT4 + tr(1,1)*cT4;
		tr2(1,2) = tr(1,2);
		tr2(2,0) = tr(2,0)*cT4 + tr(2,1)*sT4;
		tr2(2,1) = -tr(2,0)*sT4 + tr(2,1)*cT4;
		tr2(2,2) = tr(2,2);
		tr2(0,3) = tr(0,0)*30 + tr(0,3);
		tr2(1,3) = tr(1,0)*30 + tr(1,3);
		tr2(2,3) = tr(2,0)*30 + tr(2,3);
		tr = tr2;
		for (unsigned int i=0; i<3; i++){
			pos_axis(i,nj) = tr(i,3);
			pos_axis(i+3,nj) = tr(i,0);
			pos_axis(i+6,nj) = tr(i,2);
		}
		nj++;
	
		// 1rt Tip angle
		sT5 = sin(config(12));          cT5 = cos(config(12));
		tr2(0,0) = tr(0,0)*cT5 - tr(0,2)*sT5;
		tr2(0,1) = -tr(0,0)*sT5 - tr(0,2)*cT5;
		tr2(0,2) = tr(0,1);
		tr2(1,0) = tr(1,0)*cT5 - tr(1,2)*sT5;
		tr2(1,1) = -tr(1,0)*sT5 - tr(1,2)*cT5;
		tr2(1,2) = tr(1,1);
		tr2(2,0) = tr(2,0)*cT5 - tr(2,2)*sT5;
		tr2(2,1) = -tr(2,0)*sT5 - tr(2,2)*cT5;
		tr2(2,2) = tr(2,1);
		tr2(0,3) = tr(0,0)*29.5 + tr(0,3);
		tr2(1,3) = tr(1,0)*29.5 + tr(1,3);
		tr2(2,3) = tr(2,0)*29.5 + tr(2,3);
		tr = tr2;
		for (unsigned int i=0; i<3; i++){
			pos_axis(i,nj) = tr(i,3);
			pos_axis(i+3,nj) = tr(i,0);
			pos_axis(i+6,nj) = tr(i,2);
		}
		nj++;
	
		// 2nd Tip angle
		sT6 = sin(config(13));	        cT6 = cos(config(13));
		tr2(0,0) = tr(0,0)*cT6 + tr(0,2)*sT6;
		tr2(0,1) = -tr(0,0)*sT6 + tr(0,2)*cT6;
		tr2(0,2) = -tr(0,1);
		tr2(1,0) = tr(1,0)*cT6 + tr(1,2)*sT6;
		tr2(1,1) = -tr(1,0)*sT6 + tr(1,2)*cT6;
		tr2(1,2) = -tr(1,1);
		tr2(2,0) = tr(2,0)*cT6 + tr(2,2)*sT6;
		tr2(2,1) = -tr(2,0)*sT6 + tr(2,2)*cT6;
		tr2(2,2) = -tr(2,1);
		tr2(0,3) = tr(0,3);
		tr2(1,3) = tr(1,3);
		tr2(2,3) = tr(2,3);
		tr = tr2;
		for (unsigned int i=0; i<3; i++){
			pos_axis(i,nj) = tr(i,3);
			pos_axis(i+3,nj) = tr(i,0);
			pos_axis(i+6,nj) = tr(i,2);
		}
		nj++;
	
		// Storing finger position and orientation
		for (unsigned int i=0; i<3; i++){
			pos_axis(i,nj) = tr(i,3)+FT_RAD*tr(i,0);
			pos_axis(i+3,nj) = tr(i,0);
			pos_axis(i+6,nj) = tr(i,2);
		}
		nj++;
	
	
		// ---- MIDDLE ----
		// 1st Finger angle
		sT1 = sin(config(14));               cT1 = cos(config(14));
		tr2(0,0) = trp(0,2)*cT1 - trp(0,1)*sT1;
		tr2(0,1) = -trp(0,2)*sT1 - trp(0,1)*cT1;
		tr2(0,2) = trp(0,0);
		tr2(1,0) = trp(1,2)*cT1 - trp(1,1)*sT1;
		tr2(1,1) = -trp(1,2)*sT1 - trp(1,1)*cT1;
		tr2(1,2) = trp(1,0);
		tr2(2,0) = trp(2,2)*cT1 - trp(2,1)*sT1;
		tr2(2,1) = -trp(2,2)*sT1 - trp(2,1)*cT1;
		tr2(2,2) = trp(2,0);
		tr2(0,3) = -4.3*trp(0,0) + 150.15*trp(0,2) + trp(0,3);
		tr2(1,3) = -4.3*trp(1,0) + 150.15*trp(1,2) + trp(1,3);
		tr2(2,3) = -4.3*trp(2,0) + 150.15*trp(2,2) + trp(2,3);
		tr = tr2;
		for (unsigned int i=0; i<3; i++){
			pos_axis(i,nj) = tr(i,3);
			pos_axis(i+3,nj) = tr(i,0);
			pos_axis(i+6,nj) = tr(i,2);
		}
		nj++;
	
		// 2nd Finger angle
		sT2 = sin(config(15));               cT2 = cos(config(15));
		tr2(0,0) = tr(0,0)*cT2 + tr(0,2)*sT2;
		tr2(0,1) = -tr(0,0)*sT2 + tr(0,2)*cT2;
		tr2(0,2) = -tr(0,1);
		tr2(1,0) = tr(1,0)*cT2 + tr(1,2)*sT2;
		tr2(1,1) = -tr(1,0)*sT2 + tr(1,2)*cT2;
		tr2(1,2) = -tr(1,1);
		tr2(2,0) = tr(2,0)*cT2 + tr(2,2)*sT2;
		tr2(2,1) = -tr(2,0)*sT2 + tr(2,2)*cT2;
		tr2(2,2) = -tr(2,1);
		tr2(0,3) = tr(0,3);
		tr2(1,3) = tr(1,3);
		tr2(2,3) = tr(2,3);
		tr = tr2;
		for (unsigned int i=0; i<3; i++){
			pos_axis(i,nj) = tr(i,3);
			pos_axis(i+3,nj) = tr(i,0);
			pos_axis(i+6,nj) = tr(i,2);
		}
		nj++;
	
		// 3rd Finger angle
		sT3 = sin(config(16));	            cT3 = cos(config(16));
		tr2(0,0) = tr(0,0)*cT3 + tr(0,1)*sT3;
		tr2(0,1) = -tr(0,0)*sT3 + tr(0,1)*cT3;
		tr2(0,2) = tr(0,2);
		tr2(1,0) = tr(1,0)*cT3 + tr(1,1)*sT3;
		tr2(1,1) = -tr(1,0)*sT3 + tr(1,1)*cT3;
		tr2(1,2) = tr(1,2);
		tr2(2,0) = tr(2,0)*cT3 + tr(2,1)*sT3;
		tr2(2,1) = -tr(2,0)*sT3 + tr(2,1)*cT3;
		tr2(2,2) = tr(2,2);
		tr2(0,3) = tr(0,0)*67.8 + tr(0,3);
		tr2(1,3) = tr(1,0)*67.8 + tr(1,3);
		tr2(2,3) = tr(2,0)*67.8 + tr(2,3);
		tr = tr2;
		for (unsigned int i=0; i<3; i++){
			pos_axis(i,nj) = tr(i,3);
			pos_axis(i+3,nj) = tr(i,0);
			pos_axis(i+6,nj) = tr(i,2);
		}
		nj++;
	
		// 4th Finger angle
		sT4 = sin(config(17));        cT4 = cos(config(17));
		tr2(0,0) = tr(0,0)*cT4 + tr(0,1)*sT4;
		tr2(0,1) = -tr(0,0)*sT4 + tr(0,1)*cT4;
		tr2(0,2) = tr(0,2);
		tr2(1,0) = tr(1,0)*cT4 + tr(1,1)*sT4;
		tr2(1,1) = -tr(1,0)*sT4 + tr(1,1)*cT4;
		tr2(1,2) = tr(1,2);
		tr2(2,0) = tr(2,0)*cT4 + tr(2,1)*sT4;
		tr2(2,1) = -tr(2,0)*sT4 + tr(2,1)*cT4;
		tr2(2,2) = tr(2,2);
		tr2(0,3) = tr(0,0)*30 + tr(0,3);
		tr2(1,3) = tr(1,0)*30 + tr(1,3);
		tr2(2,3) = tr(2,0)*30 + tr(2,3);
		tr = tr2;
		for (unsigned int i=0; i<3; i++){
			pos_axis(i,nj) = tr(i,3);
			pos_axis(i+3,nj) = tr(i,0);
			pos_axis(i+6,nj) = tr(i,2);
		}
		nj++;
	
		// 1rt Tip angle
		sT5 = sin(config(18));          cT5 = cos(config(18));
		tr2(0,0) = tr(0,0)*cT5 - tr(0,2)*sT5;
		tr2(0,1) = -tr(0,0)*sT5 - tr(0,2)*cT5;
		tr2(0,2) = tr(0,1);
		tr2(1,0) = tr(1,0)*cT5 - tr(1,2)*sT5;
		tr2(1,1) = -tr(1,0)*sT5 - tr(1,2)*cT5;
		tr2(1,2) = tr(1,1);
		tr2(2,0) = tr(2,0)*cT5 - tr(2,2)*sT5;
		tr2(2,1) = -tr(2,0)*sT5 - tr(2,2)*cT5;
		tr2(2,2) = tr(2,1);
		tr2(0,3) = tr(0,0)*29.5 + tr(0,3);
		tr2(1,3) = tr(1,0)*29.5 + tr(1,3);
		tr2(2,3) = tr(2,0)*29.5 + tr(2,3);
		tr = tr2;
		for (unsigned int i=0; i<3; i++){
			pos_axis(i,nj) = tr(i,3);
			pos_axis(i+3,nj) = tr(i,0);
			pos_axis(i+6,nj) = tr(i,2);
		}
		nj++;
	
		// 2nd Tip angle
		sT6 = sin(config(19));	        cT6 = cos(config(19));
		tr2(0,0) = tr(0,0)*cT6 + tr(0,2)*sT6;
		tr2(0,1) = -tr(0,0)*sT6 + tr(0,2)*cT6;
		tr2(0,2) = -tr(0,1);
		tr2(1,0) = tr(1,0)*cT6 + tr(1,2)*sT6;
		tr2(1,1) = -tr(1,0)*sT6 + tr(1,2)*cT6;
		tr2(1,2) = -tr(1,1);
		tr2(2,0) = tr(2,0)*cT6 + tr(2,2)*sT6;
		tr2(2,1) = -tr(2,0)*sT6 + tr(2,2)*cT6;
		tr2(2,2) = -tr(2,1);
		tr2(0,3) = tr(0,3);
		tr2(1,3) = tr(1,3);
		tr2(2,3) = tr(2,3);
		tr = tr2;
		for (unsigned int i=0; i<3; i++){
			pos_axis(i,nj) = tr(i,3);
			pos_axis(i+3,nj) = tr(i,0);
			pos_axis(i+6,nj) = tr(i,2);
		}
		nj++;
	
		// Storing finger position and orientation
		for (unsigned int i=0; i<3; i++){
			pos_axis(i,nj) = tr(i,3)+FT_RAD*tr(i,0);
			pos_axis(i+3,nj) = tr(i,0);
			pos_axis(i+6,nj) = tr(i,2);
		}
		nj++;
	
	
		// ---- RING ----
		// 1st Finger angle
		sT1 = sin(config(20));               cT1 = cos(config(20));
		tr2(0,0) = (-0.034301*trp(0,1) + 0.99937*trp(0,2))*cT1 + (-0.99937*trp(0,1) - 0.034301*trp(0,2))*sT1;
		tr2(0,1) = (0.034301*trp(0,1) - 0.99937*trp(0,2))*sT1 + (-0.99937*trp(0,1) - 0.034301*trp(0,2))*cT1;
		tr2(0,2) = trp(0,0);
		tr2(1,0) = (-0.034301*trp(1,1) + 0.99937*trp(1,2))*cT1 + (-0.99937*trp(1,1) - 0.034301*trp(1,2))*sT1;
		tr2(1,1) = (0.034301*trp(1,1) - 0.99937*trp(1,2))*sT1 + (-0.99937*trp(1,1) - 0.034301*trp(1,2))*cT1;
		tr2(1,2) = trp(1,0);
		tr2(2,0) = (-0.034301*trp(2,1) + 0.99937*trp(2,2))*cT1 + (-0.99937*trp(2,1) - 0.034301*trp(2,2))*sT1;
		tr2(2,1) = (0.034301*trp(2,1) - 0.99937*trp(2,2))*sT1 + (-0.99937*trp(2,1) - 0.034301*trp(2,2))*cT1;
		tr2(2,2) = trp(2,0);
		tr2(0,3) = -4.3*trp(0,0) - 40.165*trp(0,1) + 145.43*trp(0,2) + trp(0,3);
		tr2(1,3) = -4.3*trp(1,0) - 40.165*trp(1,1) + 145.43*trp(1,2) + trp(1,3);
		tr2(2,3) = -4.3*trp(2,0) - 40.165*trp(2,1) + 145.43*trp(2,2) + trp(2,3);
		tr = tr2;
		for (unsigned int i=0; i<3; i++){
			pos_axis(i,nj) = tr(i,3);
			pos_axis(i+3,nj) = tr(i,0);
			pos_axis(i+6,nj) = tr(i,2);
		}
		nj++;
	
		// 2nd Finger angle
		sT2 = sin(config(21));               cT2 = cos(config(21));
		tr2(0,0) = tr(0,0)*cT2 + tr(0,2)*sT2;
		tr2(0,1) = -tr(0,0)*sT2 + tr(0,2)*cT2;
		tr2(0,2) = -tr(0,1);
		tr2(1,0) = tr(1,0)*cT2 + tr(1,2)*sT2;
		tr2(1,1) = -tr(1,0)*sT2 + tr(1,2)*cT2;
		tr2(1,2) = -tr(1,1);
		tr2(2,0) = tr(2,0)*cT2 + tr(2,2)*sT2;
		tr2(2,1) = -tr(2,0)*sT2 + tr(2,2)*cT2;
		tr2(2,2) = -tr(2,1);
		tr2(0,3) = tr(0,3);
		tr2(1,3) = tr(1,3);
		tr2(2,3) = tr(2,3);
		tr = tr2;
		for (unsigned int i=0; i<3; i++){
			pos_axis(i,nj) = tr(i,3);
			pos_axis(i+3,nj) = tr(i,0);
			pos_axis(i+6,nj) = tr(i,2);
		}
		nj++;
	
		// 3rd Finger angle
		sT3 = sin(config(22));	            cT3 = cos(config(22));
		tr2(0,0) = tr(0,0)*cT3 + tr(0,1)*sT3;
		tr2(0,1) = -tr(0,0)*sT3 + tr(0,1)*cT3;
		tr2(0,2) = tr(0,2);
		tr2(1,0) = tr(1,0)*cT3 + tr(1,1)*sT3;
		tr2(1,1) = -tr(1,0)*sT3 + tr(1,1)*cT3;
		tr2(1,2) = tr(1,2);
		tr2(2,0) = tr(2,0)*cT3 + tr(2,1)*sT3;
		tr2(2,1) = -tr(2,0)*sT3 + tr(2,1)*cT3;
		tr2(2,2) = tr(2,2);
		tr2(0,3) = tr(0,0)*67.8 + tr(0,3);
		tr2(1,3) = tr(1,0)*67.8 + tr(1,3);
		tr2(2,3) = tr(2,0)*67.8 + tr(2,3);
		tr = tr2;
		for (unsigned int i=0; i<3; i++){
			pos_axis(i,nj) = tr(i,3);
			pos_axis(i+3,nj) = tr(i,0);
			pos_axis(i+6,nj) = tr(i,2);
		}
		nj++;
	
		// 4th Finger angle
		sT4 = sin(config(23));        cT4 = cos(config(23));
		tr2(0,0) = tr(0,0)*cT4 + tr(0,1)*sT4;
		tr2(0,1) = -tr(0,0)*sT4 + tr(0,1)*cT4;
		tr2(0,2) = tr(0,2);
		tr2(1,0) = tr(1,0)*cT4 + tr(1,1)*sT4;
		tr2(1,1) = -tr(1,0)*sT4 + tr(1,1)*cT4;
		tr2(1,2) = tr(1,2);
		tr2(2,0) = tr(2,0)*cT4 + tr(2,1)*sT4;
		tr2(2,1) = -tr(2,0)*sT4 + tr(2,1)*cT4;
		tr2(2,2) = tr(2,2);
		tr2(0,3) = tr(0,0)*30 + tr(0,3);
		tr2(1,3) = tr(1,0)*30 + tr(1,3);
		tr2(2,3) = tr(2,0)*30 + tr(2,3);
		tr = tr2;
		for (unsigned int i=0; i<3; i++){
			pos_axis(i,nj) = tr(i,3);
			pos_axis(i+3,nj) = tr(i,0);
			pos_axis(i+6,nj) = tr(i,2);
		}
		nj++;
	
		// 1rt Tip angle
		sT5 = sin(config(24));          cT5 = cos(config(24));
		tr2(0,0) = tr(0,0)*cT5 - tr(0,2)*sT5;
		tr2(0,1) = -tr(0,0)*sT5 - tr(0,2)*cT5;
		tr2(0,2) = tr(0,1);
		tr2(1,0) = tr(1,0)*cT5 - tr(1,2)*sT5;
		tr2(1,1) = -tr(1,0)*sT5 - tr(1,2)*cT5;
		tr2(1,2) = tr(1,1);
		tr2(2,0) = tr(2,0)*cT5 - tr(2,2)*sT5;
		tr2(2,1) = -tr(2,0)*sT5 - tr(2,2)*cT5;
		tr2(2,2) = tr(2,1);
		tr2(0,3) = tr(0,0)*29.5 + tr(0,3);
		tr2(1,3) = tr(1,0)*29.5 + tr(1,3);
		tr2(2,3) = tr(2,0)*29.5 + tr(2,3);
		tr = tr2;
		for (unsigned int i=0; i<3; i++){
			pos_axis(i,nj) = tr(i,3);
			pos_axis(i+3,nj) = tr(i,0);
			pos_axis(i+6,nj) = tr(i,2);
		}
		nj++;
	
		// 2nd Tip angle
		sT6 = sin(config(25));	        cT6 = cos(config(25));
		tr2(0,0) = tr(0,0)*cT6 + tr(0,2)*sT6;
		tr2(0,1) = -tr(0,0)*sT6 + tr(0,2)*cT6;
		tr2(0,2) = -tr(0,1);
		tr2(1,0) = tr(1,0)*cT6 + tr(1,2)*sT6;
		tr2(1,1) = -tr(1,0)*sT6 + tr(1,2)*cT6;
		tr2(1,2) = -tr(1,1);
		tr2(2,0) = tr(2,0)*cT6 + tr(2,2)*sT6;
		tr2(2,1) = -tr(2,0)*sT6 + tr(2,2)*cT6;
		tr2(2,2) = -tr(2,1);
		tr2(0,3) = tr(0,3);
		tr2(1,3) = tr(1,3);
		tr2(2,3) = tr(2,3);
		tr = tr2;
		for (unsigned int i=0; i<3; i++){
			pos_axis(i,nj) = tr(i,3);
			pos_axis(i+3,nj) = tr(i,0);
			pos_axis(i+6,nj) = tr(i,2);
		}
		nj++;
	
		// Storing finger position and orientation
		for (unsigned int i=0; i<3; i++){
			pos_axis(i,nj) = tr(i,3)+FT_RAD*tr(i,0);
			pos_axis(i+3,nj) = tr(i,0);
			pos_axis(i+6,nj) = tr(i,2);
		}
	
		return pos_axis;
	}
	
	inline void IvKinHand::JacobianAndThumbDirKin(const boost::numeric::ublas::vector<double> &configuration){
		// *************************************************************************************************************************
		// Function that calculates the direct kinematics and the jacobian for a certain thumb configuration
		//
		//	Input variables:
		//      configuration(0) = hand rotation around thumb axis
		//      configuration(1) = thumbrot
		//		configuration(2) = thumb abduction			configuration(3) = first thumb flexion			configuration(4) = second thumb flexion
		//		configuration(5) = third thumb flexion		configuration(6) = first thumb fingertip		configuration(7) = second thumb fingertip
		//		configuration(8) = index abduction			configuration(9) = first index flexion			configuration(10) = second index flexion
		//		configuration(11) = third index flexion		configuration(12) = first index fingertip		configuration(13) = second index fingertip
		//		configuration(14) = middle abduction		configuration(15) = first middle flexion		configuration(16) = second middle flexion
		//		configuration(17) = third middle flexion	configuration(18) = first middle fingertip		configuration(19) = second middle fingertip
		//		configuration(20) = ring abduction			configuration(21) = first ring flexion			configuration(22) = second ring flexion
		//		configuration(23) = third ring flexion		configuration(24) = first ring fingertip		configuration(25) = second ring fingertip
		//
		//	Output variables:
		//	  JacNEffPos.jacobian: a 22x26 matrix. First 18x26 submatrix is the jacobian matrix. Second 4x26 matrix relates to the 
		//	   coupling of angles of the sahand.
		//
		//    JacNEffPos.effective_position:
		//	   vector of dimension 18 with the information of the finger and fingertip positions 
		//	    referenced from the thumb base as
		//		effective_position(0:2) = Index desired position		effective_position(3:5) = Index desired inside fingertip position
		//		effective_position(6:8) = Middle desired position		effective_position(9:11) = Middle desired inside fingertip position
		//		effective_position(12:14) = Ring desired position		effective_position(15:17) = Ring desired inside fingertip position
		//
		//
		// *************************************************************************************************************************
	
	
		// ----- Asserting input variables -----
		assert(configuration.size() == N_TA_VDC_CONFIG);	// Checking size
		// ----- End Asserting input variables -----
	
	
		boost::numeric::ublas::matrix<double> tdk;
		tdk = ThumbDirKin(configuration);
		thumb_dir_kin = tdk;
	
		for (unsigned int i=0; i<3; i++){
			// Index
			effective_position(i) = tdk(i,15);
			effective_position(i+3) = tdk(i,15) - FT_RAD*tdk(i+3,15);
			// Middle
			effective_position(i+6) = tdk(i,22);
			effective_position(i+9) = tdk(i,22) - FT_RAD*tdk(i+3,22);
			// Ring
			effective_position(i+12) = tdk(i,29);
			effective_position(i+15) = tdk(i,29) - FT_RAD*tdk(i+3,29);
		}
	
	
		// **** Constructing the Jacobian ****
		jacobian.clear();
	
		// Origin, z Axis, End Effector, Fingertip Position, End Effector Linear Velocity, Fingertip linear velocity
		boost::numeric::ublas::vector<double> o(3), z(3), efp(3), eft(3), lvp(3), lvt(3);    
	
		// ---- INDEX ----
		for (unsigned int i = 0; i < 3; i++){
			efp(i) = tdk(i,15);						// Obtaining end effector of finger
			eft(i) = tdk(i,14);						// Obtaining inside fingertip position
		}
		//   Thumb angle
		for (unsigned int i = 0; i < 3; i++){		
			o(i) = tdk(i,8);			// Obtaining origin of joint jt base
			z(i) = tdk(i+6,8);			// Obtaining axis z of joint jt base
		}
		
		lvp = cross_p(z,efp-o);					// Calculating linear velocity
		lvt = cross_p(z,eft-o);					// Calculating linear velocity
		for (unsigned int i = 0; i < 3; i++){            // Filling Jacobian matrix
			jacobian(i,0) = lvp(i);
			jacobian(i+3,0) = lvt(i);
		}
		for (unsigned int jt = 1; jt < 8; jt++){
			for (unsigned int i = 0; i < 3; i++){
				o(i) = tdk(i,jt);			// Obtaining origin of joint jt base
				z(i) = tdk(i+6,jt);			// Obtaining axis z of joint jt base
			}
			if (jt == 2)	z = -z;
			if (jt == 6)	z = -z;
	
	
			lvp = cross_p(z,efp-o);					// Calculating linear velocity
			lvt = cross_p(z,eft-o);					// Calculating linear velocity
	
			for (unsigned int i = 0; i < 3; i++){            // Filling Jacobian matrix
				jacobian(i,jt) = lvp(i);
				jacobian(i+3,jt) = lvt(i);
			}
		}
		for (unsigned int jt = 9; jt < 15; jt++){
			for (unsigned int i = 0; i < 3; i++){
				o(i) = tdk(i,jt);			// Obtaining origin of joint jt base
				z(i) = tdk(i+6,jt);			// Obtaining axis z of joint jt base
			}
			lvp = cross_p(z,efp-o);					// calculating linear velocity
			lvt = cross_p(z,eft-o);					// calculating linear velocity
	
			for (unsigned int i = 0; i < 3; i++){            // filling jacobian matrix
				jacobian(i,jt-1) = lvp(i);
				jacobian(i+3,jt-1) = lvt(i);
			}
		}
	
		// ---- MIDDLE ----
		for (unsigned int i = 0; i < 3; i++){
			efp(i) = tdk(i,22);						// Obtaining end effector of finger
			eft(i) = tdk(i,21);						// Obtaining inside fingertip position
		}
		//   Thumb angle
		for (unsigned int i = 0; i < 3; i++){		
			o(i) = tdk(i,8);			// Obtaining origin of joint jt base
			z(i) = tdk(i+6,8);			// Obtaining axis z of joint jt base
		}
		lvp = cross_p(z,efp-o);					// Calculating linear velocity
		lvt = cross_p(z,eft-o);					// Calculating linear velocity
		for (unsigned int i = 0; i < 3; i++){            // Filling Jacobian matrix
			jacobian(i+6,0) = lvp(i);
			jacobian(i+9,0) = lvt(i);
		}
		for (unsigned int jt = 1; jt < 8; jt++){
			for (unsigned int i = 0; i < 3; i++){
				o(i) = tdk(i,jt);			// Obtaining origin of joint jt base
				z(i) = tdk(i+6,jt);			// Obtaining axis z of joint jt base
			}
			if (jt == 2)	z = -z;
			if (jt == 6)	z = -z;
	
			lvp = cross_p(z,efp-o);					// Calculating linear velocity
			lvt = cross_p(z,eft-o);					// Calculating linear velocity
	
			for (unsigned int i = 0; i < 3; i++){            // Filling Jacobian matrix
				jacobian(i+6,jt) = lvp(i);
				jacobian(i+9,jt) = lvt(i);
			}
		}
		for (unsigned int jt = 16; jt < 22; jt++){
			for (unsigned int i = 0; i < 3; i++){
				o(i) = tdk(i,jt);			// Obtaining origin of joint jt base
				z(i) = tdk(i+6,jt);			// Obtaining axis z of joint jt base
			}
			lvp = cross_p(z,efp-o);                // Calculating linear velocity
			lvt = cross_p(z,eft-o);                // Calculating linear velocity
		
			for (unsigned int i = 0; i < 3; i++){          // Filling Jacobian matrix
				jacobian(i+6,jt-2) = lvp(i);
				jacobian(i+9,jt-2) = lvt(i);
			}
		}
	
		// ---- RING ----
		for (unsigned int i = 0; i < 3; i++){
			efp(i) = tdk(i,29);						// Obtaining end effector of finger
			eft(i) = tdk(i,28);						// Obtaining inside fingertip position
		}
		//   Thumb angle
		for (unsigned int i = 0; i < 3; i++){		
			o(i) = tdk(i,8);			// Obtaining origin of joint jt base
			z(i) = tdk(i+6,8);			// Obtaining axis z of joint jt base
		}
		lvp = cross_p(z,efp-o);					// Calculating linear velocity
		lvt = cross_p(z,eft-o);					// Calculating linear velocity
		for (unsigned int i = 0; i < 3; i++){            // Filling Jacobian matrix
			jacobian(i+12,0) = lvp(i);
			jacobian(i+15,0) = lvt(i);
		}
		for (unsigned int jt = 1; jt < 8; jt++){
			for (unsigned int i = 0; i < 3; i++){
				o(i) = tdk(i,jt);			// Obtaining origin of joint jt base
				z(i) = tdk(i+6,jt);			// Obtaining axis z of joint jt base
			}
			if (jt == 2)	z = -z;
			if (jt == 6)	z = -z;
	
			lvp = cross_p(z,efp-o);					// Calculating linear velocity
			lvt = cross_p(z,eft-o);					// Calculating linear velocity
	
			for (unsigned int i = 0; i < 3; i++){	// Filling Jacobian matrix
				jacobian(i+12,jt) = lvp(i);
				jacobian(i+15,jt) = lvt(i);
			}
		}
	
		for (unsigned int jt = 23; jt < 29; jt++){
			for (unsigned int i = 0; i < 3; i++){
				o(i) = tdk(i,jt);			// Obtaining origin of joint jt base
				z(i) = tdk(i+6,jt);			// Obtaining axis z of joint jt base
			}
	
			lvp = cross_p(z,efp-o);                // Calculating linear velocity
			lvt = cross_p(z,eft-o);                // Calculating linear velocity
	
			for (unsigned int i = 0; i < 3; i++){          // Filling Jacobian matrix
				jacobian(i+12,jt-3) = lvp(i);
				jacobian(i+15,jt-3) = lvt(i);
			}
	
		}
	
		// Coupled joints
		//	Thumb coupling
		jacobian(18,4) = 1;
		jacobian(18,5) = -1;
		//	Index coupling
		jacobian(19,10) = 1;
		jacobian(19,11) = -1;
		//	Middle coupling
		jacobian(20,16) = 1;
		jacobian(20,17) = -1;
		//	Ring coupling
		jacobian(21,22) = 1;
		jacobian(21,23) = -1;
	
	
		return;
	}
	
	
	/*************************************************************
	Funcions matem�tiques
	************************************************************/
	
	inline boost::numeric::ublas::vector<double> IvKinHand::cross_p(const boost::numeric::ublas::vector<double>& p1,
																		const boost::numeric::ublas::vector<double>& p2){
	
	/*********** INPUT VARIABLES ************
	
	---- const boost::numeric::ublas::vector<double> p1(3); ----
	---- const boost::numeric::ublas::vector<double> p2(3); ----
	
	
	*********** OUTPUT VARIABLES ************
	
	---- const boost::numeric::ublas::vector<double> cp(3); ----
	
	cp results of the cross product of p1 and p2 as:    cp = p1 x p2
	
	
	*****************************************/
	
		boost::numeric::ublas::vector<double> cp(3);
	
		cp(0) = p1(1) * p2(2) - p1(2) * p2(1);
		cp(1) = p1(2) * p2(0) - p1(0) * p2(2);
		cp(2) = p1(0) * p2(1) - p1(1) * p2(0);
	
	return cp;
	}
	
	inline double IvKinHand::angle_2_vec_F(const boost::numeric::ublas::vector<double>& vec1, 
						const boost::numeric::ublas::vector<double>& vec2) {
		// Function that calculates the angle between two vectors of dimension '3'
	
		assert( (norm_2(vec1) > 0) && (norm_2(vec2) > 0) );
	
        double tmp = prod_esc(vec1,vec2)/( norm_2(vec1)*norm_2(vec2) );
        if (tmp < -1. || tmp > 1.) throw std::invalid_argument("Called acos(x) with |x| > 1");
        double result = acos( tmp );
		return result;
	}
	
	inline double IvKinHand::prod_esc(const boost::numeric::ublas::vector<double>& vec1, const boost::numeric::ublas::vector<double>& vec2) {
		// Scalar product of two vectors of dimension 'n'
		double result = 0;
	
		unsigned i = 0;
		for (i = 0; i < 3; i++)
			result = result + vec1(i)*vec2(i);
	
		return result;
	}
	
	inline double IvKinHand::sqrmod(const boost::numeric::ublas::vector<double>& vec){
	
		assert(vec.size() > 0);
	
		double mod = 0;
	
		for (unsigned int i=0; i<vec.size(); i++)
			mod += vec(i)*vec(i);
	
		return mod;
	}
	
	
	inline void IvKinHand::DataPlot(void){
	
		// Open file
		ofstream myfile_plottraindata;
		myfile_plottraindata.open("file_output_plotdata.txt",std::ios::app);
	
	
		// Plot variables
		myfile_plottraindata << " ---------------------------------------------------------------------------------------- " << std::endl << std::endl;
	
		myfile_plottraindata << "vari1 = " << vari1 << std::endl;
		myfile_plottraindata << "vari2 = " << vari2 << std::endl;
		//myfile_plottraindata << "vari3 = " << vari3 << std::endl;
		//myfile_plottraindata << "vari4 = " << vari4 << std::endl << std::endl;
	
		//myfile_plottraindata << "vard1 = " << vard1 << std::endl;
		//myfile_plottraindata << "vard2 = " << vard2 << std::endl;
		//myfile_plottraindata << "vard3 = " << vard3 << std::endl;
		//myfile_plottraindata << "vard4 = " << vard4 << std::endl << std::endl;
	
		myfile_plottraindata << "slope_precision = " << slope_precision << std::endl;
		
	
		myfile_plottraindata << "lambda_list = " << std::endl;
		for (unsigned int i=0; i<lambda_list.size1(); i++){
			for (unsigned int j=0; j<lambda_list.size2(); j++)	myfile_plottraindata << setw(12) << lambda_list(i,j) << " ";	myfile_plottraindata << std::endl;
		}	myfile_plottraindata << std::endl;
		
		myfile_plottraindata << "ik_results.t_wrist = " << std::endl;
		for (unsigned int i=0; i<ik_results.t_wrist.size(); i++)	myfile_plottraindata << setw(12) << ik_results.t_wrist(i) << " ";	myfile_plottraindata << std::endl << std::endl;
	
		myfile_plottraindata << "ik_results.configuration = " << std::endl;
		for (unsigned int i=0; i<ik_results.configuration.size(); i++)	myfile_plottraindata << setw(12) << ik_results.configuration(i)*(180/3.141592) << " ";	myfile_plottraindata << std::endl << std::endl;
	
		/*
		myfile_plottraindata << "ik_results.t_wrist = " << std::endl;
		for (unsigned int i=0; i<ik_results.t_wrist.size1(); i++){
			for (unsigned int j=0; j<ik_results.t_wrist.size2(); j++)	myfile_plottraindata << setw(12) << ik_results.t_wrist(i,j) << " ";	myfile_plottraindata << std::endl;
		}	myfile_plottraindata << std::endl;
	
		myfile_plottraindata << "ik_results.configuration_list = " << std::endl;
		for (unsigned int i=0; i<ik_results.configuration_list.size1(); i++){
			for (unsigned int j=0; j<ik_results.configuration_list.size2(); j++)	myfile_plottraindata << setw(12) << ik_results.configuration_list(i,j)*(180/3.141592) << " ";	myfile_plottraindata << std::endl;
		}	myfile_plottraindata << std::endl;
        */
	
		// Close file
		myfile_plottraindata.close();
	
		return;
	}

#endif  // KAUTHAM_USE_GSL



