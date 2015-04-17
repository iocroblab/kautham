/*************************************************************************\
   Copyright 2014 Institute of Industrial and Control Engineering (IOC)
                 Universitat Politecnica de Catalunya
                 BarcelonaTech
    All Rights Reserved.

    This prompl::geometricram is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This prompl::geometricram is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this prompl::geometricram; if not, write to the
    Free Software Foundation, Inc.,
    59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 \*************************************************************************/

/* Author: Enrique Ajenjo, Ely Repiso */

#include "PCARRT.h"
#include "ompl/geometric/planners/rrt/RRT.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"
#include <limits>
#include <armadillo> // for compute PCA
#include "ompl/geometric/PathGeometric.h"
#include <ompl/base/spaces/SE3StateSpace.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;
using namespace arma;

namespace Kautham {
  namespace omplplanner{
/*// Inicio: funtion solve PCARRT like RRT
ob::PlannerStatus PCARRT::solve(const ob::PlannerTerminationCondition &ptc)
      {
      // ININCIO solve function (copiar aqui todo el codigo de la funcion solve de la clase ompl::RRT)

	    checkValidity();
        ob::Goal                 *goal   = pdef_->getGoal().get();
        ob::GoalSampleableRegion *goal_s = dynamic_cast<ob::GoalSampleableRegion*>(goal);

        while (const ob::State *st = pis_.nextStart())
	    {
		    Motion *motion = new Motion(si_);
		    si_->copyState(motion->state, st);
		    nn_->add(motion);
	    }

	    if (nn_->size() == 0)
	    {
		    OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
            return ob::PlannerStatus::INVALID_START;
	    }

	    if (!sampler_)
		    sampler_ = si_->allocStateSampler();

	    OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), nn_->size());

	    Motion *solution  = NULL;
	    Motion *approxsol = NULL;
	    double  approxdif = std::numeric_limits<double>::infinity();
	    Motion *rmotion   = new Motion(si_);
        ob::State *rstate = rmotion->state;
        ob::State *xstate = si_->allocState();

	    while (ptc == false)
	    {

		// sample random state (with goal biasing) 
		if (goal_s && rng_.uniform01() < goalBias_ && goal_s->canSample())
		    goal_s->sampleGoal(rstate);
		else
		    sampler_->sampleUniform(rstate);

		// find closest state in the tree 
		Motion *nmotion = nn_->nearest(rmotion);
        ob::State *dstate = rstate;

		// find state to add 
		double d = si_->distance(nmotion->state, rstate);
		if (d > maxDistance_)
		{
		    si_->getStateSpace()->interpolate(nmotion->state, rstate, maxDistance_ / d, xstate);
		    dstate = xstate;
		}

		if (si_->checkMotion(nmotion->state, dstate))
		{
		    // create a motion 
		    Motion *motion = new Motion(si_);
		    si_->copyState(motion->state, dstate);
		    motion->parent = nmotion;

		    nn_->add(motion);
		    double dist = 0.0;
		    bool sat = goal->isSatisfied(motion->state, &dist);
		    if (sat)
		    {
		        approxdif = dist;
		        solution = motion;
		        break;
		    }
		    if (dist < approxdif)
		    {
		        approxdif = dist;
		        approxsol = motion;
		    }
		}
	    }

	    bool solved = false;
	    bool approximate = false;
	    if (solution == NULL)
	    {
		solution = approxsol;
		approximate = true;
	    }

	    if (solution != NULL)
	    {
		lastGoalMotion_ = solution;

		// construct the solution path 
		std::vector<Motion*> mpath;
		while (solution != NULL)
		{
		    mpath.push_back(solution);
		    solution = solution->parent;
		}

		// set the solution path 
        og::PathGeometric *path = new og::PathGeometric(si_);
		for (int i = mpath.size() - 1 ; i >= 0 ; --i)
		    path->append(mpath[i]->state);
        pdef_->addSolutionPath(ob::PathPtr(path), approximate, approxdif);
		solved = true;
	    }

	    si_->freeState(xstate);
	    if (rmotion->state)
		si_->freeState(rmotion->state);
	    delete rmotion;

	    OMPL_INFORM("%s: Created %u states", getName().c_str(), nn_->size());

        return ob::PlannerStatus(solved, approximate);

       // FIN solve function
      }

// Final: function solve PCARRT*/  



//Our solve function
// Inicio: funtion solve PCARRT
ob::PlannerStatus PCARRT::solve(const ob::PlannerTerminationCondition &ptc)
      {
      // INICIO solve function (copiar aqui todo el codigo de la funcion solve de la clase ompl::RRT)

	    checkValidity();
        ob::Goal                 *goal   = pdef_->getGoal().get();
        ob::GoalSampleableRegion *goal_s = dynamic_cast<ob::GoalSampleableRegion*>(goal);

        while (const ob::State *st = pis_.nextStart())
	    {
		    Motion *motion = new Motion(si_);
		    si_->copyState(motion->state, st);
		    nn_->add(motion);
	    }

	    if (nn_->size() == 0)
	    {
		    OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
            return ob::PlannerStatus::INVALID_START;
	    }

	    if (!sampler_)
		    sampler_ = si_->allocStateSampler();

	    OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), nn_->size());

        Motion *solution = NULL;
	    Motion *approxsol = NULL;
	    double  approxdif = std::numeric_limits<double>::infinity();
        Motion *rmotion = new Motion(si_);
        ob::State *rstate = rmotion->state;
        ob::State *xstate = si_->allocState();

	    while (ptc == false)
	    {

		// sample random state (with goal biasing) 
		if (goal_s && rng_.uniform01() < goalBias_ && goal_s->canSample())
		    goal_s->sampleGoal(rstate);
		else
		    sampler_->sampleUniform(rstate);

		// find the k closest neighbors states in the tree  
    	Motion *nmotion = nn_->nearest(rmotion);  // iS q_near=rmotion/nmotion
		std::vector<Motion *> nbhmotion;  // nbhmotion=k neigbours vector 
    	nn_->nearestK(nmotion, kPCA,  nbhmotion);    
    	ob::State *dstate = rstate;

    	//INICIO: modify (rstate) with the PCA 
    	// matrix A, matrix U (A= covariance matrix, have in its diagona the eigenvalues, U=matrix of eigenvectors. each column are the corresponded eigenvector for the corresponded eigenvalue.)
  		
    	if(nbhmotion.size()==kPCA)
    	{
        //Calculate the new q_rand. RRT grow using PCA
             //OMPL_INFORM("RRT grow using PCA");
     // Definitions for change from planer variables (Motion,rstate) to armadillo variable (vec)
			ob::StateSpacePtr space = getSpaceInformation()->getStateSpace(); // obtain the state space 
 			ob::StateSpacePtr ssRobot0 = ((ob::StateSpacePtr) space->as<ob::CompoundStateSpace>()->getSubspace(0)); //paso 1)se extrae el subespacio correspondiente al robot 0 
        	ob::StateSpacePtr ssRobot0se3 =  ((ob::StateSpacePtr) ssRobot0->as<ob::CompoundStateSpace>()->getSubspace(0)); //paso 2) se extrae el componente se3
        	ob::ScopedState<ob::CompoundStateSpace> sstate_qnear_vec(space);      //se define un scoped state para el qnear (encapsulameinto de un state)
   			ob::ScopedState<ob::SE3StateSpace> qnearse3_vec(ssRobot0se3); //se define un scoped state para la parte se3 del qnear
 			ob::ScopedState<ob::CompoundStateSpace> sstate_qrand(space);      // definition of scoped state (encapsulation of a state)  
 			ob::ScopedState<ob::SE3StateSpace> qrandse3(ssRobot0se3); //se define un scoped state para la parte se3 del qrand
 			ob::ScopedState<ob::CompoundStateSpace> sstate_qnear(space);      //se define un scoped state para el qnear (encapsulameinto de un state)
			ob::ScopedState<ob::SE3StateSpace> qnearse3(ssRobot0se3); //se define un scoped state para la parte se3 del qnear

			//OMPL_INFORM("PCA RRT 2,kPCA=%i",kPCA);
        	//OMPL_INFORM("PCA RRT 2,nDOF=%i",nDOF);

     		//initializing matrix to compute the PCA, PCAmatrix=the columns are the k_neighbor_points
        	mat PCAmatrix(kPCA,nDOF);
        	for(unsigned int n=0;n<kPCA;n++){ // m=filas=nDOF, n=columnas=k	
          		for(unsigned int m=0;m<nDOF;m++){
      				sstate_qnear_vec = *(nbhmotion[n]->state);  //qnear_n=nbhmotion[n]->state
					sstate_qnear_vec >> qnearse3_vec; 			// fill the scoped state qnearse3 with the content of se3 of the scoped state sstate_qnear
 					if(m==0){
 						PCAmatrix(n,m)=qnearse3_vec->getX(); 
 						//OMPL_INFORM("PCAmatrix(%i,%i)=%f",n,m,PCAmatrix(n,m));
 					}else{
 						PCAmatrix(n,m)=qnearse3_vec->getY();
 						//OMPL_INFORM("PCAmatrix(%i,%i)=%f",n,m,PCAmatrix(n,m));
 					}
          		}
        	}

    	// Compute vector of eigenvalues and matrix of eigenvectors from PCA
        	vec A; // vector of eigenvalues
        	mat U; // matrix of eigenvectors in columns
	   		mat score;   
	   		
	   		 //OMPL_INFORM("A=%d ",U);
        	princomp(U,score,A,cov(PCAmatrix)); // compute A and U using the PCA points 
     	// Obtain the lambda_max of the eigenvalues for the posterior computation of q_rand using the PCA.
 		
        	double lambda_max;
        	lambda_max=A[0]; // first eigen value.
     		//OMPL_INFORM("A=%d ",A[0]);

   		//change of rstate and nmotion->state to armadillo vectors (vec)
 // pksogmpkrwgmhp
        	// q_rand
         	vec old_q_rand(2);    
            sstate_qrand = *rstate;         //obtain the qrand with rstate
 		 	sstate_qrand >> qrandse3;       //fill the scoped state qrandse3 with the content of se3 of the scoped state sstate_qrand         
         	old_q_rand[0]=qrandse3->getX(); // obtain the x of q_rand state
         	old_q_rand[1]=qrandse3->getY(); // obtain the y of q_rand state

          	// q_near
         	vec new_nmotion(2);
            sstate_qnear = *(nmotion->state);//obtain the qnear with nmotion->state
         	sstate_qnear >> qnearse3;        //fill the scoped state qnearse3 with the content of se3 del scoped state sstate_qnear
         	new_nmotion[0]=qnearse3->getX(); //obtain the x of q_near state 
         	new_nmotion[1]=qnearse3->getY(); //obtain the y of q_near state

     	//bucle, calculate the new rstate. Function: q_rand=q_near+ sum(i=1->n) dot( (Aii/lambda_max)*(q_rand-q_near) , Ui )*Ui
         	vec q_rand_new;         
         	vec rest;
    	 	rest=old_q_rand - new_nmotion;	
    	 	q_rand_new=new_nmotion; // q_rand=q_near  
    	 	//OMPL_INFORM("PCA RRT 11, cols=%i, rows=%i", U.n_cols, U.n_rows); 
        	for (unsigned int n=0;n<2; n++)  // 2=U.col size.
        	{   
          		double constante=A[n]/lambda_max;	    
          		q_rand_new=q_rand_new+dot(constante*rest,U.col(n))*U.col(n); 
        	}

     	// variable change from armadillo vector (vec) to rstate 
        	qrandse3->setX(q_rand_new[0]); //change the X of qrand
        	qrandse3->setY(q_rand_new[1]); //change the Y of qrand        	
    	}
    	else{
        // If not have sufficient RRT states for calculate the PCA, the tree growht like RRT.
         OMPL_INFORM("Normal RRT, Insufficient number of points to calculate the PCA");
    	}

    //FIN: modify rstate with the PCA
 
		// find state to add 
		double d = si_->distance(nmotion->state, rstate);
		if (d > maxDistance_)
		{
		    si_->getStateSpace()->interpolate(nmotion->state, rstate, maxDistance_ / d, xstate); //AVANCE DEL ARBOL HACIA q_rand. ANTES DE EL HAY QUE MODIFICAR rstate HACIA DONDE AVANZA, CON LA PCA
		    dstate = xstate;
		}

		if (si_->checkMotion(nmotion->state, dstate))
		{
		    // create a motion 
		    Motion *motion = new Motion(si_);
		    si_->copyState(motion->state, dstate);
		    motion->parent = nmotion;

		    nn_->add(motion);
		    double dist = 0.0;
		    bool sat = goal->isSatisfied(motion->state, &dist);
		    if (sat)
		    {
		        approxdif = dist;
		        solution = motion;
		        break;
		    }
		    if (dist < approxdif)
		    {
		        approxdif = dist;
		        approxsol = motion;
		    }
		}
	    }

	    bool solved = false;
	    bool approximate = false;
	    if (solution == NULL)
	    {
		solution = approxsol;
		approximate = true;
	    }

	    if (solution != NULL)
	    {
		lastGoalMotion_ = solution;

		// construct the solution path 
		std::vector<Motion*> mpath;
		while (solution != NULL)
		{
		    mpath.push_back(solution);
		    solution = solution->parent;
		}

		// set the solution path 
        og::PathGeometric *path = new og::PathGeometric(si_);
		for (int i = mpath.size() - 1 ; i >= 0 ; --i)
		    path->append(mpath[i]->state);
        pdef_->addSolutionPath(ob::PathPtr(path), approximate, approxdif);
		solved = true;
	    }

	    si_->freeState(xstate);
	    if (rmotion->state)
		si_->freeState(rmotion->state);
	    delete rmotion;

	    OMPL_INFORM("%s: Created %u states", getName().c_str(), nn_->size());

        return ob::PlannerStatus(solved, approximate);

       // FIN solve function
      }

// Final: function solve PCARRT

//Fin our solve function 


  }
}
