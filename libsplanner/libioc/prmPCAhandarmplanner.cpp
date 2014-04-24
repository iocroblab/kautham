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

//FIXME: this planner is done for a single TREE robot (associtated to wkSpace->robots[0])

#if defined(KAUTHAM_USE_ARMADILLO)

#include <stdio.h>
#include <time.h>
#include "prmPCAhandarmplanner.h"
#include <boost/random.hpp>

///////////////Armadillo////////
#include <iostream>

using namespace arma;
using namespace std;

//////////////////////
 
 namespace Kautham {

  namespace IOC{
		
	PRMPCAHandArmPlanner::PRMPCAHandArmPlanner(SPACETYPE stype, Sample *init, Sample *goal, SampleSet *samples, Sampler *sampler, 
           WorkSpace *ws, int cloudSize, int samplingV, KthReal cloudRad, int samplingR,float distgoal,float distsamplingpcagoal)
      :PRMHandPlanner(stype, init, goal, samples, sampler, ws,  cloudSize,  cloudRad)
	{
		_idName = "PRMPCA HandArm";
        _guiName = "PRMPCA HandArm";
		
		_deltaR=distgoal;
		addParameter("Distance Goal", _deltaR);
		_deltaI=distsamplingpcagoal;
		addParameter("Distance Sampling PCA Goal",_deltaI);
		_samplingV=samplingV;
		addParameter("Num Sampling Free PCA", _samplingV);
		_samplingR=samplingR;
		addParameter("Num Sampling Free RW",_samplingR);


		//////////////////////////////////////////////////
		_lambda = cloudRad;
        addParameter("Lambda", _lambda);

        removeParameter("Cloud deltaM");//from prmhandplanner class
        removeParameter("Cloud Radius");//from prmhandplanner class
        removeParameter("Cloud Size");//from prmhandplanner class
        removeParameter("Neigh Thresshold");//from prmplanner class
        removeParameter("P(connect to Ini-Goal)");//from prmplanner class
        removeParameter("Sampler 1(sdk),2(h),3(g),4(gl),5(r)");//from prmplanner class

		_samplingmethod=0;
		addParameter("Sampling (0:PCA, 1:Gauss, 2:Bridge)",_samplingmethod);

		//boost::mt19937 rng(time(0));
		//boost::normal_distribution<double> gaussian_dist(0,0.1); // Parametro sigma
		//_gaussianGen = new GaussianGenerator(rng, gaussian_dist);

		// _samples->setANNdatastructures(_kNeighs, _maxNumSamples*2);
		//fp = fopen("rand.txt","wt"); 

         //matPCA = new mat(500,_wkSpace->getNumRobControls());
	}


	PRMPCAHandArmPlanner::~PRMPCAHandArmPlanner(){
	}

    bool PRMPCAHandArmPlanner::setParameters()
	{
      //PRMHandPlanner::setParameters(); //why it is not called?
      try{
        HASH_S_K::iterator it = _parameters.find("Step Size");
        if(it != _parameters.end())
            _locPlanner->setStepSize(it->second);
        else
          return false;

		it = _parameters.find("Sampling (0:PCA, 1:Gauss, 2:Bridge)");
        if(it != _parameters.end())
		{
          if(it->second==2) _samplingmethod = 2;
		  else if(it->second==1) _samplingmethod = 1;
		  else _samplingmethod = 0;
	    }
        else
          return false;

        it = _parameters.find("Speed Factor");
        if(it != _parameters.end())
          _speedFactor = it->second;
        else
          return false;

        it = _parameters.find("Max. Samples");
		if(it != _parameters.end()){
          _maxNumSamples = it->second;
		  _samples->setANNdatastructures(_kNeighs, _maxNumSamples*2);
	    }
        else
          return false;

        //it = _parameters.find("Neigh Thresshold");
        //if(it != _parameters.end())
        //  _neighThress = it->second;
        //else
        //  return false;

        it = _parameters.find("Max. Neighs");
		if(it != _parameters.end()){
          _kNeighs = (int)it->second;
		  _samples->setANNdatastructures(_kNeighs, _maxNumSamples*2);
	  }
        else
          return false;

		
        it = _parameters.find("Lambda");
		if(it != _parameters.end()){
          _lambda = it->second;
		  
		}
        else
          return false;

       
		it = _parameters.find("Distance Goal");
        if(it != _parameters.end())
          _deltaR = it->second;
        else
          return false;

		it = _parameters.find("Distance Sampling PCA Goal");
        if(it != _parameters.end())
          _deltaI = it->second;
        else
          return false;

		it = _parameters.find("Num Sampling Free PCA");
        if(it != _parameters.end())
          _samplingV = it->second;
        else
          return false;

		it = _parameters.find("Num Sampling Free RW");
        if(it != _parameters.end())
          _samplingR = it->second;
        else
          return false;

        it = _parameters.find("Drawn Path Link");
        if(it != _parameters.end()){
          _drawnLink = it->second;
          for(int i=0; i<_wkSpace->getNumRobots();i++)
            _wkSpace->getRobot(i)->setLinkPathDrawn(_drawnLink);
        }else
          return false;

      }catch(...){
        return false;
      }
      return true;
    }


	//! Finds a random hand configuration for a given arm configuration.
	//! If randhand is false, the coord vector passed as a parameter is unchanged,
	//! only the thumb limits are computed; then the numPMDs parameter is not used.
	//! if randhand is true, it computes the random conf using a number numPMDs of
	//! PMDs. The value numPMDs = -1 means all of them.
  bool PRMPCAHandArmPlanner::getHandConfig(vector<KthReal>& coord,  bool randhand, int numPMDs)
	{
		vector<KthReal> coordvector;

		//If random - first find a autocollisionfree sample up to a max of maxtrials
		if(randhand)
		{
			//_gen->rand_init();
			bool autocol;
			int trials=0;
			int maxtrials=10;
			int kini = _wkSpace->getRobot(0)->getTrunk();
			//hand coords not set - sample the whole range
			bool samplewholerange;
			if(coord[kini]==-1) samplewholerange=true;
			else samplewholerange=false;

			do{
				coordvector.clear();

				//load the arm coordinates passed as a parameter
				for(int k =0; k < _wkSpace->getRobot(0)->getTrunk(); k++)
				{
					coordvector.push_back(coord[k]);
				}

				//sample the hand coordinates
				float q=0;
                q=_wkSpace->getNumRobControls()-(_wkSpace->getRobot(0)->getTrunk());

                if(numPMDs==-1 || numPMDs>(_wkSpace->getNumRobControls()-_wkSpace->getRobot(0)->getTrunk()))
                        numPMDs = _wkSpace->getNumRobControls()-_wkSpace->getRobot(0)->getTrunk();
				int k;
				//Aqui randon Hand/////////////////////////
				for(k = kini; k < kini+numPMDs; k++)
				{
					//hand coords not set - sample the whole range
					if(samplewholerange){
						//nonlinear behaviour- sample uniformly within a bigger range and then saturate
							KthReal size=1.0;//1.4;//1.2;
							double rr=_gen->d_rand();
							//fprintf(fp,"%f\n",rr);
							coord[k]=-((size-1.0)/2) + rr*size; 
							if(coord[k]<0) coord[k]=0;
						    else if(coord[k]>1) coord[k]=1;
						/*
						if(_gen->d_rand()<0.5) coord[k] = (KthReal)_gen->d_rand();
						else{
							if(k==kini) coord[k] = 0.5 + 0.1*(2*(KthReal)_gen->d_rand()-1);
							if(k==(kini+1)) coord[k] = 0.5 + 0.1*(2*(KthReal)_gen->d_rand()-1) ;
							if(k==(kini+2)) coord[k] = 0 + 0.1*(2*(KthReal)_gen->d_rand()-1);
							if(k==(kini+3)) coord[k] = 1.0 + 0.1*(2*(KthReal)_gen->d_rand()-1);
							if(k==(kini+4)) coord[k] = 1.0 + 0.1*(2*(KthReal)_gen->d_rand()-1);
							
							if(coord[k]<0) coord[k]=0;
						    else if(coord[k]>1) coord[k]=1;
						}
						*/
					}
					//hand coords already set - saple around the known values
					else{
						double rr=_gen->d_rand();
						//fprintf(fp,"%f\n",rr);
						coord[k] = coord[k]+ 0.5*(2*(KthReal)rr-1);//0.3*
						if(coord[k]<0) coord[k]=0;
						else if(coord[k]>1) coord[k]=1;
					}
					coordvector.push_back(coord[k]);
				}
                for(; k < _wkSpace->getNumRobControls(); k++)
				{
					coord[k] = 0.5;
					coordvector.push_back(coord[k]);
				}

				/*
				//load the arm coordinates passed as a parameter
				for(int k =0; k < _wkSpace->getRobot(0)->getTrunk(); k++)
				{
					coordvector.push_back(coord[k]);
				}
				*/
				//Set the new sample with the hand-arm coorinates and check for autocollision.				
				_wkSpace->getRobot(0)->control2Pose(coordvector); 
				autocol = _wkSpace->getRobot(0)->autocollision();
				trials++;
			}while(autocol==true && trials<maxtrials);
			if(autocol==true) return false;
		}
		else
		{
			//load the hand-arm coordinates passed as a parameter
            for(int k = 0; k < _wkSpace->getNumRobControls(); k++)
			{
				coordvector.push_back(coord[k]);
			}
		}
		return true;
	}




//!Given the position of the tcp of the arm, returns the six joint angles of the arm
	//! computed using the same configuration solution as the sample smp.
	//! the six first values of the vector carm are used for the input/output
	bool PRMPCAHandArmPlanner::ArmInverseKinematics(vector<KthReal> &carm, Sample *smp, bool maintainSameWrist)
	{
		//CALL TO INVERSE KINEMATICS
		RobConf rc;
		SE3Conf c;
		std::vector<KthReal> coordarmGoal; coordarmGoal.resize(6); 
		std::vector<KthReal> tmpcoordTCP; tmpcoordTCP.resize(6); 

		//load the six joint values of the goal
		for(int k=0;k<6;k++) coordarmGoal[k] = smp->getMappedConf().at(0).getRn().getCoordinate(k);

	    //load the transform of the tcp (coordarm) and convert it to a RobConf
		for(int k=0;k<6;k++) tmpcoordTCP[k] = carm[k];
		c.setCoordinates(tmpcoordTCP);

		try{
			//cal inverse kinematics - want to find a solution similar to the goal, in termns of configuration
			//parameters (l/r,ep,en,wp,wn)
				rc = _wkSpace->getRobot(0)->InverseKinematics(c.getCoordinates(), coordarmGoal, maintainSameWrist);
		}catch(InvKinEx &ex){
			std::cout << ex.what() << std::endl;
			return false;
		}
		//load the six joint values of the arm
        for(int k=0;k<6;k++) carm[k] = rc.getRn().getCoordinate(k);
		return true;
	}


 	void PRMPCAHandArmPlanner::setIniGoalSe3()
	{
		std::vector<RobConf> r = initSamp()->getMappedConf();
		RnConf q = initSamp()->getMappedConf().at(0).getRn();
		_wkSpace->getRobot(0)->Kinematics(q);
		std::vector<KthReal> tmpcoordTCPpos; 
		tmpcoordTCPpos.resize(3);  
		std::vector<KthReal> tmpcoordTCPori; 
		tmpcoordTCPori.resize(4);  
		mt::Transform ctransfini = _wkSpace->getRobot(0)->getLinkTransform(6);
		mt::Point3 ctransini = ctransfini.getTranslation();
		mt::Rotation crotini = ctransfini.getRotation();
		for(int k=0;k<3;k++) tmpcoordTCPpos[k] =  ctransini[k];
		for(int k=0;k<4;k++) tmpcoordTCPori[k] =  crotini[k];
		_inise3.setPos(tmpcoordTCPpos);
		_inise3.setOrient(tmpcoordTCPori);

		_wkSpace->getRobot(0)->Kinematics(goalSamp()->getMappedConf().at(0).getRn()); 
		mt::Transform ctransfgoal = _wkSpace->getRobot(0)->getLinkTransform(6);
		_goaltrans = ctransfgoal.getTranslation();
		mt::Rotation crotgoal = ctransfgoal.getRotation();
		for(int k=0;k<3;k++) tmpcoordTCPpos[k] =  _goaltrans[k];
		for(int k=0;k<4;k++) tmpcoordTCPori[k] =  crotgoal[k];
		_goalse3.setPos(tmpcoordTCPpos);
		_goalse3.setOrient(tmpcoordTCPori);
	}


	//!verifies if init an goal are in the same connected component, fins the path in the graph, and
	//!reports the info.
 	bool PRMPCAHandArmPlanner::verifySuccess(clock_t inittime, int nloops, int nPCAcalls)
	{
		if(goalSamp()->getConnectedComponent() == initSamp()->getConnectedComponent()) 
		{
			if(PRMPlanner::findPath())
			{
					printConnectedComponents();
					//cout << "PRM Nodes = " << _samples->getSize() << endl;
					cout << "NUMBER OF SAMPLED CONFIGURATIONS = " << _triedSamples << endl;
														
					clock_t finaltime = clock();
					cout<<"TIME TO COMPUTE THE PATH = "<<(double)(finaltime-inittime)/CLOCKS_PER_SEC<<endl;
					PRMPlanner::smoothPath();

					clock_t finalsmoothtime = clock();
					cout<<"TIME TO SMOOTH THE PATH = "<<(double)(finalsmoothtime - finaltime)/CLOCKS_PER_SEC<<endl;
														
					cout << "Number of passes = " << nloops << endl; //" deltaM = " << deltaM<< endl;
					cout << " Call PCA = "<<nPCAcalls<<endl<<flush;
					//cout << " Call Sampling World Real = "<<countwr<<endl<<flush;
					printPCAComponents();
																		
														
					_solved = true;
					_generatedEdges = weights.size();
					_totalTime = (KthReal)(finaltime - inittime)/CLOCKS_PER_SEC ;
					_smoothTime = (KthReal)(finalsmoothtime - finaltime)/CLOCKS_PER_SEC ;
								
					return true;
			}
		}
		return false;
	}


	bool PRMPCAHandArmPlanner::trySolve()
	{
		//_gen->rand_init();
		//fprintf(fp,"---------------------------------------------\n");

		wkSpace()->resetCollCheckCounter();

		if(_solved) {
			cout << "PATH ALREADY SOLVED"<<endl;
			return true;
		}

		cout<<"ENTERING TRYSOLVE!!!"<<endl;
	    clock_t entertime = clock();

		//set neigh threshold to "infinity", to conisider all k neighbors, irrespective of their distance
		_neighThress = goalSamp()->getDistance(initSamp(), CONFIGSPACE) *1000;//  * 4;
		_triedSamples=0;

		if(_samples->changed())
		{
			PRMPlanner::clearGraph(); //also puts _solved to false;
		}
			
		//Create graph with initial and goal sample
		if( !_isGraphSet )
		{
			//to try to connect ini and goal 
			_samples->findBFNeighs(_neighThress, _kNeighs);
			connectSamples();
			PRMPlanner::loadGraph();
			if(PRMPlanner::findPath()) {
				return true;
			}


		}

		//load ini and goal transforms of the robot TCP as se3conf objects, to interpolate between them.
		setIniGoalSe3();


		//////////////////////////////////////////////////////////////////////////////
		bool sampleincollision;
		double deltaM;
		Sample *tmpSample;
        std::vector<KthReal> coord(wkSpace()->getNumRobControls());
		vector<KthReal> coordvector;
		int trials = 0;
		int maxtrials = 100;
		int ig=0;
        int maxsamplesingoal=_wkSpace->getNumRobControls();//PCA Action on 11 elements (Arm:6 and Hand:5 )

		_distance.clear();//clear vector distancia
		_indexpca.clear();//clear vector indexpca
		int countwr=0;
		
		int myflag=0, z=0;
		float R=0;
		callpca=0;
		int countSamples=0;
		//////////////////////////////////////////////////////////////////////////////
	 
	  	//Sample around goal up to maxsamplesingoal (in order to have as many samples as required for a first call to PCA)
		_samples->setTypeSearch(BRUTEFORCE);
		for(ig=0; trials<maxtrials && ig<maxsamplesingoal; ig) {
		  if(getSampleInGoalRegion(_deltaI , 0.02)) 
		  {
			  ig++;
		  }	  
		}
		_samples->setTypeSearch(ANNMETHOD);

		cout << "Initial Sampling: " << (_samples->getSize()-countSamples) << " samples" << endl;
		countSamples = _samples->getSize();
	 
		///////////////////////////////////////////////////////////////////////////////////////////////////////
		int p=0;
		int n=0;
		int freepca=0;
		int freerw=0;
		//set the deltaM of the sphere where to sample randomly
		deltaM = _deltaI; 
		
		
		//getSamplesBetweenInitGoal(0.0, 0.0, false);
		//main loop, up to _maxNumSamples samples  
		do{
			try{
				//p is the loop count
				p++;
				//the deltaM increases as new passes are required
				//it is reset to initial value (delta_I) if it increases up to 1.5 times delta_R
				deltaM = deltaM + _deltaI;
				if(deltaM> _deltaR) deltaM = _deltaI;

				//add interpolated samples between init and goal
				int numinterpoations=2;
				getSamplesBetweenInitGoal(numinterpoations,deltaM, 0.02, false);
				cout << "Sampling Interpolated: " << (_samples->getSize()-countSamples) << " samples" << endl;
				countSamples = _samples->getSize();
						
				//get samples from V using PCA
				if(_samplingmethod==0)
				{
				  if(getSampleRandPCA(_deltaR))
				  {		
					//verify if the samples from V are within bounds and free
                    if(matPCA.n_rows>0)//matrand
					{
						////////////////Creación de variables////////////////////
                        vector<KthReal> coord(_wkSpace->getNumRobControls());
						Sample *tmpSample;
                        tmpSample = new Sample(_wkSpace->getNumRobControls());
						
						///////////////////////////////////////////////////////////
						//Procesamiento para comprobar las nuevas muestas 20x11, son libres de colisión
                        for(int z=0; z<matPCA.n_rows ; z++)
						{
                            rowvec pointpca=matPCA.row(z);//Fila de 11 elementos(6 primeros del brazo y los 5 ultimos de la mano)
                            for(int k=0; k < _wkSpace->getNumRobControls(); k++)
								coord[k]=pointpca(0,k);//coord:variable para almacenar los 11 elementos
							
							//load sample coordinates
							tmpSample->setCoords(coord);	

							_triedSamples++;
							//evaluate sample
							sampleincollision=_wkSpace->collisionCheck(tmpSample);
									
							if( sampleincollision == false) //comprobación si la muestra esta libre de colisiones
							{ 	
								n++;
										
								//Cinematica Directa para encontrar la posición del Robot con la nueva muestra
								_wkSpace->getRobot(0)->Kinematics(tmpSample->getMappedConf().at(0).getRn()); 
								mt::Transform temptransf = _wkSpace->getRobot(0)->getLinkTransform(6);
								mt::Point3 temptrans = temptransf.getTranslation();
								///////////////////////////////////////////////////////////
								float dist=0;//dist:variable para almacena la distancia de la muestra al objetivo
								dist=temptrans.distance(_goaltrans);//calculo de la distancia de la nueva muestra al objetivo	

								freepca++;
								_samples->add(tmpSample);//Adicionamos una nueva muestra libre de colisiones		
								_distance.push_back(dist);//Se agrega la distancia de la muestra (en centimetros)
								//connect sample to roadmap
                                connectLastSample(initSamp(), goalSamp());

								//create next sample
                                tmpSample = new Sample(_wkSpace->getNumRobControls());
								
								_indexpca.push_back(_samples->getSize()-1);//Se agrega el indice de la muestra generada por Sampling PCA									

								if(verifySuccess( entertime, p, callpca)==true)  {
									cout << " COLLISION-CHECK COUNT: "<< wkSpace()->getCollCheckCounter() << endl;
									return _solved;
								}
								
								///////////////////////////////////////////////////////////////// 
								if(freepca>=_samplingV) {freepca=0; break;}
								////////////////////////////////////////////////////////////////
							}
						}
					}
				  }
				}
				//get samples from V using Gaussian					
                else if(_samplingmethod==1)
				{
					//Alternativa al sampling de V: Gaussian sampling around the goal
					for(int t=0;t<_samplingV;t++)
					{		 
						//cout<<"deltaM = "<<deltaM<<" n = "<<n<<endl;
						if(getSampleInGoalRegionRealworldGaussian(_deltaI, 0.05, true)) 
						{	
							//t++;
							n++;
							_indexpca.push_back(_samples->getSize()-1);//Se usa para reportar las muestras generadas por el gaussian, usamos el mismo vector...
							if(verifySuccess( entertime, p, callpca)==true)  {
								cout << " COLLISION-CHECK COUNT: "<< wkSpace()->getCollCheckCounter() << endl;
								return _solved;
							}
						}
					}
					//END alternativa Gaussian
				}
				else
				{
					//Alternativa al sampling de V: Bridge-test sampling around the goal
					for(int t=0;t<_samplingV;t++)
					{		 
						//cout<<"deltaM = "<<deltaM<<" n = "<<n<<endl;
						if(getSampleInGoalRegionRealworldBridgeTest(_deltaI, 0.05, true)) 
						{	
							//t++;
							n++;
							_indexpca.push_back(_samples->getSize()-1);//Se usa para reportar las muestras generadas por el gaussian, usamos el mismo vector...
							if(verifySuccess( entertime, p, callpca)==true) {
								cout << " COLLISION-CHECK COUNT: "<< wkSpace()->getCollCheckCounter() << endl;
								return _solved;
							}
						}
					}
					//END alternativa Bridge-test
				}
			}
			catch (...){ cout<<"Data not recognize"<<endl;}
			
			cout << "Sampling from V: " << (_samples->getSize()-countSamples) << " samples" << endl;
			countSamples = _samples->getSize();

				
			if(_triedSamples > _maxNumSamples) break;

			//Sample _samplingR samples from region R_S (the "real world")
			for(int t=0;t<_samplingR;)//t++)
			{		 
				//cout<<"deltaM = "<<deltaM<<" n = "<<n<<endl;
				if(getSampleInGoalRegionRealworld(deltaM, 0.05, true)) 
				{	
					t++;
					countwr++;
					n++;
					if(verifySuccess( entertime, p, callpca)==true){
						cout << " COLLISION-CHECK COUNT: "<< wkSpace()->getCollCheckCounter() << endl;
						return _solved;
					}
				}
			}
			
			cout << "Sampling from RW: " << (_samples->getSize()-countSamples) << " samples" << endl;
			countSamples = _samples->getSize();

	  }while(_triedSamples < _maxNumSamples);

	  cout << "PRM Free Nodes = " << _samples->getSize() << endl;
	  cout<<"PATH NOT POUND"<<endl;
	  printConnectedComponents();
	  
	  clock_t finaltime = clock();
	  cout<<"ELAPSED TIME = "<<(double)(finaltime-entertime)/CLOCKS_PER_SEC<<endl;

	  //cout << "Number sampled configurations = " << n << endl;
	  cout << " number of collision-checks = "<<_triedSamples<<endl<<flush;
	  cout << "Number of passes = " << p << " deltaM = " << deltaM<< endl;
	  cout << " Call PCA = "<<callpca<<endl<<flush;
	  cout << " COLLISION-CHECK COUNT: "<< wkSpace()->getCollCheckCounter() << endl;
	  printPCAComponents();
	 


    _solved = false;
    //_triedSamples = n; ???
    _totalTime = (KthReal)(finaltime - entertime)/CLOCKS_PER_SEC ;
    _smoothTime = 0. ;
   return _solved;
	  /////////////////////////////////////////////////////////////////
	  }
	
	//!Uses PCA with the samples near the goal to compute the V region for sampling
	bool PRMPCAHandArmPlanner::getSampleRandPCA(float R)
	{
		int sizevd=0; //inicialización de la variable
		sizevd=_distance.size();//numero de elementos del vector de distancias
		
		//if not enough ssmples return
        if(sizevd<_wkSpace->getNumRobControls()) return false;
	
		//else compute the V region
		///////////////////////////////////
				
		///////////Fill Matrix PCA11PMDs to compute PCA /////////////////////
        //use those samples within a distance R, and if more than 11 (_wkSpace->getNumRobControls()) are available use those
		//that also belong to the same connected component as the goal
		vector<Sample*> pacsamplevector;
		for(int i=0;i<sizevd;i++)
		{
			//consider only those of the same connected component as the goal
            if(i>=_wkSpace->getNumRobControls() && _samples->getSampleAt(i+2)->getConnectedComponent()!=goalSamp()->getConnectedComponent()) continue;

			if(_distance.at(i)<R)//condición para muestras que se encuentran a una distancia menor a R
			{
				pacsamplevector.push_back(_samples->getSampleAt(i+2));//Conseguimos las muestras libre a partir del elemento numero 3 :(0)= Ini y (1)=Goal, 
			}
		}
        mat PCA11PMDs(pacsamplevector.size(),_wkSpace->getNumRobControls());
		PCA11PMDs.fill(0.0);//inicilización de la matriz
		for(int i=0;i<pacsamplevector.size();i++)
		{
            for(int k = 0; k <_wkSpace->getNumRobControls(); k++)
				PCA11PMDs(i,k)=pacsamplevector[i]->getCoords()[k];		
		}

		try
		{
            if(PCA11PMDs.n_rows>=_wkSpace->getNumRobControls())//Condición para el calculo de PCA(minimo _wkSpace->getNumRobControls(),11, elementos)
			{
				///////////////PCA Armadillo//////////////////////////////
				mat coeff;//coeff: principal component coefficients
				vec latent;//latent: principal component variances.
                mat score; //projected data
                //princomp(coeff, score, latent, X)
                princomp(coeff, score, latent,cov(PCA11PMDs));//Calcula el PCA


				/////////Print///////////////////////////////////////////////////////////
				//cout << "Matrix Rotation:" << endl << coeff << endl;
				//////////Contador Call PCA/////////////////////////////////////////
				callpca++;

				////////Baricentro-Media///////////////////////////
				rowvec bar  = mean(PCA11PMDs);
				////////Print Media/////////////////////////////
				//cout << "Media of Data:" << endl << bar << endl;
				//////////Lambdas/////////////////////////////////////
				 rowvec lambdapca = trans(_lambda*sqrt(latent));//4
				/////////Print Lambdas////////////////////////////
				//cout << "Lambdas:" << endl << lambdapca << endl;
				///////////Inicilización de la Matriz para almacenar 20x11 nuevas muestras en el Espacio PCA/////////////////
				//mat::fixed<50,11> matrand;
				//matrand.fill(0.0);
				int numsamplespca=500;//200;
                mat matrand(numsamplespca,_wkSpace->getNumRobControls());
                //matrand(numsamplespca,11);
                matrand.fill(0.0);

				//_gen->rand_init();
				for(int i=0; i < numsamplespca; i++)
                {
                    for(int j=0; j<_wkSpace->getNumRobControls(); j++){
						double rr=_gen->d_rand();
                        //fprintf(fp,"%f\n",rr);
                        matrand(i,j)=-lambdapca(0,j)+(2*lambdapca(0,j)*(rr));
					}

                    rowvec zeta=trans((coeff*trans(matrand.row(i)))+trans(bar));
					
                    for(int j=0; j<_wkSpace->getNumRobControls(); j++)
                        matrand(i,j)=zeta(0,j);
					
				}

                matPCA= matrand;

				return true;
			}
		}
		/////////////////////////////////
		catch (...){return false; }
}


	////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//!Print PCA components 
    void PRMPCAHandArmPlanner::printPCAComponents()
	{
		//std::map<int, SampleSet*>::iterator it;
		cout<<"Num Sampling Goal Region PCA  = "<<_indexpca.size()<<endl;
		cout << "PCA Sampling=> " ;
		for ( int i=0;i<_indexpca.size(); i++ )
		{
			cout<< _indexpca[i]<< "," ;
				
		}	
		cout << endl;
	}


	 //!resample around the goal configuration
	//!reimplemented
    bool PRMPCAHandArmPlanner::getSampleInGoalRegion(double tdeltaM, double rdeltaM)
	{
	    bool handWholeRange = false;
		return getSampleInGoalRegionRealworld( tdeltaM, rdeltaM, handWholeRange);
	}




	//!samples aroun the goal sample or around a sample that belongs to the same connected component than the goal
	bool PRMPCAHandArmPlanner::getSampleInGoalRegionRealworld(double tdeltaM, double rdeltaM, bool handWholeRange)
	{
	    int trials, maxtrials;
        vector<KthReal> coord(_wkSpace->getNumRobControls());
		bool autocol;//flag to test autocollisions
		Sample *tmpSample;
        tmpSample = new Sample(_wkSpace->getNumRobControls());

		//Set the coordinates of the robot joints 
		//Randomly set the coordinates of the robot joints at a autocollision-free conf
		trials=0;
		maxtrials=100;
		Sample *s;
		//_gen->rand_init();
		do{
			vector<KthReal> tmpSpos; tmpSpos.resize(3); 
			vector<KthReal> tmpSrot; tmpSrot.resize(3); 
			vector<KthReal> coordrobot; coordrobot.resize(6); 
	
			//randomly compute the coords of a smaple of the conecected component of the goal
			//or the coordinates of the goal
			
			double rr=_gen->d_rand();
			//fprintf(fp,"%f\n",rr);
			if(rr<0.8)
			{
				int cc = goalSamp()->getConnectedComponent();
				int kcc = _ccMap[cc]->getSize(); 
				
				double rr=_gen->d_rand();
				//fprintf(fp,"%f\n",rr);
				int indexsam = rr*kcc;
				//cout<<"sampling around "<<indexsam<<endl;
				 s = _ccMap[cc]->getSampleAt(indexsam);
			}
			else s = goalSamp();

			std::vector<KthReal> tmpcoordTCPpos; tmpcoordTCPpos.resize(3);  
			std::vector<KthReal> tmpcoordTCPori; tmpcoordTCPori.resize(4);
			_wkSpace->getRobot(0)->Kinematics(s->getMappedConf().at(0).getRn()); 
			mt::Transform ctransfgoal = _wkSpace->getRobot(0)->getLinkTransform(6);
			mt::Point3 ctransgoal = ctransfgoal.getTranslation();
			mt::Rotation crotgoal = ctransfgoal.getRotation();
			for(int k=0;k<3;k++) tmpcoordTCPpos[k] =  ctransgoal[k];
			for(int k=0;k<4;k++) tmpcoordTCPori[k] =  crotgoal[k];
			_goalse3.setPos(tmpcoordTCPpos);
			_goalse3.setOrient(tmpcoordTCPori);
			
			tmpSpos	= _goalse3.getPos();
			tmpSrot	= _goalse3.getParams();
			
						
			//sample position in a ball centered at tmpSpos and radius tdeltaM
			double rx,ry,rz,rrmod=0;
			do 
			{
				rx= (2 * _gen->d_rand()) - 1;
				ry= (2 * _gen->d_rand()) - 1;
				rz= (2 * _gen->d_rand()) - 1;
				rrmod = rx*rx + ry*ry + rz*rz;
			}while(rrmod>1);
			coordrobot[0] = tmpSpos[0] + tdeltaM*rx;
			coordrobot[1] = tmpSpos[1] + tdeltaM*ry;
			coordrobot[2] = tmpSpos[2] + tdeltaM*rz;

			//sample from box centered at tmpSpos
			//for(int k =0; k < 3; k++)
			//{
			//	double rr=_gen->d_rand();
			//	//fprintf(fp,"%f\n",rr);
			//	coordrobot[k] = tmpSpos[k] + tdeltaM*(2*(KthReal)rr-1);
			//}

			//sample orientation
			for(int k =0; k < 3; k++)
			{
				double rr=_gen->d_rand();
				//fprintf(fp,"%f\n",rr);
				coordrobot[k+3] = tmpSrot[k]+ rdeltaM*(2*(KthReal)rr-1);
				if(coordrobot[k+3]<0) coordrobot[k+3]=0;
				else if(coordrobot[k+3]>1) coordrobot[k+3]=1;
			}

			//compute inverse kinematics. solution is reloaded in same vector coord
			//bool maintainSameWrist=true;
			bool maintainSameWrist;
			//if(_gen->d_rand()<0.5) maintainSameWrist=true;
			//else maintainSameWrist=false;
			maintainSameWrist=true;
			bool invKinSolved=ArmInverseKinematics(coordrobot, goalSamp(), maintainSameWrist);
						
					
			if(invKinSolved==true) 
			{
				//load arm coordinates (normalized)
				KthReal low[6];
				KthReal high[6];
				for(int k=0; k < 6; k++)
				{
					//normalize
					low[k] = *_wkSpace->getRobot(0)->getLink(k+1)->getLimits(true);
					high[k] = *_wkSpace->getRobot(0)->getLink(k+1)->getLimits(false);
					coord[k]=(coordrobot[k]-low[k])/(high[k]-low[k]);
				}

				//Set the new sample with the arm coorinates and check for autocollision.	
                for(int k=6; k < _wkSpace->getNumRobControls(); k++)	coord[k]=goalSamp()->getCoords()[k]  ;//dummmy -  set to goal values for later call to getHandConfig
				_wkSpace->getRobot(0)->control2Pose(coord); 
				autocol = _wkSpace->getRobot(0)->autocollision(1);//test for the trunk
			}
			else{
				autocol = true;//invkinematics failed, considered as an autocolision to continue looping
			}

			trials++;
		}while(autocol==true && trials<maxtrials);

		if(autocol==true) return false;
		
		//Set the coordinates of the hand joints at a autocollision-free conf
		bool flag;
		//float r=0;
		
		//if(_gen->d_rand()<0.5) flag = false; //use as hand coords those of the goal sample
		//else flag = true; //random sample the hand 

		flag = true; //random sample the hand 
		if(handWholeRange)
		{
			//Set the coord values to -1 in order to sample within the whole hand workspace, not only around the goal
			//when calling to getHandConfig
            for(int k=6; k < _wkSpace->getNumRobControls(); k++)	coord[k]=-1;
		}
		else{
			vector<KthReal>& coordgoal = s->getCoords();
            for(int k=6; k < _wkSpace->getNumRobControls(); k++)	coord[k]= coordgoal[k];
		}
		if(getHandConfig(coord, flag, -1))	
		{
			//autocollisionfree sample found
			tmpSample->setCoords(coord);
			
			///////////////////////////////////////////////////////////////////////
				_triedSamples++;
				if( !_wkSpace->collisionCheck(tmpSample))
					{ 		
						//////////////////////////////////////////////////////////////////////
					//Cinematica Directa para encontrar la posición del Robot con la nueva muestra
					_wkSpace->getRobot(0)->Kinematics(tmpSample->getMappedConf().at(0).getRn()); 
					mt::Transform temptransf = _wkSpace->getRobot(0)->getLinkTransform(6);
					mt::Point3 temptrans = temptransf.getTranslation();
					///////////////////////////////////////////////////////////
					float dist=0;//,dif_dist;
					//Distancia de la nueva muestra libre al objetivo
					dist=temptrans.distance(_goaltrans);//goaltrans: distancia objetivo
					
					//Adicionamos una nueva muestra libre de colisiones
					//if(dist<=_deltaR)
					//{
						_samples->add(tmpSample);
						PRMPlanner::connectLastSample( );
						/////////////////////////////////////////
                        tmpSample = new Sample(_wkSpace->getNumRobControls());
						//Adicionamos la distancia de la muestra (en centimetros)
						_distance.push_back(dist);
						return true;
					//}
				}
		}
	
		return false;
	}


	
	//!samples aroun the goal sample or around a sample that belongs to the same connected component than the goal
	//!uses Gaussina sampling
	bool PRMPCAHandArmPlanner::getSampleInGoalRegionRealworldGaussian(double tdeltaM, double rdeltaM, bool handWholeRange)
	{
		boost::mt19937 rng(time(0));
		boost::normal_distribution<double> gaussian_dist(0,0.1); // Parametro sigma
		GaussianGenerator gaussianGen(rng,gaussian_dist);

	    int trials, maxtrials;
        vector<KthReal> coord(_wkSpace->getNumRobControls());
		bool autocol;//flag to test autocollisions
		Sample *tmpSample;
        tmpSample = new Sample(_wkSpace->getNumRobControls());
		Sample *tmpSample2;
        tmpSample2 = new Sample(_wkSpace->getNumRobControls());

		//Set the coordinates of the robot joints 
		//Randomly set the coordinates of the robot joints at a autocollision-free conf
		trials=0;
		maxtrials=100;
		Sample *s;
		int indexsam;
		int kcc;
		//_gen->rand_init();
		do{
			vector<KthReal> tmpSpos; tmpSpos.resize(3); 
			vector<KthReal> tmpSrot; tmpSrot.resize(3); 
			vector<KthReal> coordrobot; coordrobot.resize(6); 
	
			//randomly compute the coords of a sample of the conecected component of the goal
			//or the coordinates of the goal
			
			double rr=_gen->d_rand();
			//fprintf(fp,"%f\n",rr);
			if(rr<0.8)
			{
				int cc = goalSamp()->getConnectedComponent();
				kcc = _ccMap[cc]->getSize(); 
				
				double rr=_gen->d_rand();
				//fprintf(fp,"%f\n",rr);
				indexsam = rr*kcc;
				//cout<<"sampling around "<<indexsam<<endl;
				 s = _ccMap[cc]->getSampleAt(indexsam);
			}
			else s = goalSamp();

			std::vector<KthReal> tmpcoordTCPpos; tmpcoordTCPpos.resize(3);  
			std::vector<KthReal> tmpcoordTCPori; tmpcoordTCPori.resize(4);
			_wkSpace->getRobot(0)->Kinematics(s->getMappedConf().at(0).getRn()); 
			mt::Transform ctransfgoal = _wkSpace->getRobot(0)->getLinkTransform(6);
			mt::Point3 ctransgoal = ctransfgoal.getTranslation();
			mt::Rotation crotgoal = ctransfgoal.getRotation();
			for(int k=0;k<3;k++) tmpcoordTCPpos[k] =  ctransgoal[k];
			for(int k=0;k<4;k++) tmpcoordTCPori[k] =  crotgoal[k];
			_goalse3.setPos(tmpcoordTCPpos);
			_goalse3.setOrient(tmpcoordTCPori);
			
			tmpSpos	= _goalse3.getPos();
			tmpSrot	= _goalse3.getParams();
			
						
			//sample position in a ball centered at tmpSpos and radius tdeltaM
			double rx,ry,rz,rrmod=0;
			do 
			{
				rx= (2 * _gen->d_rand()) - 1;
				ry= (2 * _gen->d_rand()) - 1;
				rz= (2 * _gen->d_rand()) - 1;
				rrmod = rx*rx + ry*ry + rz*rz;
			}while(rrmod>1);
			double tt = tdeltaM * _gen->d_rand();
			coordrobot[0] = tmpSpos[0] + tt*rx;
			coordrobot[1] = tmpSpos[1] + tt*ry;
			coordrobot[2] = tmpSpos[2] + tt*rz;

			//sample from box centered at tmpSpos
			//for(int k =0; k < 3; k++)
			//{
			//	double rr=_gen->d_rand();
			//	//fprintf(fp,"%f\n",rr);
			//	coordrobot[k] = tmpSpos[k] + tdeltaM*(2*(KthReal)rr-1);
			//}

			//sample orientation
			for(int k =0; k < 3; k++)
			{
				double rr=_gen->d_rand();
				//fprintf(fp,"%f\n",rr);
				coordrobot[k+3] = tmpSrot[k]+ rdeltaM*(2*(KthReal)rr-1);
				if(coordrobot[k+3]<0) coordrobot[k+3]=0;
				else if(coordrobot[k+3]>1) coordrobot[k+3]=1;
			}

			//compute inverse kinematics. solution is reloaded in same vector coord
			//bool maintainSameWrist=true;
			bool maintainSameWrist;
			//if(_gen->d_rand()<0.5) maintainSameWrist=true;
			//else maintainSameWrist=false;
			maintainSameWrist=true;
			bool invKinSolved=ArmInverseKinematics(coordrobot, goalSamp(), maintainSameWrist);
						
					
			if(invKinSolved==true) 
			{
				//load arm coordinates (normalized)
				KthReal low[6];
				KthReal high[6];
				for(int k=0; k < 6; k++)
				{
					//normalize
					low[k] = *_wkSpace->getRobot(0)->getLink(k+1)->getLimits(true);
					high[k] = *_wkSpace->getRobot(0)->getLink(k+1)->getLimits(false);
					coord[k]=(coordrobot[k]-low[k])/(high[k]-low[k]);
				}

				//Set the new sample with the arm coorinates and check for autocollision.	
                for(int k=6; k < _wkSpace->getNumRobControls(); k++)	coord[k]=goalSamp()->getCoords()[k]  ;//dummmy -  set to goal values for later call to getHandConfig
				_wkSpace->getRobot(0)->control2Pose(coord); 
				autocol = _wkSpace->getRobot(0)->autocollision(1);//test for the trunk
			}
			else{
				autocol = true;//invkinematics failed, considered as an autocolision to continue looping
			}

			trials++;
		}while(autocol==true && trials<maxtrials);

		if(autocol==true) return false;
		
		//Set the coordinates of the hand joints at a autocollision-free conf
		bool flag;
		//float r=0;
		
		//if(_gen->d_rand()<0.5) flag = false; //use as hand coords those of the goal sample
		//else flag = true; //random sample the hand 

		flag = true; //random sample the hand 
		if(handWholeRange)
		{
			//Set the coord values to -1 in order to sample within the whole hand workspace, not only around the goal
			//when calling to getHandConfig
            for(int k=6; k < _wkSpace->getNumRobControls(); k++)	coord[k]=-1;
		}
		else{
			vector<KthReal>& coordgoal = s->getCoords();
            for(int k=6; k < _wkSpace->getNumRobControls(); k++)	coord[k]= coordgoal[k];
		}
		if(getHandConfig(coord, flag, -1))	
		{
			//autocollisionfree sample found
			tmpSample->setCoords(coord);
			
			///////////////////////////////////////////////////////////////////////
			_triedSamples++;
				
			trials=0;
			//compute tmpSample2
			do
			{
				KthReal v=-1.0;
				KthReal sigmaHand=1.0;//0.01;
				KthReal sigmaArm=0.05;//0.01;
				vector<KthReal>& coords = tmpSample->getCoords();
				vector<KthReal> coords2; coords2.resize(coords.size());
				//change coords
				//arm
				for(int j = 0; j < 6 ; j++)
				{
					v=-1.0;
					while(v<0.0 || v>1.0)
						v = coords.at(j) + sigmaArm * gaussianGen();
						//v = coords.at(j) + sigmaArm* (*_gaussianGen)();
						//v = coords.at(j) + 2*sigmaArm*(_gen->d_rand()-0.5);
						//v = coords.at(j);
					coords2[j]=v;
				}	
				//hand
                for(int j = 6; j < _wkSpace->getNumRobControls() ; j++)
				{
					v=-1.0;
					while(v<0.0 || v>1.0)
						v = coords.at(j) + sigmaHand * gaussianGen();
						//v = coords.at(j) + sigmaHand*(*_gaussianGen)();
						//v = coords.at(j) + 2*sigmaHand*(_gen->d_rand()-0.5);
					coords2[j]=v;
				}		
				//load coords to sample
				tmpSample2->setCoords(coords2);
				//check autocolision				
				_wkSpace->getRobot(0)->control2Pose(coords2); 
				autocol = _wkSpace->getRobot(0)->autocollision();
				trials++;
			}while( autocol == true && trials<maxtrials);
			if(autocol==true) return false;

			//now we have two close autocolision-free samples
			//we check for collision and keep the one that is free (if one is in collision and the other free)
			//otherwise we discard both
			bool tmpSample_IsCollision = _wkSpace->collisionCheck(tmpSample);
			bool tmpSample2_IsCollision = _wkSpace->collisionCheck(tmpSample2);

			/*
			if(tmpSample_IsCollision == false)
			{				
				//////////////////////////////////////////////////////////////////////
				//Cinematica Directa para encontrar la posición del Robot con la nueva muestra
				_wkSpace->getRobot(0)->Kinematics(tmpSample->getMappedConf().at(0).getRn()); 
				mt::Transform temptransf = _wkSpace->getRobot(0)->getLinkTransform(6);
				mt::Point3 temptrans = temptransf.getTranslation();
				///////////////////////////////////////////////////////////
				float dist=0;//,dif_dist;
				//Distancia de la nueva muestra libre al objetivo
				dist=temptrans.distance(_goaltrans);//goaltrans: distancia objetivo
					
				//Adicionamos una nueva muestra libre de colisiones
				_samples->add(tmpSample);
				PRMPlanner::connectLastSample( );
				_distance.push_back(dist);
			}
			if(tmpSample2_IsCollision==false)
			{			
				//////////////////////////////////////////////////////////////////////
				//Cinematica Directa para encontrar la posición del Robot con la nueva muestra
				_wkSpace->getRobot(0)->Kinematics(tmpSample2->getMappedConf().at(0).getRn()); 
				mt::Transform temptransf = _wkSpace->getRobot(0)->getLinkTransform(6);
				mt::Point3 temptrans = temptransf.getTranslation();
				///////////////////////////////////////////////////////////
				float dist=0;//,dif_dist;
				//Distancia de la nueva muestra libre al objetivo
				dist=temptrans.distance(_goaltrans);//goaltrans: distancia objetivo
					
				//Adicionamos una nueva muestra libre de colisiones
				_samples->add(tmpSample2);
				PRMPlanner::connectLastSample( );
				_distance.push_back(dist);
			}
			if(tmpSample_IsCollision == false || tmpSample2_IsCollision == false) return true;
			*/
			
			
			Sample *chosenSample;
			bool foundsample=false;
			if(tmpSample_IsCollision == true && tmpSample2_IsCollision == false )
			{
				chosenSample = tmpSample2;
				foundsample = true;
			}
			else if(tmpSample_IsCollision == false && tmpSample2_IsCollision==true)
			{
				chosenSample = tmpSample;
				foundsample = true;
			}
				
			if(foundsample==true)
			{
				//////////////////////////////////////////////////////////////////////
				//Cinematica Directa para encontrar la posición del Robot con la nueva muestra
				_wkSpace->getRobot(0)->Kinematics(chosenSample->getMappedConf().at(0).getRn()); 
				mt::Transform temptransf = _wkSpace->getRobot(0)->getLinkTransform(6);
				mt::Point3 temptrans = temptransf.getTranslation();
				///////////////////////////////////////////////////////////
				float dist=0;//,dif_dist;
				//Distancia de la nueva muestra libre al objetivo
				dist=temptrans.distance(_goaltrans);//goaltrans: distancia objetivo
					
				//Adicionamos una nueva muestra libre de colisiones
				_samples->add(chosenSample);
				PRMPlanner::connectLastSample( );
				_distance.push_back(dist);
				return true;
			}
		}
		return false;
	}


	//!samples aroun the goal sample or around a sample that belongs to the same connected component than the goal
	//!uses BridgeTest sampling
	bool PRMPCAHandArmPlanner::getSampleInGoalRegionRealworldBridgeTest(double tdeltaM, double rdeltaM, bool handWholeRange)
	{
		boost::mt19937 rng(time(0));
		boost::normal_distribution<double> gaussian_dist(0,0.1); // Parametro sigma
		GaussianGenerator gaussianGen(rng,gaussian_dist);

	    int trials, maxtrials;
        vector<KthReal> coord(_wkSpace->getNumRobControls());
		bool autocol;//flag to test autocollisions
		Sample *tmpSample;
        tmpSample = new Sample(_wkSpace->getNumRobControls());
		Sample *tmpSample2;
        tmpSample2 = new Sample(_wkSpace->getNumRobControls());

		//Set the coordinates of the robot joints 
		//Randomly set the coordinates of the robot joints at a autocollision-free conf
		trials=0;
		maxtrials=100;
		Sample *s;
		int indexsam;
		int kcc;
		//_gen->rand_init();
		do{
			vector<KthReal> tmpSpos; tmpSpos.resize(3); 
			vector<KthReal> tmpSrot; tmpSrot.resize(3); 
			vector<KthReal> coordrobot; coordrobot.resize(6); 
	
			//randomly compute the coords of a sample of the conecected component of the goal
			//or the coordinates of the goal
			
			double rr=_gen->d_rand();
			//fprintf(fp,"%f\n",rr);
			if(rr<0.8)
			{
				int cc = goalSamp()->getConnectedComponent();
				kcc = _ccMap[cc]->getSize(); 
				
				double rr=_gen->d_rand();
				//fprintf(fp,"%f\n",rr);
				indexsam = rr*kcc;
				//cout<<"sampling around "<<indexsam<<endl;
				 s = _ccMap[cc]->getSampleAt(indexsam);
			}
			else s = goalSamp();

			std::vector<KthReal> tmpcoordTCPpos; tmpcoordTCPpos.resize(3);  
			std::vector<KthReal> tmpcoordTCPori; tmpcoordTCPori.resize(4);
			_wkSpace->getRobot(0)->Kinematics(s->getMappedConf().at(0).getRn()); 
			mt::Transform ctransfgoal = _wkSpace->getRobot(0)->getLinkTransform(6);
			mt::Point3 ctransgoal = ctransfgoal.getTranslation();
			mt::Rotation crotgoal = ctransfgoal.getRotation();
			for(int k=0;k<3;k++) tmpcoordTCPpos[k] =  ctransgoal[k];
			for(int k=0;k<4;k++) tmpcoordTCPori[k] =  crotgoal[k];
			_goalse3.setPos(tmpcoordTCPpos);
			_goalse3.setOrient(tmpcoordTCPori);
			
			tmpSpos	= _goalse3.getPos();
			tmpSrot	= _goalse3.getParams();
			
						
			//sample position in a ball centered at tmpSpos and radius tdeltaM
			double rx,ry,rz,rrmod=0;
			do 
			{
				rx= (2 * _gen->d_rand()) - 1;
				ry= (2 * _gen->d_rand()) - 1;
				rz= (2 * _gen->d_rand()) - 1;
				rrmod = rx*rx + ry*ry + rz*rz;
			}while(rrmod>1);
			double tt = tdeltaM * _gen->d_rand();
			coordrobot[0] = tmpSpos[0] + tt*rx;
			coordrobot[1] = tmpSpos[1] + tt*ry;
			coordrobot[2] = tmpSpos[2] + tt*rz;

			//sample from box centered at tmpSpos
			//for(int k =0; k < 3; k++)
			//{
			//	double rr=_gen->d_rand();
			//	//fprintf(fp,"%f\n",rr);
			//	coordrobot[k] = tmpSpos[k] + tdeltaM*(2*(KthReal)rr-1);
			//}

			//sample orientation
			for(int k =0; k < 3; k++)
			{
				double rr=_gen->d_rand();
				//fprintf(fp,"%f\n",rr);
				coordrobot[k+3] = tmpSrot[k]+ rdeltaM*(2*(KthReal)rr-1);
				if(coordrobot[k+3]<0) coordrobot[k+3]=0;
				else if(coordrobot[k+3]>1) coordrobot[k+3]=1;
			}

			//compute inverse kinematics. solution is reloaded in same vector coord
			//bool maintainSameWrist=true;
			bool maintainSameWrist;
			//if(_gen->d_rand()<0.5) maintainSameWrist=true;
			//else maintainSameWrist=false;
			maintainSameWrist=true;
			bool invKinSolved=ArmInverseKinematics(coordrobot, goalSamp(), maintainSameWrist);
						
					
			if(invKinSolved==true) 
			{
				//load arm coordinates (normalized)
				KthReal low[6];
				KthReal high[6];
				for(int k=0; k < 6; k++)
				{
					//normalize
					low[k] = *_wkSpace->getRobot(0)->getLink(k+1)->getLimits(true);
					high[k] = *_wkSpace->getRobot(0)->getLink(k+1)->getLimits(false);
					coord[k]=(coordrobot[k]-low[k])/(high[k]-low[k]);
				}

				//Set the new sample with the arm coorinates and check for autocollision.	
                for(int k=6; k < _wkSpace->getNumRobControls(); k++)	coord[k]=goalSamp()->getCoords()[k]  ;//dummmy -  set to goal values for later call to getHandConfig
				_wkSpace->getRobot(0)->control2Pose(coord); 
				autocol = _wkSpace->getRobot(0)->autocollision(1);//test for the trunk
			}
			else{
				autocol = true;//invkinematics failed, considered as an autocolision to continue looping
			}

			trials++;
		}while(autocol==true && trials<maxtrials);

		if(autocol==true) return false;
		
		//Set the coordinates of the hand joints at a autocollision-free conf
		bool flag;
		//float r=0;
		
		//if(_gen->d_rand()<0.5) flag = false; //use as hand coords those of the goal sample
		//else flag = true; //random sample the hand 

		flag = true; //random sample the hand 
		if(handWholeRange)
		{
			//Set the coord values to -1 in order to sample within the whole hand workspace, not only around the goal
			//when calling to getHandConfig
            for(int k=6; k < _wkSpace->getNumRobControls(); k++)	coord[k]=-1;
		}
		else{
			vector<KthReal>& coordgoal = s->getCoords();
            for(int k=6; k < _wkSpace->getNumRobControls(); k++)	coord[k]= coordgoal[k];
		}
		if(getHandConfig(coord, flag, -1))	
		{
			//autocollisionfree sample found
			tmpSample->setCoords(coord);
			
			///////////////////////////////////////////////////////////////////////
			_triedSamples++;
				
			trials=0;
			//compute tmpSample2
			do
			{
				KthReal v=-1.0;
				KthReal sigmaHand=1.0;//0.01;
				KthReal sigmaArm=0.05;//0.01;
				vector<KthReal>& coords = tmpSample->getCoords();
				vector<KthReal> coords2; coords2.resize(coords.size());
				//change coords
				//arm
				for(int j = 0; j < 6 ; j++)
				{
					v=-1.0;
					while(v<0.0 || v>1.0)
						v = coords.at(j) + sigmaArm * gaussianGen();
						//v = coords.at(j) + sigmaArm* (*_gaussianGen)();
						//v = coords.at(j) + 2*sigmaArm*(_gen->d_rand()-0.5);
						//v = coords.at(j);
					coords2[j]=v;
				}	
				//hand
                for(int j = 6; j < _wkSpace->getNumRobControls() ; j++)
				{
					v=-1.0;
					while(v<0.0 || v>1.0)
						v = coords.at(j) + sigmaHand * gaussianGen();
						//v = coords.at(j) + sigmaHand*(*_gaussianGen)();
						//v = coords.at(j) + 2*sigmaHand*(_gen->d_rand()-0.5);
					coords2[j]=v;
				}		
				//load coords to sample
				tmpSample2->setCoords(coords2);
				//check autocolision				
				_wkSpace->getRobot(0)->control2Pose(coords2); 
				autocol = _wkSpace->getRobot(0)->autocollision();
				trials++;
			}while( autocol == true && trials<maxtrials);
			if(autocol==true) return false;

			//now we have two close autocolision-free samples
			//we check for collision and keep the one that is free (if one is in collision and the other free)
			//otherwise we discard both
			bool tmpSample_IsCollision = _wkSpace->collisionCheck(tmpSample);
			bool tmpSample2_IsCollision = _wkSpace->collisionCheck(tmpSample2);
			
			if(tmpSample_IsCollision == true && tmpSample2_IsCollision == true )
			{
				Sample *midpointSample;
				KthReal midpoint;
				int maxmidpointtrials=10;
				int midpointtrials=0;

				while(midpointtrials<maxmidpointtrials)
				{
					midpointtrials++;
					KthReal midpoint = _gen->d_rand();

					midpointSample = tmpSample->interpolate(tmpSample2,midpoint);

					if(_wkSpace->collisionCheck(midpointSample)==false)
					{
						//////////////////////////////////////////////////////////////////////
						//Cinematica Directa para encontrar la posición del Robot con la nueva muestra
						_wkSpace->getRobot(0)->Kinematics(midpointSample->getMappedConf().at(0).getRn()); 
						mt::Transform temptransf = _wkSpace->getRobot(0)->getLinkTransform(6);
						mt::Point3 temptrans = temptransf.getTranslation();
						///////////////////////////////////////////////////////////
						float dist=0;//,dif_dist;
						//Distancia de la nueva muestra libre al objetivo
						dist=temptrans.distance(_goaltrans);//goaltrans: distancia objetivo
					
						//Adicionamos una nueva muestra libre de colisiones
						_samples->add(midpointSample);
						PRMPlanner::connectLastSample( );
						_distance.push_back(dist);
						return true;
					}
				}
			}
		}
		return false;
	}




		//!samples around the line connecting init and goal configurations 
	int PRMPCAHandArmPlanner::getSamplesBetweenInitGoal(int maxinterpolatedpoints, double tdeltaM, double rdeltaM, bool handWholeRange)
	{
		int samplesadded=0;
	    int trials, maxtrials;
        vector<KthReal> coord(_wkSpace->getNumRobControls());
		bool autocol;//flag to test autocollisions
		Sample *tmpSample;
        tmpSample = new Sample(_wkSpace->getNumRobControls());

		//Set the coordinates of the robot joints 
		//Randomly set the coordinates of the robot joints at a autocollision-free conf
		trials=0;
		maxtrials=100;
		Sample *s;

		
		//t is the interpolation parameter
		double t; 
		for(int i=0;i<=maxinterpolatedpoints;i++)
		{
			t=((double)i)/maxinterpolatedpoints;
			
			//start do-while loop to find a configuration of tha arm between the init and goal
			//that is free from autocolision
			do{
				vector<KthReal> tmpSpos; tmpSpos.resize(3); 
				vector<KthReal> tmpSrot; tmpSrot.resize(3); 
				vector<KthReal> coordrobot; coordrobot.resize(6); 
				
				//Position of the wrist at the NEW sample
				//comuted by adding random noise to the interpolated position
				for(int k =0; k < 3; k++)
				{
					tmpSpos[k]	= _inise3.getPos()[k] + t*(_goalse3.getPos()[k]-_inise3.getPos()[k]);
					double rr=_gen->d_rand();
					//fprintf(fp,"%f\n",rr);
					coordrobot[k] = tmpSpos[k] + tdeltaM*(2*(KthReal)rr-1);
				}
				//Orientation of the wrist at the NEW sample
				//computed by adding random noise around ini or goal orientation 
				for(int k =0; k < 3; k++)
				{
					double rr=_gen->d_rand();
					//fprintf(fp,"%f\n",rr);
					if(t<0.5) coordrobot[k+3] = _inise3.getParams()[k]+ rdeltaM*(2*(KthReal)rr-1);
					else coordrobot[k+3] = _goalse3.getParams()[k]+ rdeltaM*(2*(KthReal)rr-1);
					if(coordrobot[k+3]<0) coordrobot[k+3]=0;
					else if(coordrobot[k+3]>1) coordrobot[k+3]=1;
				}

				//compute inverse kinematics. solution is reloaded in same vector coord
				bool maintainSameWrist;
				//if(_gen->d_rand()<0.5) maintainSameWrist=true;
				//else maintainSameWrist=false;
				maintainSameWrist=true;
				bool invKinSolved=ArmInverseKinematics(coordrobot, goalSamp(), maintainSameWrist);
				if(invKinSolved==false) invKinSolved=ArmInverseKinematics(coordrobot, initSamp(), maintainSameWrist);
						
					
				if(invKinSolved==true) 
				{
					//load arm coordinates (normalized)
					KthReal low[6];
					KthReal high[6];
					for(int k=0; k < 6; k++)
					{
						//normalize
						low[k] = *_wkSpace->getRobot(0)->getLink(k+1)->getLimits(true);
						high[k] = *_wkSpace->getRobot(0)->getLink(k+1)->getLimits(false);
						coord[k]=(coordrobot[k]-low[k])/(high[k]-low[k]);
					}

					//Set the new sample with the arm coorinates and check for autocollision.	
                    for(int k=6; k < _wkSpace->getNumRobControls(); k++)	coord[k]=goalSamp()->getCoords()[k]  ;//dummmy -  set to goal values for later call to getHandConfig
					_wkSpace->getRobot(0)->control2Pose(coord); 
					autocol = _wkSpace->getRobot(0)->autocollision(1);//test for the trunk
				}
				else{
					autocol = true;//invkinematics failed, considered as an autocolision to continue looping
				}

				trials++;
			}while(autocol==true && trials<maxtrials);
		
			//if failed to find an arm configuration (trials==maxtrials) continue looping the for loop
			if(autocol==true) continue;
		

			//Set the coordinates of the hand joints at a autocollision-free conf
			bool flag = true; //random sample the hand 
			if(handWholeRange)
			{
				//Set the coord values to -1 in order to sample within the whole hand workspace, not only around the goal
				//when calling to getHandConfig
                for(int k=6; k < _wkSpace->getNumRobControls(); k++)	coord[k]=-1;
			}
			else{
				vector<KthReal>& coordgoal = goalSamp()->getCoords();
				vector<KthReal>& coordini = initSamp()->getCoords();
                for(int k=6; k < _wkSpace->getNumRobControls(); k++)	coord[k]= coordini[k] + t*(coordgoal[k]-coordini[k]);
			}
			if(getHandConfig(coord, flag, -1))	
			{
				//autocollisionfree sample found
				tmpSample->setCoords(coord);
			
				///////////////////////////////////////////////////////////////////////
				_triedSamples++;
				if( !_wkSpace->collisionCheck(tmpSample))
					{ 		
						//////////////////////////////////////////////////////////////////////
					//Cinematica Directa para encontrar la posición del Robot con la nueva muestra
					_wkSpace->getRobot(0)->Kinematics(tmpSample->getMappedConf().at(0).getRn()); 
					mt::Transform temptransf = _wkSpace->getRobot(0)->getLinkTransform(6);
					mt::Point3 temptrans = temptransf.getTranslation();
					///////////////////////////////////////////////////////////
					float dist=0;//,dif_dist;
					//Distancia de la nueva muestra libre al objetivo
					dist=temptrans.distance(_goaltrans);//goaltrans: distancia objetivo
					
					//Adicionamos una nueva muestra libre de colisiones
					//if(dist<=_deltaR)
					//{
						_samples->add(tmpSample);
						samplesadded++;
						//PRMPlanner::connectLastSample( );
						PRMPlanner::connectLastSample(initSamp(), goalSamp());
						/////////////////////////////////////////
                        tmpSample = new Sample(_wkSpace->getNumRobControls());
						//Adicionamos la distancia de la muestra (en centimetros)
						_distance.push_back(dist);
					//}
				}
			}
		//end for
		}
	
		return samplesadded;
	}




    void PRMPCAHandArmPlanner::saveData()
	{
		//stores the simulationpath samples i(both robot and hand) in a single file 
		if(_solved) 
		{
			vector<KthReal> coordvector;
			RobConf* joints;

			cout << "Save PATH to FILE"<<endl;
			FILE *fp;
			fp = fopen ("PCAhandrobotconf.txt","wt");

			for(unsigned i = 0; i < _simulationPath.size(); i++){
				coordvector.clear();
				//convert from controls to real coordinates
                for(int k = 0; k < _wkSpace->getNumRobControls(); k++)
					coordvector.push_back( _simulationPath[i]->getCoords()[k] ); 
								
				_wkSpace->getRobot(0)->control2Pose(coordvector);
				joints = _wkSpace->getRobot(0)->getCurrentPos();

				//arm coordinates
				int j;
				for(j =0; j < 6; j++)
					fprintf(fp,"%.2f ",joints->getRn().getCoordinate(j)*180.0/PI);
				
				//hand coordinates
				for(; j < joints->getRn().getDim(); j++)
				{
					if(j==6 || j==11 || j==15 || j==19 || j==23) continue;
					fprintf(fp,"%.2f ",joints->getRn().getCoordinate(j)*180.0/PI);
				}
				fprintf(fp,"\n");
			}
			fclose(fp);
		}
		else{
			cout << "Sorry: Path not yet found"<<endl;
		}
	
    }

  }
};

#endif // KAUTHAM_USE_ARMADILLO
