
//FIXME: this planner is done for a single TREE robot (associtated to wkSpace->robots[0])

#if defined(KAUTHAM_USE_ARMADILLO)

#include <stdio.h>
#include <time.h>
#include "prmPCAhandarmplanner.h"
#include "ML_locplan.h"

///////////////Armadillo////////
#include <iostream>

using namespace arma;
using namespace std;

//////////////////////
 
 namespace libPlanner {
  namespace PRM{
		
	PRMPCAHandArmPlanner::PRMPCAHandArmPlanner(SPACETYPE stype, Sample *init, Sample *goal, SampleSet *samples, Sampler *sampler, 
           WorkSpace *ws, LocalPlanner *lcPlan, KthReal ssize,int cloudSize, int samplingV, KthReal cloudRad, int samplingR,float distgoal,float distsamplingpcagoal)
      :PRMHandPlanner(stype, init, goal, samples, sampler, ws, lcPlan,  ssize, cloudSize,  cloudRad)
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

		removeParameter("Neigh Thresshold");
		removeParameter("Cloud deltaM");
		removeParameter("Cloud Radius");
		removeParameter("Cloud Size");
	    removeParameter("P(connect to Ini-Goal)");

		//fp = fopen("rand.txt","wt"); 
	}


	PRMPCAHandArmPlanner::~PRMPCAHandArmPlanner(){
	}

    bool PRMPCAHandArmPlanner::setParameters()
	{
      //PRMHandPlanner::setParameters(); //why it is not called?
      try{
        HASH_S_K::iterator it = _parameters.find("Step Size");
        if(it != _parameters.end())
			setStepSize(it->second);//also changes stpssize of localplanner
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
		  _samples->setANNdatastructures(_kNeighs, _maxNumSamples);
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
		  _samples->setANNdatastructures(_kNeighs, _maxNumSamples);
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
				q=_wkSpace->getDimension()-(_wkSpace->getRobot(0)->getTrunk());

				if(numPMDs==-1 || numPMDs>(_wkSpace->getDimension()-_wkSpace->getRobot(0)->getTrunk())) 
						numPMDs = _wkSpace->getDimension()-_wkSpace->getRobot(0)->getTrunk();
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
				for(; k < _wkSpace->getDimension(); k++)
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
			for(int k = 0; k < _wkSpace->getDimension(); k++)
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



	bool PRMPCAHandArmPlanner::trySolve()
	{
		//_gen->rand_init();
		//fprintf(fp,"---------------------------------------------\n");

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
		std::vector<KthReal> coord(wkSpace()->getDimension());
		vector<KthReal> coordvector;
		int trials = 0;
		int maxtrials = 100;
		int ig=0;
		int maxsamplesingoal=11;//PCA Action on 11 elements (Arm:6 and Hand:5 )

		_distance.clear();//clear vector distancia
		_indexpca.clear();//clear vector indexpca
		int countwr=0;
		
		int myflag=0, z=0;
		float R=0;
		callpca=0;
		//////////////////////////////////////////////////////////////////////////////
	 
	  	//Sample around goal up to maxsamplesingoal (in order to have as many samples as required for a first call to PCA)
		for(ig=0; trials<maxtrials && ig<maxsamplesingoal; ig) {
		  if(getSampleInGoalRegion(_deltaI , 0.02)) 
		  {
			  ig++;
		  }	  
		}
	 
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
				getSamplesBetweenInitGoal(deltaM, 0.02, false);
						
				//get samples from V
				if(getSampleRandPCA(_deltaR))
				{		
					//verify if the samples from V are within bounds and free
					if(matPCA.n_rows>0)//matrand
					{
						////////////////Creación de variables////////////////////
						vector<KthReal> coord(_wkSpace->getDimension());
						Sample *tmpSample;
						tmpSample = new Sample(_wkSpace->getDimension());
						
						///////////////////////////////////////////////////////////
						//Procesamiento para comprobar las nuevas muestas 20x11, son libres de colisión
						for(int z=0; z<matPCA.n_rows ; z++)
						{
							rowvec pointpca=matPCA.row(z);//Fila de 11 elementos(6 primeros del brazo y los 5 ultimos de la mano)
							for(int k=0; k < 11; k++)
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
								PRMPlanner::connectLastSample(initSamp(), goalSamp());

								//create next sample
								tmpSample = new Sample(_wkSpace->getDimension());
								
								_indexpca.push_back(_samples->getSize()-1);//Se agrega el indice de la muestra generada por Sampling PCA									

								if(goalSamp()->getConnectedComponent() == initSamp()->getConnectedComponent()) 
								{
									if(PRMPlanner::findPath())
									{
										printConnectedComponents();
										cout << "PRM Nodes = " << _samples->getSize() << endl;
										cout << "Number sampled configurations = " << n << endl;
														
										clock_t finaltime = clock();
										cout<<"TIME TO COMPUTE THE PATH = "<<(double)(finaltime-entertime)/CLOCKS_PER_SEC<<endl;
										PRMPlanner::smoothPath();

										clock_t finalsmoothtime = clock();
										cout<<"TIME TO SMOOTH THE PATH = "<<(double)(finalsmoothtime - finaltime)/CLOCKS_PER_SEC<<endl;
														
										cout << "Number of passes = " << p << " deltaM = " << deltaM<< endl;
										cout << " Call PCA = "<<callpca<<endl<<flush;
										cout << " Call Sampling World Real = "<<countwr<<endl<<flush;
										printPCAComponents();
																		
										cout << " number of collision-checks = "<<_triedSamples<<endl<<flush;
														
										_solved = true;
										_triedSamples = n;
										_generatedEdges = weights.size();
										_totalTime = (KthReal)(finaltime - entertime)/CLOCKS_PER_SEC ;
										_smoothTime = (KthReal)(finalsmoothtime - finaltime)/CLOCKS_PER_SEC ;
											
										
										return _solved;
									}
								}
								///////////////////////////////////////////////////////////////// 
								if(freepca>=_samplingV) {freepca=0; break;}
								////////////////////////////////////////////////////////////////
							}
						}
					}
				}
			}
			catch (...){ cout<<"Data not recognize"<<endl;}
						
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
				}
			}
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
	  printPCAComponents();
	 


    _solved = false;
    _triedSamples = n;
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
		if(sizevd<11) return false;
	
		//else compute the V region
		///////////////////////////////////
				
		///////////Fill Matrix PCA11PMDs to compute PCA /////////////////////
		//use those samples within a distance R, and if more than 11 are available use those 
		//that also belong to the same connected component as the goal
		vector<Sample*> pacsamplevector;
		for(int i=0;i<sizevd;i++)
		{
			//consider only those of the same connected component as the goal
			if(i>=11 && _samples->getSampleAt(i+2)->getConnectedComponent()!=goalSamp()->getConnectedComponent()) continue;

			if(_distance.at(i)<R)//condición para muestras que se encuentran a una distancia menor a R
			{
				pacsamplevector.push_back(_samples->getSampleAt(i+2));//Conseguimos las muestras libre a partir del elemento numero 3 :(0)= Ini y (1)=Goal, 
			}
		}
		mat PCA11PMDs(pacsamplevector.size(),11);
		PCA11PMDs.fill(0.0);//inicilización de la matriz
		for(int i=0;i<pacsamplevector.size();i++)
		{
			for(int k = 0; k <11; k++) 
				PCA11PMDs(i,k)=pacsamplevector[i]->getCoords()[k];		
		}

		try
		{
			if(PCA11PMDs.n_rows>=11)//Condición para el calculo de PCA(minimo 11 elementos)
			{
				///////////////PCA Armadillo//////////////////////////////
				mat coeff;//coeff: principal component coefficients
				vec latent;//latent: principal component variances.
				vec explained;//explained: percentage of the total variance explained by each principal component. 
				princomp_cov(coeff, latent, explained,PCA11PMDs);//Calcula el PCA
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
				mat matrand(numsamplespca,11);
				//matrand(numsamplespca,11);
				matrand.fill(0.0);

				//_gen->rand_init();
				for(int i=0; i < numsamplespca; i++)
				{
					for(int j=0; j<11; j++){
						double rr=_gen->d_rand();
						//fprintf(fp,"%f\n",rr);
						matrand(i,j)=-lambdapca(0,j)+(2*lambdapca(0,j)*(rr));
					}
					
					rowvec zeta=trans((coeff*trans(matrand.row(i)))+trans(bar));
					
					for(int j=0; j<11; j++)
						matrand(i,j)=zeta(0,j);
					
				}

				matPCA= matrand;
				matrand.~Mat();
				PCA11PMDs.~Mat();

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
		vector<KthReal> coord(_wkSpace->getDimension());
		bool autocol;//flag to test autocollisions
		Sample *tmpSample;
        tmpSample = new Sample(_wkSpace->getDimension());

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
			
						
			//Add random noise to tmpS (only translational part)
			for(int k =0; k < 3; k++)
			{
				double rr=_gen->d_rand();
				//fprintf(fp,"%f\n",rr);
				coordrobot[k] = tmpSpos[k] + tdeltaM*(2*(KthReal)rr-1);
			}
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
				for(int k=6; k < _wkSpace->getDimension(); k++)	coord[k]=goalSamp()->getCoords()[k]  ;//dummmy -  set to goal values for later call to getHandConfig		
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
			for(int k=6; k < _wkSpace->getDimension(); k++)	coord[k]=-1;
		}
		else{
			vector<KthReal>& coordgoal = s->getCoords();
			for(int k=6; k < _wkSpace->getDimension(); k++)	coord[k]= coordgoal[k];
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
						tmpSample = new Sample(_wkSpace->getDimension());
						//Adicionamos la distancia de la muestra (en centimetros)
						_distance.push_back(dist);
						return true;
					//}
			}
		}
	
		return false;
	}


		//!samples around the line connecting init and goal configurations 
	int PRMPCAHandArmPlanner::getSamplesBetweenInitGoal(double tdeltaM, double rdeltaM, bool handWholeRange)
	{
		int samplesadded=0;
	    int trials, maxtrials;
		vector<KthReal> coord(_wkSpace->getDimension());
		bool autocol;//flag to test autocollisions
		Sample *tmpSample;
        tmpSample = new Sample(_wkSpace->getDimension());

		//Set the coordinates of the robot joints 
		//Randomly set the coordinates of the robot joints at a autocollision-free conf
		trials=0;
		maxtrials=100;
		Sample *s;

		int maxinterpolatedpoints=10;
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
					for(int k=6; k < _wkSpace->getDimension(); k++)	coord[k]=goalSamp()->getCoords()[k]  ;//dummmy -  set to goal values for later call to getHandConfig		
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
				for(int k=6; k < _wkSpace->getDimension(); k++)	coord[k]=-1;
			}
			else{
				vector<KthReal>& coordgoal = goalSamp()->getCoords();
				vector<KthReal>& coordini = initSamp()->getCoords();
				for(int k=6; k < _wkSpace->getDimension(); k++)	coord[k]= coordini[k] + t*(coordgoal[k]-coordini[k]);
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
						tmpSample = new Sample(_wkSpace->getDimension());
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
				for(int k = 0; k < _wkSpace->getDimension(); k++)
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
