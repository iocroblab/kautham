
//FIXME: this planner is done for a single TREE robot (associtated to wkSpace->robots[0])

#if defined(KAUTHAM_USE_ARMADILLO)

#include <stdio.h>
#include <time.h>
#include "prmhandplannerArmHandPCA.h"
#include "ML_locplan.h"

///////////////Armadillo////////
#include <iostream>

using namespace arma;
using namespace std;

//////////////////////
 
 namespace libPlanner {
  namespace PRM{
		
	PRMHandPlannerArmHandPCA::PRMHandPlannerArmHandPCA(SPACETYPE stype, Sample *init, Sample *goal, SampleSet *samples, Sampler *sampler, 
           WorkSpace *ws, LocalPlanner *lcPlan, KthReal ssize,int cloudSize, int samplingV, KthReal cloudRad, int samplingR,float distgoal,float distsamplingpcagoal)
      :PRMHandPlanner(stype, init, goal, samples, sampler, ws, lcPlan,  ssize, cloudSize,  cloudRad)
	{
		_idName = "PRM RobotArmHand PCA";
		
		_distancegoal=distgoal;
		addParameter("Distance Goal", _distancegoal);
		_distsamplingpcagoal=distsamplingpcagoal;
		addParameter("Distance Sampling PCA Goal",_distsamplingpcagoal);
		_samplingV=samplingV;
		addParameter("Num Sampling Free PCA", _samplingV);
		_samplingR=samplingR;
		addParameter("Num Sampling Free RW",_samplingR);


		//////////////////////////////////////////////////
		_cloudRadiusMax = cloudRad;
        addParameter("Lambda", _cloudRadiusMax);

		removeParameter("Neigh Thresshold");
		removeParameter("Cloud Radius");
		removeParameter("Cloud Size");
	}


	PRMHandPlannerArmHandPCA::~PRMHandPlannerArmHandPCA(){
			
	}

    bool PRMHandPlannerArmHandPCA::setParameters()
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
          _cloudRadiusMax = it->second;
		  
		}
        else
          return false;

       
		it = _parameters.find("Distance Goal");
        if(it != _parameters.end())
          _distancegoal = it->second;
        else
          return false;

		it = _parameters.find("Distance Sampling PCA Goal");
        if(it != _parameters.end())
          _distsamplingpcagoal = it->second;
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


	//! finds a random hand configuration for a given arm configuration
	//! returns a range for the thumb joint for which there is no collisions
	//! If randhand is false, the coord vector passed as a parameter is unchanged,
	//! only the thumb limits are computed; then the numPMDs parameter is not used.
	//! if randhand is true, it computes the random conf using a number numPMDs of
	//! PMDs. The value numPMDs = -1 means all of them.
  bool PRMHandPlannerArmHandPCA::getHandConfig(vector<KthReal>& coord,  bool randhand, int numPMDs)
	{
		vector<KthReal> coordvector;

		//If random - first find a autocollisionfree sample up to a max of maxtrials
		if(randhand)
		{
			bool autocol;
			int trials=0;
			int maxtrials=10;
			do{
				coordvector.clear();
				//sample the hand coordinates
				float q=0;
				q=_wkSpace->getDimension()-(_wkSpace->getRobot(0)->getTrunk());

				if(numPMDs==-1 || numPMDs>(_wkSpace->getDimension()-_wkSpace->getRobot(0)->getTrunk())) 
						numPMDs = _wkSpace->getDimension()-_wkSpace->getRobot(0)->getTrunk();
				int k;
				int kini = _wkSpace->getRobot(0)->getTrunk();

				//Aqui randon Hand/////////////////////////
				for(k = kini; k < kini+numPMDs; k++)
				{
					//hand coords not set - sample the whole range
					if(coord[k]==-1){
						//nonlinear behaviour- sample uniformly within a bigger range and then saturate
							KthReal size=1.4;//1.2;
							coord[k]=-((size-1.0)/2) + _gen->d_rand()*size; 
							//if(coord[k]<0) coord[k]=0;
						    //else if(coord[k]>1) coord[k]=1;
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
						coord[k] = coord[k]+ 0.3*(2*(KthReal)_gen->d_rand()-1);//0.5*
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

				//load the arm coordinates passed as a parameter
				for(int k =0; k < _wkSpace->getRobot(0)->getTrunk(); k++)
				{
					coordvector.push_back(coord[k]);
				}
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
	bool PRMHandPlannerArmHandPCA::ArmInverseKinematics(vector<KthReal> &carm, Sample *smp, bool maintainSameWrist)
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


 	void PRMHandPlannerArmHandPCA::setIniGoalSe3()
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
		mt::Point3 ctransgoal = ctransfgoal.getTranslation();
		mt::Rotation crotgoal = ctransfgoal.getRotation();
		for(int k=0;k<3;k++) tmpcoordTCPpos[k] =  ctransgoal[k];
		for(int k=0;k<4;k++) tmpcoordTCPori[k] =  crotgoal[k];
		_goalse3.setPos(tmpcoordTCPpos);
		_goalse3.setOrient(tmpcoordTCPori);
	}



 	bool PRMHandPlannerArmHandPCA::trySolve()
	{
		_neighThress = goalSamp()->getDistance(initSamp(), CONFIGSPACE) *1000;//  * 4;
//			cout<<"...._neighThress = "<<_neighThress<<endl<<flush;

		
		_triedSamples=0;

		if(_samples->changed())
		{
			PRMPlanner::clearGraph(); //also puts _solved to false;
		}

		if(_solved) {
			cout << "PATH ALREADY SOLVED"<<endl;
			return true;
		}
		
		cout<<"ENTERING TRYSOLVE!!!"<<endl;
	    clock_t entertime = clock();
		
		double radius;

    std::vector<KthReal> coord(wkSpace()->getDimension());
      Sample *tmpSample;
	  vector<KthReal> coordvector;
	  int trials, maxtrials;

	
	  //Create graph with initial and goal sample
	  if( !_isGraphSet )
	  {
		  //to try to connect ini and goal set threshold to 4.1*_neighThress
		  //Take into account that _neighThress is set to dist_ini_to_goal/4.
		_samples->findBFNeighs(_neighThress*4.1, _kNeighs);

		vector<KthReal>& coordinit = initSamp()->getCoords();
		getHandConfig(coordinit, false, 0);
		vector<KthReal>& coordgoal = goalSamp()->getCoords();
		getHandConfig(coordgoal, false, 0);

        connectSamples();
		PRMPlanner::loadGraph();
		if(PRMPlanner::findPath()) return true;
	  }

	  //load ini and goal transforms of the robot TCP as se3conf objects, to interpolate between them.
	  setIniGoalSe3();



	  int ig=0;
	  trials=0;
	  maxtrials=100;//100
	  int maxsamplesingoal=11;//_wkSpace->getDimension();//20;

	  //////////////////////////////////////////////////////////////////////////////
	  _distance.clear();//clear vector distancia
	  _indexpca.clear();//clear vector indexpca
	  int countwr=0;
	   ////////////////PCA Action on 11 elements (Arm:6 and Hand:5 )
	  int myflag=0, z=0;
	  float R=0;
	  callpca=0;
	  distanceig=0;
	  ///////////////////////////////////////////////////////
	  //Cinematica Directa para encontrar la posición del Robot en el objetivo
		_wkSpace->getRobot(0)->Kinematics(goalSamp()->getMappedConf().at(0).getRn()); 
		mt::Transform goaltransf = _wkSpace->getRobot(0)->getLinkTransform(6);
		goaltrans = goaltransf.getTranslation();
	  //////////////////////////////////////////////////////////
	  //Cinematica Directa para encontrar la posición del Robot en el objetivo
		_wkSpace->getRobot(0)->Kinematics(initSamp()->getMappedConf().at(0).getRn()); 
		mt::Transform initransf = _wkSpace->getRobot(0)->getLinkTransform(6);
		mt::Point3 initrans = initransf.getTranslation();

		distanceig=initrans.distance(goaltrans);
	  ///////////////////////////////////////////////////////
	 
	  //////////////////////////////////////////////////////////////////////////////
	 
	  
	  //sample around goal
	  //Población de Muestras Libres en el Mundo Real
	  //for(ig=0;trials<maxtrials && ig<maxsamplesingoal;trials++) {
	  for(ig=0;trials<maxtrials && ig<maxsamplesingoal;ig) {
		  if(getSampleInGoalRegion(_distsamplingpcagoal , 0.02)) 
		  
		  {
			  ig++;
			  // PRMPlanner::connectLastSample( goalSamp() );
		  }
		 
		 		  
	  }
	 
	  ///////////////////////////////////////////////////////////////////////////////////////////////////////
		float routegoal=0,routeinit=0,routeneighs=0,centreroute=0;
		centreroute=distanceig/2;
		routegoal=centreroute/2;
		routeinit=routegoal+centreroute;
		

		int p=0;
		int n=0;
		int freepca=0;
		int freerw=0;
		
			//set the radius of the sphere where to sample randomly
			//the radius increases as new passes are required
			

		radius = _distsamplingpcagoal; //_cloudRadius;
		do{
		try{		
					p++;
					//radius = _distsamplingpcagoal * p; //_cloudRadius * p;
					//if(radius>0.75*distanceig){
					//	p = 0;
					//	radius = _distsamplingpcagoal; //_cloudRadius;
					//}
					radius = radius+25;
					//if(radius>(distanceig)) radius = _distsamplingpcagoal;// p = 1;
					if(radius> _distancegoal*1.5) radius = _distsamplingpcagoal;
							
					if(getSampleRandPCA(_distancegoal))
							
					{
						
						if(matPCA.n_rows>0)//matrand
						{
							////////////////Creación de variables////////////////////
							vector<KthReal> coord(_wkSpace->getDimension());
							Sample *tmpSample;
							tmpSample = new Sample(_wkSpace->getDimension());
							//float r=0;
							///////////////////////////////////////////////////////////
							//Procesamiento para comprobar las nuevas muestas 20x11, son libres de colisión
							bool samplearmoutofbounds;
							bool samplehandoutofbounds;

							for(int z=0; z<matPCA.n_rows ; z++)
							{
								rowvec pointpca=matPCA.row(z);//Fila de 11 elementos(6 primeros del brazo y los 5 ultimos de la mano)

								samplearmoutofbounds=false;
								samplehandoutofbounds=false;
								for(int k=0; k < 11; k++)
								{
									coord[k]=pointpca(0,k);//coord:variable para almacenar los 11 elementos
									//if(coord[k]>1.0 || coord[k]<0.0) {
									//	if(k<6){
									//		samplearmoutofbounds=true;
									//		//break;
									//	}
									//	else{
									//		samplehandoutofbounds=true;
									//	//	if(coord[k]>1.0) coord[k] = 1.0;
									//	//	else coord[k] = 0.0;
									//	}

									//}
								}
								//if(sampleoutofbounds) continue;
								
								tmpSample->setCoords(coord);
								
								
									samplefree=true;

									_triedSamples++;
									samplefree=_wkSpace->collisionCheck(tmpSample);
									/*if(samplearmoutofbounds==true && samplefree==false)
									{
										int k=0;
										k++;
									}
									if(samplehandoutofbounds==true && samplefree==false)
									{
										int k=0;
										k++;
									}*/

									if( !(samplefree)) //comprobación si la muestra esta libre de colisiones
									{ 	
									
										n++;
										
										//Cinematica Directa para encontrar la posición del Robot con la nueva muestra
										_wkSpace->getRobot(0)->Kinematics(tmpSample->getMappedConf().at(0).getRn()); 
										mt::Transform temptransf = _wkSpace->getRobot(0)->getLinkTransform(6);
										mt::Point3 temptrans = temptransf.getTranslation();
										///////////////////////////////////////////////////////////
										float dist=0,dif_dist=0;//dist:variable para almacena la distancia de la muestra al objetivo
										dist=temptrans.distance(goaltrans);//calculo de la distancia de la nueva muestra al objetivo
										
										if(dist<=_distancegoal)
										{
											freepca++;
											_samples->add(tmpSample);//Adicionamos una nueva muestra libre de colisiones
											tmpSample = new Sample(_wkSpace->getDimension());

											//////////////////////////////////////////////////////////////////
											int _index=0;						
											_index=_distance.size()+2;
											_indexpca.push_back(_index);//Se agrega el indice de la muestra generada por Sampling PCA
											//diferencia de distancia desde el objetivo hasta la nueva muestra
											//dif_dist=distanceig-dist;
											_distance.push_back(dist);//Se agrega la distancia de la muestra (en centimetros)
											//PRMPlanner::connectLastSample( goalSamp() );//Conexión con el objetivo

												///////////////////////////////////////////////////////////////////
												//if(dist>=routeinit) 
												//		{
												//			PRMPlanner::connectLastSample( initSamp() );
												//			//cout << "Conexion with Init Sampling " <<endl;
												//		}
												//else if (dist<=routegoal) 
												//		{
												//			PRMPlanner::connectLastSample( goalSamp() );
												//			//cout << "Conexion with Goal Sampling " <<endl;
												//		}
												//else 
												//		{
												//			PRMPlanner::connectLastSample( );
												//			//cout << "Conexion with Neighs " <<endl;
												//		}
												///////////////////////////////////////////////////////////////////
												if (dist<=routegoal) 
														{
															PRMPlanner::connectLastSample( goalSamp() );
															//cout << "Conexion with Goal Sampling " <<endl;
														}
												else 
														{
															PRMPlanner::connectLastSample(initSamp());
															//cout << "Conexion with Neighs " <<endl;
														}
												///////////////////////////////////////////////////////////////////
												int lastComponent = _samples->getSampleAt(_samples->getSize() - 1)->getConnectedComponent();
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

														
														cout << "Number of passes = " << p << " radius = " << radius<< endl;
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
									
										
										/////////////////////////////////////////////////////////////////////
												if(freepca>=_samplingV) {freepca=0; break;}
										////////////////////////////////////////////////////////////////
								
										}
									}
							 }
							}
						//////////////////////////////////////////////////////////
						}
						}
						catch (...){ cout<<"Data not recognize"<<endl;}
						
						if(_triedSamples > _maxNumSamples) break;
						for(int t=0;t<_samplingR;)//t++)
						{		 
							//cout<<"radius = "<<radius<<" n = "<<n<<endl;
							if(getSampleInGoalRegionRealworld(radius , 0.05, true)) 
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
	  cout << "Number of passes = " << p << " radius = " << radius<< endl;
	  cout << " Call PCA = "<<callpca<<endl<<flush;
	  printPCAComponents();
	 


    _solved = false;
    _triedSamples = n;
    _totalTime = (KthReal)(finaltime - entertime)/CLOCKS_PER_SEC ;
    _smoothTime = 0. ;
    return _solved;
	  /////////////////////////////////////////////////////////////////
	  }
	
bool PRMHandPlannerArmHandPCA::getSampleRandPCA(float R)
{
	///////////////////////////////////////////////////////////////////////
	int sizevd=0; //inicialización de la variable

		sizevd=_distance.size();//numero de elementos del vector de distancias
		
		if(sizevd>=11)
		{	
			///////////////////////////////////
			mat PCA11PMDs(sizevd,11);
			
			//PCA11PMDs(sizevd,11);
			PCA11PMDs.fill(0.0);//inicilización de la matriz
			int rowpca=0;//contador de filas para la matriz
			std::vector<KthReal> coordpca(wkSpace()->getDimension());//variable para recuperar las muestras libres hasta el momento

			for(int i=0;i<sizevd;i++)
			{
				//consider only those of the same connected component as the goal
				//if(_samples->getSampleAt(i+2)->getConnectedComponent()!=goalSamp()->getConnectedComponent()) continue;

				if(_distance.at(i)<R)//condición para muestras que se encuentran a una distancia menor a R
				{
					
					
					///////////////////////////////////////////////////
					Sample *samplefree = _samples->getSampleAt(i+2);//Conseguimos las muestras libre a partir del elemento numero 3 :(0)= Ini y (1)=Goal, 
					
					for(int k=0; k <11; k++) coordpca[k]=samplefree->getCoords()[k];//Almacenamos el vector de 11 elementos en coordpca
					
					///////////Fill Matrix/////////////////////
					for(int jk = 0; jk <11; jk++) 
					{
						PCA11PMDs(rowpca,jk)=coordpca[jk];
					}
				
					rowpca++;
					
				}
			}
			
			//int x=0,y=0;
			//x=row;
			//y=PCA_11PMDs.n_rows;
			//PCA_11PMDs.shed_rows(row-1,sizevd-1);
			//y=PCA_11PMDs.n_rows;
		
			int freesamples=PCA11PMDs.n_rows;//numero de muestras libres de 11 elementos
			try{
			if(freesamples>=11)//Condición para el calculo de PCA(minimo 11 elementos)
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
				 rowvec lambdapca = trans(_cloudRadiusMax*sqrt(latent));//4
				/////////Print Lambdas////////////////////////////
				//cout << "Lambdas:" << endl << lambdapca << endl;
				///////////Inicilización de la Matriz para almacenar 20x11 nuevas muestras en el Espacio PCA/////////////////
				//mat::fixed<50,11> matrand;
				//matrand.fill(0.0);
				int numsamplespca=500;//200;
				mat matrand(numsamplespca,11);
				//matrand(numsamplespca,11);
				matrand.fill(0.0);

				///////////////////////////////////////Varibles para los 11 lambdas////////////////////////////////////////
				float lambdaC1,lambdaC2,lambdaC3,lambdaC4,lambdaC5,lambdaC6,lambdaC7,lambdaC8,lambdaC9,lambdaC10,lambdaC11;
				////////Asignación de los 11 lambdas///////////////////////
				lambdaC1=lambdapca(0,0);			  
				lambdaC2=lambdapca(0,1);
				lambdaC3=lambdapca(0,2);
				lambdaC4=lambdapca(0,3);
				lambdaC5=lambdapca(0,4);
				lambdaC6=lambdapca(0,5);			  
				lambdaC7=lambdapca(0,6);
				lambdaC8=lambdapca(0,7);
				lambdaC9=lambdapca(0,8);
				lambdaC10=lambdapca(0,9);
				lambdaC11=lambdapca(0,10);
		
			//////////////Numero de Muestras por cada vez que se llama a Sampling PCA=20//////////////////
			for(int i=0; i < numsamplespca; i++)
			{
				
			  for(int j=0; j<11; j++)
		       { 
				switch(j)
				{
					///////////Sampling Random desde [-lambda,+lambda]: C1...C11.
					
				   //Para la componente J1
				  case 0: matrand(i,j)=-lambdaC1+(2*lambdaC1*(_gen->d_rand())); break;
				  //Para la componente J2
				  case 1: matrand(i,j)=-lambdaC2+(2*lambdaC2*(_gen->d_rand()));break;
				  //Para la componente J3		
				  case 2:  matrand(i,j)=-lambdaC3+(2*lambdaC3*(_gen->d_rand()));break;
				  //Para la componente J4	   
				  case 3: matrand(i,j)=-lambdaC4+(2*lambdaC4*(_gen->d_rand()));break;
				 //Para la componente J5				 
				  case 4: matrand(i,j)=-lambdaC5+(2*lambdaC5*(_gen->d_rand()));break;
				 //Para la componente J6
				  case 5: matrand(i,j)=-lambdaC6+(2*lambdaC6*(_gen->d_rand()));break;
				  //Para la componente Thumb
				  case 6: matrand(i,j)=-lambdaC7+(2*lambdaC7*(_gen->d_rand()));break;
				  //Para la componente C1		
				  case 7:  matrand(i,j)=-lambdaC8+(2*lambdaC8*(_gen->d_rand()));break;
				  //Para la componente C2	   
				  case 8: matrand(i,j)=-lambdaC9+(2*lambdaC9*(_gen->d_rand()));break;
				 //Para la componente C3				 
				  case 9: matrand(i,j)=-lambdaC10+(2*lambdaC10*(_gen->d_rand()));break;
				 //Para la componente C4				 
				  case 10: matrand(i,j)=-lambdaC11+(2*lambdaC11*(_gen->d_rand()));break;
				  default:
					  break;
				}	   
			  }
			}
			//Procesamiento de las 50x11 muestras, para regresar al Espacio del Mundo Real
			for(int i=0; i<matrand.n_rows ; i++)
				 { 
					rowvec pointpcas=matrand.row(i);//Fila de 11 elementos(6 primeros del brazo y los 5 ultimos de la mano)
						
					rowvec zeta=trans((coeff*trans(pointpcas))+trans(bar));//Transformacion del Space PCA al Space Real

					matrand(i,0)=zeta(0,0);//J1
					matrand(i,1)=zeta(0,1);//J2
					matrand(i,2)=zeta(0,2);//J3
					matrand(i,3)=zeta(0,3);//J4
					matrand(i,4)=zeta(0,4);//J5
					matrand(i,5)=zeta(0,5);//J6
					matrand(i,6)=zeta(0,6);//Thumb
					matrand(i,7)=zeta(0,7);//C1
					matrand(i,8)=zeta(0,8);//C2
					matrand(i,9)=zeta(0,9);//C3
					matrand(i,10)=zeta(0,10);//C4
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
		return false;
	///////////////////////////////////////////////////////////////////////
	
}


	////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//!Print PCA components 
    void PRMHandPlannerArmHandPCA::printPCAComponents()
	{
		//std::map<int, SampleSet*>::iterator it;
		cout<<"Num Sampling Goal Region PCA  = "<<_indexpca.size()<<endl;
		cout << "PCA Sampling=> " ;
		for ( int i=0;i<_indexpca.size(); i++ )
		{
			cout<< _indexpca[i]<< "," ;
				
		}	
		cout << endl;

		cout<<"Distance Init to Goal = "<<distanceig<<endl;
	}


	 //!resample around the goal configuration
	//!reimplemented
    bool PRMHandPlannerArmHandPCA::getSampleInGoalRegion(double tradius, double rradius)
	{
	    bool handWholeRange = false;
		return getSampleInGoalRegionRealworld( tradius, rradius, handWholeRange);
	}




	
	bool PRMHandPlannerArmHandPCA::getSampleInGoalRegionRealworld(double tradius, double rradius, bool handWholeRange)
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
		do{
			vector<KthReal> tmpSpos; tmpSpos.resize(3); 
			vector<KthReal> tmpSrot; tmpSrot.resize(3); 
			vector<KthReal> coordrobot; coordrobot.resize(6); 
			tmpSpos	= _goalse3.getPos();
			tmpSrot	= _goalse3.getParams();
						
			//Add random noise to tmpS (only translational part)
			for(int k =0; k < 3; k++)
			{
				coordrobot[k] = tmpSpos[k] + tradius*(2*(KthReal)_gen->d_rand()-1);
			}
			for(int k =0; k < 3; k++)
			{
				coordrobot[k+3] = tmpSrot[k]+ rradius*(2*(KthReal)_gen->d_rand()-1);
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
					dist=temptrans.distance(goaltrans);//goaltrans: distancia objetivo
					//diferencia de distancia desde el objetivo hasta la nueva muestra
					//dif_dist=distanceig-dist;
					//Adicionamos una nueva muestra libre de colisiones
					//if(dist<=_distancegoal)
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


    void PRMHandPlannerArmHandPCA::writeFiles(FILE *fpr, FILE *fph, RobConf* joints)
	{
		//write arm coordinates
		int j;
		for(j =0; j < 6; j++)
			fprintf(fpr,"%.2f ",joints->getRn().getCoordinate(j)*180.0/PI);
		fprintf(fpr,"\n");

		//write hand coordinates
		for(; j < joints->getRn().getDim(); j++)
		{
			if(j==6 || j==11 || j==15 || j==19 || j==23) continue;
			fprintf(fph,"%.2f ",joints->getRn().getCoordinate(j)*180.0/PI);
		}
		fprintf(fph,"\n");
	}



    void PRMHandPlannerArmHandPCA::saveData()
	{

	
    }


  }
};

#endif // KAUTHAM_USE_ARMADILLO
