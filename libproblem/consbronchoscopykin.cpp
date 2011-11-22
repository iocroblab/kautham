#include "consbronchoscopykin.h"
#include "robot.h"
#include <mt/mt.h> 

namespace libProblem {
	
	ConsBronchoscopyKin::ConsBronchoscopyKin(Robot* const rob):ConstrainedKinematic(rob){
		
		_robot = rob;
		_robConf.setRn(_robot->getNumJoints());	
		_currentvalues[0] = 0.; 
		_currentvalues[1] = 0.; 
		_currentvalues[2] = 0.;
		_initialminbending_RAD = (KthReal) (-M_PI/2);//-90degrees
		_initialmaxbending_RAD = (KthReal) (2*M_PI/3);//120 degrees
		_initialminalpha_RAD = (KthReal) (-M_PI/3);//-60degrees;
		_initialmaxalpha_RAD = (KthReal) (M_PI/2);//90degrees

		//set _minbending_RAD, _maxbending_RAD, _minalpha_RAD, _maxalpha_RAD
		setAngleLimits(_initialminbending_RAD, _initialmaxbending_RAD, _initialminalpha_RAD, _initialmaxalpha_RAD);

		_rangealpha_RAD = _maxalpha_RAD - _minalpha_RAD;
		_rangebending_RAD = _maxbending_RAD - _minbending_RAD;

	}
	
	ConsBronchoscopyKin::~ConsBronchoscopyKin()
	{

	}

	bool ConsBronchoscopyKin::solve(){
		_robConf = solve(_target);
		return true;
	}


	RobConf ConsBronchoscopyKin::solve(vector<KthReal> &values)
	{
		//values contain the alpha,beta and zeta parameters
		//values[0] and values[1] in the range -1..1

		//Links characteristic
		Link* _linkn = _robot->getLink(_robot->getNumJoints()-1);
		KthReal l = abs(_linkn->getA()); //one link length
		int n=_robot->getNumJoints()-1; //number of bending joints (do not consider base(fixed) and link0(alpha rotation))

		// Current values
		//KthReal curr_alpha = getvalues(0);
		//KthReal curr_beta = getvalues(1);
		//KthReal curr_z = getvalues(2); // it will not be used cause the slider already gives a Delta
		
		//cout<<"alpha = "<<curr_alpha<<" psi = "<<curr_beta<<endl;

		// Read values
		//KthReal Dalpha = _maxalpha *(values[0]-curr_alpha);

/*
		//computation of beta values to update the limits 
		KthReal currbeta_RAD;
		if(getvalues(1)>0) currbeta_RAD = _maxbending_RAD*getvalues(1);
		else currbeta_RAD = -_minbending_RAD*getvalues(1); //recall minbending is negative
		KthReal newbeta_RAD;
		if(values[1]>0) newbeta_RAD = _maxbending_RAD*values[1];
		else newbeta_RAD = -_minbending_RAD*values[1]; //recall minbending is negative
		//update the limits of beta as a function of the accumulated motions 
		KthReal Delta_beta_RAD=0;
		if(newbeta_RAD>0)
		{
			if(currbeta_RAD<newbeta_RAD)
			{
				if(currbeta_RAD>0) Delta_beta_RAD = newbeta_RAD - currbeta_RAD;
				else Delta_beta_RAD = newbeta_RAD;
			}
		}
		else
		{
			if(currbeta_RAD>newbeta_RAD)
			{
				if(currbeta_RAD<0) Delta_beta_RAD = newbeta_RAD - currbeta_RAD;
				else Delta_beta_RAD = newbeta_RAD;
			}
		}
		setAngleLimits(_minbending_RAD-Delta_beta_RAD,_maxbending_RAD-Delta_beta_RAD,
			 			   _minalpha_RAD, _maxalpha_RAD);

*/


		//KthReal Dalpha;
		KthReal Alpha_RAD;
		if(values[0]>=0) 
		{
			//both values[0] and curr_alpha positive
			/*
			if(curr_alpha>=0) 
			{
				Dalpha = _maxalpha *(values[0]-curr_alpha); 
			}
			else
			{
			    //values[0] positive and curr_alpha negative (Delta is positive)
				Dalpha = curr_alpha*_minalpha + values[0]*_maxalpha;
			}
			*/
			Alpha_RAD = _maxalpha_RAD *values[0]; 
		}
		else 
		{
			//both values[0] and curr_alpha negative
			/*
			if(curr_alpha<0) Dalpha = -_minalpha *(values[0]-curr_alpha);//the signs of delta depends on the diference,
																			//recall that _minalpha is a negative value
			else
			{
			    //curr_alpha  positive and values[0] negative (Delta is negative)
				Dalpha = -curr_alpha*_maxalpha - values[0]*_minalpha;
			}
			*/
			Alpha_RAD = -_minalpha_RAD *values[0]; 
		}

		KthReal psi_RAD;
		if(values[1]>0) psi_RAD = _maxbending_RAD*(values[1]/n); // values[1]=total bending angle read from the slider
		else psi_RAD = _minbending_RAD*(-values[1]/n);
		KthReal Dzeta = (KthReal)-values[2];//directly the value read from the slider //-1000*(values[2]-currentvalues2);
   


		// sin and cos of rotation angles
		//KthReal calpha=cos(Dalpha); 
		//KthReal salpha=sin(Dalpha);
		KthReal calpha=cos(Alpha_RAD); 
		KthReal salpha=sin(Alpha_RAD);
    
		// alpha rotation around z-axis
		 mt::Matrix3x3 RzDalpha=Matrix3x3( calpha,   -salpha,  0,
											salpha,    calpha,  0,
											 0,         0,  1);
		mt::Transform T_RzDalpha;
		T_RzDalpha.setRotation(mt::Rotation(RzDalpha));
		T_RzDalpha.setTranslation(mt::Vector3(0,0,0));
    
		//Current Robot config
		RobConf* _CurrentPos = _robot->getCurrentPos();
		SE3Conf& _SE3Conf = _CurrentPos->getSE3();

		vector<KthReal>& PosSE3 = _SE3Conf.getPos();
		vector<KthReal>& OriSE3 = _SE3Conf.getOrient();

		mt::Transform currTran;
		currTran.setRotation(mt::Rotation(OriSE3[0], OriSE3[1], OriSE3[2], OriSE3[3]));
		currTran.setTranslation(mt::Point3(PosSE3[0], PosSE3[1], PosSE3[2]));
		//mt::Transform Tbase;

		//advance rotating
		if (abs(psi_RAD)>0.0001){
			KthReal d=l/2*cos(psi_RAD/2)/sin(psi_RAD/2);	 //dradio = -length/(2*tan(theta/2));	
			KthReal Dbeta_RAD = Dzeta/d; //(2*zeta*tan(theta/2))/length;
  		
			KthReal cpsi=cos(psi_RAD); 
			KthReal spsi=sin(psi_RAD);
			KthReal cbeta=cos(Dbeta_RAD);  
			KthReal sbeta=sin(Dbeta_RAD);

			mt::Transform T_Y_d_Z_l2;
			T_Y_d_Z_l2.setRotation(mt::Rotation(0.0,0.0,0.0,1));
			T_Y_d_Z_l2.setTranslation(mt::Vector3(0,d,-l/2));


			// Rotation of psi along the x axis, to corrctly locat the Xc center of rotation
			mt::Matrix3x3 RxPsi=Matrix3x3( 1,      0,      0,
											0.0,  cpsi, -spsi,
											0.0,  spsi,  cpsi);
			mt::Transform T_RxPsi;
			T_RxPsi.setRotation(mt::Rotation(RxPsi));
			T_RxPsi.setTranslation(mt::Vector3(0,0,0));

			// Rotation of beta along the x axis -> the bronchoscope go frwd(rotate) in local z-y plane
			mt::Matrix3x3 RxDbeta=Matrix3x3( 1,      0,      0,
											0.0,  cbeta, -sbeta,
											0.0,  sbeta,  cbeta);
                                      
			mt::Transform T_RxDbeta;
			T_RxDbeta.setRotation(mt::Rotation(RxDbeta));
			T_RxDbeta.setTranslation(mt::Vector3(0,0,0));

			// updating the base transform
			//mt::Transform 
			//Tbase=currTran*T_RzDalpha*T_Y_d_Z_l2*T_RxDbeta*T_Y_d_Z_l2.inverse(); // beta around x axis
			
			//the last inverse is becaus finally the alpha rotation is assigend to the first link, not to
			//the roatation of the base
			Tbase=currTran*T_RzDalpha*T_RxPsi*T_Y_d_Z_l2*T_RxDbeta*T_Y_d_Z_l2.inverse()*T_RxPsi.inverse()*T_RzDalpha.inverse(); // beta around x axis
		}
		//else rectilinear advance
		else {
			mt::Transform T_X0_Y0_Zvel;
			T_X0_Y0_Zvel.setRotation(mt::Rotation(0.0,0.0,0.0,1));
			//JAN : changed Dzeta to -Dzeta in next line 2011-03-26
			T_X0_Y0_Zvel.setTranslation(mt::Vector3(0,0,-Dzeta));
      
			//mt::Transform 
			Tbase=currTran*T_RzDalpha*T_X0_Y0_Zvel*T_RzDalpha.inverse(); // beta around x axis
		}

		//update joint values
		RnConf& _RnConf = _CurrentPos->getRn();
		vector<KthReal>& rncoor= _RnConf.getCoordinates();


		//COMPTE rncoord hauria d'estar entre 0 i 1 i values sembla que va de -1 a 1.
		//o rncoord hauria d'estar en el seu rang en radiants i values va de -1 a 1...

		//if(values[0]>0) rncoor[0] = values[0]*_maxalpha; //alpha
		//else rncoor[0] = -values[0]*_minalpha;
		 
		//rncoor[0] += Dalpha;
		rncoor[0] = Alpha_RAD;

		//OK//rncoor[0] = values[0]; //alpha
		//rncoor[0] = (values[0]+1)/2; //alpha
		for(int i =1; i< rncoor.size(); i++)
		{
			rncoor[i] = psi_RAD;
			//if(values[1]>0) rncoor[i] = _maxbending*values[1]/n;
			//else rncoor[i] = -(_minbending*values[1]/n);
			//OK//rncoor[i] = values[1]/n;//i.e. psi/_maxbending;
			//rncoor[i] = ((values[1]+1)/2) /n;//i.e. psi/_maxbending;
		}


    // updating the _currentvalues variable with the new read values
		setvalues(values[0],0);
		setvalues(values[1],1);
		setvalues(values[2],2);		

		const mt::Point3& basePos = Tbase.getTranslationRef(); //resTra=result.getTranslationRef();

		vector<KthReal> coords(7);
		for(int i =0; i < 3; i++)
			coords[i] = basePos[i];

		const mt::Rotation& baseRot = Tbase.getRotationRef(); //resRot=result.getRotationRef();

		for(int i =0; i < 4; i++)
			coords[i+3] = baseRot[i];

		_robConf.setSE3(coords);
		_robConf.setRn(_RnConf);
		_robot->Kinematics(_robConf);


		return _robConf;
	}


	//computes the normalized values alpha and beta corresponding to the current robot configuration
	void ConsBronchoscopyKin::setAngleLimits(KthReal mb, KthReal Mb, KthReal ma, KthReal Ma)
	{
		if(mb>0) mb=0;//min value is negative allways
		else if(mb<_initialminbending_RAD) mb = _initialminbending_RAD;
		if(Mb<0) Mb=0;//max value is positive allways
		else if(Mb>_maxbending_RAD) Mb = _initialmaxbending_RAD;
		
		if(ma>0) ma=0;//min value is negative allways
		else if(ma<_initialminalpha_RAD) ma = _initialminalpha_RAD;
		if(Ma<0) Ma=0;//max value is positive allways
		else if(Ma>_initialmaxalpha_RAD) Ma = _initialmaxalpha_RAD;

		_minbending_RAD = mb;
		_maxbending_RAD = Mb;
		_minalpha_RAD = ma;
		_maxalpha_RAD = Ma;

		_rangealpha_RAD = _maxalpha_RAD - _minalpha_RAD;
		_rangebending_RAD = _maxbending_RAD - _minbending_RAD;

		cout<<"mb = "<<_minbending_RAD<<" Mb = "<<_maxbending_RAD<<" ma = "<<_minalpha_RAD<<" Ma = "<<_maxalpha_RAD<<endl;
	}

	//computes the normalized values alpha and beta corresponding to the current robot configuration
	void ConsBronchoscopyKin::registerValues()
	{
		//read configuration
		RobConf* _CurrentPos = _robot->getCurrentPos();
		RnConf& _RnConf = _CurrentPos->getRn();
		vector<KthReal>& rncoor= _RnConf.getCoordinates();
		//store noramalized anpha and beta values (in range -1..1) in  variable values
		
		KthReal a_11;
		if(rncoor[0]>0) a_11=rncoor[0]/_maxalpha_RAD;
		else a_11=-rncoor[0]/_minalpha_RAD;
		setvalues(a_11,0);//alpha

		KthReal b_11;
		if(rncoor[1]>0) b_11=rncoor[1]*(rncoor.size()-1)/_maxbending_RAD;
		else b_11=-rncoor[1]*(rncoor.size()-1)/_minbending_RAD;
		setvalues(b_11,1);//beta
	}

	bool ConsBronchoscopyKin::setParameters(){
		return true;
	}

//apply inverseJacobian
	bool ConsBronchoscopyKin::ApplyInverseJ(int k, KthReal alpha, KthReal xi, KthReal vx, KthReal vy, 
		KthReal vz, KthReal *Dalpha, KthReal *Dxi, KthReal *Dz)
	{
		KthReal invJ0[3];
		KthReal invJ1[3];
		KthReal invJ2[3]; 
		
		if(iJ(k,0,0,alpha,xi,&invJ0[0]) == false) return false;
		if(iJ(k,0,1,alpha,xi, &invJ0[1])== false) return false;
		if(iJ(k,0,2,alpha,xi, &invJ0[2])== false) return false;
		if(iJ(k,1,0,alpha,xi, &invJ1[0])== false) return false;
		if(iJ(k,1,1,alpha,xi, &invJ1[1])== false) return false;
		if(iJ(k,1,2,alpha,xi, &invJ1[2])== false) return false;
		if(iJ(k,2,0,alpha,xi, &invJ2[0])== false) return false;
		if(iJ(k,2,1,alpha,xi, &invJ2[1])== false) return false;
		if(iJ(k,2,2,alpha,xi, &invJ2[2])== false) return false;

		*Dalpha = invJ0[0]*vx + invJ0[1]*vy + invJ0[2]*vz;
		*Dxi    = invJ1[0]*vx + invJ1[1]*vy + invJ1[2]*vz;
		*Dz     = invJ2[0]*vx + invJ2[1]*vy + invJ2[2]*vz;

		return true;
	}
//inverseJacobian
	bool ConsBronchoscopyKin::iJ(int k, int row, int column, KthReal alpha, KthReal xi, KthReal *coef)
	{
	//k is the index for the link
	//this data od the Jacobian is valid from k=2 upwards
		if(k<2 || k>=_robot->getNumJoints()-1) return false;

	KthReal num,den;
		if(row==0 && column==0)
		{
			den = 15+30*cos(xi)+50*cos(xi*k-xi)-48*cos(xi*k)-12*cos(4*xi)-34*cos(-4*xi+xi*k)+78*cos(xi*k-5*xi)+12*cos(2*xi*k-xi)-51*cos(2*xi*k-6*xi)+40*cos(2*xi*k-5*xi)+12*cos(2*xi*k-9*xi)+19*cos(2*xi*k-8*xi)+75*cos(2*xi*k-4*xi)-30*cos(2*xi*k-3*xi)-65*cos(2*xi*k-2*xi)-30*cos(2*xi*k-7*xi)+30*cos(5*xi)+33*cos(2*xi*k)-3*cos(2*xi*k-10*xi)+30*cos(-6*xi+xi*k)-15*cos(xi*k-8*xi)-34*cos(-7*xi+xi*k)+6*cos(xi*k-9*xi)-30*cos(xi*k+4*xi)-cos(8*xi+xi*k)+3*cos(-10*xi+xi*k)+12*cos(6*xi)-9*cos(2*xi+2*xi*k)+9*cos(6*xi+xi*k)-2*cos(xi+2*xi*k)-2*cos(-11*xi+2*xi*k)+cos(4*xi+2*xi*k)+2*cos(5*xi+xi*k)-6*cos(7*xi)-3*cos(8*xi)-12*cos(2*xi)+36*cos(xi*k-2*xi)+50*cos(xi*k+2*xi)-6*cos(xi*k+xi)-54*cos(3*xi)-90*cos(xi*k-3*xi)-6*cos(xi*k+3*xi);

			if(den==0) return false;

			num = -42*sin(3*xi+alpha)+42*sin(-3*xi+alpha)+14*sin(alpha-4*xi)-14*sin(alpha+4*xi)-77*sin(alpha-xi-xi*k)-15*sin(alpha+7*xi-xi*k)+35*sin(alpha-xi*k-3*xi)-7*sin(alpha+6*xi-xi*k)-6*sin(-6*xi+alpha)+21*sin(alpha+2*xi+xi*k)-7*sin(alpha+4*xi+xi*k)+sin(alpha+6*xi+xi*k)-35*sin(alpha+3*xi+xi*k)+77*sin(alpha+xi+xi*k)+9*sin(alpha+5*xi+xi*k)-sin(alpha+7*xi+xi*k)+7*sin(alpha-4*xi-xi*k)-sin(alpha-6*xi-xi*k)+sin(alpha-xi*k+8*xi)-21*sin(alpha-2*xi-xi*k)-9*sin(alpha-5*xi-xi*k)+sin(alpha-7*xi-xi*k)+6*sin(6*xi+alpha)+15*sin(alpha-7*xi+xi*k)+14*sin(alpha+5*xi)-14*sin(alpha-5*xi)+2*sin(alpha-7*xi)+sin(-8*xi+alpha)-sin(8*xi+alpha)-2*sin(alpha+7*xi)+7*sin(alpha-6*xi+xi*k)-sin(alpha+xi*k-8*xi)-2*sin(-9*xi+alpha+xi*k)+2*sin(9*xi+alpha-xi*k)+70*sin(xi+alpha)-70*sin(-xi+alpha)+35*sin(alpha+xi*k-2*xi)-35*sin(alpha-xi*k+2*xi)+14*sin(alpha+2*xi)-14*sin(alpha-2*xi)+91*sin(alpha+xi*k-3*xi)-91*sin(alpha-xi*k+3*xi)+21*sin(4*xi+alpha-xi*k)+35*sin(alpha-xi*k)-35*sin(alpha+xi*k)-21*sin(-4*xi+alpha+xi*k)+49*sin(alpha-xi*k+5*xi)+105*sin(xi+alpha-xi*k)-105*sin(-xi+alpha+xi*k)-49*sin(alpha+xi*k-5*xi);

			*coef = num/den;
			return true;
		}

		else if(row==0 && column==1)
		{
			den = 15+30*cos(xi)+50*cos(xi*k-xi)-48*cos(xi*k)-12*cos(4*xi)-34*cos(-4*xi+xi*k)+78*cos(xi*k-5*xi)+12*cos(2*xi*k-xi)-51*cos(2*xi*k-6*xi)+40*cos(2*xi*k-5*xi)+12*cos(2*xi*k-9*xi)+19*cos(2*xi*k-8*xi)+75*cos(2*xi*k-4*xi)-30*cos(2*xi*k-3*xi)-65*cos(2*xi*k-2*xi)-30*cos(2*xi*k-7*xi)+30*cos(5*xi)+33*cos(2*xi*k)-3*cos(2*xi*k-10*xi)+30*cos(-6*xi+xi*k)-15*cos(xi*k-8*xi)-34*cos(-7*xi+xi*k)+6*cos(xi*k-9*xi)-30*cos(xi*k+4*xi)-cos(8*xi+xi*k)+3*cos(-10*xi+xi*k)+12*cos(6*xi)-9*cos(2*xi+2*xi*k)+9*cos(6*xi+xi*k)-2*cos(xi+2*xi*k)-2*cos(-11*xi+2*xi*k)+cos(4*xi+2*xi*k)+2*cos(5*xi+xi*k)-6*cos(7*xi)-3*cos(8*xi)-12*cos(2*xi)+36*cos(xi*k-2*xi)+50*cos(xi*k+2*xi)-6*cos(xi*k+xi)-54*cos(3*xi)-90*cos(xi*k-3*xi)-6*cos(xi*k+3*xi);

			if(den==0) return false;

			num = -49*cos(alpha-xi*k+5*xi)+35*cos(alpha+xi*k)+49*cos(alpha+xi*k-5*xi)-35*cos(alpha-xi*k)+77*cos(alpha-xi-xi*k)-15*cos(alpha-7*xi+xi*k)+21*cos(alpha-2*xi-xi*k)-7*cos(alpha-6*xi+xi*k)+15*cos(alpha+7*xi-xi*k)-77*cos(alpha+xi+xi*k)+7*cos(alpha+6*xi-xi*k)-21*cos(alpha+2*xi+xi*k)+14*cos(alpha+4*xi)-14*cos(alpha-4*xi)+cos(alpha+xi*k-8*xi)+35*cos(alpha+3*xi+xi*k)-9*cos(alpha+5*xi+xi*k)+cos(alpha+7*xi+xi*k)-7*cos(alpha-4*xi-xi*k)+cos(alpha-6*xi-xi*k)-35*cos(alpha-xi*k-3*xi)+9*cos(alpha-5*xi-xi*k)-cos(alpha-7*xi-xi*k)+7*cos(alpha+4*xi+xi*k)-cos(alpha+6*xi+xi*k)+14*cos(alpha-5*xi)-14*cos(alpha+5*xi)-6*cos(6*xi+alpha)+6*cos(-6*xi+alpha)+2*cos(alpha+7*xi)-2*cos(alpha-7*xi)-cos(alpha-xi*k+8*xi)+cos(8*xi+alpha)-cos(-8*xi+alpha)-2*cos(9*xi+alpha-xi*k)+2*cos(-9*xi+alpha+xi*k)+70*cos(-xi+alpha)-70*cos(xi+alpha)+35*cos(alpha-xi*k+2*xi)-35*cos(alpha+xi*k-2*xi)-91*cos(alpha+xi*k-3*xi)+14*cos(alpha-2*xi)-14*cos(alpha+2*xi)+91*cos(alpha-xi*k+3*xi)-42*cos(-3*xi+alpha)+42*cos(3*xi+alpha)-105*cos(xi+alpha-xi*k)+105*cos(-xi+alpha+xi*k)-21*cos(4*xi+alpha-xi*k)+21*cos(-4*xi+alpha+xi*k);

			*coef = num/den;
			return true;
		}
		else if(row==0 && column==2)
		{
			*coef = 0.0;
			return true;
		}
		else if(row==1 && column==0)
		{
			den = 12+24*cos(xi)+24*cos(xi*k-xi)-26*cos(xi*k)-12*cos(4*xi)-18*cos(-4*xi+xi*k)+44*cos(xi*k-5*xi)+4*cos(2*xi*k-xi)-26*cos(2*xi*k-6*xi)+24*cos(2*xi*k-5*xi)+4*cos(2*xi*k-9*xi)+6*cos(2*xi*k-8*xi)+44*cos(2*xi*k-4*xi)-16*cos(2*xi*k-3*xi)-36*cos(2*xi*k-2*xi)-16*cos(2*xi*k-7*xi)+12*cos(5*xi)+14*cos(2*xi*k)+18*cos(-6*xi+xi*k)-6*cos(xi*k-8*xi)-12*cos(-7*xi+xi*k)-14*cos(xi*k+4*xi)+6*cos(6*xi)-2*cos(2*xi+2*xi*k)+2*cos(6*xi+xi*k)-6*cos(2*xi)+14*cos(xi*k-2*xi)+30*cos(xi*k+2*xi)+4*cos(xi*k+xi)-36*cos(3*xi)-56*cos(xi*k-3*xi)-4*cos(xi*k+3*xi);

			if(den==0) return false;

			num = -40*sin(alpha)-12*sin(alpha-4*xi)-12*sin(alpha+4*xi)-6*sin(9*xi+alpha-2*xi*k)+sin(-xi+alpha-2*xi*k)-sin(-10*xi+alpha+xi*k)-15*sin(alpha-xi-xi*k)-sin(alpha+7*xi-xi*k)+6*sin(alpha-xi*k-3*xi)-15*sin(alpha+6*xi-xi*k)+2*sin(-6*xi+alpha)-sin(alpha+2*xi+xi*k)+6*sin(alpha+3*xi+xi*k)-15*sin(alpha+xi+xi*k)-sin(alpha+5*xi+xi*k)+6*sin(alpha-xi*k+8*xi)-sin(alpha-2*xi-xi*k)-sin(alpha-5*xi-xi*k)+2*sin(6*xi+alpha)-sin(alpha-7*xi+xi*k)-15*sin(alpha-6*xi+xi*k)+6*sin(alpha+xi*k-8*xi)-sin(10*xi+alpha-xi*k)+sin(xi+alpha+2*xi*k)-6*sin(-9*xi+alpha+2*xi*k)-6*sin(xi+alpha-2*xi*k)+15*sin(alpha-2*xi*k+3*xi)+15*sin(7*xi+alpha-2*xi*k)+15*sin(-7*xi+alpha+2*xi*k)+15*sin(alpha+2*xi*k-3*xi)-20*sin(alpha+2*xi*k-5*xi)-20*sin(alpha-2*xi*k+5*xi)-6*sin(-xi+alpha+2*xi*k)-15*sin(alpha+xi*k-2*xi)-15*sin(alpha-xi*k+2*xi)+30*sin(alpha+2*xi)+30*sin(alpha-2*xi)-15*sin(alpha+xi*k-3*xi)-15*sin(alpha-xi*k+3*xi)+sin(-11*xi+alpha+2*xi*k)+sin(11*xi+alpha-2*xi*k)+20*sin(4*xi+alpha-xi*k)+6*sin(alpha-xi*k)+6*sin(alpha+xi*k)+20*sin(-4*xi+alpha+xi*k)+6*sin(alpha-xi*k+5*xi)+20*sin(xi+alpha-xi*k)+20*sin(-xi+alpha+xi*k)+6*sin(alpha+xi*k-5*xi);

			*coef = num/den;
			return true;
		}
		else if(row==1 && column==1)
		{
			den = 12+24*cos(xi)+24*cos(xi*k-xi)-26*cos(xi*k)-12*cos(4*xi)-18*cos(-4*xi+xi*k)+44*cos(xi*k-5*xi)+4*cos(2*xi*k-xi)-26*cos(2*xi*k-6*xi)+24*cos(2*xi*k-5*xi)+4*cos(2*xi*k-9*xi)+6*cos(2*xi*k-8*xi)+44*cos(2*xi*k-4*xi)-16*cos(2*xi*k-3*xi)-36*cos(2*xi*k-2*xi)-16*cos(2*xi*k-7*xi)+12*cos(5*xi)+14*cos(2*xi*k)+18*cos(-6*xi+xi*k)-6*cos(xi*k-8*xi)-12*cos(-7*xi+xi*k)-14*cos(xi*k+4*xi)+6*cos(6*xi)-2*cos(2*xi+2*xi*k)+2*cos(6*xi+xi*k)-6*cos(2*xi)+14*cos(xi*k-2*xi)+30*cos(xi*k+2*xi)+4*cos(xi*k+xi)-36*cos(3*xi)-56*cos(xi*k-3*xi)-4*cos(xi*k+3*xi);

			if(den==0) return false;

			num = 40*cos(alpha)+20*cos(alpha-2*xi*k+5*xi)+20*cos(alpha+2*xi*k-5*xi)-15*cos(7*xi+alpha-2*xi*k)-15*cos(alpha-2*xi*k+3*xi)-15*cos(alpha+2*xi*k-3*xi)-15*cos(-7*xi+alpha+2*xi*k)-6*cos(alpha-xi*k+5*xi)-6*cos(alpha+xi*k)-6*cos(alpha+xi*k-5*xi)-6*cos(alpha-xi*k)+15*cos(alpha-xi-xi*k)+cos(alpha-7*xi+xi*k)+cos(alpha-2*xi-xi*k)+15*cos(alpha-6*xi+xi*k)+cos(alpha+7*xi-xi*k)+15*cos(alpha+xi+xi*k)+15*cos(alpha+6*xi-xi*k)+cos(alpha+2*xi+xi*k)+6*cos(xi+alpha-2*xi*k)-cos(-xi+alpha-2*xi*k)+6*cos(9*xi+alpha-2*xi*k)-cos(-11*xi+alpha+2*xi*k)-cos(11*xi+alpha-2*xi*k)+12*cos(alpha+4*xi)+12*cos(alpha-4*xi)+cos(10*xi+alpha-xi*k)+cos(-10*xi+alpha+xi*k)+6*cos(-xi+alpha+2*xi*k)+6*cos(-9*xi+alpha+2*xi*k)-cos(xi+alpha+2*xi*k)-6*cos(alpha+xi*k-8*xi)-6*cos(alpha+3*xi+xi*k)+cos(alpha+5*xi+xi*k)-6*cos(alpha-xi*k-3*xi)+cos(alpha-5*xi-xi*k)-2*cos(6*xi+alpha)-2*cos(-6*xi+alpha)-6*cos(alpha-xi*k+8*xi)+15*cos(alpha-xi*k+2*xi)+15*cos(alpha+xi*k-2*xi)+15*cos(alpha+xi*k-3*xi)-30*cos(alpha-2*xi)-30*cos(alpha+2*xi)+15*cos(alpha-xi*k+3*xi)-20*cos(xi+alpha-xi*k)-20*cos(-xi+alpha+xi*k)-20*cos(4*xi+alpha-xi*k)-20*cos(-4*xi+alpha+xi*k);

			*coef = num/den;
			return true;
		}
		else if(row==1 && column==2)
		{
			den = 6+12*cos(xi)+12*cos(xi*k-xi)-13*cos(xi*k)-6*cos(4*xi)-9*cos(-4*xi+xi*k)+22*cos(xi*k-5*xi)+2*cos(2*xi*k-xi)-13*cos(2*xi*k-6*xi)+12*cos(2*xi*k-5*xi)+2*cos(2*xi*k-9*xi)+3*cos(2*xi*k-8*xi)+22*cos(2*xi*k-4*xi)-8*cos(2*xi*k-3*xi)-18*cos(2*xi*k-2*xi)-8*cos(2*xi*k-7*xi)+6*cos(5*xi)+7*cos(2*xi*k)+9*cos(-6*xi+xi*k)-3*cos(xi*k-8*xi)-6*cos(-7*xi+xi*k)-7*cos(xi*k+4*xi)+3*cos(6*xi)-cos(2*xi+2*xi*k)+cos(6*xi+xi*k)-3*cos(2*xi)+7*cos(xi*k-2*xi)+15*cos(xi*k+2*xi)+2*cos(xi*k+xi)-18*cos(3*xi)-28*cos(xi*k-3*xi)-2*cos(xi*k+3*xi);

			if(den==0) return false;

			num = 20*sin(xi*k-xi)+6*sin(xi*k)+20*sin(-4*xi+xi*k)-15*sin(-6*xi+xi*k)+6*sin(xi*k-5*xi)-sin(-7*xi+xi*k)+15*sin(2*xi*k-3*xi)+15*sin(2*xi*k-7*xi)-20*sin(2*xi*k-5*xi)+6*sin(xi*k-8*xi)-sin(-10*xi+xi*k)-6*sin(2*xi*k-9*xi)-6*sin(2*xi*k-xi)+sin(-11*xi+2*xi*k)+sin(xi+2*xi*k)-sin(5*xi+xi*k)-sin(xi*k+2*xi)-15*sin(xi*k-2*xi)-15*sin(xi*k+xi)+6*sin(xi*k+3*xi)-15*sin(xi*k-3*xi);

			*coef = num/den;
			return true;
		}
		else if(row==2 && column==0)
		{
			den = 60+120*cos(xi)+200*cos(xi*k-xi)-192*cos(xi*k)-48*cos(4*xi)-136*cos(-4*xi+xi*k)+312*cos(xi*k-5*xi)+48*cos(2*xi*k-xi)-204*cos(2*xi*k-6*xi)+160*cos(2*xi*k-5*xi)+48*cos(2*xi*k-9*xi)+76*cos(2*xi*k-8*xi)+300*cos(2*xi*k-4*xi)-120*cos(2*xi*k-3*xi)-260*cos(2*xi*k-2*xi)-120*cos(2*xi*k-7*xi)+120*cos(5*xi)+132*cos(2*xi*k)-12*cos(2*xi*k-10*xi)+120*cos(-6*xi+xi*k)-60*cos(xi*k-8*xi)-136*cos(-7*xi+xi*k)+24*cos(xi*k-9*xi)-120*cos(xi*k+4*xi)-4*cos(8*xi+xi*k)+12*cos(-10*xi+xi*k)+48*cos(6*xi)-36*cos(2*xi+2*xi*k)+36*cos(6*xi+xi*k)-8*cos(xi+2*xi*k)-8*cos(-11*xi+2*xi*k)+4*cos(4*xi+2*xi*k)+8*cos(5*xi+xi*k)-24*cos(7*xi)-12*cos(8*xi)-48*cos(2*xi)+144*cos(xi*k-2*xi)+200*cos(xi*k+2*xi)-24*cos(xi*k+xi)-216*cos(3*xi)-360*cos(xi*k-3*xi)-24*cos(xi*k+3*xi);

			if(den==0) return false;

			num = 14*k*cos(-9*xi+alpha+2*xi*k)+6*k*cos(xi+alpha+2*xi*k)-28*k*cos(alpha-5*xi)+28*k*cos(alpha+5*xi)+14*k*cos(alpha+2*xi*k)+28*k*cos(alpha-4*xi)-2*k*cos(2*xi+alpha+2*xi*k)-12*k*cos(-6*xi+alpha)-k*cos(3*xi+alpha+2*xi*k)+4*k*cos(alpha-7*xi)-6*k*cos(-11*xi+alpha+2*xi*k)-4*k*cos(alpha+7*xi)+70*k*cos(alpha-2*xi*k+6*xi)-42*k*cos(alpha-2*xi*k+8*xi)-70*k*cos(alpha-2*xi*k+4*xi)+14*k*cos(10*xi+alpha-2*xi*k)+2*k*cos(-8*xi+alpha)+42*k*cos(alpha-2*xi*k+2*xi)-2*k*cos(12*xi+alpha-2*xi*k)-14*k*cos(alpha-2*xi*k+3*xi)+14*k*cos(xi+alpha-2*xi*k)+14*k*cos(7*xi+alpha-2*xi*k)-6*k*cos(-xi+alpha-2*xi*k)-14*k*cos(9*xi+alpha-2*xi*k)-14*k*cos(alpha-2*xi*k)+2*k*cos(-2*xi+alpha-2*xi*k)+k*cos(-3*xi+alpha-2*xi*k)+6*k*cos(11*xi+alpha-2*xi*k)+2*k*cos(-12*xi+alpha+2*xi*k)-42*k*cos(alpha+2*xi*k-2*xi)-28*k*cos(alpha-2*xi)-2*k*cos(8*xi+alpha)+45*cos(alpha-2*xi*k+5*xi)+185*cos(alpha-2*xi*k+4*xi)-45*cos(alpha+2*xi*k-5*xi)-185*cos(alpha+2*xi*k-4*xi)-165*cos(alpha-2*xi*k+6*xi)+165*cos(alpha+2*xi*k-6*xi)-67*cos(7*xi+alpha-2*xi*k)+3*cos(alpha-2*xi*k+3*xi)-123*cos(alpha-2*xi*k+2*xi)-3*cos(alpha+2*xi*k-3*xi)+123*cos(alpha+2*xi*k-2*xi)+67*cos(-7*xi+alpha+2*xi*k)+87*cos(alpha-2*xi*k+8*xi)-87*cos(alpha+2*xi*k-8*xi)+204*cos(alpha-xi*k+5*xi)+244*cos(alpha+xi*k)-204*cos(alpha+xi*k-5*xi)-244*cos(alpha-xi*k)-132*cos(alpha-xi-xi*k)+108*cos(alpha-7*xi+xi*k)+144*cos(alpha-2*xi-xi*k)-176*cos(alpha-6*xi+xi*k)-108*cos(alpha+7*xi-xi*k)+132*cos(alpha+xi+xi*k)+176*cos(alpha+6*xi-xi*k)-144*cos(alpha+2*xi+xi*k)-25*cos(xi+alpha-2*xi*k)+15*cos(-xi+alpha-2*xi*k)+45*cos(9*xi+alpha-2*xi*k)+15*cos(-11*xi+alpha+2*xi*k)-15*cos(11*xi+alpha-2*xi*k)+4*cos(-11*xi+alpha+xi*k)+22*cos(alpha+4*xi)-56*cos(alpha+xi*k-3*xi)*k-22*cos(alpha-4*xi)+24*cos(10*xi+alpha-xi*k)+56*k*cos(alpha+xi*k-5*xi)+42*k*cos(-xi+alpha+xi*k)-42*k*cos(xi+alpha-xi*k)-56*k*cos(alpha-xi*k+5*xi)-k*cos(alpha-7*xi-xi*k)-k*cos(-12*xi+alpha+xi*k)-3*cos(-3*xi+alpha-2*xi*k)+cos(11*xi+alpha)+k*cos(alpha+7*xi+xi*k)-9*cos(9*xi+alpha)-24*cos(-10*xi+alpha+xi*k)+25*cos(-10*xi+alpha+2*xi*k)-45*cos(alpha+2*xi*k)-14*k*cos(-7*xi+alpha+2*xi*k)-14*k*cos(-xi+alpha+2*xi*k)+84*k*cos(-3*xi+alpha)-84*k*cos(3*xi+alpha)+25*cos(-xi+alpha+2*xi*k)-45*cos(-9*xi+alpha+2*xi*k)-15*cos(xi+alpha+2*xi*k)+45*cos(alpha-2*xi*k)-25*cos(10*xi+alpha-2*xi*k)-14*k*cos(-10*xi+alpha+2*xi*k)+70*k*cos(alpha+2*xi*k-4*xi)+12*k*cos(6*xi+alpha)-16*k*cos(alpha+xi*k-8*xi)+36*k*cos(alpha+2*xi+xi*k)+13*k*cos(alpha-4*xi-xi*k)-28*k*cos(alpha+6*xi-xi*k)-36*k*cos(alpha-7*xi+xi*k)+13*k*cos(-9*xi+alpha+xi*k)+36*k*cos(alpha+7*xi-xi*k)-56*cos(alpha-xi*k+2*xi)*k+7*cos(2*xi+alpha+2*xi*k)+3*cos(-12*xi+alpha+xi*k)-3*cos(-12*xi+alpha+2*xi*k)+k*cos(-13*xi+alpha+2*xi*k)-k*cos(13*xi+alpha-2*xi*k)+56*cos(alpha-xi*k+3*xi)*k+9*cos(-9*xi+alpha)-cos(-11*xi+alpha)+84*cos(alpha+xi*k-8*xi)-68*cos(alpha+3*xi+xi*k)+24*cos(alpha+5*xi+xi*k)-4*cos(alpha+7*xi+xi*k)-51*cos(alpha-4*xi-xi*k)+8*cos(alpha-6*xi-xi*k)+68*cos(alpha-xi*k-3*xi)-24*cos(alpha-5*xi-xi*k)+4*cos(alpha-7*xi-xi*k)+51*cos(alpha+4*xi+xi*k)-8*cos(alpha+6*xi+xi*k)+163*cos(alpha-5*xi)-163*cos(alpha+5*xi)-3*cos(6*xi+alpha)+3*cos(-6*xi+alpha)+47*cos(alpha+7*xi)-47*cos(alpha-7*xi)-84*cos(alpha-xi*k+8*xi)-3*cos(8*xi+alpha)+3*cos(-8*xi+alpha)+32*cos(9*xi+alpha-xi*k)-32*cos(-9*xi+alpha+xi*k)+cos(10*xi+alpha)+14*k*cos(alpha+2*xi*k-3*xi)-140*k*cos(-xi+alpha)+140*k*cos(xi+alpha)-2*k*cos(-11*xi+alpha+xi*k)+28*k*cos(alpha-xi-xi*k)-13*k*cos(9*xi+alpha-xi*k)+28*k*cos(alpha-6*xi+xi*k)-36*k*cos(alpha-2*xi-xi*k)-28*k*cos(alpha+xi+xi*k)+16*k*cos(alpha+3*xi+xi*k)-6*k*cos(alpha+5*xi+xi*k)-16*k*cos(alpha-xi*k-3*xi)-13*k*cos(alpha+4*xi+xi*k)+2*k*cos(alpha+6*xi+xi*k)+16*k*cos(alpha-xi*k+8*xi)-6*k*cos(10*xi+alpha-xi*k)-7*cos(-2*xi+alpha-2*xi*k)+2*cos(13*xi+alpha-2*xi*k)-4*cos(11*xi+alpha-xi*k)+56*k*cos(alpha-xi*k)+42*k*cos(4*xi+alpha-xi*k)+560*cos(-xi+alpha)-560*cos(xi+alpha)+288*cos(alpha-xi*k+2*xi)-288*cos(alpha+xi*k-2*xi)+56*cos(alpha+xi*k-2*xi)*k-42*k*cos(-4*xi+alpha+xi*k)-56*k*cos(alpha+xi*k)+2*k*cos(11*xi+alpha-xi*k)+6*k*cos(alpha-5*xi-xi*k)-2*k*cos(alpha-6*xi-xi*k)+6*k*cos(-10*xi+alpha+xi*k)+k*cos(12*xi+alpha-xi*k)-2*cos(-13*xi+alpha+2*xi*k)-3*cos(12*xi+alpha-xi*k)+3*cos(12*xi+alpha-2*xi*k)+42*k*cos(alpha+2*xi*k-8*xi)-70*k*cos(alpha+2*xi*k-6*xi)+28*k*cos(alpha+2*xi)-28*k*cos(alpha+4*xi)-cos(-10*xi+alpha)+244*cos(alpha+xi*k-3*xi)+28*cos(alpha-2*xi)-28*cos(alpha+2*xi)-244*cos(alpha-xi*k+3*xi)+3*cos(3*xi+alpha+2*xi*k)-372*cos(-3*xi+alpha)+372*cos(3*xi+alpha)+204*cos(xi+alpha-xi*k)-204*cos(-xi+alpha+xi*k)-258*cos(4*xi+alpha-xi*k)+258*cos(-4*xi+alpha+xi*k);

			*coef = num/den;
			return true;
		}
		else if(row==2 && column==1)
		{
			den = 60+120*cos(xi)+200*cos(xi*k-xi)-192*cos(xi*k)-48*cos(4*xi)-136*cos(-4*xi+xi*k)+312*cos(xi*k-5*xi)+48*cos(2*xi*k-xi)-204*cos(2*xi*k-6*xi)+160*cos(2*xi*k-5*xi)+48*cos(2*xi*k-9*xi)+76*cos(2*xi*k-8*xi)+300*cos(2*xi*k-4*xi)-120*cos(2*xi*k-3*xi)-260*cos(2*xi*k-2*xi)-120*cos(2*xi*k-7*xi)+120*cos(5*xi)+132*cos(2*xi*k)-12*cos(2*xi*k-10*xi)+120*cos(-6*xi+xi*k)-60*cos(xi*k-8*xi)-136*cos(-7*xi+xi*k)+24*cos(xi*k-9*xi)-120*cos(xi*k+4*xi)-4*cos(8*xi+xi*k)+12*cos(-10*xi+xi*k)+48*cos(6*xi)-36*cos(2*xi+2*xi*k)+36*cos(6*xi+xi*k)-8*cos(xi+2*xi*k)-8*cos(-11*xi+2*xi*k)+4*cos(4*xi+2*xi*k)+8*cos(5*xi+xi*k)-24*cos(7*xi)-12*cos(8*xi)-48*cos(2*xi)+144*cos(xi*k-2*xi)+200*cos(xi*k+2*xi)-24*cos(xi*k+xi)-216*cos(3*xi)-360*cos(xi*k-3*xi)-24*cos(xi*k+3*xi);

			if(den==0) return false;

			num = 36*k*sin(alpha+7*xi-xi*k)+16*k*sin(alpha+3*xi+xi*k)-13*k*sin(9*xi+alpha-xi*k)+28*k*sin(alpha-xi-xi*k)-6*k*sin(alpha+5*xi+xi*k)+16*k*sin(alpha-xi*k+8*xi)-36*k*sin(alpha-2*xi-xi*k)+6*k*sin(-10*xi+alpha+xi*k)-28*k*sin(alpha+6*xi-xi*k)-6*k*sin(10*xi+alpha-xi*k)+28*k*sin(alpha-6*xi+xi*k)+k*sin(12*xi+alpha-xi*k)+36*k*sin(alpha+2*xi+xi*k)-16*k*sin(alpha+xi*k-8*xi)-28*k*sin(alpha+xi+xi*k)-36*k*sin(alpha-7*xi+xi*k)-16*k*sin(alpha-xi*k-3*xi)-3*sin(12*xi+alpha-xi*k)-13*k*sin(alpha+4*xi+xi*k)+13*k*sin(alpha-4*xi-xi*k)+2*k*sin(alpha+6*xi+xi*k)+13*k*sin(-9*xi+alpha+xi*k)-2*k*sin(-11*xi+alpha+xi*k)+6*k*sin(alpha-5*xi-xi*k)-2*k*sin(alpha-6*xi-xi*k)+4*sin(-11*xi+alpha+xi*k)-4*sin(11*xi+alpha-xi*k)-sin(-10*xi+alpha)-9*sin(9*xi+alpha)+sin(10*xi+alpha)+9*sin(-9*xi+alpha)+k*sin(alpha+7*xi+xi*k)+2*k*sin(11*xi+alpha-xi*k)-k*sin(-12*xi+alpha+xi*k)+3*sin(-12*xi+alpha+xi*k)-3*sin(-12*xi+alpha+2*xi*k)-k*sin(alpha-7*xi-xi*k)-28*k*sin(alpha+4*xi)+28*k*sin(alpha+2*xi)-70*k*sin(alpha+2*xi*k-6*xi)+42*k*sin(alpha+2*xi*k-8*xi)+12*k*sin(6*xi+alpha)+70*k*sin(alpha+2*xi*k-4*xi)-14*k*sin(-10*xi+alpha+2*xi*k)-2*k*sin(8*xi+alpha)-28*k*sin(alpha-2*xi)-42*k*sin(alpha+2*xi*k-2*xi)+2*k*sin(-12*xi+alpha+2*xi*k)+140*k*sin(xi+alpha)-140*k*sin(-xi+alpha)+14*k*sin(alpha+2*xi*k-3*xi)-84*k*sin(3*xi+alpha)+84*k*sin(-3*xi+alpha)-14*k*sin(-xi+alpha+2*xi*k)-14*k*sin(-7*xi+alpha+2*xi*k)+28*k*sin(alpha+5*xi)-28*k*sin(alpha-5*xi)+6*k*sin(xi+alpha+2*xi*k)+14*k*sin(-9*xi+alpha+2*xi*k)+28*k*sin(alpha-4*xi)+14*k*sin(alpha+2*xi*k)+7*sin(2*xi+alpha+2*xi*k)-12*k*sin(-6*xi+alpha)-2*k*sin(2*xi+alpha+2*xi*k)+4*k*sin(alpha-7*xi)-k*sin(3*xi+alpha+2*xi*k)-4*k*sin(alpha+7*xi)-6*k*sin(-11*xi+alpha+2*xi*k)-42*k*sin(alpha-2*xi*k+8*xi)+70*k*sin(alpha-2*xi*k+6*xi)+14*k*sin(10*xi+alpha-2*xi*k)-70*k*sin(alpha-2*xi*k+4*xi)-2*k*sin(12*xi+alpha-2*xi*k)+42*k*sin(alpha-2*xi*k+2*xi)+2*k*sin(-8*xi+alpha)-14*k*sin(alpha-2*xi*k+3*xi)+14*k*sin(7*xi+alpha-2*xi*k)+14*k*sin(xi+alpha-2*xi*k)-14*k*sin(9*xi+alpha-2*xi*k)-6*k*sin(-xi+alpha-2*xi*k)-14*k*sin(alpha-2*xi*k)+3*sin(3*xi+alpha+2*xi*k)+3*sin(12*xi+alpha-2*xi*k)-7*sin(-2*xi+alpha-2*xi*k)-3*sin(-3*xi+alpha-2*xi*k)+2*k*sin(-2*xi+alpha-2*xi*k)+k*sin(-3*xi+alpha-2*xi*k)+6*k*sin(11*xi+alpha-2*xi*k)+k*sin(-13*xi+alpha+2*xi*k)-2*sin(-13*xi+alpha+2*xi*k)-k*sin(13*xi+alpha-2*xi*k)+2*sin(13*xi+alpha-2*xi*k)+sin(11*xi+alpha)+372*sin(3*xi+alpha)-372*sin(-3*xi+alpha)-22*sin(alpha-4*xi)+22*sin(alpha+4*xi)+42*k*sin(4*xi+alpha-xi*k)+56*k*sin(alpha-xi*k)-56*k*sin(alpha+xi*k)-42*k*sin(-4*xi+alpha+xi*k)-56*k*sin(alpha-xi*k+5*xi)-42*k*sin(xi+alpha-xi*k)+45*sin(9*xi+alpha-2*xi*k)+15*sin(-xi+alpha-2*xi*k)-24*sin(-10*xi+alpha+xi*k)-132*sin(alpha-xi-xi*k)-108*sin(alpha+7*xi-xi*k)+68*sin(alpha-xi*k-3*xi)+176*sin(alpha+6*xi-xi*k)+3*sin(-6*xi+alpha)-144*sin(alpha+2*xi+xi*k)+51*sin(alpha+4*xi+xi*k)-8*sin(alpha+6*xi+xi*k)-68*sin(alpha+3*xi+xi*k)+132*sin(alpha+xi+xi*k)+24*sin(alpha+5*xi+xi*k)-4*sin(alpha+7*xi+xi*k)-51*sin(alpha-4*xi-xi*k)+8*sin(alpha-6*xi-xi*k)-84*sin(alpha-xi*k+8*xi)+144*sin(alpha-2*xi-xi*k)-24*sin(alpha-5*xi-xi*k)+4*sin(alpha-7*xi-xi*k)-3*sin(6*xi+alpha)+108*sin(alpha-7*xi+xi*k)-163*sin(alpha+5*xi)+163*sin(alpha-5*xi)-47*sin(alpha-7*xi)+3*sin(-8*xi+alpha)-3*sin(8*xi+alpha)+47*sin(alpha+7*xi)-176*sin(alpha-6*xi+xi*k)+84*sin(alpha+xi*k-8*xi)-32*sin(-9*xi+alpha+xi*k)+32*sin(9*xi+alpha-xi*k)+24*sin(10*xi+alpha-xi*k)+56*sin(alpha+xi*k-2*xi)*k-15*sin(xi+alpha+2*xi*k)-45*sin(-9*xi+alpha+2*xi*k)-25*sin(10*xi+alpha-2*xi*k)+45*sin(alpha-2*xi*k)-25*sin(xi+alpha-2*xi*k)-56*sin(alpha-xi*k+2*xi)*k-sin(-11*xi+alpha)+3*sin(alpha-2*xi*k+3*xi)-123*sin(alpha-2*xi*k+2*xi)-67*sin(7*xi+alpha-2*xi*k)-165*sin(alpha-2*xi*k+6*xi)-87*sin(alpha+2*xi*k-8*xi)+67*sin(-7*xi+alpha+2*xi*k)-185*sin(alpha+2*xi*k-4*xi)-3*sin(alpha+2*xi*k-3*xi)+185*sin(alpha-2*xi*k+4*xi)+87*sin(alpha-2*xi*k+8*xi)+165*sin(alpha+2*xi*k-6*xi)+123*sin(alpha+2*xi*k-2*xi)-45*sin(alpha+2*xi*k-5*xi)+45*sin(alpha-2*xi*k+5*xi)+25*sin(-xi+alpha+2*xi*k)+42*k*sin(-xi+alpha+xi*k)+56*k*sin(alpha+xi*k-5*xi)-45*sin(alpha+2*xi*k)+25*sin(-10*xi+alpha+2*xi*k)-560*sin(xi+alpha)+560*sin(-xi+alpha)-288*sin(alpha+xi*k-2*xi)+288*sin(alpha-xi*k+2*xi)-28*sin(alpha+2*xi)+28*sin(alpha-2*xi)+244*sin(alpha+xi*k-3*xi)-244*sin(alpha-xi*k+3*xi)+56*sin(alpha-xi*k+3*xi)*k-56*sin(alpha+xi*k-3*xi)*k+15*sin(-11*xi+alpha+2*xi*k)-15*sin(11*xi+alpha-2*xi*k)-258*sin(4*xi+alpha-xi*k)-244*sin(alpha-xi*k)+244*sin(alpha+xi*k)+258*sin(-4*xi+alpha+xi*k)+204*sin(alpha-xi*k+5*xi)+204*sin(xi+alpha-xi*k)-204*sin(-xi+alpha+xi*k)-204*sin(alpha+xi*k-5*xi);

			*coef = num/den;
			return true;
		}
		else if(row==2 && column==2)
		{
			den = 30+60*cos(xi)+100*cos(xi*k-xi)-96*cos(xi*k)-24*cos(4*xi)-68*cos(-4*xi+xi*k)+156*cos(xi*k-5*xi)+24*cos(2*xi*k-xi)-102*cos(2*xi*k-6*xi)+80*cos(2*xi*k-5*xi)+24*cos(2*xi*k-9*xi)+38*cos(2*xi*k-8*xi)+150*cos(2*xi*k-4*xi)-60*cos(2*xi*k-3*xi)-130*cos(2*xi*k-2*xi)-60*cos(2*xi*k-7*xi)+60*cos(5*xi)+66*cos(2*xi*k)-6*cos(2*xi*k-10*xi)+60*cos(-6*xi+xi*k)-30*cos(xi*k-8*xi)-68*cos(-7*xi+xi*k)+12*cos(xi*k-9*xi)-60*cos(xi*k+4*xi)-2*cos(8*xi+xi*k)+6*cos(-10*xi+xi*k)+24*cos(6*xi)-18*cos(2*xi+2*xi*k)+18*cos(6*xi+xi*k)-4*cos(xi+2*xi*k)-4*cos(-11*xi+2*xi*k)+2*cos(4*xi+2*xi*k)+4*cos(5*xi+xi*k)-12*cos(7*xi)-6*cos(8*xi)-24*cos(2*xi)+72*cos(xi*k-2*xi)+100*cos(xi*k+2*xi)-12*cos(xi*k+xi)-108*cos(3*xi)-180*cos(xi*k-3*xi)-12*cos(xi*k+3*xi);

			if(den==0) return false;

			num = -9-28*k*cos(xi*k+xi)-14*k*cos(2*xi*k-7*xi)+56*k*cos(xi*k-5*xi)+28*k*cos(-6*xi+xi*k)-2*cos(xi*k+7*xi)-3*cos(2*xi*k-12*xi)+9*cos(9*xi)-cos(11*xi)+56*cos(xi*k-2*xi)*k-18*cos(xi)-36*k*cos(-7*xi+xi*k)+13*k*cos(xi*k-9*xi)-2*k*cos(-11*xi+xi*k)-16*k*cos(xi*k-8*xi)+6*k*cos(-10*xi+xi*k)-k*cos(-12*xi+xi*k)+2*k*cos(2*xi*k-12*xi)+6*k*cos(xi+2*xi*k)-k*cos(2*xi*k+3*xi)-114*cos(xi*k-xi)+54*cos(xi*k)-14*k*cos(2*xi*k-xi)-14*k*cos(2*xi*k-10*xi)+60*cos(4*xi)+14*k*cos(2*xi*k)+96*cos(-4*xi+xi*k)-226*cos(xi*k-5*xi)+25*cos(2*xi*k-xi)+165*cos(2*xi*k-6*xi)-45*cos(2*xi*k-5*xi)-45*cos(2*xi*k-9*xi)-87*cos(2*xi*k-8*xi)-185*cos(2*xi*k-4*xi)-3*cos(2*xi*k-3*xi)+123*cos(2*xi*k-2*xi)+67*cos(2*xi*k-7*xi)+15*cos(5*xi)-45*cos(2*xi*k)+25*cos(2*xi*k-10*xi)-6*k*cos(-11*xi+2*xi*k)-6*k*cos(5*xi+xi*k)+k*cos(xi*k+7*xi)-114*cos(-6*xi+xi*k)-56*cos(xi*k-3*xi)*k+74*cos(xi*k-8*xi)+126*cos(-7*xi+xi*k)-42*k*cos(2*xi*k-2*xi)-36*cos(xi*k-9*xi)+29*cos(xi*k+4*xi)-24*cos(-10*xi+xi*k)-45*cos(6*xi)+7*cos(2*xi+2*xi*k)-6*cos(6*xi+xi*k)-15*cos(xi+2*xi*k)+15*cos(-11*xi+2*xi*k)+6*cos(5*xi+xi*k)-23*cos(7*xi)+13*cos(8*xi)+42*k*cos(2*xi*k-8*xi)+42*k*cos(xi*k-xi)+14*k*cos(2*xi*k-9*xi)-56*k*cos(xi*k)-42*k*cos(-4*xi+xi*k)-18*cos(2*xi)-58*cos(xi*k-2*xi)-54*cos(xi*k+2*xi)+26*cos(xi*k+xi)+18*cos(3*xi)+222*cos(xi*k-3*xi)-6*cos(xi*k+3*xi)+70*k*cos(2*xi*k-4*xi)+14*k*cos(2*xi*k-3*xi)-70*k*cos(2*xi*k-6*xi)+k*cos(2*xi*k-13*xi)+2*k*cos(6*xi+xi*k)-2*cos(2*xi*k-13*xi)+4*cos(-11*xi+xi*k)+3*cos(2*xi*k+3*xi)+3*cos(-12*xi+xi*k)-cos(10*xi)-2*k*cos(2*xi+2*xi*k)+16*k*cos(xi*k+3*xi)+36*k*cos(xi*k+2*xi)-13*k*cos(xi*k+4*xi);

			*coef = num/den;
			return true;
		}
	}



/*withDz=1 file Jinverse7.mw
		KthReal num,den;
		if(row==0 && column==0)
		{
			den=(-475+140*k+198*cos(xi)+772*cos(2*xi)+656*cos(xi*k-2*xi)+334*cos(xi*k+2*xi)-
			318*cos(xi*k+xi)-390*cos(3*xi)-1410*cos(xi*k-3*xi)+18*cos(xi*k+3*xi)+970*cos(xi*k-xi)-660*cos(xi*k)-
			56*k*cos(xi)-404*cos(4*xi)-220*cos(xi*k-2*xi)*k+500*cos(xi*k-3*xi)*k+60*k*cos(-4*xi+xi*k)+
			228*k*cos(xi*k)+112*k*cos(3*xi)-396*k*cos(xi*k-5*xi)-340*k*cos(xi*k-xi)-254*cos(-4*xi+xi*k)+
			1110*cos(xi*k-5*xi)+40*cos(2*xi*k-9*xi)+5*cos(2*xi*k-8*xi)+270*cos(5*xi)+96*cos(2*xi*k-xi)-
			9*cos(2*xi*k)-170*cos(2*xi*k-3*xi)+5*cos(2*xi*k-2*xi)-114*cos(2*xi*k-7*xi)-9*cos(2*xi*k-6*xi)+
			5*cos(2*xi*k-4*xi)+180*cos(2*xi*k-5*xi)+78*cos(xi*k-9*xi)+101*cos(xi*k-8*xi)-82*cos(xi*k+4*xi)-
			cos(2*xi*k-10*xi)-78*cos(xi*k-6*xi)-458*cos(-7*xi+xi*k)-224*k*cos(2*xi)+112*k*cos(4*xi)-
			80*k*cos(5*xi)+108*k*cos(xi*k+xi)+60*k*cos(xi*k-6*xi)+164*k*cos(-7*xi+xi*k)+4*k*cos(8*xi)-
			108*k*cos(xi*k+2*xi)-90*cos(7*xi)+12*cos(9*xi)+4*cos(3*xi+2*xi*k)-cos(4*xi+2*xi*k)-
			30*cos(xi+2*xi*k)+5*cos(2*xi+2*xi*k)-6*cos(-11*xi+2*xi*k)-cos(8*xi+xi*k)-
			25*cos(-10*xi+xi*k)-17*cos(8*xi)+124*cos(6*xi)-32*k*cos(6*xi)+28*k*cos(7*xi)-
			4*k*cos(xi*k+3*xi)-52*k*cos(xi*k-8*xi)+20*k*cos(xi*k+4*xi)-28*k*cos(xi*k-9*xi)-4*k*cos(9*xi)-
			4*k*cos(5*xi+xi*k)+12*k*cos(-10*xi+xi*k)+10*cos(5*xi+xi*k)+9*cos(6*xi+xi*k));

			if(den==0) return false;

			num=(770*sin(xi+alpha)+770*sin(xi-alpha)+35*sin(alpha+xi*k-2*xi)+35*sin(-alpha+xi*k-2*xi)-
			126*sin(alpha+2*xi)-126*sin(-alpha+2*xi)+91*sin(alpha+xi*k-3*xi)+91*sin(-alpha+xi*k-3*xi)-
			462*sin(3*xi+alpha)-462*sin(3*xi-alpha)-105*sin(-xi+alpha+xi*k)-105*sin(-xi-alpha+xi*k)-
			21*sin(-4*xi+alpha+xi*k)-21*sin(-4*xi-alpha+xi*k)+126*sin(-alpha+4*xi)+126*sin(alpha+4*xi)-
			35*sin(alpha+xi*k)-49*sin(-alpha+xi*k-5*xi)-49*sin(alpha+xi*k-5*xi)-35*sin(-alpha+xi*k)-
			56*k*sin(-alpha+4*xi)+24*k*sin(6*xi+alpha)+168*k*sin(3*xi+alpha)+168*k*sin(3*xi-alpha)-
			56*k*sin(alpha+5*xi)-56*k*sin(-alpha+5*xi)+56*k*sin(alpha+2*xi)-56*k*sin(alpha+4*xi)+
			56*k*sin(-alpha+2*xi)-54*sin(6*xi-alpha)+9*sin(8*xi-alpha)-22*sin(-alpha+7*xi)-
			22*sin(alpha+7*xi)+154*sin(alpha+5*xi)+154*sin(-alpha+5*xi)-54*sin(6*xi+alpha)+
			24*k*sin(6*xi-alpha)-4*k*sin(8*xi-alpha)-280*k*sin(xi-alpha)-280*k*sin(xi+alpha)+
			8*k*sin(-alpha+7*xi)+8*k*sin(alpha+7*xi)-4*k*sin(8*xi+alpha)-35*sin(-alpha+xi*k+3*xi)+
			15*sin(-alpha-7*xi+xi*k)+77*sin(alpha+xi+xi*k)-35*sin(alpha+xi*k+3*xi)+15*sin(alpha-
			7*xi+xi*k)+7*sin(-alpha-6*xi+xi*k)+9*sin(8*xi+alpha)+77*sin(-alpha+xi+xi*k)+
			21*sin(-alpha+2*xi+xi*k)-sin(-alpha+xi*k-8*xi)+7*sin(alpha-6*xi+xi*k)+
			21*sin(alpha+2*xi+xi*k)-sin(alpha+xi*k-8*xi)-7*sin(4*xi-alpha+xi*k)-7*sin(4*xi+alpha+xi*k)-
			2*sin(-9*xi-alpha+xi*k)-2*sin(-9*xi+alpha+xi*k)+9*sin(-alpha+5*xi+xi*k)-sin(-alpha+7*xi+xi*k)+
			sin(alpha+6*xi+xi*k)+9*sin(alpha+5*xi+xi*k)-sin(alpha+7*xi+xi*k)+
			sin(-alpha+6*xi+xi*k));

			*coef = num/den;
			return true;
		}

		else if(row==0 && column==1)
		{
			den = (-475+140*k+198*cos(xi)+
			772*cos(2*xi)+656*cos(xi*k-2*xi)+334*cos(xi*k+2*xi)-318*cos(xi*k+xi)-390*cos(3*xi)-1410*cos(xi*k-3*xi)+
			18*cos(xi*k+3*xi)+970*cos(xi*k-xi)-660*cos(xi*k)-56*k*cos(xi)-404*cos(4*xi)-220*cos(xi*k-2*xi)*k+
			500*cos(xi*k-3*xi)*k+60*k*cos(-4*xi+xi*k)+228*k*cos(xi*k)+112*k*cos(3*xi)-396*k*cos(xi*k-5*xi)-
			340*k*cos(xi*k-xi)-254*cos(-4*xi+xi*k)+1110*cos(xi*k-5*xi)+40*cos(2*xi*k-9*xi)+5*cos(2*xi*k-8*xi)+
			270*cos(5*xi)+96*cos(2*xi*k-xi)-9*cos(2*xi*k)-170*cos(2*xi*k-3*xi)+5*cos(2*xi*k-2*xi)-
			114*cos(2*xi*k-7*xi)-9*cos(2*xi*k-6*xi)+5*cos(2*xi*k-4*xi)+180*cos(2*xi*k-5*xi)+
			78*cos(xi*k-9*xi)+101*cos(xi*k-8*xi)-82*cos(xi*k+4*xi)-cos(2*xi*k-10*xi)-
			78*cos(xi*k-6*xi)-458*cos(-7*xi+xi*k)-224*k*cos(2*xi)+112*k*cos(4*xi)-
			80*k*cos(5*xi)+108*k*cos(xi*k+xi)+60*k*cos(xi*k-6*xi)+164*k*cos(-7*xi+xi*k)+
			4*k*cos(8*xi)-108*k*cos(xi*k+2*xi)-90*cos(7*xi)+12*cos(9*xi)+4*cos(3*xi+2*xi*k)-
			cos(4*xi+2*xi*k)-30*cos(xi+2*xi*k)+5*cos(2*xi+2*xi*k)-6*cos(-11*xi+2*xi*k)-
			cos(8*xi+xi*k)-25*cos(-10*xi+xi*k)-17*cos(8*xi)+124*cos(6*xi)-32*k*cos(6*xi)+
			28*k*cos(7*xi)-4*k*cos(xi*k+3*xi)-52*k*cos(xi*k-8*xi)+20*k*cos(xi*k+4*xi)-
			28*k*cos(xi*k-9*xi)-4*k*cos(9*xi)-4*k*cos(5*xi+xi*k)+12*k*cos(-10*xi+xi*k)+10*cos(5*xi+xi*k)+
			9*cos(6*xi+xi*k));

			if(den==0) return false;

			num = (-cos(-alpha+xi*k-8*xi)+21*cos(-alpha+2*xi+xi*k)-7*cos(4*xi-alpha+xi*k)+
			cos(-alpha+6*xi+xi*k)+2*cos(-9*xi+alpha+xi*k)-2*cos(-9*xi-alpha+xi*k)+770*cos(xi-alpha)-
			770*cos(xi+alpha)+35*cos(-alpha+xi*k-2*xi)-35*cos(alpha+xi*k-2*xi)-35*cos(-alpha+xi*k)-
			126*cos(-alpha+2*xi)+126*cos(alpha+2*xi)+91*cos(-alpha+xi*k-3*xi)-91*cos(alpha+xi*k-3*xi)+
			280*k*cos(xi+alpha)-56*k*cos(-alpha+5*xi)-168*k*cos(3*xi+alpha)+8*k*cos(-alpha+7*xi)-
			56*k*cos(alpha+2*xi)+56*k*cos(alpha+4*xi)-24*k*cos(6*xi+alpha)+56*k*cos(-alpha+2*xi)+
			4*k*cos(8*xi+alpha)+56*k*cos(alpha+5*xi)-8*k*cos(alpha+7*xi)-56*k*cos(-alpha+4*xi)+
			24*k*cos(6*xi-alpha)-4*k*cos(8*xi-alpha)-462*cos(3*xi-alpha)+462*cos(3*xi+alpha)-
			105*cos(-xi-alpha+xi*k)+105*cos(-xi+alpha+xi*k)-21*cos(-4*xi-alpha+xi*k)+
			21*cos(-4*xi+alpha+xi*k)-49*cos(-alpha+xi*k-5*xi)+35*cos(alpha+xi*k)+49*cos(alpha+xi*k-5*xi)+
			77*cos(-alpha+xi+xi*k)-7*cos(alpha-6*xi+xi*k)-77*cos(alpha+xi+xi*k)-15*cos(alpha-7*xi+xi*k)-
			126*cos(alpha+4*xi)+126*cos(-alpha+4*xi)-35*cos(-alpha+xi*k+3*xi)+9*cos(-alpha+5*xi+xi*k)+
			cos(alpha+7*xi+xi*k)-cos(-alpha+7*xi+xi*k)+15*cos(-alpha-7*xi+xi*k)+cos(alpha+xi*k-8*xi)+
			7*cos(-alpha-6*xi+xi*k)-54*cos(6*xi-alpha)+9*cos(8*xi-alpha)-21*cos(alpha+2*xi+xi*k)+
			7*cos(4*xi+alpha+xi*k)-cos(alpha+6*xi+xi*k)-280*k*cos(xi-alpha)+168*k*cos(3*xi-alpha)+
			154*cos(-alpha+5*xi)-22*cos(-alpha+7*xi)+54*cos(6*xi+alpha)-9*cos(8*xi+alpha)-
			154*cos(alpha+5*xi)+22*cos(alpha+7*xi)+35*cos(alpha+xi*k+3*xi)-9*cos(alpha+5*xi+xi*k));

			*coef = num/den;
			return true;
		}
		else if(row==0 && column==2)
		{
			*coef = 0.0;
			return true;
		}
		else if(row==1 && column==0)
		{
			den=(-268+80*k+144*cos(xi)+414*cos(2*xi)+414*cos(xi*k-2*xi)+
			134*cos(xi*k+2*xi)-76*cos(xi*k+xi)-252*cos(3*xi)-856*cos(xi*k-3*xi)-20*cos(xi*k+3*xi)+504*cos(xi*k-xi)-
			386*cos(xi*k)-40*k*cos(xi)-180*cos(4*xi)-144*cos(xi*k-2*xi)*k+304*cos(xi*k-3*xi)*k+16*k*cos(-4*xi+xi*k)+
			136*k*cos(xi*k)+72*k*cos(3*xi)-216*k*cos(xi*k-5*xi)-176*k*cos(xi*k-xi)-98*cos(-4*xi+xi*k)+604*cos(xi*k-5*xi)+
			12*cos(2*xi*k-9*xi)+2*cos(2*xi*k-8*xi)+132*cos(5*xi)+44*cos(2*xi*k-xi)-6*cos(2*xi*k)-96*cos(2*xi*k-3*xi)+
			4*cos(2*xi*k-2*xi)-56*cos(2*xi*k-7*xi)-6*cos(2*xi*k-6*xi)+4*cos(2*xi*k-4*xi)+104*cos(2*xi*k-5*xi)+
			50*cos(xi*k-8*xi)-14*cos(xi*k+4*xi)-102*cos(xi*k-6*xi)-156*cos(-7*xi+xi*k)-120*k*cos(2*xi)+
			48*k*cos(4*xi)-40*k*cos(5*xi)+24*k*cos(xi*k+xi)+56*k*cos(xi*k-6*xi)+56*k*cos(-7*xi+xi*k)-
			40*k*cos(xi*k+2*xi)-24*cos(7*xi)-8*cos(xi+2*xi*k)+2*cos(2*xi+2*xi*k)+34*cos(6*xi)-8*k*cos(6*xi)+
			8*k*cos(7*xi)+8*k*cos(xi*k+3*xi)-24*k*cos(xi*k-8*xi)+2*cos(6*xi+xi*k));

			if(den==0) return false;

			num = (-4*sin(-alpha+2*xi*k)+4*sin(-10*xi-alpha+2*xi*k)+sin(-10*xi-alpha+xi*k)-sin(-10*xi+alpha+xi*k)+
			4*sin(alpha+2*xi*k)-4*sin(-10*xi+alpha+2*xi*k)+2*sin(-xi+alpha+2*xi*k)-sin(xi+alpha+2*xi*k)-
			14*sin(-9*xi+alpha+2*xi*k)-2*sin(-xi-alpha+2*xi*k)+sin(xi-alpha+2*xi*k)+14*sin(-9*xi-alpha+2*xi*k)-
			3*sin(-11*xi-alpha+2*xi*k)+3*sin(-11*xi+alpha+2*xi*k)+5*sin(-3*xi+alpha+2*xi*k)-20*sin(alpha+2*xi*k-2*xi)-
			25*sin(-7*xi-alpha+2*xi*k)-5*sin(-3*xi-alpha+2*xi*k)+20*sin(-alpha+2*xi*k-2*xi)+20*sin(alpha+2*xi*k-8*xi)-
			20*sin(-alpha+2*xi*k-8*xi)-40*sin(alpha)-15*sin(alpha+xi*k-2*xi)+15*sin(-alpha+xi*k-2*xi)+30*sin(alpha+2*xi)-
			30*sin(-alpha+2*xi)-15*sin(alpha+xi*k-3*xi)+15*sin(-alpha+xi*k-3*xi)+20*sin(-xi+alpha+xi*k)-
			20*sin(-xi-alpha+xi*k)+20*sin(-4*xi+alpha+xi*k)-20*sin(-4*xi-alpha+xi*k)+12*sin(-alpha+4*xi)-
			12*sin(alpha+4*xi)+6*sin(alpha+xi*k)-6*sin(-alpha+xi*k-5*xi)+6*sin(alpha+xi*k-5*xi)-6*sin(-alpha+xi*k)-
			40*sin(-alpha+2*xi*k-4*xi)-40*sin(alpha+2*xi*k-6*xi)+40*sin(-alpha+2*xi*k-6*xi)+25*sin(-7*xi+alpha+2*xi*k)-
			20*sin(alpha+2*xi*k-5*xi)+40*sin(alpha+2*xi*k-4*xi)+20*sin(-alpha+2*xi*k-5*xi)-2*sin(6*xi-alpha)+
			2*sin(6*xi+alpha)-6*sin(-alpha+xi*k+3*xi)+sin(-alpha-7*xi+xi*k)-15*sin(alpha+xi+xi*k)+6*sin(alpha+xi*k+3*xi)-
			sin(alpha-7*xi+xi*k)+15*sin(-alpha-6*xi+xi*k)+15*sin(-alpha+xi+xi*k)+sin(-alpha+2*xi+xi*k)-
			6*sin(-alpha+xi*k-8*xi)-15*sin(alpha-6*xi+xi*k)-sin(alpha+2*xi+xi*k)+6*sin(alpha+xi*k-8*xi)+
			sin(-alpha+5*xi+xi*k)-sin(alpha+5*xi+xi*k));

			*coef = num/den;
			return true;
		}
		else if(row==1 && column==1)
		{
			den = (-268+80*k+144*cos(xi)+414*cos(2*xi)+
			414*cos(xi*k-2*xi)+134*cos(xi*k+2*xi)-76*cos(xi*k+xi)-252*cos(3*xi)-856*cos(xi*k-3*xi)-
			20*cos(xi*k+3*xi)+504*cos(xi*k-xi)-386*cos(xi*k)-40*k*cos(xi)-180*cos(4*xi)-144*cos(xi*k-2*xi)*k+
			304*cos(xi*k-3*xi)*k+16*k*cos(-4*xi+xi*k)+136*k*cos(xi*k)+72*k*cos(3*xi)-216*k*cos(xi*k-5*xi)-
			176*k*cos(xi*k-xi)-98*cos(-4*xi+xi*k)+604*cos(xi*k-5*xi)+12*cos(2*xi*k-9*xi)+2*cos(2*xi*k-8*xi)+
			132*cos(5*xi)+44*cos(2*xi*k-xi)-6*cos(2*xi*k)-96*cos(2*xi*k-3*xi)+4*cos(2*xi*k-2*xi)-
			56*cos(2*xi*k-7*xi)-6*cos(2*xi*k-6*xi)+4*cos(2*xi*k-4*xi)+104*cos(2*xi*k-5*xi)+
			50*cos(xi*k-8*xi)-14*cos(xi*k+4*xi)-102*cos(xi*k-6*xi)-156*cos(-7*xi+xi*k)-120*k*cos(2*xi)+
			48*k*cos(4*xi)-40*k*cos(5*xi)+24*k*cos(xi*k+xi)+56*k*cos(xi*k-6*xi)+56*k*cos(-7*xi+xi*k)-
			40*k*cos(xi*k+2*xi)-24*cos(7*xi)-8*cos(xi+2*xi*k)+2*cos(2*xi+2*xi*k)+34*cos(6*xi)-
			8*k*cos(6*xi)+8*k*cos(7*xi)+8*k*cos(xi*k+3*xi)-24*k*cos(xi*k-8*xi)+2*cos(6*xi+xi*k));

			if(den==0) return false;

			num = (20*cos(alpha+2*xi*k-5*xi)-40*cos(-alpha+2*xi*k-4*xi)+40*cos(-alpha+2*xi*k-6*xi)-
			6*cos(-alpha+xi*k-8*xi)+cos(-alpha+2*xi+xi*k)+40*cos(alpha)+15*cos(-alpha+xi*k-2*xi)+
			15*cos(alpha+xi*k-2*xi)-6*cos(-alpha+xi*k)-30*cos(-alpha+2*xi)-30*cos(alpha+2*xi)+
			15*cos(-alpha+xi*k-3*xi)+15*cos(alpha+xi*k-3*xi)-20*cos(-xi-alpha+xi*k)-20*cos(-xi+alpha+xi*k)-
			20*cos(-4*xi-alpha+xi*k)-20*cos(-4*xi+alpha+xi*k)-6*cos(-alpha+xi*k-5*xi)-6*cos(alpha+xi*k)-
			6*cos(alpha+xi*k-5*xi)+40*cos(alpha+2*xi*k-6*xi)-5*cos(-3*xi+alpha+2*xi*k)-5*cos(-3*xi-alpha+2*xi*k)-
			25*cos(-7*xi+alpha+2*xi*k)-25*cos(-7*xi-alpha+2*xi*k)+15*cos(-alpha+xi+xi*k)+15*cos(alpha-6*xi+xi*k)+
			15*cos(alpha+xi+xi*k)+20*cos(-alpha+2*xi*k-2*xi)+20*cos(alpha+2*xi*k-2*xi)-20*cos(-alpha+2*xi*k-8*xi)-
			20*cos(alpha+2*xi*k-8*xi)+cos(alpha-7*xi+xi*k)+12*cos(alpha+4*xi)+12*cos(-alpha+4*xi)-
			6*cos(-alpha+xi*k+3*xi)+cos(-alpha+5*xi+xi*k)+cos(-alpha-7*xi+xi*k)-6*cos(alpha+xi*k-8*xi)+
			15*cos(-alpha-6*xi+xi*k)-2*cos(6*xi-alpha)+cos(alpha+2*xi+xi*k)-2*cos(6*xi+alpha)-
			6*cos(alpha+xi*k+3*xi)+cos(alpha+5*xi+xi*k)+cos(-10*xi+alpha+xi*k)+cos(-10*xi-alpha+xi*k)+
			4*cos(-10*xi-alpha+2*xi*k)-4*cos(-alpha+2*xi*k)-2*cos(-xi-alpha+2*xi*k)+14*cos(-9*xi-alpha+2*xi*k)+
			cos(xi-alpha+2*xi*k)+4*cos(-10*xi+alpha+2*xi*k)-4*cos(alpha+2*xi*k)-2*cos(-xi+alpha+2*xi*k)+
			14*cos(-9*xi+alpha+2*xi*k)+cos(xi+alpha+2*xi*k)-3*cos(-11*xi-alpha+2*xi*k)-3*cos(-11*xi+alpha+2*xi*k)-
			40*cos(alpha+2*xi*k-4*xi)+20*cos(-alpha+2*xi*k-5*xi));

			*coef = num/den;
			return true;
		}
		else if(row==1 && column==2)
		{
			den = (-134+40*k+72*cos(xi)+
			207*cos(2*xi)+207*cos(xi*k-2*xi)+67*cos(xi*k+2*xi)-38*cos(xi*k+xi)-126*cos(3*xi)-
			428*cos(xi*k-3*xi)-10*cos(xi*k+3*xi)+252*cos(xi*k-xi)-193*cos(xi*k)-20*k*cos(xi)-
			90*cos(4*xi)-72*cos(xi*k-2*xi)*k+152*cos(xi*k-3*xi)*k+8*k*cos(-4*xi+xi*k)+
			68*k*cos(xi*k)+36*k*cos(3*xi)-108*k*cos(xi*k-5*xi)-88*k*cos(xi*k-xi)-49*cos(-4*xi+xi*k)+
			302*cos(xi*k-5*xi)+6*cos(2*xi*k-9*xi)+cos(2*xi*k-8*xi)+66*cos(5*xi)+22*cos(2*xi*k-xi)-
			3*cos(2*xi*k)-48*cos(2*xi*k-3*xi)+2*cos(2*xi*k-2*xi)-28*cos(2*xi*k-7*xi)-3*cos(2*xi*k-6*xi)+
			2*cos(2*xi*k-4*xi)+52*cos(2*xi*k-5*xi)+25*cos(xi*k-8*xi)-7*cos(xi*k+4*xi)-51*cos(xi*k-6*xi)-
			78*cos(-7*xi+xi*k)-60*k*cos(2*xi)+24*k*cos(4*xi)-20*k*cos(5*xi)+12*k*cos(xi*k+xi)+
			28*k*cos(xi*k-6*xi)+28*k*cos(-7*xi+xi*k)-20*k*cos(xi*k+2*xi)-12*cos(7*xi)-4*cos(xi+2*xi*k)+
			cos(2*xi+2*xi*k)+17*cos(6*xi)-4*k*cos(6*xi)+4*k*cos(7*xi)+4*k*cos(xi*k+3*xi)-12*k*cos(xi*k-8*xi)+cos(6*xi+xi*k));

			if(den==0) return false;

			num = (-sin(xi*k+2*xi)+2*sin(2*xi*k-xi)-sin(xi+2*xi*k)-14*sin(2*xi*k-9*xi)-80*sin(xi)+
			6*sin(xi*k-8*xi)-15*sin(xi*k-2*xi)-15*sin(xi*k+xi)+20*sin(xi*k-xi)+6*sin(xi*k)+
			6*sin(xi*k+3*xi)-15*sin(xi*k-3*xi)+20*sin(2*xi)-sin(-10*xi+xi*k)+4*sin(2*xi*k)-
			4*sin(2*xi*k-10*xi)+3*sin(-11*xi+2*xi*k)-sin(5*xi+xi*k)+40*sin(3*xi)+20*sin(-4*xi+xi*k)-
			15*sin(xi*k-6*xi)+5*sin(2*xi*k-3*xi)-40*sin(2*xi*k-6*xi)+40*sin(2*xi*k-4*xi)-
			20*sin(2*xi*k-5*xi)+4*sin(6*xi)-sin(-7*xi+xi*k)+6*sin(xi*k-5*xi)-8*sin(5*xi)-16*sin(4*xi)+
			25*sin(2*xi*k-7*xi)-20*sin(2*xi*k-2*xi)+20*sin(2*xi*k-8*xi));

			*coef = num/den;
			return true;
		}
		else if(row==2 && column==0)
		{
			den = (-1900+560*k+792*cos(xi)+
			3088*cos(2*xi)+2624*cos(xi*k-2*xi)+1336*cos(xi*k+2*xi)-1272*cos(xi*k+xi)-1560*cos(3*xi)-
			5640*cos(xi*k-3*xi)+72*cos(xi*k+3*xi)+3880*cos(xi*k-xi)-2640*cos(xi*k)-224*k*cos(xi)-
			1616*cos(4*xi)-880*cos(xi*k-2*xi)*k+2000*cos(xi*k-3*xi)*k+240*k*cos(-4*xi+xi*k)+
			912*k*cos(xi*k)+448*k*cos(3*xi)-1584*k*cos(xi*k-5*xi)-1360*k*cos(xi*k-xi)-
			1016*cos(-4*xi+xi*k)+4440*cos(xi*k-5*xi)+160*cos(2*xi*k-9*xi)+20*cos(2*xi*k-8*xi)+
			1080*cos(5*xi)+384*cos(2*xi*k-xi)-36*cos(2*xi*k)-680*cos(2*xi*k-3*xi)+
			20*cos(2*xi*k-2*xi)-456*cos(2*xi*k-7*xi)-36*cos(2*xi*k-6*xi)+20*cos(2*xi*k-4*xi)+
			720*cos(2*xi*k-5*xi)+312*cos(xi*k-9*xi)+404*cos(xi*k-8*xi)-328*cos(xi*k+4*xi)-
			4*cos(2*xi*k-10*xi)-312*cos(xi*k-6*xi)-1832*cos(-7*xi+xi*k)-896*k*cos(2*xi)+
			448*k*cos(4*xi)-320*k*cos(5*xi)+432*k*cos(xi*k+xi)+240*k*cos(xi*k-6*xi)+
			656*k*cos(-7*xi+xi*k)+16*k*cos(8*xi)-432*k*cos(xi*k+2*xi)-360*cos(7*xi)+
			48*cos(9*xi)+16*cos(3*xi+2*xi*k)-4*cos(4*xi+2*xi*k)-120*cos(xi+2*xi*k)+
			20*cos(2*xi+2*xi*k)-24*cos(-11*xi+2*xi*k)-4*cos(8*xi+xi*k)-100*cos(-10*xi+xi*k)-
			68*cos(8*xi)+496*cos(6*xi)-128*k*cos(6*xi)+112*k*cos(7*xi)-16*k*cos(xi*k+3*xi)-
			208*k*cos(xi*k-8*xi)+80*k*cos(xi*k+4*xi)-112*k*cos(xi*k-9*xi)-16*k*cos(9*xi)-
			16*k*cos(5*xi+xi*k)+48*k*cos(-10*xi+xi*k)+40*cos(5*xi+xi*k)+36*cos(6*xi+xi*k));

			if(den==0) return false;

			num = (-cos(10*xi-alpha)-9*cos(9*xi+alpha)+cos(11*xi+alpha)-925*cos(alpha+2*xi*k-5*xi)-
			555*cos(-alpha+2*xi*k-4*xi)+495*cos(-alpha+2*xi*k-6*xi)+9*cos(9*xi-alpha)-cos(11*xi-alpha)-
			3*cos(3*xi-alpha+2*xi*k)-284*cos(-alpha+xi*k-8*xi)+304*cos(-alpha+2*xi+xi*k)-
			89*cos(4*xi-alpha+xi*k)+12*cos(-alpha+6*xi+xi*k)-24*cos(-9*xi+alpha+xi*k)+
			24*cos(-9*xi-alpha+xi*k)-4*cos(-11*xi-alpha+xi*k)+56*cos(alpha+xi*k-3*xi)*k-56*cos(-alpha+xi*k-3*xi)*k+
			168*cos(alpha+xi*k-2*xi)*k-168*cos(-alpha+xi*k-2*xi)*k+280*k*cos(alpha+2*xi*k-5*xi)-
			154*k*cos(-3*xi+alpha+2*xi*k)-294*k*cos(-7*xi+alpha+2*xi*k)+42*k*cos(-xi+alpha+2*xi*k)+
			182*k*cos(-9*xi+alpha+2*xi*k)-2*k*cos(xi+alpha+2*xi*k)+210*k*cos(alpha+2*xi*k-6*xi)-
			210*k*cos(alpha+2*xi*k-4*xi)-126*k*cos(alpha+2*xi*k-8*xi)+126*k*cos(alpha+2*xi*k-2*xi)+
			42*k*cos(-10*xi+alpha+2*xi*k)-42*k*cos(alpha+2*xi*k)-62*k*cos(-11*xi+alpha+2*xi*k)-
			k*cos(3*xi+alpha+2*xi*k)+2240*cos(xi-alpha)-2240*cos(xi+alpha)+904*cos(-alpha+xi*k-2*xi)-
			904*cos(alpha+xi*k-2*xi)-182*k*cos(-4*xi+alpha+xi*k)-112*k*cos(alpha+xi*k)+
			182*k*cos(-4*xi-alpha+xi*k)+112*k*cos(-alpha+xi*k)-98*k*cos(-xi+alpha+xi*k)+
			98*k*cos(-xi-alpha+xi*k)-636*cos(-alpha+xi*k)-252*cos(-alpha+2*xi)+252*cos(alpha+2*xi)+
			204*cos(-alpha+xi*k-3*xi)-204*cos(alpha+xi*k-3*xi)+k*cos(-alpha+7*xi+xi*k)-
			3*k*cos(-12*xi+alpha+xi*k)+9*cos(-12*xi+alpha+xi*k)+cos(10*xi+alpha)+62*k*cos(-11*xi-alpha+2*xi*k)+
			k*cos(3*xi-alpha+2*xi*k)+6*k*cos(-12*xi-alpha+2*xi*k)-6*k*cos(2*xi-alpha+2*xi*k)-
			15*k*cos(4*xi+alpha+xi*k)+72*k*cos(-alpha+xi*k-8*xi)-52*k*cos(-alpha+2*xi+xi*k)+
			11*k*cos(-9*xi+alpha+xi*k)+2*k*cos(-11*xi-alpha+xi*k)+10*k*cos(alpha+5*xi+xi*k)+
			40*k*cos(-alpha+xi*k+3*xi)+22*k*cos(-10*xi+alpha+xi*k)+84*k*cos(alpha+xi+xi*k)+
			20*k*cos(-alpha-7*xi+xi*k)-20*k*cos(alpha-7*xi+xi*k)-40*k*cos(alpha+xi*k+3*xi)-
			11*k*cos(-9*xi-alpha+xi*k)-84*k*cos(-alpha+xi+xi*k)-22*k*cos(-10*xi-alpha+xi*k)+
			2*k*cos(alpha+6*xi+xi*k)+700*k*cos(xi+alpha)-140*k*cos(-alpha+5*xi)-420*k*cos(3*xi+alpha)+
			20*k*cos(-alpha+7*xi)-84*k*cos(alpha+2*xi)+84*k*cos(alpha+4*xi)-36*k*cos(6*xi+alpha)+
			84*k*cos(-alpha+2*xi)+6*k*cos(8*xi+alpha)+140*k*cos(alpha+5*xi)-20*k*cos(alpha+7*xi)-
			84*k*cos(-alpha+4*xi)+36*k*cos(6*xi-alpha)-6*k*cos(8*xi-alpha)-9*cos(-12*xi-alpha+2*xi*k)+
			21*cos(2*xi-alpha+2*xi*k)+9*k*cos(-13*xi+alpha+2*xi*k)-1380*cos(3*xi-alpha)+1380*cos(3*xi+alpha)-
			356*cos(-xi-alpha+xi*k)+356*cos(-xi+alpha+xi*k)-902*cos(-4*xi-alpha+xi*k)+902*cos(-4*xi+alpha+xi*k)-
			20*cos(-alpha+xi*k-5*xi)+636*cos(alpha+xi*k)+20*cos(alpha+xi*k-5*xi)-495*cos(alpha+2*xi*k-6*xi)+
			573*cos(-3*xi+alpha+2*xi*k)-573*cos(-3*xi-alpha+2*xi*k)+867*cos(-7*xi+alpha+2*xi*k)-
			867*cos(-7*xi-alpha+2*xi*k)+316*cos(-alpha+xi+xi*k)-624*cos(alpha-6*xi+xi*k)-
			316*cos(alpha+xi+xi*k)+369*cos(-alpha+2*xi*k-2*xi)-369*cos(alpha+2*xi*k-2*xi)-
			261*cos(-alpha+2*xi*k-8*xi)+261*cos(alpha+2*xi*k-8*xi)+44*cos(alpha-7*xi+xi*k)-
			258*cos(alpha+4*xi)+258*cos(-alpha+4*xi)-156*cos(-alpha+xi*k+3*xi)+40*cos(-alpha+5*xi+xi*k)+
			4*cos(alpha+7*xi+xi*k)-4*cos(-alpha+7*xi+xi*k)-44*cos(-alpha-7*xi+xi*k)+284*cos(alpha+xi*k-8*xi)+
			624*cos(-alpha-6*xi+xi*k)-117*cos(6*xi-alpha)+23*cos(8*xi-alpha)-304*cos(alpha+2*xi+xi*k)+
			89*cos(4*xi+alpha+xi*k)-12*cos(alpha+6*xi+xi*k)+15*k*cos(4*xi-alpha+xi*k)+140*k*cos(alpha-6*xi+xi*k)-
			2*k*cos(-alpha+6*xi+xi*k)-72*k*cos(alpha+xi*k-8*xi)+52*k*cos(alpha+2*xi+xi*k)+3*cos(3*xi+alpha+2*xi*k)-
			2*k*cos(-11*xi+alpha+xi*k)-10*k*cos(-alpha+5*xi+xi*k)+4*cos(-11*xi+alpha+xi*k)-9*cos(-12*xi-alpha+xi*k)-
			700*k*cos(xi-alpha)+420*k*cos(3*xi-alpha)+499*cos(-alpha+5*xi)-95*cos(-alpha+7*xi)+117*cos(6*xi+alpha)-
			23*cos(8*xi+alpha)-499*cos(alpha+5*xi)+95*cos(alpha+7*xi)-9*k*cos(-13*xi-alpha+2*xi*k)-
			140*k*cos(-alpha-6*xi+xi*k)+18*cos(-13*xi-alpha+2*xi*k)-k*cos(alpha+7*xi+xi*k)+
			3*k*cos(-12*xi-alpha+xi*k)-18*cos(-13*xi+alpha+2*xi*k)+6*k*cos(2*xi+alpha+2*xi*k)-
			280*k*cos(-alpha+2*xi*k-5*xi)+154*k*cos(-3*xi-alpha+2*xi*k)+294*k*cos(-7*xi-alpha+2*xi*k)-
			42*k*cos(-xi-alpha+2*xi*k)-182*k*cos(-9*xi-alpha+2*xi*k)+2*k*cos(xi-alpha+2*xi*k)-
			210*k*cos(-alpha+2*xi*k-6*xi)+210*k*cos(-alpha+2*xi*k-4*xi)+126*k*cos(-alpha+2*xi*k-8*xi)-
			126*k*cos(-alpha+2*xi*k-2*xi)-42*k*cos(-10*xi-alpha+2*xi*k)+42*k*cos(-alpha+2*xi*k)+
			156*cos(alpha+xi*k+3*xi)-40*cos(alpha+5*xi+xi*k)-76*cos(-10*xi+alpha+xi*k)+
			76*cos(-10*xi-alpha+xi*k)+75*cos(-10*xi-alpha+2*xi*k)-135*cos(-alpha+2*xi*k)+
			183*cos(-xi-alpha+2*xi*k)+477*cos(-9*xi-alpha+2*xi*k)-17*cos(xi-alpha+2*xi*k)-
			75*cos(-10*xi+alpha+2*xi*k)+135*cos(alpha+2*xi*k)-183*cos(-xi+alpha+2*xi*k)-
			477*cos(-9*xi+alpha+2*xi*k)+17*cos(xi+alpha+2*xi*k)-143*cos(-11*xi-alpha+2*xi*k)+
			143*cos(-11*xi+alpha+2*xi*k)+555*cos(alpha+2*xi*k-4*xi)+925*cos(-alpha+2*xi*k-5*xi)-
			6*k*cos(-12*xi+alpha+2*xi*k)+9*cos(-12*xi+alpha+2*xi*k)-21*cos(2*xi+alpha+2*xi*k));

			*coef = num/den;
			return true;
		}
		else if(row==2 && column==1)
		{
			den = (-1900+560*k+792*cos(xi)+
			3088*cos(2*xi)+2624*cos(xi*k-2*xi)+1336*cos(xi*k+2*xi)-1272*cos(xi*k+xi)-
			1560*cos(3*xi)-5640*cos(xi*k-3*xi)+72*cos(xi*k+3*xi)+3880*cos(xi*k-xi)-2640*cos(xi*k)-
			224*k*cos(xi)-1616*cos(4*xi)-880*cos(xi*k-2*xi)*k+2000*cos(xi*k-3*xi)*k+
			240*k*cos(-4*xi+xi*k)+912*k*cos(xi*k)+448*k*cos(3*xi)-1584*k*cos(xi*k-5*xi)-
			1360*k*cos(xi*k-xi)-1016*cos(-4*xi+xi*k)+4440*cos(xi*k-5*xi)+160*cos(2*xi*k-9*xi)+
			20*cos(2*xi*k-8*xi)+1080*cos(5*xi)+384*cos(2*xi*k-xi)-36*cos(2*xi*k)-680*cos(2*xi*k-3*xi)+
			20*cos(2*xi*k-2*xi)-456*cos(2*xi*k-7*xi)-36*cos(2*xi*k-6*xi)+20*cos(2*xi*k-4*xi)+
			720*cos(2*xi*k-5*xi)+312*cos(xi*k-9*xi)+404*cos(xi*k-8*xi)-328*cos(xi*k+4*xi)-
			4*cos(2*xi*k-10*xi)-312*cos(xi*k-6*xi)-1832*cos(-7*xi+xi*k)-896*k*cos(2*xi)+448*k*cos(4*xi)-
			320*k*cos(5*xi)+432*k*cos(xi*k+xi)+240*k*cos(xi*k-6*xi)+656*k*cos(-7*xi+xi*k)+
			16*k*cos(8*xi)-432*k*cos(xi*k+2*xi)-360*cos(7*xi)+48*cos(9*xi)+16*cos(3*xi+2*xi*k)-
			4*cos(4*xi+2*xi*k)-120*cos(xi+2*xi*k)+20*cos(2*xi+2*xi*k)-24*cos(-11*xi+2*xi*k)-
			4*cos(8*xi+xi*k)-100*cos(-10*xi+xi*k)-68*cos(8*xi)+496*cos(6*xi)-128*k*cos(6*xi)+
			112*k*cos(7*xi)-16*k*cos(xi*k+3*xi)-208*k*cos(xi*k-8*xi)+80*k*cos(xi*k+4*xi)-
			112*k*cos(xi*k-9*xi)-16*k*cos(9*xi)-16*k*cos(5*xi+xi*k)+48*k*cos(-10*xi+xi*k)+
			40*cos(5*xi+xi*k)+36*cos(6*xi+xi*k));

			if(den==0) return false;

			num = (135*sin(-alpha+2*xi*k)-75*sin(-10*xi-alpha+2*xi*k)-76*sin(-10*xi-alpha+xi*k)-
			76*sin(-10*xi+alpha+xi*k)+135*sin(alpha+2*xi*k)-75*sin(-10*xi+alpha+2*xi*k)-
			183*sin(-xi+alpha+2*xi*k)+17*sin(xi+alpha+2*xi*k)-477*sin(-9*xi+alpha+2*xi*k)-
			183*sin(-xi-alpha+2*xi*k)+17*sin(xi-alpha+2*xi*k)-477*sin(-9*xi-alpha+2*xi*k)+
			143*sin(-11*xi-alpha+2*xi*k)+143*sin(-11*xi+alpha+2*xi*k)+168*sin(alpha+xi*k-2*xi)*k+
			168*sin(-alpha+xi*k-2*xi)*k+573*sin(-3*xi+alpha+2*xi*k)-369*sin(alpha+2*xi*k-2*xi)+
			867*sin(-7*xi-alpha+2*xi*k)+573*sin(-3*xi-alpha+2*xi*k)+sin(11*xi-alpha)-
			369*sin(-alpha+2*xi*k-2*xi)+261*sin(alpha+2*xi*k-8*xi)+261*sin(-alpha+2*xi*k-8*xi)-
			2240*sin(xi+alpha)-2240*sin(xi-alpha)-904*sin(alpha+xi*k-2*xi)-904*sin(-alpha+xi*k-2*xi)+
			252*sin(alpha+2*xi)+252*sin(-alpha+2*xi)-204*sin(alpha+xi*k-3*xi)-204*sin(-alpha+xi*k-3*xi)+
			56*sin(-alpha+xi*k-3*xi)*k-112*k*sin(-alpha+xi*k)-182*k*sin(-4*xi-alpha+xi*k)-112*k*sin(alpha+xi*k)-
			182*k*sin(-4*xi+alpha+xi*k)-98*k*sin(-xi-alpha+xi*k)-98*k*sin(-xi+alpha+xi*k)+
			140*k*sin(alpha-6*xi+xi*k)+52*k*sin(-alpha+2*xi+xi*k)+52*k*sin(alpha+2*xi+xi*k)-
			72*k*sin(alpha+xi*k-8*xi)-15*k*sin(4*xi-alpha+xi*k)+140*k*sin(-alpha-6*xi+xi*k)-
			20*k*sin(alpha-7*xi+xi*k)+84*k*sin(-alpha+xi+xi*k)+84*k*sin(alpha+xi+xi*k)+
			11*k*sin(-9*xi+alpha+xi*k)-40*k*sin(-alpha+xi*k+3*xi)-20*k*sin(-alpha-7*xi+xi*k)+
			22*k*sin(-10*xi+alpha+xi*k)-72*k*sin(-alpha+xi*k-8*xi)-15*k*sin(4*xi+alpha+xi*k)+
			2*k*sin(-alpha+6*xi+xi*k)-2*k*sin(-11*xi+alpha+xi*k)+11*k*sin(-9*xi-alpha+xi*k)-
			40*k*sin(alpha+xi*k+3*xi)+10*k*sin(-alpha+5*xi+xi*k)+4*sin(-11*xi+alpha+xi*k)+
			22*k*sin(-10*xi-alpha+xi*k)+2*k*sin(alpha+6*xi+xi*k)-2*k*sin(-11*xi-alpha+xi*k)+
			10*k*sin(alpha+5*xi+xi*k)-k*sin(-alpha+7*xi+xi*k)-3*k*sin(-12*xi+alpha+xi*k)+
			9*sin(-12*xi+alpha+xi*k)-k*sin(alpha+7*xi+xi*k)-3*k*sin(-12*xi-alpha+xi*k)-
			154*k*sin(-3*xi-alpha+2*xi*k)+280*k*sin(-alpha+2*xi*k-5*xi)+9*sin(-12*xi-alpha+xi*k)+
			42*k*sin(-xi-alpha+2*xi*k)-294*k*sin(-7*xi-alpha+2*xi*k)-2*k*sin(xi-alpha+2*xi*k)+
			182*k*sin(-9*xi-alpha+2*xi*k)-210*k*sin(-alpha+2*xi*k-4*xi)+210*k*sin(-alpha+2*xi*k-6*xi)+
			126*k*sin(-alpha+2*xi*k-2*xi)-126*k*sin(-alpha+2*xi*k-8*xi)-
			42*k*sin(-alpha+2*xi*k)+42*k*sin(-10*xi-alpha+2*xi*k)-62*k*sin(-11*xi-alpha+2*xi*k)-
			k*sin(3*xi-alpha+2*xi*k)+3*sin(3*xi-alpha+2*xi*k)+9*sin(-12*xi-alpha+2*xi*k)-
			6*k*sin(-12*xi-alpha+2*xi*k)+6*k*sin(2*xi-alpha+2*xi*k)-154*k*sin(-3*xi+alpha+2*xi*k)+
			280*k*sin(alpha+2*xi*k-5*xi)+42*k*sin(-xi+alpha+2*xi*k)-294*k*sin(-7*xi+alpha+2*xi*k)-
			2*k*sin(xi+alpha+2*xi*k)+182*k*sin(-9*xi+alpha+2*xi*k)-210*k*sin(alpha+2*xi*k-4*xi)+
			210*k*sin(alpha+2*xi*k-6*xi)+126*k*sin(alpha+2*xi*k-2*xi)-126*k*sin(alpha+2*xi*k-8*xi)-
			42*k*sin(alpha+2*xi*k)+42*k*sin(-10*xi+alpha+2*xi*k)-21*sin(2*xi-alpha+2*xi*k)-
			62*k*sin(-11*xi+alpha+2*xi*k)-k*sin(3*xi+alpha+2*xi*k)-6*k*sin(-12*xi+alpha+2*xi*k)+
			6*k*sin(2*xi+alpha+2*xi*k)+sin(11*xi+alpha)+3*sin(3*xi+alpha+2*xi*k)+
			9*sin(-12*xi+alpha+2*xi*k)-21*sin(2*xi+alpha+2*xi*k)+sin(10*xi+alpha)+
			1380*sin(3*xi+alpha)+1380*sin(3*xi-alpha)+356*sin(-xi+alpha+xi*k)+
			356*sin(-xi-alpha+xi*k)+902*sin(-4*xi+alpha+xi*k)+902*sin(-4*xi-alpha+xi*k)-
			258*sin(-alpha+4*xi)-258*sin(alpha+4*xi)+636*sin(alpha+xi*k)+20*sin(-alpha+xi*k-5*xi)+
			20*sin(alpha+xi*k-5*xi)+636*sin(-alpha+xi*k)+555*sin(-alpha+2*xi*k-4*xi)-
			495*sin(alpha+2*xi*k-6*xi)-495*sin(-alpha+2*xi*k-6*xi)+867*sin(-7*xi+alpha+2*xi*k)-
			925*sin(alpha+2*xi*k-5*xi)+555*sin(alpha+2*xi*k-4*xi)-925*sin(-alpha+2*xi*k-5*xi)+
			84*k*sin(-alpha+4*xi)-36*k*sin(6*xi+alpha)-420*k*sin(3*xi+alpha)-420*k*sin(3*xi-alpha)+
			140*k*sin(alpha+5*xi)+140*k*sin(-alpha+5*xi)-84*k*sin(alpha+2*xi)+84*k*sin(alpha+4*xi)-
			84*k*sin(-alpha+2*xi)+117*sin(6*xi-alpha)-23*sin(8*xi-alpha)+95*sin(-alpha+7*xi)+
			95*sin(alpha+7*xi)-499*sin(alpha+5*xi)-499*sin(-alpha+5*xi)+117*sin(6*xi+alpha)-
			36*k*sin(6*xi-alpha)+6*k*sin(8*xi-alpha)+700*k*sin(xi-alpha)+700*k*sin(xi+alpha)-
			20*k*sin(-alpha+7*xi)-20*k*sin(alpha+7*xi)+6*k*sin(8*xi+alpha)+156*sin(-alpha+xi*k+3*xi)+
			44*sin(-alpha-7*xi+xi*k)-316*sin(alpha+xi+xi*k)+156*sin(alpha+xi*k+3*xi)+
			44*sin(alpha-7*xi+xi*k)-624*sin(-alpha-6*xi+xi*k)-23*sin(8*xi+alpha)-
			316*sin(-alpha+xi+xi*k)-304*sin(-alpha+2*xi+xi*k)+284*sin(-alpha+xi*k-8*xi)-
			624*sin(alpha-6*xi+xi*k)-304*sin(alpha+2*xi+xi*k)+284*sin(alpha+xi*k-8*xi)+
			89*sin(4*xi-alpha+xi*k)+89*sin(4*xi+alpha+xi*k)-24*sin(-9*xi-alpha+xi*k)-
			24*sin(-9*xi+alpha+xi*k)-40*sin(-alpha+5*xi+xi*k)+4*sin(-alpha+7*xi+xi*k)-12*sin(alpha+6*xi+xi*k)-
			40*sin(alpha+5*xi+xi*k)+4*sin(alpha+7*xi+xi*k)-12*sin(-alpha+6*xi+xi*k)+
			56*sin(alpha+xi*k-3*xi)*k+4*sin(-11*xi-alpha+xi*k)+9*k*sin(-13*xi-alpha+2*xi*k)-
			18*sin(-13*xi-alpha+2*xi*k)-18*sin(-13*xi+alpha+2*xi*k)+9*k*sin(-13*xi+alpha+2*xi*k)-
			9*sin(9*xi+alpha)-9*sin(9*xi-alpha)+sin(10*xi-alpha));

			*coef = num/den;
			return true;
		}
		else if(row==2 && column==2)
		{
			den = (-950+280*k+396*cos(xi)+1544*cos(2*xi)+
			1312*cos(xi*k-2*xi)+668*cos(xi*k+2*xi)-636*cos(xi*k+xi)-780*cos(3*xi)-2820*cos(xi*k-3*xi)+
			36*cos(xi*k+3*xi)+1940*cos(xi*k-xi)-1320*cos(xi*k)-112*k*cos(xi)-808*cos(4*xi)-440*cos(xi*k-2*xi)*k+
			1000*cos(xi*k-3*xi)*k+120*k*cos(-4*xi+xi*k)+456*k*cos(xi*k)+224*k*cos(3*xi)-792*k*cos(xi*k-5*xi)-
			680*k*cos(xi*k-xi)-508*cos(-4*xi+xi*k)+2220*cos(xi*k-5*xi)+80*cos(2*xi*k-9*xi)+10*cos(2*xi*k-8*xi)+
			540*cos(5*xi)+192*cos(2*xi*k-xi)-18*cos(2*xi*k)-340*cos(2*xi*k-3*xi)+10*cos(2*xi*k-2*xi)-
			228*cos(2*xi*k-7*xi)-18*cos(2*xi*k-6*xi)+10*cos(2*xi*k-4*xi)+360*cos(2*xi*k-5*xi)+156*cos(xi*k-9*xi)+
			202*cos(xi*k-8*xi)-164*cos(xi*k+4*xi)-2*cos(2*xi*k-10*xi)-156*cos(xi*k-6*xi)-916*cos(-7*xi+xi*k)-
			448*k*cos(2*xi)+224*k*cos(4*xi)-160*k*cos(5*xi)+216*k*cos(xi*k+xi)+120*k*cos(xi*k-6*xi)+
			328*k*cos(-7*xi+xi*k)+8*k*cos(8*xi)-216*k*cos(xi*k+2*xi)-180*cos(7*xi)+24*cos(9*xi)+8*cos(3*xi+2*xi*k)-
			2*cos(4*xi+2*xi*k)-60*cos(xi+2*xi*k)+10*cos(2*xi+2*xi*k)-12*cos(-11*xi+2*xi*k)-2*cos(8*xi+xi*k)-
			50*cos(-10*xi+xi*k)-34*cos(8*xi)+248*cos(6*xi)-64*k*cos(6*xi)+56*k*cos(7*xi)-8*k*cos(xi*k+3*xi)-
			104*k*cos(xi*k-8*xi)+40*k*cos(xi*k+4*xi)-56*k*cos(xi*k-9*xi)-8*k*cos(9*xi)-8*k*cos(5*xi+xi*k)+
			24*k*cos(-10*xi+xi*k)+20*cos(5*xi+xi*k)+18*cos(6*xi+xi*k));

			if(den==0) return false;

			num = (171+182*k*cos(2*xi*k-9*xi)-126*k*cos(2*xi*k-8*xi)-cos(10*xi)-98*cos(xi)-cos(11*xi)+
			280*k*cos(2*xi*k-5*xi)-274*cos(2*xi)-534*cos(xi*k-2*xi)-130*cos(xi*k+2*xi)-114*cos(xi*k+xi)+
			162*cos(3*xi)+138*cos(xi*k-3*xi)+78*cos(xi*k+3*xi)+6*k*cos(2*xi+2*xi*k)+26*cos(xi*k-xi)+306*cos(xi*k)-
			42*k*cos(2*xi*k)+42*k*cos(2*xi*k-10*xi)-210*k*cos(2*xi*k-4*xi)+140*cos(4*xi)+168*cos(xi*k-2*xi)*k+
			56*cos(xi*k-3*xi)*k-182*k*cos(-4*xi+xi*k)-112*k*cos(xi*k)-98*k*cos(xi*k-xi)+656*cos(-4*xi+xi*k)-
			198*cos(xi*k-5*xi)-477*cos(2*xi*k-9*xi)+261*cos(2*xi*k-8*xi)-65*cos(5*xi)-183*cos(2*xi*k-xi)+
			135*cos(2*xi*k)+573*cos(2*xi*k-3*xi)-369*cos(2*xi*k-2*xi)+867*cos(2*xi*k-7*xi)-495*cos(2*xi*k-6*xi)+
			555*cos(2*xi*k-4*xi)-925*cos(2*xi*k-5*xi)-36*cos(xi*k-9*xi)+270*cos(xi*k-8*xi)+39*cos(xi*k+4*xi)-
			75*cos(2*xi*k-10*xi)-534*cos(xi*k-6*xi)+122*cos(-7*xi+xi*k)+84*k*cos(xi*k+xi)+140*k*cos(xi*k-6*xi)-
			294*k*cos(2*xi*k-7*xi)+210*k*cos(2*xi*k-6*xi)-154*k*cos(2*xi*k-3*xi)+126*k*cos(2*xi*k-2*xi)-
			18*cos(2*xi*k-13*xi)+4*cos(xi*k-11*xi)-k*cos(xi*k+7*xi)-2*k*cos(xi+2*xi*k)-k*cos(3*xi+2*xi*k)-
			20*k*cos(-7*xi+xi*k)-6*k*cos(2*xi*k-12*xi)-3*k*cos(-12*xi+xi*k)+9*cos(-12*xi+xi*k)+42*k*cos(2*xi*k-xi)+
			52*k*cos(xi*k+2*xi)-7*cos(7*xi)+9*cos(9*xi)+3*cos(3*xi+2*xi*k)+17*cos(xi+2*xi*k)-21*cos(2*xi+2*xi*k)+
			143*cos(-11*xi+2*xi*k)-76*cos(-10*xi+xi*k)+9*cos(8*xi)-45*cos(6*xi)-40*k*cos(xi*k+3*xi)-
			72*k*cos(xi*k-8*xi)-15*k*cos(xi*k+4*xi)+11*k*cos(xi*k-9*xi)+10*k*cos(5*xi+xi*k)+22*k*cos(-10*xi+xi*k)-
			22*cos(5*xi+xi*k)-6*cos(6*xi+xi*k)+9*cos(2*xi*k-12*xi)-62*k*cos(-11*xi+2*xi*k)+9*k*cos(2*xi*k-13*xi)-
			2*k*cos(xi*k-11*xi)+2*k*cos(6*xi+xi*k)+2*cos(xi*k+7*xi));

			*coef = num/den;
			return true;
		}
	}
*/
}
