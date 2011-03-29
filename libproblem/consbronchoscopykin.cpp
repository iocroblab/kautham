#include "ConsBronchoscopyKin.h"
#include "robot.h"
#include <mt/mt.h> 

namespace libProblem {
	
	ConsBronchoscopyKin::ConsBronchoscopyKin(Robot* const rob):ConstrainedKinematic(rob){
		
		_robot = rob;
		_robConf.setRn(_robot->getNumJoints());	
		_currentvalues[0] = 0.; 
		_currentvalues[1] = 0.; 
		_currentvalues[2] = 0.;

	}
	
	ConsBronchoscopyKin::~ConsBronchoscopyKin(){
	}

  bool ConsBronchoscopyKin::solve(){
    _robConf = solve(_target);
    return true;
  }

	RobConf ConsBronchoscopyKin::solve(vector<KthReal> &values){

    vector<KthReal> coords(7);
    //Links characteristic
    Link* _linkn = _robot->getLink(_robot->getNumJoints()-1);
    KthReal l = abs(_linkn->getA()); //one link length
		int n=_robot->getNumJoints()-1; //number of joints (not considering base and TCP which are just shperes)

    // Current values
		KthReal curr_alpha = getvalues(0);
		KthReal curr_psi = getvalues(1);
		//KthReal curr_z = getvalues(2); // it will not be used cause the slider already gives a Delta
		
//cout<<"alpha = "<<curr_alpha<<" psi = "<<curr_psi<<endl;

    // Read values
		KthReal Dalpha = (KthReal) M_PI *(values[0]-curr_alpha); // slider goes from -1000 to 1000
    KthReal psi = (KthReal) M_PI/2*(values[1]/(n-1)); // values[1]=total bending angle read from the slider
    KthReal Dzeta = (KthReal)-values[2];//directly the value read from the slider //-1000*(values[2]-currentvalues2);
   

    // sin and cos of rotation angles
    KthReal calpha=cos(Dalpha); KthReal salpha=sin(Dalpha);
    
    
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

    //psi = psi ==0?  psi+0.001 : psi; // ¡¡¡¡¡¡¡ TO CHANGE!!!!!!!!
    if (abs(psi)>0.0001){
      KthReal d=l/2*cos(psi/2)/sin(psi/2);	 //dradio = -length/(2*tan(theta/2));	
      KthReal Dbeta = Dzeta/d; //(2*zeta*tan(theta/2))/length;
		  //mt::Matrix3x3 BroncoRotMatrix;
		  //mt::Point3    BroncoPos;
  		
      KthReal cpsi=cos(psi); KthReal spsi=sin(psi);
      KthReal cbeta=cos(Dbeta);  KthReal sbeta=sin(Dbeta);

      mt::Transform T_Y_d_Z_l2;
      T_Y_d_Z_l2.setRotation(mt::Rotation(0.0,0.0,0.0,1));
      T_Y_d_Z_l2.setTranslation(mt::Vector3(0,d,-l/2));


      // Rotation of beta along the x axis -> the bronchoscope go frwd(rotate) in local z-y plane
      mt::Matrix3x3 RxDbeta=Matrix3x3( 1,      0,      0,
                                      0.0,  cbeta, -sbeta,
                                      0.0,  sbeta,  cbeta);
                                      
      mt::Transform T_RxDbeta;
      T_RxDbeta.setRotation(mt::Rotation(RxDbeta));
      T_RxDbeta.setTranslation(mt::Vector3(0,0,0));

      // updating the base transform
      //mt::Transform 
        Tbase=currTran*T_RzDalpha*T_Y_d_Z_l2*T_RxDbeta*T_Y_d_Z_l2.inverse(); // beta around x axis
    }
    else {

      mt::Transform T_X0_Y0_Zvel;
      T_X0_Y0_Zvel.setRotation(mt::Rotation(0.0,0.0,0.0,1));
		//JAN : changed Dzeta to -Dzeta in next line 2011-03-26
      T_X0_Y0_Zvel.setTranslation(mt::Vector3(0,0,-Dzeta));
      
      //mt::Transform 
        Tbase=currTran*T_RzDalpha*T_X0_Y0_Zvel; // beta around x axis
    }

		/*BroncoPos[0] = (-dradio*cos(alpha)*cos(beta))+((length/2)*cos(alpha)*sin(beta))+(cos(alpha)*dradio);
		BroncoPos[1] = (-dradio*sin(alpha)*cos(beta))+((length/2)*sin(alpha)*sin(beta))+(sin(alpha)*dradio);
		BroncoPos[2] = (dradio*sin(beta))+((length/2)*cos(beta))-((length/2));
		BroncoRotMatrix[0][0] = cos(alpha)*cos(beta);	BroncoRotMatrix[0][1] = -sin(alpha);	BroncoRotMatrix[0][2] = cos(alpha)*sin(beta);
		BroncoRotMatrix[1][0] = sin(alpha)*cos(beta);	BroncoRotMatrix[1][1] = cos(alpha);		BroncoRotMatrix[1][2] = sin(alpha)*sin(beta);
		BroncoRotMatrix[2][0] = -sin(beta);				BroncoRotMatrix[2][1] = 0;				BroncoRotMatrix[2][2] = cos(beta);*/
				
		/*mt::Transform broncoTra;
		broncoTra.setRotation(mt::Rotation(BroncoRotMatrix));
		broncoTra.setTranslation(mt::Point3(BroncoPos[0],BroncoPos[1],BroncoPos[2]));*/
		
		/*RobConf* _CurrentPos = _robot->getCurrentPos();
		RnConf& _RnConf = _CurrentPos->getRn();
		SE3Conf& _SE3Conf = _CurrentPos->getSE3();

		vector<KthReal>& PosSE3 = _SE3Conf.getPos();
		vector<KthReal>& OriSE3 = _SE3Conf.getOrient();*/

    RnConf& _RnConf = _CurrentPos->getRn();
		vector<KthReal>& rncoor= _RnConf.getCoordinates();
		for(int i =0; i< rncoor.size(); i++)
			rncoor[i] = psi;

		/*mt::Transform currTran;
		currTran.setRotation(mt::Rotation(OriSE3[0], OriSE3[1], OriSE3[2], OriSE3[3]));
		currTran.setTranslation(mt::Point3(PosSE3[0], PosSE3[1], PosSE3[2]));*/

		//const mt::Transform result= currTran* broncoTra;

    // updating the _currentvalues variable with the new read values
		setvalues(values[0],0);
		setvalues(values[1],1);
		setvalues(values[2],2);		

		const mt::Point3& basePos = Tbase.getTranslationRef(); //resTra=result.getTranslationRef();

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

	bool ConsBronchoscopyKin::setParameters(){
		return true;
	}

  vector<float> ConsBronchoscopyKin::constrainedinterpolate(vector<float> coords, vector<KthReal> &values){
	  vector<KthReal> vecTmp;
	  _robot->control2Parameters(coords,vecTmp);
	  vector<KthReal> vecTmpSE3;
	  vecTmpSE3.resize(7);

	  for (int i=0; i<7; i++){
		  vecTmpSE3[i] = vecTmp[i];
	  }

    //vector<KthReal> currentcoords = _robot->deNormalizeSE3(vecTmpSE3);
	  //_currentConf.setSE3(currentcoords);
  			
	  RobConf _RobConf = solve(values);
	  _currentvalues[0] = 0.;
	  _currentvalues[1] = 0.;
	  _currentvalues[2] = 0.;
	  SE3Conf & _SE3Conf = _RobConf.getSE3();
	  RnConf & _RnConf = _RobConf.getRn();
	  vector<float>& coordSE3 = _SE3Conf.getCoordinates();
	  vector<float> Params = _SE3Conf.getParams();
	  vector<float>& coordRn = _RnConf.getCoordinates();
	  vector<float> coord;
	  coord.resize(7);

	  coord [0] = (coordSE3[0] / 1000) + 0.5;
	  coord [1] = (coordSE3[1] / 1000) + 0.5;
	  coord [2] = (coordSE3[2] / 1000) + 0.5;
	  coord [3] = Params[0];
	  coord [4] = Params[1];
	  coord [5] = Params[2] - 0.5;
	  coord [6] = values[1];
	  return coord;
  }

  vector<KthReal> ConsBronchoscopyKin::constrainedparameter2Pose(vector<KthReal> &values){
    std::vector<KthReal> coords(6);
    SE3Conf tmp;

    RobConf* _CurrentPos = _robot->getCurrentPos();
	  RnConf& _RnConf = _CurrentPos->getRn();
	  SE3Conf& _SE3Conf = _CurrentPos->getSE3();

	  vector<KthReal>& PosSE3 = _SE3Conf.getPos();
	  vector<KthReal>& OriSE3 = _SE3Conf.getOrient();

    mt::Point3 tempTran(PosSE3[0],PosSE3[1],PosSE3[2]);
    mt::Rotation tempRot(OriSE3[0],OriSE3[1],OriSE3[2],OriSE3[3]);

    //mt::Transform in_home, in_world;
    //in_home.setRotation(tempRot);
    //in_home.setTranslation(tempTran);

    //in_world = _homeTrans * in_home;      // Obtaining it in the world frame.
    //tempTran = in_world.getTranslation();
    //tempRot  = in_world.getRotation();

    coords.resize(7); // Resizing is needed to use quaternions
    coords[0] = tempTran[0];
    coords[1] = tempTran[1];
    coords[2] = tempTran[2];
    coords[3] = tempRot[0];
    coords[4] = tempRot[1];
    coords[5] = tempRot[2];
    coords[6] = tempRot[3];

    return coords;
  }

  void ConsBronchoscopyKin::SolveConstrainedKinematics(vector<KthReal> &values){
    vector<KthReal> vecTmp;
    //_robot->_hasChanged = true;
    _robot->control2Parameters(values,vecTmp);
    vector<KthReal> coords = constrainedparameter2Pose(vecTmp);
      _robot->getCurrentPos()->setSE3(coords);    
    RobConf _robconf = solve(values);
    _robot->Kinematics(_robconf);
  }
}
