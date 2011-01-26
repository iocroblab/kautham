#include "consbroncoscopykin.h"
#include "robot.h"
#include <mt/mt.h> 

namespace libProblem {
	
	ConsBroncoscopyKin::ConsBroncoscopyKin(Robot* const rob):ConstrainedKinematic(rob){
		
		_robot = rob;
		_robConf.setRn(_robot->getNumJoints());	
		_currentvalues[0] = 0.; 
		_currentvalues[1] = 0.; 
		_currentvalues[2] = 0.;

	}
	
	ConsBroncoscopyKin::~ConsBroncoscopyKin(){
	}

  bool ConsBroncoscopyKin::solve(){
    _robConf = solve(_target);
    return true;
  }

	RobConf ConsBroncoscopyKin::solve(vector<KthReal> &values){

		vector<KthReal> coords(7);
		Link* _linkn = _robot->getLink(2);
						
		KthReal currentvalues0 = getvalues(0);
		KthReal currentvalues1 = getvalues(1);
		KthReal currentvalues2 = getvalues(2);
		KthReal length = _linkn->getA();
		KthReal alpha = 2* M_PI *(values[0]-currentvalues0);
		KthReal theta = M_PI*(values[1])/4;
		KthReal zeta = -1000*(values[2]-currentvalues2);		
		theta = theta ==0?  theta+0.001 : theta;
		KthReal beta = (2*zeta*tan(theta/2))/length;
		KthReal dradio = -length/(2*tan(theta/2));		
		mt::Matrix3x3 BroncoRotMatrix;
		mt::Point3    BroncoPos;
		
		BroncoPos[0] = (-dradio*cos(alpha)*cos(beta))+((length/2)*cos(alpha)*sin(beta))+(cos(alpha)*dradio);
		BroncoPos[1] = (-dradio*sin(alpha)*cos(beta))+((length/2)*sin(alpha)*sin(beta))+(sin(alpha)*dradio);
		BroncoPos[2] = (dradio*sin(beta))+((length/2)*cos(beta))-((length/2));
		BroncoRotMatrix[0][0] = cos(alpha)*cos(beta);	BroncoRotMatrix[0][1] = -sin(alpha);	BroncoRotMatrix[0][2] = cos(alpha)*sin(beta);
		BroncoRotMatrix[1][0] = sin(alpha)*cos(beta);	BroncoRotMatrix[1][1] = cos(alpha);		BroncoRotMatrix[1][2] = sin(alpha)*sin(beta);
		BroncoRotMatrix[2][0] = -sin(beta);				BroncoRotMatrix[2][1] = 0;				BroncoRotMatrix[2][2] = cos(beta);
				
		mt::Transform broncoTra;
		broncoTra.setRotation(mt::Rotation(BroncoRotMatrix));
		broncoTra.setTranslation(mt::Point3(BroncoPos[0],BroncoPos[1],BroncoPos[2]));
		
		RobConf* _CurrentPos = _robot->getCurrentPos();
		RnConf& _RnConf = _CurrentPos->getRn();
		SE3Conf& _SE3Conf = _CurrentPos->getSE3();

		vector<KthReal>& PosSE3 = _SE3Conf.getPos();
		vector<KthReal>& OriSE3 = _SE3Conf.getOrient();

		vector<KthReal>& rncoor= _RnConf.getCoordinates();
		for(int i =0; i< rncoor.size(); i++)
			rncoor[i] = theta;

		mt::Transform currTran;
		currTran.setRotation(mt::Rotation(OriSE3[0], OriSE3[1], OriSE3[2], OriSE3[3]));
		currTran.setTranslation(mt::Point3(PosSE3[0], PosSE3[1], PosSE3[2]));

		const mt::Transform result= currTran* broncoTra;

		setvalues(values[0],0);
		setvalues(values[1],1);
		setvalues(values[2],2);		

		const mt::Point3& resTra = result.getTranslationRef();

		for(int i =0; i < 3; i++)
			coords[i] = resTra[i];

		const mt::Rotation& resRot = result.getRotationRef();

		for(int i =0; i < 4; i++)
			coords[i+3] = resRot[i];

			_robConf.setSE3(coords);
			_robConf.setRn(_RnConf);
			//_robot->Kinematics(_robConf);
		
		return _robConf;

	}

	bool ConsBroncoscopyKin::setParameters(){
		return true;
	}

  vector<float> ConsBroncoscopyKin::constrainedinterpolate(vector<float> coords, vector<KthReal> &values){
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

  vector<KthReal> ConsBroncoscopyKin::constrainedparameter2Pose(vector<KthReal> &values){
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

  void ConsBroncoscopyKin::SolveConstrainedKinematics(vector<KthReal> &values){
    vector<KthReal> vecTmp;
    //_robot->_hasChanged = true;
    _robot->control2Parameters(values,vecTmp);
    vector<KthReal> coords = constrainedparameter2Pose(vecTmp);
      _robot->getCurrentPos()->setSE3(coords);    
    RobConf _robconf = solve(values);
    _robot->Kinematics(_robconf);
  }
}
