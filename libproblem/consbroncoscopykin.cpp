#include "consbroncoscopykin.h"
#include "robot.h"
#include <mt/mt.h> 

namespace libProblem {
	
	ConsBroncoscopyKin::ConsBroncoscopyKin(Robot* const rob):ConstrainedKinematic(rob){
		
		_robot = rob;
		_robConf.setRn(rob->getNumJoints());	
		_currentvalues[0] = 0; 
		_currentvalues[1] = 0; 
		_currentvalues[2] = 0;

	}
	
	ConsBroncoscopyKin::~ConsBroncoscopyKin(){
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

	bool      ConsBroncoscopyKin::setParameters(){
		return true;
	}

}
