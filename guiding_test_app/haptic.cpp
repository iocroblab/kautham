/***************************************************************************
 *   Copyright (C) 2007 by Emmanuel Nuño                                   *
 *   emmanuel.nuno@upc.edu                                                 *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

#include "haptic.h"
#include <Windows.h>


namespace haptic
{

Haptic::Haptic(bool &init){

	m_phState=new HapticState;

	m_phState->phMotorTorque[0]=0;
	m_phState->phMotorTorque[1]=0;
	m_phState->phMotorTorque[2]=0;
	m_phState->phMotorTorque[3]=0;
	m_phState->phMotorTorque[4]=0;
	m_phState->phMotorTorque[5]=0;

	m_hHD = hdInitDevice(HD_DEFAULT_DEVICE);
    if (HD_DEVICE_ERROR(m_error = hdGetError())) 
    {
		std::cout<<"Failed to initialize haptic device"<<std::endl;
		init=false;
		m_init=init;
		return;
    }

	init=true;
	m_init=init;
	std::cout<<"Found Hapic: "<<hdGetString(HD_DEVICE_MODEL_TYPE)<<std::endl;

	hdEnable(HD_FORCE_OUTPUT);
}

bool Haptic::calibrate(){

	if (m_init==false) return false;

	int supportedCalibrationStyles;
    int calibrationStyle;

	std::cout<<"Starting Haptic Calibration"<<std::endl;

	hdGetIntegerv(HD_CALIBRATION_STYLE, &supportedCalibrationStyles);

	if (supportedCalibrationStyles & HD_CALIBRATION_ENCODER_RESET)
    {
        calibrationStyle = HD_CALIBRATION_ENCODER_RESET;
    }
    if (supportedCalibrationStyles & HD_CALIBRATION_INKWELL)
    {
        calibrationStyle = HD_CALIBRATION_INKWELL;
    }
    if (supportedCalibrationStyles & HD_CALIBRATION_AUTO)
    {
        calibrationStyle = HD_CALIBRATION_AUTO;
    }

	if (calibrationStyle == HD_CALIBRATION_ENCODER_RESET)
    {
		std::cout<<"Please prepare for manual calibration."<<std::endl;
		std::cout<<"Place the Haptic at reset positon and press Enter"<<std::endl;

		//std::cin.get();

        hdUpdateCalibration(calibrationStyle);
        if (hdCheckCalibration() == HD_CALIBRATION_OK)
        {
			std::cout<<"Calibration complete"<<std::endl;
        }
        if (HD_DEVICE_ERROR(m_error = hdGetError()))
        {
			std::cout<<"Reset encoders reset failed."<<std::endl;
            return false;           
        }
    }
	return true;
}

bool Haptic::start(){

	if (m_init==false) return false;

	hdScheduleAsynchronous(hdState, (void *)m_phState, HD_MAX_SCHEDULER_PRIORITY);
	

	hdStartScheduler();
	
    if (HD_DEVICE_ERROR(m_error = hdGetError()))
    {
		std::cout<<"Failed to start the scheduler"<<std::endl;
        return false;           
    }

	std::cout<<"Scheduler Started"<<std::endl;

#ifdef WIN32
	Sleep(15);
#elif
    sleep(15);
#endif
	return true;
}

bool Haptic::stop(){

	if (m_init==false) return false;

	hdStopScheduler();
    hdDisableDevice(m_hHD);

	std::cout<<"Haptic has been Stoped"<<std::endl;
	return true;
}

bool Haptic::getPositionTX(mt::Transform &trans)
{
	if (m_init==false) return false;
	
	mt::Matrix3x3 rotM(m_phState->phPos[0],   m_phState->phPos[4],  m_phState->phPos[8],
					   m_phState->phPos[1],   m_phState->phPos[5],  m_phState->phPos[9],
					   m_phState->phPos[2],   m_phState->phPos[6],  m_phState->phPos[10]);

	//rotM.setIdentity();
	mt::Matrix3x3 rz(0,1, 0,
		             -1, 0, 0,
					 0, 0, 1);

	mt::Matrix3x3 ry(0, 0,-1,
		             0, 1, 0,
					 1, 0, 0);


	mt::Matrix3x3 compose(0, 0, 1,
		                  1, 0, 0,
					      0, 1, 0);
	rotM=rotM*rz;
	rotM=rotM*ry; 

	rotM=compose*rotM;

	const mt::Rotation phRot(rotM);
	const mt::Vector3  T(m_phState->phPos[14],m_phState->phPos[12],m_phState->phPos[13]);
	//const mt::Vector3  T(0.0,0.0,m_phState->phPos[13]);
	trans.setRotation(phRot);
	trans.setTranslation(T);
	
	return true;
}

bool Haptic::getPosition(mt::Transform &trans)
{
	if (m_init==false) return false;
	
	mt::Matrix3x3 rotM(m_phState->phPos[0],   m_phState->phPos[4],  m_phState->phPos[8],
					   m_phState->phPos[1],   m_phState->phPos[5],  m_phState->phPos[9],
					   m_phState->phPos[2],   m_phState->phPos[6],  m_phState->phPos[10]);

	const mt::Rotation phRot(rotM);
	const mt::Vector3  T(m_phState->phPos[12],m_phState->phPos[13],m_phState->phPos[14]);
	trans.setRotation(phRot);
	trans.setTranslation(T);
	
	return true;
}



bool Haptic::getJointPosition(Vect6 &phJoints)
{
	if (m_init==false) return false;

	phJoints[0]=m_phState->phBaseJoints[0];
	phJoints[1]=m_phState->phBaseJoints[1];
	phJoints[2]=m_phState->phBaseJoints[2];

	phJoints[3]=m_phState->phGimbalJoints[0];
	phJoints[4]=m_phState->phGimbalJoints[1];
	phJoints[5]=m_phState->phGimbalJoints[2];

	return true;
}

bool Haptic::getVelocity(Vect6 &phVel)
{
	if (m_init==false) return false;

	phVel[0]=m_phState->phVel[2];
	phVel[1]=m_phState->phVel[0];
	phVel[2]=m_phState->phVel[1];
	phVel[3]=m_phState->phAngVel[0];
	phVel[4]=m_phState->phAngVel[1];
	phVel[5]=m_phState->phAngVel[2];

	return true;
}

bool Haptic::getButtom()
{
	if (m_init==false) return false;

	if (m_phState->phButton==1)	return true;

	return false;
}

bool Haptic::setForce(const Vect6 force)
{
	if (m_init==false) return false;

	m_phState->phForce[0]=mt::getValue(force[0]);
	m_phState->phForce[1]=mt::getValue(force[1]);
	m_phState->phForce[2]=mt::getValue(force[2]);

	m_phState->phTorqe[0]=mt::getValue(force[3]);
	m_phState->phTorqe[1]=mt::getValue(force[4]);
	m_phState->phTorqe[2]=mt::getValue(force[5]);

	return true;
}

bool Haptic::setMotorTorque(const Vect6 motorTorque)
{
	if (m_init==false) return false;

	m_phState->phMotorTorque[0]=mt::getValue(motorTorque[0]);
	m_phState->phMotorTorque[1]=mt::getValue(motorTorque[1]);
	m_phState->phMotorTorque[2]=mt::getValue(motorTorque[2]);
	m_phState->phMotorTorque[3]=mt::getValue(motorTorque[3]);
	m_phState->phMotorTorque[4]=mt::getValue(motorTorque[4]);
	m_phState->phMotorTorque[5]=mt::getValue(motorTorque[5]);

	return true;
}


bool Haptic::getJacobian(ublas::matrix<mt::Scalar> &j)
{
  Vect6 q(6);
  getJointPosition(q);

  const mt::Scalar l1 = 0.215;
  const mt::Scalar l2 = 0.170;

  const mt::Scalar s1 = sin(q[0]);
  const mt::Scalar s2 = sin(q[1]);
  const mt::Scalar s3 = sin(q[2]);
  const mt::Scalar s4 = sin(q[3]);
  const mt::Scalar s5 = sin(q[4]);
  

  const mt::Scalar c1 = cos(q[0]);
  const mt::Scalar c2 = cos(q[1]);
  const mt::Scalar c3 = cos(q[2]);
  const mt::Scalar c4 = cos(q[3]);
  const mt::Scalar c5 = cos(q[4]);


  j(0,0)= c1*(l1*c1 + l2*s3);
  j(1,0)= 0.0;
  j(2,0)= -s1*(l1*c2 + l2*s3);

  j(0,1)= s1*( -l1*s2 + l2*c3);
  j(1,1)= l1*c2 + l2*s3;
  j(2,1)= c1*(-l1*s2+l2*c3);

  j(0,2)= l2*s1*c3;
  j(1,2)= l2*s3;
  j(2,2)= l2*c1*c3;

  j(0,3)= 0.0;
  j(1,3)= 0.0;
  j(2,3)= 0.0;

  j(0,4)= 0.0;
  j(1,4)= 0.0;
  j(2,4)= 0.0;

  j(0,5)= 0.0;
  j(1,5)= 0.0;
  j(2,5)= 0.0;

  j(3,0)= 0.0;
  j(4,0)= 1.0;
  j(5,0)= 0.0;

  j(3,1)= -c1;
  j(4,1)= 0.0;
  j(5,1)= s1;

  j(3,2)= -c1;
  j(4,2)= 0.0;
  j(5,2)= s1;

  j(3,3)= s1*s3;
  j(4,3)= -c3;
  j(5,3)= c1*s3;

  j(3,4)= -s1*c3*s4 - c1*c4;
  j(4,4)= -s3*s4;
  j(5,4)= -c1*c3*s4 + s1*c4;

  j(3,5)= s1*c3*c4*c5 - c1*s4*c5 - s1*s3*s5;
  j(4,5)= s3*c4*c5 + c3*s5;
  j(5,5)= c1*c3*c4*c5 + s1*s4*c5 - c1*s3*s5;

  return true;
}

bool Haptic::getJacobianTranspose(ublas::matrix<mt::Scalar> &j)
{
  Vect6 q(6);
  getJointPosition(q);

  const mt::Scalar l1 = 0.215;
  const mt::Scalar l2 = 0.170;

  const mt::Scalar s1 = sin(q[0]);
  const mt::Scalar s2 = sin(q[1]);
  const mt::Scalar s3 = sin(q[2]);
  const mt::Scalar s4 = sin(q[3]);
  const mt::Scalar s5 = sin(q[4]);
  

  const mt::Scalar c1 = cos(q[0]);
  const mt::Scalar c2 = cos(q[1]);
  const mt::Scalar c3 = cos(q[2]);
  const mt::Scalar c4 = cos(q[3]);
  const mt::Scalar c5 = cos(q[4]);


  j(0,0)= c1*(l1*c1 + l2*s3);
  j(0,1)= 0.0;
  j(0,2)= -s1*(l1*c2 + l2*s3);

  j(1,0)= s1*( -l1*s2 + l2*c3);
  j(1,1)= l1*c2 + l2*s3;
  j(1,2)= c1*(-l1*s2+l2*c3);

  j(2,0)= l2*s1*c3;
  j(2,1)= l2*s3;
  j(2,2)= l2*c1*c3;

  j(3,0)= 0.0;
  j(3,1)= 0.0;
  j(3,2)= 0.0;

  j(4,0)= 0.0;
  j(4,1)= 0.0;
  j(4,2)= 0.0;

  j(5,0)= 0.0;
  j(5,1)= 0.0;
  j(5,2)= 0.0;

  j(0,3)= 0.0;
  j(0,4)= 1.0;
  j(0,5)= 0.0;

  j(1,3)= -c1;
  j(1,4)= 0.0;
  j(1,5)= s1;

  j(2,3)= -c1;
  j(2,4)= 0.0;
  j(2,5)= s1;

  j(3,3)= s1*s3;
  j(3,4)= -c3;
  j(3,5)= c1*s3;

  j(4,3)= -s1*c3*s4 - c1*c4;
  j(4,4)= -s3*s4;
  j(4,5)= -c1*c3*s4 + s1*c4;

  j(5,3)= s1*c3*c4*c5 - c1*s4*c5 - s1*s3*s5;
  j(5,4)= s3*c4*c5 + c3*s5;
  j(5,5)= c1*c3*c4*c5 + s1*s4*c5 - c1*s3*s5;

  return true;
}

HDCallbackCode HDCALLBACK hdState(void *pState)
{
	HapticState *phState=(HapticState *)pState;
    hdBeginFrame(hdGetCurrentDevice());
	hdGetIntegerv(HD_CURRENT_BUTTONS, &phState->phButton);
	hdGetDoublev(HD_CURRENT_TRANSFORM, phState->phPos);
	hdGetDoublev(HD_CURRENT_JOINT_ANGLES, phState->phBaseJoints);
	hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, phState->phGimbalJoints);
	hdGetDoublev(HD_CURRENT_VELOCITY, phState->phVel);
	hdGetDoublev(HD_CURRENT_ANGULAR_VELOCITY, phState->phAngVel);
	hdSetDoublev(HD_CURRENT_FORCE, phState->phForce);
	hdSetDoublev(HD_CURRENT_TORQUE, phState->phTorqe);
	//hdSetLongv(HD_CURRENT_MOTOR_DAC_VALUES, phState->phMotorTorque);
    hdEndFrame(hdGetCurrentDevice());

//Execute this for ever. Use HD_CALLBACK_ONCE for one time check
    return HD_CALLBACK_CONTINUE;
}


}
