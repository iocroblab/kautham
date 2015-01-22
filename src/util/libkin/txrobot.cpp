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

/* Author: Leopold Palomo, Ignacio Flores, Adolfo Rodriguez and Emmanuel Nuño               */


/////////////////////////////// PREPROCESSOR DIRECTIVES //////////////////////

// MT LIBRARY HEADERS
#include <mt/matrix3x3.h>
#include <mt/rotation.h>
#include <cmath>


// LOCAL HEADERS
#include "txrobot.h"
using namespace std;



/////////////////////////////// NAMESPACE SCOPE //////////////////////////////

namespace TXrobot
{

/////////////////////////////// CLASS IMPLEMENTATION /////////////////////////
//============================= PUBLIC =======================================

TXRobot::TXRobot(TXtype &robot){
    
    //default configuration for all robots
    TXconfig.sh = slefty;
    TXconfig.el = epositive;
    TXconfig.wr = wpositive;

    TXtolerance = 1e-5;

//TX40, TX60, TX60L, TX90, TX90L, TX90XL

    switch (robot)
    {
         case TX40:
            exit(0);break;

         case TX60:
            exit(0);break;

        case TX60L:
            exit(0);break;

         case TX90:
            a1 = 50;
            a2 = 425;
            a3 = 50;
            a4 = 425;
            a5 = 0.0;
            a6 = 100;   //Distance from wrist center to tool
	//		a6= 0.0;

			aJ5 = a6;

            //limitations of the joint 1 in rads
            TXlimits[0].range.setValue(mt::degToRad(-180.), mt::degToRad(180.));
            TXlimits[0].speed = mt::degToRad(400.); //in rads/s
            
            //limitations of the joint 2 in rads
            TXlimits[1].range.setValue(mt::degToRad(-130.), mt::degToRad(147.5));
            TXlimits[1].speed = mt::degToRad(400.); //in rads/s
            
            //limitations of the joint 3 in rads
            TXlimits[2].range.setValue(mt::degToRad(-145.), mt::degToRad(145.));
            TXlimits[2].speed = mt::degToRad(400.); //in rads/s
            
            //limitations of the joint 4 in rads
            TXlimits[3].range.setValue(mt::degToRad(-270.), mt::degToRad(270.));
            TXlimits[3].speed = mt::degToRad(400.); //in rads/s
            
            //limitations of the joint 5 in rads
            TXlimits[4].range.setValue(mt::degToRad(-115.), mt::degToRad(140.));
            TXlimits[4].speed = mt::degToRad(400.); //in rads/s
            
            //limitations of the joint 6 in rads
            TXlimits[5].range.setValue(mt::degToRad(-270.), mt::degToRad(270.));
            TXlimits[5].speed = mt::degToRad(400.); //in rads/s
            
            break;

         case TX90L:
            exit(0);break;

          case TX90XL:
            exit(0);break;
    }
}

TXerror TXRobot::fwdKin(const Vect6& q, mt::Transform &Pos)
{
  TXerror error = SUCCESS;
  unsigned int n = 0;
  
  while (error == SUCCESS && n < 6) 
  {
      if(!mt::isContained(q[n], TXlimits[n].range))
      {
        if(n==0)
          error = ERROR_J1;
        else if(n==1)
          error = ERROR_J2;
        else if(n==2)
          error = ERROR_J3;
        else if(n==3)
          error = ERROR_J4;
        else if(n==4)
          error = ERROR_J5;
        else
          error = ERROR_J6;

        return error;
      }
      n++;
  }
  // Trascendental function evaluation
  const mt::Scalar s1 = sin(q[0]);
  const mt::Scalar s2 = sin(q[1]);
  const mt::Scalar s3 = sin(q[2]);
  const mt::Scalar s4 = sin(q[3]);
  const mt::Scalar s5 = sin(q[4]);
  const mt::Scalar s6 = sin(q[5]);

  const mt::Scalar c1 = cos(q[0]);
  const mt::Scalar c2 = cos(q[1]);
  const mt::Scalar c3 = cos(q[2]);
  const mt::Scalar c4 = cos(q[3]);
  const mt::Scalar c5 = cos(q[4]);
  const mt::Scalar c6 = cos(q[5]);

  // Translational part of transformation
  const mt::Scalar x = a6 * s5 * c1 * c2 * c3 * c4 -
                   a6 * s5 * s1 * s4 -
                   a6 * s5 * c1 * s2 * s3 * c4 +
                   a6 * c5 * c1 * c2 * s3 +
                   a6 * c5 * c1 * s2 * c3 +
                   a2 * c1 * c2 * s3 +
                   a2 * c1 * s2 * c3 -
                   a1 * s1 +
                   a2 * c1 * s2 +
                   a1 * c1;

  const mt::Scalar y = a6 * s5 * s1 * c2 * c3 * c4 +
                   a6 * s5 * c1 * s4 -
                   a6 * s5 * s1 * s2 * s3 * c4 +
                   a6 * c5 * s1 * c2 * s3 +
                   a6 * c5 * s1 * s2 * c3 +
                   a2 * s1 * c2 * s3 +
                   a2 * s1 * s2 * c3 +
                   a1 * c1 +
                   a2 * s1 * s2 +
                   a1 * s1;

  const mt::Scalar z =-a6 * s5 * s2 * c3 * c4 -
                   a6 * s5 * c2 * s3 * c4 -
                   a6 * c5 * s2 * s3 +
                   a6 * c5 * c2 * c3 -
                   a2 * s2 * s3 +
                   a2 * c2 * c3 +
                   a2 * c2;

  

  // Rotational part of transformation
  const mt::Scalar r11 = c6 * c5 * c1 * c2 * c3 * c4 -
                     c6 * c5 * s1 * s4 -
                     c6 * c5 * c1 * s2 * s3 * c4 -
                     c6 * s5 * c1 * c2 * s3 -
                     c6 * s5 * c1 * s2 * c3 -
                     s6 * c1 * c2 * c3 * s4 -
                     s6 * s1 * c4 +
                     s6 * c1 * s2 * s3 * s4;

  const mt::Scalar r12 =-s6 * c5 * c1 * c2 * c3 * c4 +
                     s6 * c5 * s1 * s4 +
                     s6 * c5 * c1 * s2 * s3 * c4 +
                     s6 * s5 * c1 * c2 * s3 +
                     s6 * s5 * c1 * s2 * c3 -
                     c6 * c1 * c2 * c3 * s4 -
                     c6 * s1 * c4 +
                     c6 * c1 * s2 * s3 * s4;

  const mt::Scalar r13 = s5 * c1 * c2 * c3 * c4 -
                     s5 * s1 * s4 -
                     s5 * c1 * s2 * s3 * c4 +
                     c5 * c1 * c2 * s3 +
                     c5 * c1 * s2 * c3;

  const mt::Scalar r21 = c6 * c5 * s1 * c2 * c3 * c4 +
                     c6 * c5 * c1 * s4 -
                     c6 * c5 * s1 * s2 * s3 * c4 -
                     c6 * s5 * s1 * c2 * s3 -
                     c6 * s5 * s1 * s2 * c3 -
                     s6 * s1 * c2 * c3 * s4 +
                     s6 * c1 * c4 +
                     s6 * s1 * s2 * s3 * s4;

  const mt::Scalar r22 =-s6 * c5 * s1 * c2 * c3 * c4 -
                     s6 * c5 * c1 * s4 +
                     s6 * c5 * s1 * s2 * s3 * c4 +
                     s6 * s5 * s1 * c2 * s3 +
                     s6 * s5 * s1 * s2 * c3 -
                     c6 * s1 * c2 * c3 * s4 +
                     c6 * c1 * c4 +
                     c6 * s1 * s2 * s3 * s4;

  const mt::Scalar r23 = s5 * s1 * c2 * c3 * c4 +
                     s5 * c1 * s4 -
                     s5 * s1 * s2 * s3 * c4 +
                     c5 * s1 * c2 * s3 +
                     c5 * s1 * s2 * c3;

  const mt::Scalar r31 =-c6 * c5 * s2 * c3 * c4 -
                     c6 * c5 * c2 * s3 * c4 +
                     c6 * s5 * s2 * s3 -
                     c6 * s5 * c2 * c3 +
                     s6 * s2 * c3 * s4 +
                     s6 * c2 * s3 * s4;

  const mt::Scalar r32 = s6 * c5 * s2 * c3 * c4 +
                     s6 * c5 * c2 * s3 * c4 -
                     s6 * s5 * s2 * s3 +
                     s6 * s5 * c2 * c3 +
                     c6 * s2 * c3 * s4 +
                     c6 * c2 * s3 * s4;

  const mt::Scalar r33 =-s5 * s2 * c3 * c4 -
                     s5 * c2 * s3 * c4 -
                     c5 * s2 * s3 +
                     c5 * c2 * c3;

  const mt::Matrix3x3 M (r11, r12, r13,
                     r21, r22, r23,
                     r31, r32, r33);

  const mt::Rotation R(M);

  // Rigid transformation of robot end-effector
  
  Pos.setRotation(R);
  Pos.setTranslation(mt::Point3(x,y,z));

  return SUCCESS;
}


/// Computes inverse kinematics from Cartesian pos values
/// (expressed in meters) and a configuration. The result goes to 
/// Vect6 (radians). The function returns an error code.

TXerror TXRobot::invKin(const mt::Transform& p, Vect6& q, const config& conf, Vect6 qant)
{
	mt::Transform target;
    target.setIdentity();
    target.setTranslation(mt::Point3(0.,0.,a6));

    const mt::Transform wc = p * target.inverse();
    const mt::Vector3 pHwc = wc.getTranslation();


	// Solve for theta(1)
	const mt::Scalar d = mt::sqrt(mt::sq(pHwc[0]) + mt::sq(pHwc[1]));
    const mt::Scalar d2 = mt::sq(d);

    //must test if d*d < a3²
    mt::Scalar tt = d2 - mt::sq(a3);
    
    if(tt <  TXtolerance){
       #ifdef VERBOSE
        std::cout<<"No solution found: fails theta1"<<std::endl;
       #endif
       return ERROR_J1;
    }
      
	mt::Scalar r=mt::sqrt(tt);

	mt::Scalar sj1,cj1;
	mt::Scalar sj2,cj2;
	mt::Scalar sj21,cj21;
	mt::Scalar sj3,cj3;
	mt::Scalar sj31,cj31;

	if (d>a3)
	{
    if(conf.sh==slefty)
		{
			sj1=(pHwc[1] * r - pHwc[0] * a3)/d2;
			cj1=(pHwc[0] * r + pHwc[1] * a3)/d2;
	    }
		else
		{
			sj1=(-pHwc[1] * r - pHwc[0] * a3)/d2;
			cj1=(-pHwc[0] * r + pHwc[1] * a3)/d2;
		}

        q[0]= atan2(sj1,cj1);
        if(!mt::isContained(q[0], TXlimits[0].range)){
          #ifdef VERBOSE
            std::cout<<"No solution found: fails theta1"<<std::endl;
          #endif
          return ERROR_J1;
        }
	}
	else
	{
		if (fabs(mt::getValue(d-a3)) < mt::getValue(TXtolerance))
		{
			//Only one solution for theta1
			q[0]= atan2(-pHwc[0],pHwc[1]);
            if(!mt::isContained(q[0], TXlimits[0].range)){
                #ifdef VERBOSE
                   std::cout<<"No solution found: fails theta1"<<std::endl;
                #endif
                return ERROR_J1;
            }
		}
		else //no solution for theta1
		{    
            #ifdef VERBOSE
       			std::cout<<"No solution found: fails theta1"<<std::endl;
            #endif
            return ERROR_J1;
		}
	}

 
    //Solving for theta2 and theta3
    
  if (conf.sh == srighty) r = -r;
	
	const mt::Scalar rho = mt::sqrt(mt::sq(pHwc[2]) + mt::sq(r-a1));
    const mt::Scalar semip = (a2 + a4 + rho)*0.5;
    const mt::Scalar gamma = a2 + a4;
    
    //temp = semip / (semip -a4);
	const mt::Scalar t1=mt::sqrt(((semip-a2)*(semip-rho))/(semip*(semip-a4)));
	const mt::Scalar t2=mt::sqrt(((semip-a4)*(semip-rho))/(semip*(semip-a2)));

    if (mt::getValue(rho) > mt::getValue(gamma) || mt::getValue(rho) < fabs(mt::getValue(a2-a4)))
	{
        #ifdef VERBOSE
          std::cout<<"No solution found: fails theta2"<<std::endl;
        #endif
        return ERROR_J2;
	}
    else
	{
        const mt::Scalar tmp1 = 1.0 + mt::sq(t1);
        //(rho*(mt::sq(t1) + 1.0)
        const mt::Scalar tmp2 = rho * tmp1;
        
        const mt::Scalar tmp4 = 1.0 + mt::sq(t2);
        
        //if (rho == gamma)
        if(fabs(mt::getValue(rho-gamma)) < mt::getValue(TXtolerance))
		{
			sj2=(r-a1)/rho;
			cj2=pHwc[2]/rho;
			q[1]=atan2(sj2,cj2);
			q[2]=0.;
            if(!mt::isContained(q[1], TXlimits[1].range)){
              #ifdef VERBOSE
                  std::cout<<"No solution found: fails theta2"<<std::endl;
              #endif 
              return ERROR_J2;
            }
            if(!mt::isContained(q[2], TXlimits[2].range)){
              #ifdef VERBOSE
                std::cout<<"No solution found: fails theta3"<<std::endl;
              #endif
              return ERROR_J3;
            }
		}
        else if (fabs(mt::getValue(rho)) < mt::getValue(TXtolerance))
		{
            sj2 = ((r - a1)*(1.0 - mt::sq(t1)) - 2.0 * pHwc[2] * t1)/tmp2;
            cj2 = (pHwc[2]*(1.0 - mt::sq(t1))+ 2.0 * (r-a1)*t1) / tmp2;
            q[1] = atan2(sj2,cj2);
            q[2] = -mt::PI;
            if(!mt::isContained(q[1], TXlimits[1].range)){
              #ifdef VERBOSE
                std::cout<<"No solution found: fails theta2"<<std::endl;
              #endif 
              return ERROR_J2;
            }
            if(!mt::isContained(q[2], TXlimits[2].range)){
              #ifdef VERBOSE
                std::cout<<"No solution found: fails theta2"<<std::endl;
              #endif 
              return ERROR_J2;
            }
		}

       	if (conf.el == epositive)
		{
     		sj2= ((r-a1)*(1.0 - mt::sq(t1))- 2.0 * pHwc[2] * t1)/tmp2;
     		cj2 = (pHwc[2]*(1.0 - mt::sq(t1)) + 2.0*t1*(r-a1))/tmp2;
     		sj3 = 2.0*(t1*(1.0 - mt::sq(t2)) + (1.0 - mt::sq(t1)) * t2)/(tmp4*tmp1);
     		cj3 = ((1.0 - mt::sq(t1))*(1.0 - mt::sq(t2)) - 4.0 * t1 * t2)/(tmp4*tmp1);
     		q[1]=atan2(sj2,cj2);
     		q[2]=atan2(sj3,cj3);
   	        if(!mt::isContained(q[1], TXlimits[1].range)){
             #ifdef VERBOSE
               std::cout<<"No solution found: fails theta2"<<std::endl;
             #endif
             return ERROR_J2;
            }
            if(!mt::isContained(q[2], TXlimits[2].range)){
              #ifdef VERBOSE
                std::cout<<"No solution found: fails theta2"<<std::endl;
              #endif 
              return ERROR_J2;
            }//disp('elbow positive')
		}
        else
        {
     		sj21=((r-a1)*(1.0 - mt::sq(t1)) + 2.0*pHwc[2]*t1) / tmp2;
     		cj21=(pHwc[2]*(1.0 - mt::sq(t1)) - 2.0 * (r-a1) * t1) / tmp2;
     		sj31=-2.0 * (t1 * (1.0 - mt::sq(t2)) + t2 * (1.0 - mt::sq(t1))) / (tmp4 * tmp1);
     		cj31=((1.0 - mt::sq(t1))* (1.0 - mt::sq(t2)) - 4.0 * t1*t2)/ (tmp4 * tmp1);
     		q[1]=atan2(sj21,cj21);
     		q[2]=atan2(sj31,cj31);
 			if(!mt::isContained(q[1], TXlimits[1].range)){
              #ifdef VERBOSE
                std::cout<<"No solution found: fails theta2"<<std::endl;
              #endif
              return ERROR_J2;
            }
            if(!mt::isContained(q[2], TXlimits[2].range)){
              #ifdef VERBOSE
                std::cout<<"No solution found: fails theta3"<<std::endl;
              #endif
              return ERROR_J3;
            }//disp('elbow negative')
		}
	}



    const mt::Rotation rotz(mt::Unit3(0,0,1),q[0]), roty(mt::Unit3(0,1,0),q[1]), rotyy(mt::Unit3(0,1,0),q[2]);
    const mt::Rotation rb=rotz * roty * rotyy;

	mt::Matrix3x3 r36 = rb.getMatrix().transpose() * p.getRotation().getMatrix();
    
    if(conf.wr == wpositive)
		q[4] = atan2(mt::sqrt(mt::sq(r36[2][0]) + mt::sq(r36[2][1])),r36[2][2]); 
    else 
		q[4] = -atan2(mt::sqrt(mt::sq(r36[2][0]) + mt::sq(r36[2][1])),r36[2][2]);
    
    if(!mt::isContained(q[4], TXlimits[4].range)){
      #ifdef VERBOSE
        std::cout<<"No solution found: fails theta5"<<std::endl;
      #endif  
      return ERROR_J5;
    }

	//Changes for rotations on 30/07/2008 by Adolfo, Leo and Emmanuel
	if(fabs(mt::getValue(q[4]-mt::PI)) < mt::getValue(TXtolerance))
    {
        //be carefull. Tolerance, then q[4] must be PI
        q[3] = qant[3];
        q[5]= q[3] + atan2(r36[0][1],-r36[0][0]);

		//Applying the first filter, the proximity filter
		setClosestAngle(q[5],qant[5]);
        
        if(!mt::isContained(q[3], TXlimits[3].range)){
          #ifdef VERBOSE
            std::cout<<"No solution found: fails theta4"<<std::endl;
          #endif
          return ERROR_J4;
        }
        if(!mt::isContained(q[5], TXlimits[5].range)){
          #ifdef VERBOSE
            std::cout<<"No solution found: fails theta6"<<std::endl;
          #endif
          return ERROR_J6;
        }
	}
	else if (fabs(mt::getValue(q[4])) < 0.065)
    {
		//std::cout<<q[4]<<std::endl;

		//q[3] = qant[3];
		//q[5] = atan2(-r36[0][1],r36[0][0])-q[3];
		
        q[3] = qant[3];
        q[5] = 0.5*(atan2(-r36[0][1],r36[0][0]) - qant[3]);
		q[3] = 0.5*atan2(-r36[0][1],r36[0][0]) + 0.5*qant[3];


		//Applying the first filter, the proximity filter
		setClosestAngle(q[3],qant[3]);
		setClosestAngle(q[5],qant[5]);

        if(!mt::isContained(q[3], TXlimits[3].range)){
          #ifdef VERBOSE
            std::cout<<"No solution found: fails theta4"<<std::endl;
          #endif
          return ERROR_J4;
        }
        if(!mt::isContained(q[5], TXlimits[5].range)){
          #ifdef VERBOSE
            std::cout<<"No solution found: fails theta6"<<std::endl;
          #endif
          return ERROR_J6;
        }//disp('elbow negative')

	}
	//Nonsingular case
	else 
    {
        if(fabs(mt::getValue(r36[1][2])) < mt::getValue(TXtolerance))
        {   
            r36[1][2]= 0.0;
        } 

		const mt::Scalar s4=sin(q[4]);
         
		q[3] = atan2(r36[1][2]/s4,r36[0][2]/s4);

		//Applying the first filter, the proximity filter
		setClosestAngle(q[3],qant[3]);
		

        if(!mt::isContained(q[3], TXlimits[3].range)){
          #ifdef VERBOSE
            std::cout<<"No solution found: fails theta4"<<std::endl;
          #endif
          return ERROR_J4;
        }

		q[5] = atan2(r36[2][1]/s4,-r36[2][0]/s4);
		
		//Applying the first filter, the proximity filter
		setClosestAngle(q[5],qant[5]);

        if(fabs(fabs(mt::getValue(q[5])) - mt::getValue(mt::PI)) <  mt::getValue(TXtolerance))
          q[5] = mt::PI;
        
        if(!mt::isContained(q[5], TXlimits[5].range)){
          #ifdef VERBOSE
            std::cout<<"No solution found: fails theta6"<<std::endl;
          #endif
          return ERROR_J6;
        }
	}

	//std::cout<<conf.wr<<" q5 "<<q[4]<<std::endl;

    return SUCCESS;
}

/// Computes inverse kinematics from Cartesian pos values
/// (expressed in milimeters), the current configuration of the robot and the last
/// position of a path (Vect6 (radians)
/// The result goes to Vect6 (radians). The function returns an error code.
TXerror TXRobot::invKin(const mt::Transform& p, Vect6& qNew, const Vect6& qOld, const config& conf)
{
  static config confIn;
  static bool first=true;

  Vect6 qOpt(6);
  mt::Scalar auxOpt, auxNew;

  
  //Just the first time invKin is called
  if (first)
  {
	  confIn=conf;
	  first=false;
  }

  TXerror result=invKin(p, qNew, confIn, qOld);

  

  if ((fabs(mt::getValue(qNew[5]-qOld[5])) > (5*3.1416/180) || fabs(mt::getValue(qNew[3]-qOld[3])) > (5*3.1416/180)) )
  {
	  //std::cout<<std::endl<<"previous q4: "<<qNew[3]<<"  "<<qOld[3]<<std::endl;
	  //std::cout<<"previous q5: "<<qNew[4]<<"  "<<qOld[4]<<std::endl;
	  //std::cout<<"previous q6: "<<qNew[5]<<"  "<<qOld[5]<<std::endl;

	  if (confIn.wr==wpositive)
	  {
	   confIn.wr=wnegative;
	   result=invKin(p, qOpt, confIn, qOld);
	   //std::cout<<"opt n q4: "<<qOpt[3]<<"  "<<qOld[3]<<std::endl;
	   //std::cout<<"opt n q6: "<<qOpt[5]<<"  "<<qOld[5]<<std::endl;
	  }
	  else if (confIn.wr==wnegative)
	  {
	   confIn.wr=wpositive;
	   result=invKin(p, qOpt, confIn, qOld);
	  // std::cout<<"opt p q4: "<<qOpt[3]<<"  "<<qOld[3]<<std::endl;
	  // std::cout<<"opt p q6: "<<qOpt[5]<<"  "<<qOld[5]<<std::endl;
	  }

  

	if (fabs(mt::getValue(qOpt[3]-qOld[3])) > fabs(mt::getValue(qOpt[5]-qOld[5])))
	 auxOpt=fabs(mt::getValue(qOpt[3]-qOld[3]));
	else auxOpt=fabs(mt::getValue(qOpt[5]-qOld[5]));

	if (fabs(mt::getValue(qNew[3]-qOld[3])) > fabs(mt::getValue(qNew[5]-qOld[5])))
	 auxNew=fabs(mt::getValue(qNew[3]-qOld[3]));
	else auxNew=fabs(mt::getValue(qNew[5]-qOld[5]));


	if (mt::getValue(auxNew-auxOpt)>0.1) qNew=qOpt;
	else if (confIn.wr==wpositive) confIn.wr=wnegative;
	else confIn.wr=wpositive;

	

	//std::cout<<std::endl<<"taken q4: "<<qNew[3]<<"  "<<qOld[3]<<std::endl;
	//std::cout<<"taken q5: "<<qNew[4]<<"  "<<qOld[4]<<std::endl;
	//std::cout<<"taken q6: "<<qNew[5]<<"  "<<qOld[5]<<std::endl;

	}

  return result;

}

//Function to place a new angle as close as possible with the previous
TXerror TXRobot::setClosestAngle(mt::Scalar &q, const mt::Scalar &qAnt)
{
	mt::Scalar tmp; 
	if (mt::getValue(q) > 0.0) 
		tmp = q - mt::TWO_PI; 
    else 
		tmp = mt::TWO_PI + q;

	if(fabs(mt::getValue(tmp - qAnt)) < fabs(mt::getValue(q - qAnt)))
          q = tmp;
	
	return SUCCESS;
}


/// Computes inverse kinematics from Cartesian pos values
/// (expressed in milimeters), and after it crashes your program
// TXerror TXRobot::invKin(const mt::Transform& p,  Vect6& q , int  foo,  const config& conf)
// {
//    TXerror result=  invKin(p, q, conf);
// 
//     double *crash = new double[10];
//     crash[100] = 3.5; 
//     return result; //;-), just to not show in the warning
// }

TXerror TXRobot::invKin(const mt::Transform& p,  Vect6& q , const config& preferred,
                        config& solution, const Vect6& qNear){
 TXerror result;
 typedef std::pair<config,Vect6> candidate;
 std::vector<candidate> candiSet;
 config tmpConfig;

 //result = invKin(p,q,preferred);
 //if(result == SUCCESS ){
 //   solution.set(preferred.sh, preferred.el, preferred.wr);
 //}else{
   tmpConfig.set(srighty,epositive,wpositive);
   if(tmpConfig.matchMask(solution)){
     result = invKin(p,q,tmpConfig);
     if(result == SUCCESS){
        candidate cand1(tmpConfig,q);
        candiSet.push_back(cand1);
     }
   }
   tmpConfig.set(srighty,epositive,wnegative);
   if(tmpConfig.matchMask(solution)){
     result = invKin(p,q,tmpConfig);
     if(result == SUCCESS){
        candidate cand1(tmpConfig,q);
        candiSet.push_back(cand1);
     }
   }
   tmpConfig.set(srighty,enegative,wpositive);
   if(tmpConfig.matchMask(solution)){
     result = invKin(p,q,tmpConfig);
     if(result == SUCCESS){
        candidate cand1(tmpConfig,q);
        candiSet.push_back(cand1);
     }
   }
   tmpConfig.set(srighty,enegative,wnegative);
   if(tmpConfig.matchMask(solution)){
     result = invKin(p,q,tmpConfig);
     if(result == SUCCESS){
        candidate cand1(tmpConfig,q);
        candiSet.push_back(cand1);
     }
   }
   tmpConfig.set(slefty,epositive,wpositive);
   if(tmpConfig.matchMask(solution)){
     result = invKin(p,q,tmpConfig);
     if(result == SUCCESS){
        candidate cand1(tmpConfig,q);
        candiSet.push_back(cand1);
     }
   }
   tmpConfig.set(slefty,epositive,wnegative);
   if(tmpConfig.matchMask(solution)){
     result = invKin(p,q,tmpConfig);
     if(result == SUCCESS){
        candidate cand1(tmpConfig,q);
        candiSet.push_back(cand1);
     }
   }
   tmpConfig.set(slefty,enegative,wpositive);
   if(tmpConfig.matchMask(solution)){
     result = invKin(p,q,tmpConfig);
     if(result == SUCCESS){
        candidate cand1(tmpConfig,q);
        candiSet.push_back(cand1);
     }
   }
   tmpConfig.set(slefty,enegative,wnegative);
   if(tmpConfig.matchMask(solution)){
     result = invKin(p,q,tmpConfig);
     if(result == SUCCESS){
        candidate cand1(tmpConfig,q);
        candiSet.push_back(cand1);
     }
   }
   if(candiSet.size() > 0){
     float* dist = new float[candiSet.size()];
     int idxNear = 0;
     for(unsigned int i = 0; i < candiSet.size(); i++){
       dist[i] = 0.;
       Vect6& tmpVec6 = candiSet.at(i).second;

       for(int j = 0; j < 6; j++)
         dist[i] += mt::sq(mt::normalize(tmpVec6[i] - qNear[i],-mt::HALF_PI, mt::HALF_PI));

       dist[i] = sqrt(dist[i]);
       if(dist[i] < dist[idxNear])
         idxNear = i;
     }
     result = SUCCESS;
     config& tmpC = candiSet.at(idxNear).first;
     Vect6& tmpVec6 = candiSet.at(idxNear).second;
     solution.set(tmpC.sh, tmpC.el, tmpC.wr);
     
     for(int j = 0; j < 6; j++)
       q[j] = tmpVec6[j];

   }else{
     result = ERROR_UNDEFINED;
   }
 //}
 return result;
}

TXerror TXRobot::getJacobian(const Vect6& q, ublas::matrix<mt::Scalar> &j) const
{
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

  j(0,0)=-aJ5 * s5 * c4 * s1 * c2 * c3 + 
		  aJ5 * s5 * c4 * s1 * s2 * s3 - 
		  aJ5 * s5 * c1 * s4 - 
		  aJ5 * c5 * s1 * c2 * s3 - 
		  aJ5 * c5 * s1 * s2 * c3 - 
		  a4 * s1 * c2 * s3 - 
		  a4 * s1 * s2 * c3 - 
		  c1 * a3 - 
		  s1 * s2 * a2 - 
		  s1 * a1;
	
  j(0,1)= c1 * (-aJ5 * c4 * s5 * s2 * c3 - 
		  aJ5 * c4 * s5 * c2 * s3 - 
		  aJ5 * c5 * s2 * s3 + 
		  aJ5 * c5 * c2 * c3 - 
		  a4 * s2 * s3 + 
		  a4 * c2 * c3 + 
		  c2 * a2);

  j(0,2)= c1 * (-aJ5 * c4 * s5 * s2 * c3 - 
		  aJ5 * c4 * s5 * c2 * s3 - 
		  aJ5 * c5 * s2 * s3 + 
		  aJ5 * c5 * c2 * c3 - 
		  a4 * s2 * s3 + 
		  a4 * c2 * c3);

  j(0,3)=-s5 * (s1 * c4 - 
		  s2 * s3 * c1 * s4 + 
		  c2 * c3 * c1 * s4) * aJ5;

  j(0,4)= aJ5 * (-s4 * s1 * c5 - 
		  c1 * c4 * c5 * s2 * s3 + 
		  c1 * c4 * c5 * c2 * c3 - 
		  s2 * c3 * s5 * c1 - 
		  c2 * s3 * s5 * c1);
  
  j(0,5)=0;

  j(1,0)= aJ5 * s5 * c4 * c1 * c2 * c3 - 
	      aJ5 * s5 * c4 * c1 * s2 * s3 - 
		  aJ5 * s5 * s1 * s4 + 
		  aJ5 * c5 * c1 * c2 * s3 + 
		  aJ5 * c5 * c1 * s2 * c3 + 
		  a4 * c1 * c2 * s3 + 
		  a4 * c1 * s2 * c3 - 
		  s1 * a3 + 
		  c1 * s2 * a2 + 
		  c1 * a1;

  j(1,1)= s1 * (-aJ5 * c4 * s5 * s2 * c3 - 
		  aJ5 * c4 * s5 * c2 * s3 - 
		  aJ5 * c5 * s2 * s3 + 
		  aJ5 * c5 * c2 * c3 - 
		  a4 * s2 * s3 + 
		  a4 * c2 * c3 + 
		  c2 * a2);

  j(1,2)= s1 * (-aJ5 * c4 * s5 * s2 * c3 - 
		  aJ5 * c4 * s5 * c2 * s3 - 
		  aJ5 * c5 * s2 * s3 + 
		  aJ5 * c5 * c2 * c3 - 
		  a4 * s2 * s3 + 
		  a4 * c2 * c3);

  j(1,3)= -s5 * (-c1 * c4 - 
		   s4 * s1 * s2 * s3 + 
		   s4 * s1 * c2 * c3) * aJ5;

  j(1,4)= aJ5 * (-s2 * c3 * s5 * s1 - 
		  c2 * s3 * s5 * s1 + 
		  c1 * s4 * c5 - 
		  c4 * s1 * c5 * s2 * s3 + 
		  c4 * s1 * c5 * c2 * c3);

  j(1,5)=0;

  j(2,0)=0;

  j(2,1)=-aJ5 * s5 * c4 * c2 * c3 + 
		  aJ5 * s5 * c4 * s2 * s3 - 
		  aJ5 * c5 * c2 * s3 - 
		  aJ5 * c5 * s2 * c3 - 
		  a4 * c2 * s3 - 
		  a4 * s2 * c3 - 
		  s2 * a2;

  j(2,2)=-aJ5 * s5 * c4 * c2 * c3 + 
		  aJ5 * s5 * c4 * s2 * s3 - 
		  aJ5 * c5 * c2 * s3 - 
		  aJ5 * c5 * s2 * c3 - 
		  a4 * c2 * s3 - 
		  a4 * s2 * c3;

  j(2,3)= s5 * s4 * (c2 * s3 + s2 * c3) * aJ5;

  j(2,4)=-aJ5 * (c4 * c5 * c2 * s3 + 
	      c4 * c5 * s2 * c3 + 
		  c2 * c3 * s5 - 
		  s2 * s3 * s5);
	
  j(2,5)=0;

  j(3,0)=0;

  j(3,1)= -s1;

  j(3,2)= -s1;

  j(3,3)= c1 * (c2 * s3 + s2 * c3);

  j(3,4)=-s1 * c4 + s2 * s3 * c1 * s4 - c2 * c3 * c1 * s4;

  j(3,5)= s5 * c4 * c1 * c2 * c3 - 
		  s5 * c4 * c1 * s2 * s3 - 
		  s5 * s1 * s4 + 
		  c5 * c1 * c2 * s3 + 
		  c5 * c1 * s2 * c3;

  j(4,0)=0;

  j(4,1)= c1;

  j(4,2)= c1;

  j(4,3)= s1 * (c2 * s3 + s2 * c3);

  j(4,4)=-s4 * s1 * c2 * c3 + s4 * s1 * s2 * s3 + c1 * c4;

  j(4,5)= s5 * c4 * s1 * c2 * c3 - 
		  s5 * c4 * s1 * s2 * s3 + 
		  s5 * c1 * s4 + 
		  c5 * s1 * c2 * s3 + 
		  c5 * s1 * s2 * c3;

  j(5,0)= 1;

  j(5,1)= 0;

  j(5,2)= 0;

  j(5,3)= -s2 * s3 + c2 * c3;

  j(5,4)= (c2 * s3 + s2 * c3) * s4;

  j(5,5)=-c4 * s5 * s2 * c3 - c4 * s5 * c2 * s3 - c5 * s2 * s3 + c5 * c2 * c3;

	return SUCCESS;
}



}// TXrobot
