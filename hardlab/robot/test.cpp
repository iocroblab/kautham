/***************************************************************************
                          test.cpp  -  description
                             -------------------
    begin                : dl feb  4 18:53:07 CET 2008
    copyright            : (C) 2008 by Leopold Palomo Avellaneda
    email                : leopold.palomo@upc.edu

    copyright:             (C) 2010 by Alexander Pérez Ruiz
    email:                 alexander.perez@upc.edu    
 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <cmath>
#include <ctime>
#include <fstream>

#include "txrobot.h"
#include <mt/mt.h>

using namespace std;

using namespace robot;

int main (int argc, char *argv[])
{

   TXRobot *tx;
   TXtype tipus = TX90;

   tx = new TXRobot(tipus);
   TXerror error;
   config TXConf;
   char op;
   mt::Scalar value, x, y, z, yaw, pitch, roll;
   Vect6 q(6), qsol(6);
   clock_t inici, final;
   mt::Transform Atarget;
   mt::Transform AHand;
   mt::Matrix3x3 mat;
   mt::Vector3 vect;
   
   float factor = 10;  //number of times to make the calculus. Put 1000 if you want to test times.
   

   #ifdef VERBOSE
    std::cout << "Init done " << std::endl;
   #endif 
   for (;;)
   {
      cout << "i: inverse kinematics (m00,m10,m20,m01,m11,m21.." << endl;
      cout << "e: inverse kinematics (x,y,z,yaw,pitch,roll ...degree and m) Craig" << endl;
      cout << "a: inverse kinematics (x,y,z,yaw,pitch, roll ...degree and m)) Craig and show all sols" << endl;
      cout << "f: forward kinematics q1,q2,q3,q4,q5,q6" << endl;
      cout << "s: inverse kinematics (x,y,z, yaw, pitch, roll .... degree and mm!!!)" << endl;
      cout << "q: quit" << endl;

      cout << "Select and option: " << endl;
      cin >> op;
      switch (op)
      {
         case 'q':
           exit(0);
         
         case 'i':
            cin >> mat[0][0];
            cin >> mat[1][0];
            cin >> mat[2][0];

            cin >> mat[0][1];
            cin >> mat[1][1];
            cin >> mat[2][1];

            cin >> mat[0][2];
            cin >> mat[1][2];
            cin >> mat[2][2];

            cin >> vect[0];
            cin >> vect[1];
            cin >> vect[2];
            
            Atarget.setRotation(mat);
            Atarget.setTranslation(vect);

            inici = clock ();
            for (int i= 1; i < (int)factor ; i++)
              error = tx->invKin(Atarget, q);
            final = clock ();
  
            cout << "Time used : " << (difftime (final, inici) /CLOCKS_PER_SEC)/factor << "s" << endl;
            
            if(error == SUCCESS)
            {
                for(unsigned int i = 0; i < q.size();i++)
                  std::cout << "Solution q(" << i << ")= " << q[i] << " rads, " << mt::radToDeg(q[i]) << " degrees." << std::endl;

            }
            else
                std::cout << "Error: " << error << std::endl;

         break;

         case 'e':
            cin >> x;
            cin >> y;
            cin >> z;
            cin >> yaw;
            cin >> pitch;
            cin >> roll;
            
            Atarget.setRotation(mt::Rotation(mt::degToRad(yaw),mt::degToRad(pitch),mt::degToRad(roll)));
            Atarget.setTranslation(mt::Vector3::Vector3(x,y,z));
            
            inici = clock ();
            for (int i= 1; i < (int)factor ; i++)
              error = tx->invKin(Atarget, q);
            final = clock ();
  
            cout << "Time used : " << (difftime (final, inici) /CLOCKS_PER_SEC)/factor << "s" << endl;

            if(error == SUCCESS)
            {
                for(unsigned int i = 0; i < q.size();i++)
                  std::cout << "Solution q(" << i << ")= " << q[i] << " rads, " << mt::radToDeg(q[i]) << " degrees." << std::endl;

            }
            else
                std::cout << "Error: " << error << std::endl;


         break;

         case 'a':
            cin >> x;
            cin >> y;
            cin >> z;
            cin >> yaw;
            cin >> pitch;
            cin >> roll;

            Atarget.setRotation(mt::Rotation(mt::degToRad(yaw),mt::degToRad(pitch),mt::degToRad(roll)));
            Atarget.setTranslation(mt::Vector3::Vector3(x,y,z));

            for(unsigned int c = 0; c < 8; c++)
            {
              switch (c)
              {
                  case 0:
                         TXConf.sh = srighty;
                         TXConf.el = epositive;
                         TXConf.wr = wpositive;
                  break;
                  case 1:
                         TXConf.sh = slefty;
                         TXConf.el = epositive;
                         TXConf.wr = wpositive;

                  break;
                  case 2:
                         TXConf.sh = srighty;
                         TXConf.el = enegative;
                         TXConf.wr = wpositive;

                  break;
                  case 3:
                         TXConf.sh = slefty;
                         TXConf.el = enegative;
                         TXConf.wr = wpositive;

                  break;
                  case 4:
                         TXConf.sh = srighty;
                         TXConf.el = enegative;
                         TXConf.wr = wnegative;
                  break;
                  case 5:
                         TXConf.sh = slefty;
                         TXConf.el = enegative;
                         TXConf.wr = wnegative;

                  break;
                  case 6:
                         TXConf.sh = srighty;
                         TXConf.el = epositive;
                         TXConf.wr = wnegative;

                  break;
                  case 7:
                         TXConf.sh = slefty;
                         TXConf.el = epositive;
                         TXConf.wr = wnegative;

                  break;
                }
                error = tx->invKin(Atarget, q, TXConf);

                if(error == SUCCESS)
                {
                  std::cout << "------------------------------------------------------" << std::endl;
                  for(unsigned int i = 0; i < q.size();i++)
                    std::cout << "Solution q(" << i << ")= " << q[i] << " rads, " << mt::radToDeg(q[i]) << " degrees." << std::endl;
                }
                else
                  std::cout << "Error: " << error << std::endl;   

            }
         break;

         case 'f':
            cin >> value; q[0] = mt::degToRad(value);
            cin >> value; q[1] = mt::degToRad(value);
            cin >> value; q[2] = mt::degToRad(value);
            cin >> value; q[3] = mt::degToRad(value);
            cin >> value; q[4] = mt::degToRad(value);
            cin >> value; q[5] = mt::degToRad(value);

            std::cout << "Doint the forward kinematics ..." << std::endl;
            inici = clock ();
            for (int i= 1; i < (int)factor ; i++)
                  error = tx->fwdKin(q,AHand);
            final = clock ();
  
            cout << "Time used : " << (difftime (final, inici) /CLOCKS_PER_SEC)/factor << "s" << endl;

            if(error == SUCCESS)
            {
              std::cout << "Rotation:" << std::endl << AHand.getRotation().getMatrix() << std::endl;
              std::cout << "Translation:" << std::endl << AHand.getTranslation() << std::endl;
            }
            else
              std::cout << "Error: " << error << std::endl;
         break;
        
          case 's':
            cin >> x;
            cin >> y;
            cin >> z;
            cin >> yaw;
            cin >> pitch;
            cin >> roll;
            
            const mt::Rotation rotx(mt::Unit3(1,0,0),mt::degToRad(yaw)), roty(mt::Unit3(0,1,0),mt::degToRad(pitch)), rotz(mt::Unit3(0,0,1),mt::degToRad(roll));
            mt::Rotation rb; 
            rb = rotx * roty * rotz;

            
            Atarget.setRotation(rb);
            Atarget.setTranslation(mt::Vector3::Vector3(x/1000.,y/1000.,z/1000.));
            
            
            inici = clock ();
            for (int i= 1; i < (int)factor ; i++)
                error = tx->invKin(Atarget, q);
            final = clock ();
  
            cout << "Time used : " << (difftime (final, inici) /CLOCKS_PER_SEC)/factor << "s" << endl;

            
            if(error == SUCCESS)
            {
                for(unsigned int i = 0; i < q.size();i++)
                  std::cout << "Solution q(" << i << ")= " << q[i] << " rads, " << mt::radToDeg(q[i]) << " degrees." << std::endl;

            }
            else
                std::cout << "Error: " << error << std::endl;


         break;

      }
   }

   return EXIT_SUCCESS;

}


/* 
  std::cout << "Doing the inv inverse kinematics ..." << std::endl;
  std::cout << "The result must be: " << std::endl;
  for(unsigned int i = 0; i < q.size();i++)
      std::cout << "Solution q(" << i << ")= " << q[i] << " rads, " << mt::radToDeg(q[i]) << " degrees." << std::endl;
  
  std::cout << "---------------------------" << std::endl;

  inici = clock ();
  error = tx->invKin(AHand, q);
  final = clock ();
  
  cout << "Temps empreat: " << difftime (final, inici) /CLOCKS_PER_SEC << "s" << endl;

  if(error == SUCCESS)
  {
    for(unsigned int i = 0; i < q.size();i++)
      std::cout << "Solution q(" << i << ")= " << q[i] << " rads, " << mt::radToDeg(q[i]) << " degrees." << std::endl;
    
  }
  else
    std::cout << "Error: " << error << std::endl;

  return EXIT_SUCCESS;

}

*/

