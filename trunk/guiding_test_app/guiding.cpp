/***************************************************************************
 *   Copyright (C) 2006 by Emmanuel Nuño                                   *
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



//mt library
#include <mt/mt.h>

//haptic library
#include <haptic.h>

#if defined(WIN32)
#include <Windows.h>
#endif

#include <cstdlib>
#include <pugixml.hpp>
#include <string>
#include <vector>
#include <cmath>

using namespace std;
using namespace pugi;

#if !defined(M_PI)
#define M_PI       3.14159265358979323846
#endif

haptic::Haptic* myHaptic = NULL;

void reachFrame( mt::Transform& target, double _threshold ){  
	mt::Transform myPos;
	mt::Vector3 pos;
  

	Vect6 forceV(6);

	forceV[0]=0;
	forceV[1]=0;
	forceV[2]=0;
	forceV[3]=0;
	forceV[4]=0;
	forceV[5]=0;

	const mt::Scalar K=0.05;
  const mt::Scalar Kr=2;
  mt::Scalar y,p,r;

  do{
    myHaptic->getPosition(myPos);
    pos = target.getTranslation() - myPos.getTranslation();

	if( pos.length() <= _threshold ){
		forceV[0]=0;
		forceV[1]=0;
		forceV[2]=0;
		forceV[3]=0;
		forceV[4]=0;
		forceV[5]=0;
		myHaptic->setForce(forceV);
		return;
	}

    mt::Rotation difRot = target.getRotation() - myPos.getRotation();
    difRot.getYpr(y,p,r);

	forceV[0]=K*pos[0];
	forceV[1]=K*pos[1];
	forceV[2]=K*pos[2];
    forceV[3]=Kr*r;
    forceV[4]=Kr*p;
    forceV[5]=Kr*y;

	for(int i = 0; i < 6; i++)
      std::cout << forceV[i] << "\t" ;
    std::cout << std::endl;

		myHaptic->setForce(forceV);

#if defined(WIN32)
    Sleep(5);
#else
    sleep(5);
#endif
  }while(true);

  return ;
}

int main(int argc, char *argv[])
{

  const double toRad = M_PI/180.0;
  vector<mt::Transform> _transforms;
  double _threshold = 3.;

  string fileName="frames.xml";
  try{
	  if( argc < 2 )
		_threshold = 3.;
	  else if( argc < 3 )
		_threshold = atof(argv[1]);
	  else{
		_threshold = atof(argv[1]);
		fileName.assign(argv[2]);
	  }
  }catch(...){
	  cout << "Usage: guiding_test [Threshold] [frames_filename]\n";
	  exit(1);
  }

  // Opening the file with the new pugiXML library.
  xml_document doc;
  xml_parse_result result = doc.load_file(fileName.c_str());

  if(result){
    //  Once the file has been parsed, it contains the frames information
    xml_node framesNode = doc.child("Frames");

    mt::Transform aTrans;
    mt::Point3  aPoint;
    mt::Rotation aRot;
    mt::Scalar   angle;
    for(xml_node_iterator it = framesNode.begin(); it != framesNode.end(); ++it){
      xml_node aFrame = (*it);
      if( aFrame != NULL ){
        aPoint.setValue(aFrame.attribute("X").as_double(),
                        aFrame.attribute("Y").as_double(),
                        aFrame.attribute("Z").as_double() );

        angle = aFrame.attribute("WX").as_double() * toRad;

        aRot.setAxisAngle( mt::Unit3( aFrame.attribute("WX").as_double(),
                          aFrame.attribute("WY").as_double(),
                          aFrame.attribute("WZ").as_double() ),
                          angle );
        aTrans.setTranslation( aPoint );
        aTrans.setRotation( aRot );
        _transforms.push_back( aTrans );
      }
    }
  }

  if(_transforms.size() < 1 ){
    cout << "There are not frames to be used as targets.\n";
    exit(1);
  }


  bool active = false;
	myHaptic = new haptic::Haptic(active);
  if( active ){
	  myHaptic->calibrate();
	  myHaptic->start();

    int i = 0;
    do{
      cout << "\n\nBasic Menu:\n";
      cout << " -1 : Exit\n";
      cout << " n : Index of the frame desired\n";
      cout << " 0 < n < " << _transforms.size() << endl;
      cout << "Choose an option: ";
      cin >> i;
      //i = atoi(opt);
      if( i >= 0 && i < _transforms.size() )
        reachFrame( _transforms[i], _threshold );
      else
        cout << "Mind the limits!!\n" ;
    }while( i != -1 );


	  myHaptic->stop();

	  return 0;
  }else{
    cout << "Please verify the correct function of the haptic device.\n";
  }
}
