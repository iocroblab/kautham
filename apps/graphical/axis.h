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

/* Author: Alexander Perez, Jan Rosell */
#ifndef _axis_H_
#define _axis_H_

#include <Inventor/SoDB.h>
#include <Inventor/SoInput.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/actions/SoSearchAction.h>


/** \addtogroup Application
 *  @{
 */


static const char *axis_str[] = {
    "#Inventor V2.1 ascii\n",
    "DEF root Separator {\n",
    "  DEF X Separator {\n",
    "    Material {\n",
    "      ambientColor 1 0 0\n",
    "      diffuseColor 1 0 0\n",
    "    }\n",
    "    Rotation {\n",
    "      rotation -0 -0 -1  1.5707999\n",
    "    }\n",
    "    Translation {\n",
    "      translation 0 50 0\n",
    "    }\n",
    "    Cylinder {\n",
    "      height 100\n",
    "    }\n",
    "    Translation {\n",
    "      translation 0 50 0\n",
    "    }\n",
    "    Cone {\n",
    "      bottomRadius 5\n",
    "      height 15\n",
    "    }\n",
    "    Text2 {\n",
    "      string \"X\"\n",
    "    }\n",
    "  }\n",
    "  DEF Y Separator {\n",
    "    Material {\n",
    "      ambientColor 0 1 0\n",
    "      diffuseColor 0 1 0\n",
    "    }\n",
    "    Translation {\n",
    "      translation 0 50 0\n",
    "    }\n",
    "    Cylinder {\n",
    "      height 100\n",
    "    }\n",
    "    Translation {\n",
    "      translation 0 50 0\n",
    "    }\n",
    "    Cone {\n",
    "      bottomRadius 5\n",
    "      height 15\n",
    "    }\n",
    "    Text2 {\n",
    "      string \"Y\"\n",
    "    }\n",
    "  }\n",
    "  DEF Z Separator {\n",
    "    Material {\n",
    "      ambientColor 0 0 1\n",
    "      diffuseColor 0 0 1\n",
    "    }\n",
    "    Rotation {\n",
    "      rotation 1 0 0  1.5707999\n",
    "    }\n",
    "    Translation {\n",
    "      translation 0 50 0\n",
    "    }\n",
    "    Cylinder {\n",
    "      height 100\n",
    "    }\n",
    "    Translation {\n",
    "      translation 0 50 0\n",
    "    }\n",
    "    Cone {\n",
    "      bottomRadius 5\n",
    "      height 15\n",
    "    }\n",
    "    Text2 {\n",
    "      string \"Z\"\n",
    "    }\n",
    "  }\n",
    "}\n",
    NULL
};

class Axis : public SoSeparator
{
public:
  Axis(float transparency = 0.)
  {
    SoInput in;
    in.setStringArray(axis_str);
    SoSeparator *sep = SoDB::readAll(&in);
    sep->ref();

    if( transparency != 0. ){
      // Search for all file nodes
      SoSearchAction sa;
      sa.setType(SoMaterial::getClassTypeId());
      sa.setInterest(SoSearchAction::ALL);
      sa.setSearchingAll(TRUE);

      sa.apply(sep);

//      SoFullPath *p = (SoFullPath *) sa.getPath();
      SoPathList &p = sa.getPaths();
/*      if(p == NULL)
          std::cout << "NO SoMaterial NODES FOUND" << std::endl;

      while (p != NULL) {
        SoMaterial* tmpMat = (SoMaterial *)p->getTail();
        tmpMat->transparency.setValue( transparency );
        sa.apply(sep);
        p = (SoFullPath *) sa.getPath();
      }
      */
      for(int i = 0; i < p.getLength(); ++i){
        SoMaterial* tmpMat = (SoMaterial *)p[i]->getTail();
        tmpMat->transparency.setValue( transparency );
      }
    }

    while (sep->getNumChildren() > 0){ 
      this->addChild( sep->getChild(0) );
      sep->removeChild(0);
    }
    sep->unref();

  }
};


/** @}   end of Doxygen module "Applications" */
#endif
