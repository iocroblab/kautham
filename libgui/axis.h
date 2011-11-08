//File created with coindesigner
#ifndef _axis_H_
#define _axis_H_

#include <Inventor/SoDB.h>
#include <Inventor/SoInput.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/actions/SoSearchAction.h>

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
#endif
