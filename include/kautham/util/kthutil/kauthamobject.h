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


 
#if !defined(_KAUTHAMOBJECT_H)
#define _KAUTHAMOBJECT_H

#include <kautham/util/kthutil/kauthamdefs.h>

using namespace std;
using namespace Kautham;

namespace Kautham{
  class KauthamObject{
    public:
      virtual bool    setParameters()=0;
      inline          KauthamObject(string name){_guiName = name;}
      inline string   getGuiName() const {return _guiName;}
      inline void     addParameter(string key, KthReal value){_parameters[key] = value;}
      inline bool     removeParameter(string key){
			HASH_S_K::iterator it = _parameters.find(key);
			if(it != _parameters.end())	{
				_parameters.erase(it);
				return true;
			}
			else return false;}
      KthReal         getParameter(string key);

      string          getParametersAsString();

      bool            setParameter(string key, KthReal value);

      bool            setParametersFromString(const string& par);
  
      inline KauthamObject(){}
      virtual ~KauthamObject() {}
    protected:
      HASH_S_K      _parameters;
      string        _guiName;
  };
}

#endif	//_KAUTHAMOBJECT_H
