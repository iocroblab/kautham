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



#include <kautham/util/kthutil/kauthamobject.h>
#include <string>
#include <sstream>
#include <cstdio>
#include <cstdlib>
#include <boost/algorithm/string.hpp>

namespace Kautham{

KthReal KauthamObject::getParameter(string key){
    HASH_S_K::iterator it = _parameters.find(key);
    if(it != _parameters.end() )
        return it->second;
    else
        throw -1;
}

string KauthamObject::getParametersAsString(){
    std::stringstream par;
    string p;
    par.precision(10);
    HASH_S_K::iterator it;
    for(it = _parameters.begin(); it != _parameters.end(); ++it) {
        par << it->first << "|";
        par << it->second << "|";
    }
    p = par.str();
    return p.substr(0,p.length()-1);
}

bool KauthamObject::setParameter(string key, KthReal value){
    HASH_S_K::iterator it = _parameters.find(key);
    if(it != _parameters.end() ){
        it->second = value;
        return true;
    }
    return false;
}

bool KauthamObject::setParametersFromString(const string& par){
    if (par == "") return false;
    KthReal val = 0.0;
    HASH_S_K::iterator it;
    vector<string> tokens;
    boost::split(tokens, par, boost::is_any_of("|"));
    
    for(unsigned i=0; i<tokens.size(); i=i+2){
        try{
            it = _parameters.find(tokens[i]);
            val = (KthReal)atof(tokens[i+1].c_str());
            if(it != _parameters.end())
                it->second = val;
            else{
                cout << "Error - parameter not found:" << tokens[i] << "\n";
                throw -1;
            }
        }catch(...){
            cout << "Error - parameter not found:" << tokens[i] << "\n";
            throw -2;
        }
    }
    
    return setParameters();
}
}
