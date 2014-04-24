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

#include <libpugixml/pugixml.hpp>
#include <sstream>
#include <string>
#include <iostream>
#include <boost/algorithm/string.hpp>
#include "pathparse.h"

using namespace std;
using namespace pugi;

PathParse::PathParse()
{
	dimpoint = -1;
}

PathParse::PathParse(int d)
{
	dimpoint = d;
}

bool PathParse::savePath2File(string filepath, vector<double*> &path){
	if(getDimPoint()==-1){
		std::cout << "ERROR: dimpoint not set.\n";
		return false;
	}
	else
	{
		stringstream ss;
		string sp;
		xml_document doc;
		xml_node pathNode = doc.append_child();
		pathNode.set_name("Path");
		for(int i = 0; i< path.size(); i++){
			xml_node tmpPoint = pathNode.append_child();
			tmpPoint.set_name("Point");
			ss.str("");
			for(int j = 0; j < dimpoint; j++ )
				ss << path[i][j] << " ";
			sp = ss.str();
			sp.erase( sp.length() - 1 );
			tmpPoint.append_child(node_pcdata).set_value( sp.c_str() );
		 }
		 return doc.save_file( filepath.c_str() );
	}

}

vector<double*>  PathParse::loadPathFromFile(string filepath){
	vector<double*> vecPoints;
	if(getDimPoint()==-1){
		std::cout << "ERROR: dimpoint not set.\n";
	}
	else
	{
		double* tmpPoint = NULL;
		xml_document doc;
		xml_parse_result result = doc.load_file(filepath.c_str());

		xml_node tempNode = doc.child("Path");
		xml_node::iterator it;
		for(it = tempNode.begin(); it != tempNode.end(); ++it ){
			string sentence = it->child_value();
			vector<string> tokens;
			boost::split(tokens, sentence, boost::is_any_of("| "));

			if(tokens.size() != dimpoint){
				std::cout << "Incorrect path point.\n";
				break;
			}

			double *tmpPoint = new double[dimpoint];

			for(int i = 0; i < dimpoint; i++ )
				tmpPoint[i] = atof( tokens[i].c_str() );

			vecPoints.push_back( tmpPoint );
		}
	}
	return vecPoints; 
}
