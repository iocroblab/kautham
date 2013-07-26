
#include "pugixml.hpp"
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
