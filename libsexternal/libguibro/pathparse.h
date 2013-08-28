#ifndef PATHPARSE_H
#define PATHPARSE_H

#include <string>
#include <vector>

using namespace std;


class PathParse{
public:
  bool             savePath2File(string filepath, vector<double*> &path);
  vector<double*>  loadPathFromFile(string filepath);
  inline int getDimPoint(){return dimpoint;};
  inline void setDimPoint(int d){dimpoint=d;};

  PathParse();
  PathParse(int d);
private:
  int dimpoint;

};

#endif // PATHPARSE_H
