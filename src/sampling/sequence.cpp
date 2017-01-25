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

 
#include <kautham/sampling/sequence.h>
#include <external/lcprng.h>

namespace Kautham {


  Sequence::Sequence(int dim, int M, bool randOffset){
    LCPRNG* gen1 = new LCPRNG(3141592621, 1, 0, ((unsigned int)time(NULL) & 0xfffffffe) + 1);//LCPRNG(15485341);//15485341 is a big prime number
    _index=0;
    _dim=dim;
    _maxSamplingLevel=M;
    _maxNumCells=0x01<<(_dim*_maxSamplingLevel);
    _V=NULL;
    _indexes = NULL;
    _lastCode=0;
    _offset=0;
    if(randOffset)_offset= (unsigned long)(gen1->d_rand()*_maxNumCells);
  }

  Sequence::~Sequence(){
      if (_V) {
          for(unsigned int i = 0; i < _dim; ++i)
              delete[] _V[i];
          delete[] _V;
      }

      delete[] _indexes;

  }

  int* Sequence::getIndexes(void){
		return _indexes;
	}

  int* Sequence::getIndexes(unsigned long int code){
    if(code != _lastCode) 
      _V = getVMatrix(code);
        if(_indexes == NULL) _indexes = new int[_dim];
        for(unsigned int i=0; i<_dim; i++) {
			_indexes[i]=0;
            for(unsigned int j=0; j < _maxSamplingLevel; j++) {
				_indexes[i]+= _V[i][j]<<(_maxSamplingLevel-j-1);
			}
		}
		return _indexes;
	}

  unsigned long Sequence::getSequenceCode(unsigned long index){
        int **v = getVMatrix(index);
        int **tv = _tMat->multiply( v, _maxSamplingLevel );
		int loop = _index / _maxNumCells;
		unsigned long ret=0;
    for(unsigned int i=0; i<_dim; i++){
            for(unsigned int j=0; j<_maxSamplingLevel; j++){
				if(_wMat->getRow(i, j)!=-1){
					ret += _wMat->getRow(i, j ) * tv[i][_maxSamplingLevel-j-1];
				}
			}
    }
    if( _index <= _maxNumCells )
      return ret;
    else
      return ret+loop*_maxNumCells;
	}

  unsigned long Sequence::getSequenceCode(){
    return getSequenceCode( _offset + _index++ );
  }

  int** Sequence::getVMatrix(void){
    return _V;
  }

  int** Sequence::getVMatrix(unsigned long int code){
		long temp;
		code = code % _maxNumCells;
		if(_V == NULL){
            _V = new int*[_dim];
            for(unsigned int i=0;i<_dim;i++)
                _V[i]= new int[_maxSamplingLevel];
		}

        for(unsigned int i=0;i<_dim;i++)
            for(unsigned int j=0; j<_maxSamplingLevel; j++){
				temp = code & _wMat->getRow(i, j);
				if(temp == 0)
					_V[i][j] = 0;
				else
					_V[i][j] = 1;
			}
		return _V;
	}

  unsigned long Sequence::getCode(int *indexes){
    long temp;
    if(_V == NULL){
            _V = new int*[_dim];
            for(unsigned int i=0;i<_dim;i++)
                _V[i]= new int[_maxSamplingLevel];
		}
    for(unsigned i=0; i< _dim; i++){
      for(unsigned j=0; j<_maxSamplingLevel; j++){
        temp = ((long)indexes[i]) & (0x01<<(_maxSamplingLevel-j-1));
				if(temp == 0)
					_V[i][j] = 0;
				else
					_V[i][j] = 1;
      }
    }
    
		unsigned long ret=0;
    for(unsigned int i=0; i<_dim; i++){
            for(unsigned int j=0; j<_maxSamplingLevel; j++){
				if(_wMat->getRow(i, j)!=-1){
					ret += _wMat->getRow(i, j ) * _V[i][j];
				}
			}
    }
		return ret;
  }

}
