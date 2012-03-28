/***************************************************************************
*                                                                          *
*           Institute of Industrial and Control Engineering                *
*                 Technical University of Catalunya                        *
*                        Barcelona, Spain                                  *
*                                                                          *
*                Project Name:       Kautham Planner                       *
*                                                                          *
*     Copyright (C) 2007 - 2009 by Alexander Pérez and Jan Rosell          *
*            alexander.perez@upc.edu and jan.rosell@upc.edu                *
*                                                                          *
*             This is a motion planning tool to be used into               *
*             academic environment and it's provided without               *
*                     any warranty by the authors.                         *
*                                                                          *
*          Alexander Pérez is also with the Escuela Colombiana             *
*          de Ingeniería "Julio Garavito" placed in Bogotá D.C.            *
*             Colombia.  alexander.perez@escuelaing.edu.co                 *
*                                                                          *
***************************************************************************/
/***************************************************************************
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
 
 
 

#if !defined(_DEVICE_H)
#define _DEVICE_H

#include <libutil/kauthamdefs.h>
#include <libutil/kauthamobject.h>
#include <string>
#include <QObject>
#include <mt/transform.h>

using namespace Kautham;
using namespace std;
namespace libDevice{
  class Device: public QObject, public KauthamObject{
    Q_OBJECT
  signals:
    void                  updated();
  public:
    Device(string name);
    //~Device();
    
    virtual bool            connect(string url = "", string port = "", bool tcp = true)=0;
    virtual bool            disconnect()=0;
    virtual bool            setParameters()=0;
    virtual bool            getButtonState()=0;
    virtual mt::Transform&  getHIP()=0;
    virtual void            setSE3Force(KthReal forces[])=0;
    inline void             addIncomingParameter(string key, KthReal value){_incomingParameters[key] = value;}
    inline void             emmitUpdated(bool em){_emmitUpdated = em;}
    inline bool             emmitUpdated(){return _emmitUpdated;}
    KthReal                 getIncomingParameter(string key);
    string                  getIncomingParametersAsString();
    bool                    setIncomingParameter(string key, KthReal value);
    inline const KthReal*   maxForces() const {return _maxForces;}
    inline const KthReal*   deviceLimits(unsigned int dof) const{
                                  if(dof >= 0 && dof < 6)
                                    return _deviceLimits[dof];
                                  return NULL;
    }
    inline bool             isReadDirectly(){return _directReading;}
    virtual inline void     setReadDirectly(bool rd){ _directReading = rd;}
  private:
    Device();
  protected:
    HASH_S_K              _incomingParameters;
    string                _url;
    string                _port;
    bool                  _tcp;
    bool                  _emmitUpdated;
    //!> This object can connet to device in two ways: Directly or using socket to HapticServer
    bool                  _directReading;
    //bool                  _emulated;
    KthReal               _maxForces[6];
    KthReal               _deviceLimits[6][2];
  };
}

#endif //_DEVICE_H
