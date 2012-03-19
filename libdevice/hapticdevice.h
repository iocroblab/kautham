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

#ifndef HAPTICDEVICE_H
#define HAPTICDEVICE_H

#include "device.h"
#include <QTimer>
#include <mt/mt.h>

//haptic library
#ifdef KAUTHAM_USE_OPENHAPTICS
  #include <haptic/haptic.h>
#endif

using namespace Kautham;
using namespace std;
using namespace mt;

namespace libDevice{
  class HapticDevice : public Device {
    Q_OBJECT
    private slots:
      void                          update();
    public:
      HapticDevice(string name, unsigned int upPeriod);
      ~HapticDevice();
      bool                          connect(string url = "", string port = "", bool tcp = true);
      bool                          disconnect();
      bool                          setParameters();
      inline mt::Transform&         getHIP(){return _hipTransform;}
      inline bool                   getButtonState(){return _buttonPressed;}
      void                          setSE3Force(KthReal forces[]);
      inline void                   setReadDirectly(bool rd){
#ifdef KAUTHAM_USE_OPENHAPTICS
        _directReading = rd;
#else
        _directReading = false;
#endif
      }

    private:
      HapticDevice();

      //// components for the client
      //ioc_comm::Client*             _client;

      unsigned int                  _updatingPeriod;
      QTimer                        _updateDeviceTimer;

      ////Generic Exchanged data structures
      //ioc_comm::vecData             _serverData;
      //ioc_comm::vecData             _sendingData;

      //// Haptic information
      //ioc_comm::cartesian::position _position;
      //ioc_comm::cartesian::force    _force;
      bool                          _buttonPressed;

      mt::Transform                 _hipTransform;

#ifdef KAUTHAM_USE_OPENHAPTICS
      haptic::Haptic*               _hapticDevice;
#endif

      KthReal                       _fMax, _tMax;

  };
}
#endif // HAPTICDEVICE_H
