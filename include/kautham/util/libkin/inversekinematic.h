
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

/* Author: Alexander Perez, Jan Rosell and Nestor Garcia Hidalgo */

#if !defined(_INVERSEKINEMATIC_H)
#define _INVERSEKINEMATIC_H

#include <util/kthutil/kauthamdefs.h>
#include <util/kthutil/kauthamobject.h>

#include <sampling/robconf.h>
#include <sampling/se3conf.h>
#include <sampling/rnconf.h>


namespace Kautham {
/** \addtogroup libKin
 *  @{
 */
  class Robot;
  //! This vector contains the layout of the robot adopted by the solution. The layout is a descriptive
  //! configuration of the pose of the robot. It is normally used as shoulder left/right, or elbow up/down.
  //typedef vector<bool> RobLayout;
  class RobLayout: public vector<bool>{
  public:
    bool operator!=(const RobLayout & b)const {
      if( this->size() == b.size() ){
        for(size_t i = 0; i < size(); ++i)
          if( this->at(i) != b.at(i) )
            return true;
        
        return false;
      }
      return true;
    }
/*
    RobLayout& operator =(const RobLayout& b){
      if(size() != b.size() )
        resize( b.size() );

      for(size_t i = 0; i < b.size(); ++i)
        at(i) = b.at(i);
      
      return *this;
    }
*/
  };

  class InvKinEx: public exception {
    public:
     InvKinEx(int cau) : exception(), causes(cau) {}

     inline const char* what() const throw() {
        switch(causes) {
        case 0:
          return "IK has not solution. Take care with the robot workspace and its singularities."  ;
        case 1:
          return "Joint out of range";
        }
        return "Unspected error in the Inverse Kinematic resolution";
     }
    private:
     int causes;
  };



  class InverseKinematic:public KauthamObject{
  public:
    InverseKinematic(Robot* const rob);

      //!	This method returns the type of the inverse kinematics.
    virtual INVKINECLASS type() {return NOINVKIN;}

      //!	This method returns the name of the inverse kinematics.
    virtual string       name() {return "";}

	  //!	This method must be implemented in order to obtain a solution
	  //! of the inverse kinematic model for a specific target.
    virtual bool         solve() = 0;

	  //!	This method allows extract the information from the Hash_Map
	  //! and setup the internal variables.
    virtual bool         setParameters() = 0;

    //! This method returns the high level configuration description of the robot i.e. shoulder left/right or 
    //! elbow up/down for the target parameter. This function should be virtual pure.
    virtual RobLayout&   getRobLayout(vector<KthReal> &target); // = 0;

	  //!	This method allow to setup the target. Commonly, the target is se3.coordintes.
    void              setTarget(vector<KthReal> &target, const string& param=string(""));
    virtual void	    setTarget(vector<KthReal> &target, vector<KthReal> masterconf, bool maintainSameWrist);
    inline void       setTarget(mt::Transform& tcp){_targetTrans = tcp;}
    inline SE3Conf&   getSE3(){return _robConf.getSE3();}
    inline RnConf&    getRn(){return _robConf.getRn();}
    inline RobConf&   getRobConf(){return _robConf;}
    inline Robot&     getRobot(){return *_robot;}
    inline RobLayout& getRobLayout(){return _robLay;}
    inline void       setRobLayout(RobLayout& robLay){copy(robLay.begin(), robLay.end(), _robLay.begin());}

  private:
    InverseKinematic();
  protected:

	  //!	This is a robot pointer. It will be free, chain or tree robot.
    Robot*          _robot;
    RobConf         _robConf;
    RobLayout       _robLay;

	  //! This vector contains the target to be solved.
    vector<KthReal> _target;      //!< This is a generic way to set up the target.
    mt::Transform   _targetTrans; //!< This is the Scene target

  };

  /** @}   end of Doxygen module "Util */
}
#endif  //_INVERSEKINEMATIC_H
