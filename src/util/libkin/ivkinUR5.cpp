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

/* Author: Nestor Garcia Hidalgo */

#include <vector>
#include <iomanip>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include <kautham/util/libkin/ivkinUR5.h>
#include <kautham/util/libkin/UR5_kinematics.h>


IvKinUR5::IvKinUR5(Robot* const rob): Kautham::InverseKinematic(rob){
    _target.resize(10);  // This contains the pos and quaternion as a vector
    _eulPos.resize(6);
    _robConf.setRn(6);
    _robLay.resize(3);

    addParameter("Px", _eulPos.at(0));
    addParameter("Py", _eulPos.at(1));
    addParameter("Pz", _eulPos.at(2));
    addParameter("Rx", _eulPos.at(3));
    addParameter("Ry", _eulPos.at(4));
    addParameter("Rz", _eulPos.at(5));

    addParameter("Shoulder Lefty?", 1.);
    addParameter("Elbow Positive?", 1.);
    addParameter("Wrist Positive?", 1.);
}

IvKinUR5::~IvKinUR5(){

}

bool IvKinUR5::solve(){

    auto plotTF = [](mt::Transform tf, const char* tf_name) {
        mt::Vector3 pos = tf.getTranslation();
        mt::Rotation rot = tf.getRotation();
        mt::Matrix3x3 rot_mat = rot.getMatrix();
        mt::Scalar y, p, r;
        rot.getYpr(y,p,r);
        mt::Quaternion q = rot;
        std::cout << "-- " << tf_name << " ------------------------" << std::endl;
        std::cout << "DK: pos = " <<pos[0]<<" " <<pos[1]<<" " <<pos[2]<<" " <<std::endl;
//        std::cout << "DK: rot = " << std::endl;
//        for (unsigned int i=0; i<3; ++i){
//            for (unsigned int j=0; j<3; ++j)    std::cout << std::setw(12) << rot_mat[i][j] << " ";
//            std::cout << std::endl;
//        }
//        std::cout << "DK: rpy = " <<r<<" " <<p<<" " <<y<<" " <<std::endl;
        std::cout << "DK: q   = " <<q[0]<<" " <<q[1]<<" " <<q[2]<<" | " <<q[3]<<" " <<std::endl;
    };


    std::cout << "+++++++++++++++++++++++ SOLVING IK +++++++++++++++++++++++" << std::endl;

    bool shoulder(true), elbow(true), wrist(true);
    if (_target.size() > 7){
        shoulder = ! (_target.at(7) == shoulder_left);
        elbow =    ! (_target.at(8) == elbow_up);
        wrist =    ! (_target.at(9) == wrist_in);
    }


    // Offset transformation between the kinematic model in UR5_Kinematics and the kautham model such that:
    //  T_kautham = T_{UR5_Kinematics} * T_offset
    mt::Transform offsetTF;
//    offsetTF.setTranslation(mt::Vector3(0.0, 0.0822999, 0.0));
    offsetTF.setTranslation(mt::Vector3(0.0, 0.0, 0.0));
    offsetTF.setRotation(mt::Rotation(mt::Quaternion(0.707107, 0.0, 0.0, -0.707107)));


    // DK test
    double theta_test[6];
    if (true){

        // Test configurations

//        theta_test[0] = 0.0;
//        theta_test[1] = -M_PI/2.0;
//        theta_test[2] = 0.0;
//        theta_test[3] = -M_PI/2.0;
//        theta_test[4] = -M_PI/2.0;
//        theta_test[5] = 0.0;

        theta_test[0] = 0.1;
        theta_test[1] = -2.3;
        theta_test[2] = 1.29;
        theta_test[3] = -5.89;
        theta_test[4] = -1.77;
        theta_test[5] = 0.67;

        theta_test[0] = -0.211;
        theta_test[1] = -0.3;
        theta_test[2] = -2.29;
        theta_test[3] = 1.89;
        theta_test[4] = 0.37;
        theta_test[5] = -5.67;

//        theta_test[0] = M_PI/2.0;
//        theta_test[1] = 0.0;
//        theta_test[2] = 0.0;
//        theta_test[3] = -M_PI/2.0;
//        theta_test[4] = 0.0;
//        theta_test[5] = 0.0;

        // Vertical
//        for (unsigned int i=0; i<6; ++i)    theta_test[i] = 0.0;
//        theta_test[1] = -M_PI/2.0;
//        theta_test[3] = -M_PI/2.0;

        std::cout << "DK: theta = "; for (unsigned int i=0; i<6; ++i)    std::cout << theta_test[i] << " "; std::cout << std::endl;

        // Set joints to robot
        for (unsigned int i = 0; i<6; ++i)  _robot->getLink(i+1)->setValue(theta_test[i]);

        // Direct kinematics ------------------------------------------------------------------
        mt::Transform kthDKTF = _robot->getLastLinkTransform();
        plotTF(kthDKTF, "kthDKTF");

//        std::cout << "lib tfs +++++++++++++++++++++++++++++++++++++++++++" << std::endl;
//        for (unsigned int i=0; i<6; ++i){
//            mt::Transform libTF = UR5_dir_kin(theta_test, i);
//            std::cout << i << std::endl;
//            plotTF(libTF, "libTF");
//        }

//        std::cout << "kth tfs +++++++++++++++++++++++++++++++++++++++++++" << std::endl;
//        for (unsigned int i=0; i<6; ++i){
//            mt::Transform kthTF = _robot->getLinkTransform(i+1);
//            std::cout << i << std::endl;
//            plotTF(kthTF, "kthTF");
//        }

        // 'lbTF' and 'corrLbTF' have to be equal
        mt::Transform lbTF = UR5_dir_kin(theta_test);
        mt::Transform corrLbTF = _robot->getLastLinkTransform() * offsetTF;
        plotTF(lbTF, "lbTF");
        plotTF(corrLbTF, "corrLbTF");


        // Test IK ------------------------------------------------------------------
        mt::Transform ikTF = UR5_dir_kin(theta_test);

        double q_ik[6];
        int ik_err_code = UR5_inv_kin(ikTF, shoulder, wrist, elbow, q_ik);
        std::cout << "ik_err_code = "<<ik_err_code;

        mt::Transform ikTF2 = UR5_dir_kin(q_ik);

        plotTF(ikTF, "ikTF");
        plotTF(ikTF2, "ikTF2");
    }



    std::cout << "_target ----------------------------------------- " <<std::endl;
    std::cout << "_target: UR5 target pos = " << _target.at(0) << " " << _target.at(1) << " " << _target.at(2) << std::endl;
    std::cout << "_target: UR5 target q   = " << _target.at(3) << " " << _target.at(4) << " " << _target.at(5) << " | " << _target.at(6) << std::endl;
    std::cout << "_target: shoulder / elbow / wrist = " << shoulder << " / " << elbow << " / " << wrist << std::endl;

    _targetTrans.setTranslation( mt::Point3(_target.at(0), _target.at(1), _target.at(2)) );
    _targetTrans.setRotation( mt::Rotation(_target.at(3), _target.at(4), _target.at(5), _target.at(6)) );
    plotTF(_targetTrans, "_targetTrans");

    int ik_err_code = UR5_inv_kin(_targetTrans * offsetTF, shoulder, wrist, elbow, _result);
    if ( ik_err_code == UR5_NO_ERROR ) {

        double control[6];
        UR5_controls(control,_result);
        cout << "Joint values are:" << endl;
        for (int j = 0; j < 6; j++) {
            cout << "  theta" << j+1 << " = " << _result[j]
                 << " (" << control[j] << ")" << endl;
        }

        std::vector<KthReal> qn(6);
        for (unsigned int i = 0; i<6; ++i){
            _robot->getLink(i+1)->setValue(_result[i]);
            qn.at(i) = _result[i];
        }
        _robConf.setRn(qn);

        mt::Transform result_ikTF = _robot->getLastLinkTransform();
        plotTF(result_ikTF, "result_ikTF");

        return true;
    } else {
        cout << "Inverse kinematics failed: error " << ik_err_code << endl;
        return false;
    }
}

bool IvKinUR5::setParameters(){
    try{
        HASH_S_K::iterator it = _parameters.find("Px");
        if(it != _parameters.end())     _eulPos.at(0) = it->second;
        else                            return false;

        it = _parameters.find("Py");
        if(it != _parameters.end())     _eulPos.at(1) = it->second;
        else                            return false;

        it = _parameters.find("Pz");
        if(it != _parameters.end())     _eulPos.at(2) = it->second;
        else                            return false;

        it = _parameters.find("Rx");
        if(it != _parameters.end())     _eulPos.at(3) = it->second;
        else                            return false;

        it = _parameters.find("Ry");
        if(it != _parameters.end())     _eulPos.at(4) = it->second;
        else                            return false;

        it = _parameters.find("Rz");
        if(it != _parameters.end())     _eulPos.at(5) = it->second;
        else                            return false;

        it = _parameters.find("Shoulder Lefty?" );
        if(it != _parameters.end()){
          if(it->second == 1)   _UR5Conf.sh = shoulder_left;
          else                  _UR5Conf.sh = shoulder_right;
        }else                   return false;

        it = _parameters.find("Elbow Positive?" );
        if(it != _parameters.end()){
          if(it->second == 1)   _UR5Conf.el = elbow_up;
          else                  _UR5Conf.el = elbow_down;
        }else                   return false;

        it = _parameters.find("Wrist Positive?" );
        if(it != _parameters.end()){
          if(it->second == 1)   _UR5Conf.wr = wrist_in;
          else                  _UR5Conf.wr = wrist_out;
        }else                   return false;


        _targetTrans.setTranslation(mt::Point3(_eulPos.at(0), _eulPos.at(1), _eulPos.at(2)));
        _targetTrans.setRotation(mt::Rotation(_eulPos.at(5), _eulPos.at(4), _eulPos.at(3)));

        for( int i = 0; i < 3; i++)
          _target.at(i) = _targetTrans.getTranslation().at(i);

        for( int i = 3; i < 7; i++)
          _target.at(i) = _targetTrans.getRotation().at(i-3);

        if(_target.size() > 7){
          _target.at(7) = _UR5Conf.sh == shoulder_right ? 1. : 0. ;
          _target.at(8) = _UR5Conf.el == elbow_up ? 1. : 0. ;
          _target.at(9) = _UR5Conf.wr == wrist_in ? 1. : 0. ;
        }

      }catch(...){
        return false;
      }
      return true;
}

void IvKinUR5::setTarget(vector<KthReal> &target, vector<KthReal> masterconf, bool maintainSameWrist){
    (void)masterconf;//unused
    (void)maintainSameWrist;//unused

    //loads the target: the tcp transform
    _target.clear();
    for(unsigned i =0; i< target.size(); i++)
        _target.push_back(target.at(i));
}
