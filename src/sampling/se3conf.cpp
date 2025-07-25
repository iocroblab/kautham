/*************************************************************************\
   Copyright 2014-2024  Institute of Industrial and Control Engineering (IOC)
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


#include <kautham/sampling/se3conf.h>
#include <cmath>

#if !defined(M_PI)
#define M_PI 3.1415926535897932384626433832795
#endif


namespace Kautham {

SE3Conf::SE3Conf():Conf(SE3),_pos(3,0.), _ori(4,0.), _axisAn(4,0.){
    // 3 for position and 4 as a quaternion.
    dim = 7;
    coord.resize(dim);
    coord.at(6) = (double)1.0; // Quaternion initialization
}

bool SE3Conf::setPose(const Eigen::AffineCompact3d& _pose) {
    // Set translation components:
    this->coord[0] = _pose.translation().x();
    this->coord[1] = _pose.translation().y();
    this->coord[2] = _pose.translation().z();

    // Extract rotation matrix and convert to quaternion:
    Eigen::Quaterniond quat(_pose.rotation());

    // Check for NaN/Inf:
    if (!quat.coeffs().allFinite()) {
        return false;
    }

    // Check unit quaternion:
    if (std::abs(quat.norm() - 1.0) > 1e-6) {
        return false;
    }

    // Store quaternion components:
    this->coord[3] = quat.x();
    this->coord[4] = quat.y();
    this->coord[5] = quat.z();
    this->coord[6] = quat.w();

    return true;
}

bool SE3Conf::getPose(Eigen::AffineCompact3d& _pose) {

    // Validate coordinate storage capacity:
    if (this->coord.size() < 7) {
        return false;
    }

    // Set translation from stored coordinates:
    _pose.translation() = Eigen::Vector3d(
        this->coord[0],
        this->coord[1],
        this->coord[2]
    );

    // Reconstruct quaternion from stored components
    Eigen::Quaterniond quat(
        this->coord[6],  // qw
        this->coord[3],  // qx
        this->coord[4],  // qy
        this->coord[5]   // qz
    );

    // Validate quaternion integrity:
    if(!quat.coeffs().allFinite()) {
        return false;
    }

    if (std::abs(quat.norm() - 1.0) > 1e-6) {
        return false;
    }

    // Apply rotation to pose:
    _pose.linear() = quat.normalized().toRotationMatrix();
    
    return true;
}


void SE3Conf::normalizeQ(){
    double tmp = (double) 0.0;
    for(int i = 3; i < 7; i++)
        tmp += pow(coord[i],2);

    tmp = sqrt(tmp);

    for(int i = 3; i < 7; i++)
        coord[i] /= tmp;
}

// Changes between Kautham values[4] in
void SE3Conf::fromAxisToQuaternion(double values[]){
    double angle = values[3];
    double cosang2 = cos(angle/(double)2.0);
    double sinang2 = sin(angle/(double)2.0);

    for(int i = 0; i < 3; i++)
        values[i] = values[i] * sinang2;

    values[3] = cosang2;
}

void SE3Conf::fromAxisToQuaternion(vector<double> &values){
    if(values.size() == 4 ){
        double angle = values.at(3);
        double cosang2 = cos(angle/(double)2.0);
        double sinang2 = sin(angle/(double)2.0);

        for(int i = 0; i < 3; i++)
            values.at(i) = values.at(i) * sinang2;

        values.at(3) = cosang2;

    }else if(values.size() == 7 ){
        double angle = values.at(6);
        double cosang2 = cos(angle/(double)2.0);
        double sinang2 = sin(angle/(double)2.0);

        for(int i = 3; i < 6; i++)
            values.at(i) = values.at(i) * sinang2;

        values.at(6) = cosang2;
    }
}

void SE3Conf::setPos(vector<double>& pos){
    for(int i=0;i<3;i++)
        coord[i] = pos[i];
}

vector<double>& SE3Conf::getPos() {
    for(int i=0;i<3;i++)
        _pos.at(i) = coord.at(i);
    return _pos;
}

void SE3Conf::setOrient(vector<double>& ori){
    for(int i=0;i<4;i++)
        coord[i+3] = ori[i];
}

vector<double>& SE3Conf::getOrient() {
    for(int i=0;i<4;i++)
        _ori.at(i) = coord.at(i+3);
    return _ori;
}

bool SE3Conf::setAngle(double angle){
    vector<double>& axisAngle = getAxisAngle();
    vector<double> axistmp(3);
    for(unsigned int i=0; i<3; i++)
        axistmp.at(i) =  axisAngle.at(i);

    return setAxisAngle(axistmp,angle);
}

double SE3Conf::getAngle() {
    if (coord.at(6) < -1. || coord.at(6) > 1.) throw std::invalid_argument("Called acos(x) with |x| > 1");
    return 2.*acos(coord.at(6));
}

bool SE3Conf::setAxis(vector<double>& axis) {
    if(axis.size() == 3){
        double ang = getAngle();
        return setAxisAngle(axis, ang);
    }else
        return false;
}

vector<double>& SE3Conf::getAxisAngle() {
    double sin_angle_2 = sin(getAngle()/2);

    if(sin_angle_2 != 0)
        for(int i=0;i<3;i++)
            _axisAn.at(i) = coord.at(i+3)/sin_angle_2;
    else{
        _axisAn.at(0) = 0;
        _axisAn.at(1) = 0;
        _axisAn.at(2) = 1; //it is a dummy axis since the angle is zero
    }


    _axisAn.at(3) = getAngle();
    return _axisAn;
}

bool SE3Conf::setAxisAngle(vector<double>& axis, double angle){
    try{
        double sin_angle_2 = sin(angle/2);
        coord.at(3) = axis.at(0) * sin_angle_2;	//qx
        coord.at(4) = axis.at(1) * sin_angle_2;	//qy
        coord.at(5) = axis.at(2) * sin_angle_2;	//qz
        coord.at(6) = cos(angle / 2);           //w
        normalizeQ();
        return true;
    }catch(...){
        return false;
    }
}

std::string SE3Conf::print() {
    std::ostringstream s;
    for(unsigned int i=0;i < dim; i++){
        s << "coord[" << i << "]=" << coord[i] << ";" << std::endl ;
    }
    return s.str();
}

bool SE3Conf::setCoordinates(std::vector<double>& local_coord){
    switch(local_coord.size()){
    case 7:  // The seven values (pos + quaternion) directly.
        for(int i=0; i<7; i++)
            coord.at(i) = local_coord.at(i);
        normalizeQ(); // Be sure of quaternion norm = 1;
        return true;
    case 6:
        // Coords in sampling space using Kuffner method to generate
        // uniformly-distributed random unit quaternions.
        for(int i=0; i<3; i++)
            coord.at(i) = local_coord.at(i);

        double Theta1 = 2.0 * M_PI * local_coord.at(4);
        double Theta2 = M_PI * local_coord.at(5); //- 0.5*M_PI;
        double r2 = sqrt(1 - local_coord.at(3));
        double r1 = sqrt(local_coord.at(3));
        coord.at(3) = (double)(sin(Theta1)*r1); //qx
        coord.at(4) = (double)(cos(Theta1)*r1); //qy
        coord.at(5) = (double)(sin(Theta2)*r2); //qz
        coord.at(6) = (double)(cos(Theta2)*r2); //w
        return true;
    }
    return false;
}

// This method is able to convert a SE2 configuration into a SE3 one.
bool SE3Conf::setCoordinates(SE2Conf* conf){
    if( conf == NULL ) return false;
    try{
        coord.at(0) = conf->getCoordinate(0);
        coord.at(1) = conf->getCoordinate(1);
        coord.at(2) = 0.0;
        vector<double> tmp(3, 0.0);
        tmp[2] = (double)1.0;
        setAxisAngle(tmp,conf->getCoordinate(2));
        return true;
    }catch(...){
        return false;
    }
}


double SE3Conf::getDistance2(Conf* conf, double& transWeight, double& rotWeight ){
    return getDistance2(*((SE3Conf*)conf), transWeight, rotWeight );
}

double SE3Conf::getDistance2(SE3Conf& conf, double& transWeight, double& rotWeight){
    //    if( conf == NULL ) return -1.0;
    if(conf.getType() == SE3 ){
        double dist = (double) 0.0;
        double diff = (double) 0.0;
        double rot = (double) 0.0;
        double trasl = (double) 0.0;

        // The weight is for rotational part.
        // First get the distance for translational part.
        for(unsigned int i = 0; i < 3; i++ ){
            diff = coord.at(i) - conf.getCoordinate(i);
            dist += diff * transWeight * diff * transWeight; // pow(a,2)
        }
        trasl = dist;

        //  Second get the distance for rotational part as angle.
        dist = 0.0;
        for(unsigned int i = 3; i < 7; i++ )
            dist += coord.at(i) * conf.getCoordinate(i); // inner product

        if(dist>1.0) dist=1.0; //coorect the bounds if problems with numerical errors
        else if(dist<1.0) dist=-1.0;

        dist = acos(fabs(dist));//the same as min(acos(dist),acos(-dist));
        rot = dist * rotWeight * dist * rotWeight;
        dist = trasl + rot;
        return dist;
    }
    return -1;
}

double SE3Conf::getDistance2(Conf* conf, std::vector<double>& weights){
    return getDistance2(*((SE3Conf*)conf), weights);
}

double SE3Conf::getDistance2(SE3Conf& conf, std::vector<double>& weights){
    //    if( conf == NULL ) return -1.0;
    if(conf.getType() == SE3 ){
        double dist = (double) 0.0;
        double diff = (double) 0.0;
        double rot = (double) 0.0;
        double trasl = (double) 0.0;

        if(weights.size() == 1){
            // The weight is for rotational part.
            // First get the distance for translational part.
            for(unsigned int i = 0; i < 3; i++ ){
                diff = coord.at(i) - conf.getCoordinate(i);
                dist += diff * diff; // pow(a,2)
            }
            trasl = dist;

            //  Second get the distance for rotational part as angle.
            dist = 0.0;
            for(unsigned int i = 3; i < 7; i++ )
                dist += coord.at(i) * conf.getCoordinate(i); // inner product

            if (dist < -1. || dist > 1.) throw std::invalid_argument("Called acos(x) with |x| > 1");
            dist = acos(fabs(dist));//the same as min(acos(dist),acos(-dist));
            rot = dist * weights.at(0) * dist * weights.at(0);
            dist = trasl + rot;

        }else if(weights.size() == this->dim ){
            // Each coordinate has their own weigth to will be applied
            for(unsigned int i = 0; i < coord.size(); i++ ){
                diff = coord.at(i) - conf.getCoordinate(i);
                dist += diff * weights.at(i) * diff * weights.at(i); // pow(a,2)
            }
        }else
            dist = -1;

        return dist;
    }
    return -1;
}

double SE3Conf::getDistance2(Conf* conf){
    return getDistance2(*((SE3Conf*)conf));
}

double SE3Conf::getDistance2(SE3Conf& conf){
    //    if( conf == NULL ) return -1.0;
    if(conf.getType() == SE3 ){
        double dist = (double) 0.0;
        double diff = (double) 0.0;
        double rot = (double) 0.0;
        double trasl = (double) 0.0;

        // The weight is for rotational part.
        // First get the distance for translational part.
        for(unsigned int i = 0; i < 3; i++ ){
            diff = coord.at(i) - conf.getCoordinate(i);
            dist += diff * diff; // pow(a,2)
        }
        trasl = dist;

        //  Second get the distance for rotational part as angle.
        dist = 0.0;
        for(unsigned int i = 3; i < 7; i++ )
            dist += coord.at(i) * conf.getCoordinate(i); // inner product

        if (dist < -1. || dist > 1.) throw std::invalid_argument("Called acos(x) with |x| > 1");
        dist = acos(fabs(dist));//the same as min(acos(dist),acos(-dist));
        rot = dist * dist;
        dist = trasl + rot;

        return dist;
    }
    return -1;
}

//! Returns the interpolated configuration based on coordinates.
//! Due to SE3 has a quaternion, this interpolation uses slerp.
SE3Conf SE3Conf::interpolate(SE3Conf& se3, double fraction){
    SE3Conf tmp;
    // Interpolating the translational part.
    vector<double>& other = se3.getCoordinates();
    for(unsigned int i = 0; i < 3; i++)
        tmp.coord.at(i) = coord.at(i) + fraction*(other.at(i) - coord.at(i));

    // Now the slerp
    mt::Quaternion a(coord.at(3), coord.at(4), coord.at(5), coord.at(6));
    mt::Quaternion b(other.at(3), other.at(4), other.at(5), other.at(6));

    mt::Quaternion res = a.slerp(b,fraction);

    for(unsigned int i = 0; i < 4; i++)
        tmp.coord.at(i+3) = res[i];

    normalizeQ(); // if needed

    return tmp;
}

vector<double> SE3Conf::getParams(){
    vector<double> temp(3);
    double qx,qy,qz,qw;
    try{
        //
        //Take either the original quaternion or its antipodal:
        //In order to have atan2(qx,qy) always between -pi/2 and pi/2,
        //qy is set always positive so as to have atan(qx/qy) in the first or fourth quadrant.
        //
        if(coord.at(4)>0){  //original
            qx = coord.at(3);
            qy = coord.at(4);
            qz = coord.at(5);
            qw = coord.at(6);
        }else{            //antipodal
            qx = -coord.at(3);
            qy = -coord.at(4);
            qz = -coord.at(5);
            qw = -coord.at(6);
        }
        //Parameters X1, X2 and X3 are in the range [0,1]
        temp.at(0) = qx*qx+qy*qy; //X1 = x^2+y^2
        double a = atan2(qz,qw); //returns between -pi and pi
        temp.at(2) = (double) ( a )/(M_PI); // X3 =  atan2(z/w)/pi
        a = atan2(qx,qy);         //returns between -pi and pi
        if(a < 0) a += 2. * M_PI;
        temp.at(1) = (double) (a / (2. * M_PI)); // X2 = atan2(x/y)/(2*pi)
    }catch(...){}

    return temp;
}

}

