/***************************************************************************
 *   Copyright (C) 2014 by Nestor Garcia Hidalgo                           *
 *   nestor.garcia.hidalgo@upc.edu                                         *
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


#include "UR5_kinematics.h"


Solution::Solution() {
  valid = false;
}


Solution_set::Solution_set() {
  num_solutions = 0;
}


Transform DH_transform(double alpha, double a, double theta, double d) {
    Matrix3x3 matrix;
    Point3 point;

    double c_alpha = cos(alpha);
    double s_alpha = sin(alpha);
    double c_theta = cos(theta);
    double s_theta = sin(theta);

    matrix[0][0] = c_theta;
    matrix[0][1] = -s_theta;
    matrix[0][2] = 0.0;

    matrix[1][0] = c_alpha*s_theta;
    matrix[1][1] = c_alpha*c_theta;
    matrix[1][2] = -s_alpha;

    matrix[2][0] = s_alpha*s_theta;
    matrix[2][1] = s_alpha*c_theta;
    matrix[2][2] = c_alpha;

    point[0] = a;
    point[1] = -d*s_alpha;
    point[2] = d*c_alpha;

    return (Transform(Rotation(matrix),point));
}


Transform UR5_dir_kin(double *theta) {
    double alpha[] = {alpha0, alpha1, alpha2, alpha3, alpha4, alpha5};
    double a[] = {a0, a1, a2, a3, a4, a5};
    double d[] = {d1, d2, d3, d4, d5, d6};

    Transform transform;

    for (int i = 0; i < 6; i++) {
        transform *= DH_transform(alpha[i],a[i],theta[i],d[i]);
    }

    return(transform);
}


bool UR5_inv_kin(Transform transform, bool shoulder, bool elbow, bool wrist, double *theta, double default_theta6) {
    double nx = transform.getRotation().getMatrix()[0][0];
    double ny = transform.getRotation().getMatrix()[1][0];
    double nz = transform.getRotation().getMatrix()[2][0];

    double ox = transform.getRotation().getMatrix()[0][1];
    double oy = transform.getRotation().getMatrix()[1][1];
    double oz = transform.getRotation().getMatrix()[2][1];

    double ax = transform.getRotation().getMatrix()[0][2];
    double ay = transform.getRotation().getMatrix()[1][2];
    double az = transform.getRotation().getMatrix()[2][2];

    double px = transform.getTranslation()[0];
    double py = transform.getTranslation()[1];
    double pz = transform.getTranslation()[2];


    switch (shoulder) {
    case right:
        theta[0] = atan2(py-d6*ay,px-d6*ax) + acos(d4/hypot(px-d6*ax,py-d6*ay)) + PI/2.0;
        break;
    case left:
        theta[0] = atan2(py-d6*ay,px-d6*ax) - acos(d4/hypot(px-d6*ax,py-d6*ay)) + PI/2.0;
        break;
    }
    theta[0] = offset1 + saturate(theta[0] - offset1);
    if (isnan(theta[0]) || isinf(theta[0]) || (low1 > theta[0] - offset1) && (theta[0] - offset1 > high1)) {
        return (false);
    }

    double s1 = sin(theta[0]);
    double c1 = cos(theta[0]);

    switch (elbow) {
    case up:
        theta[4] = acos((px*s1-py*c1-d4)/d6);
        break;
    case down:
        theta[4] = -acos((px*s1-py*c1-d4)/d6);
        break;
    }
    theta[4] = offset5 + saturate(theta[4] - offset5);
    if (isnan(theta[4]) || isinf(theta[4]) || (low5 > theta[4] - offset5) || (theta[4] - offset5 > high5)) {
        return (false);
    }

    double c5 = cos(theta[4]);
    double s5 = sin(theta[4]);

    if (s5 != 0.0) {
        theta[5] = atan2((-s1*ox+c1*oy)/s5,(s1*nx-c1*ny)/s5);
    } else {
        theta[5] = default_theta6;
    }
    theta[5] = offset6 + saturate(theta[5] - offset6);
    if (isnan(theta[5]) || isinf(theta[5]) || (low6 > theta[5] - offset6) || (theta[5] - offset6 > high6)) {
        return (false);
    }

    double c6 = cos(theta[5]);
    double s6 = sin(theta[5]);

    double x = s1*(d5*(s6*ny+c6*oy)-ay*d6+py)+c1*(d5*(s6*nx+c6*ox)-ax*d6+px);
    double y = d5*(nz*s6+oz*c6)-az*d6-d1+pz;

    switch (wrist) {
    case in:
        theta[1] = atan2(-y,-x) + acos((pow(a3,2.0)-pow(a2,2.0)-pow(x,2.0)-pow(y,2.0))/(2*a2*hypot(x,y)));
        break;
    case out:
        theta[1] = atan2(-y,-x) - acos((pow(a3,2.0)-pow(a2,2.0)-pow(x,2.0)-pow(y,2.0))/(2*a2*hypot(x,y)));
        break;
    }
    theta[1] = offset2 + saturate(theta[1] - offset2);
    if (isnan(theta[1]) || isinf(theta[1]) || (low2 > theta[1] - offset2) || (theta[1] - offset2 > high2)) {
        return (false);
    }

    double c2 = cos(theta[1]);
    double s2 = sin(theta[1]);

    theta[2] = atan2(y-a2*s2,x-a2*c2)-theta[1];
    theta[2] = offset3 + saturate(theta[2] - offset3);
    if (isnan(theta[2]) || isinf(theta[2]) || (low3 > theta[2] - offset3) || (theta[2] - offset3 > high3)) {
        return (false);
    }

    theta[3] = atan2(c5*(c6*nz-s6*oz)-s5*az,nz*s6+oz*c6) - theta[1] - theta[2];
    theta[3] = offset4 + saturate(theta[3] - offset4);
    if (isnan(theta[3]) || isinf(theta[3]) || (low4 > theta[3] - offset4) || (theta[3] - offset4 > high4)) {
        return (false);
    }

    return (true);
}


bool UR5_inv_kin(Transform transform, Solution_set *solution_set, double default_theta6) {
    solution_set->solution = new Solution[8];

    find_theta1(transform,solution_set->solution,default_theta6);

    for (int i = 0; i < 8; i++) {
        if (solution_set->solution[i].valid) {
            solution_set->num_solutions += 1;
        }
    }

    return(solution_set->num_solutions > 0);
}


bool UR5_inv_kin(Transform transform, double *theta, double *theta_ref){
    Solution_set solution_set;

    //if UR5 inverse kinematics suceed
    if (UR5_inv_kin(transform,&solution_set,theta_ref[5])) {
        for (int i = 0; i < 8; i++) {
            if (solution_set.solution[i].valid) {

                //adjust solution
                adjust(solution_set.solution[i].theta,theta_ref);
            }
        }

        //fins nearest solution next to the reference configuration
        nearest(solution_set,theta,theta_ref);

        return (true);
    } else {
        return (false);
    }
}


void UR5_controls(double *control, double *theta) {
    double offset[] = {offset1, offset2, offset3, offset4, offset5, offset6};
    double low[] = {low1, low2, low3, low4, low5, low6};
    double high[] = {high1, high2, high3, high4, high5, high6};

    for (int i = 0; i < 6; i++) {
        control[i] = (theta[i] - offset[i] - low[i])/(high[i] - low[i]);
    }
}


void UR5_thetas(double *control, double *theta) {
    double offset[] = {offset1, offset2, offset3, offset4, offset5, offset6};
    double low[] = {low1, low2, low3, low4, low5, low6};
    double high[] = {high1, high2, high3, high4, high5, high6};

    for (int i = 0; i < 6; i++) {
        theta[i] = offset[i] + low[i] + control[i]*(high[i]-low[i]);
    }
}


void find_theta1(Transform transform, Solution *solution, double default_theta6) {
    double ax = transform.getRotation().getMatrix()[0][2];
    double ay = transform.getRotation().getMatrix()[1][2];

    double px = transform.getTranslation()[0];
    double py = transform.getTranslation()[1];

    double theta1;
    theta1 = atan2(py-d6*ay,px-d6*ax) + acos(d4/hypot(px-d6*ax,py-d6*ay)) + PI/2.0;
    theta1 = offset1 + saturate(theta1 - offset1);
    if (!isnan(theta1) && !isinf(theta1) && (low1 <= theta1 - offset1) && (theta1 - offset1 <= high1)) {
        for (int i = 0; i < 4; i++) {
            solution[i].theta[0] = theta1;
        }
        find_theta5(transform,solution,default_theta6);
    }
    theta1 = atan2(py-d6*ay,px-d6*ax) - acos(d4/hypot(px-d6*ax,py-d6*ay)) + PI/2.0;
    theta1 = offset1 + saturate(theta1 - offset1);
    if (!isnan(theta1) && !isinf(theta1) && (low1 <= theta1 - offset1) && (theta1 - offset1 <= high1)) {
        for (int i = 4; i < 8; i++) {
            solution[i].theta[0] = theta1;
        }
        find_theta5(transform,&(solution[4]),default_theta6);
    }
}


void find_theta2(Transform transform, Solution *solution) {
    double nx = transform.getRotation().getMatrix()[0][0];
    double ny = transform.getRotation().getMatrix()[1][0];
    double nz = transform.getRotation().getMatrix()[2][0];

    double ox = transform.getRotation().getMatrix()[0][1];
    double oy = transform.getRotation().getMatrix()[1][1];
    double oz = transform.getRotation().getMatrix()[2][1];

    double ax = transform.getRotation().getMatrix()[0][2];
    double ay = transform.getRotation().getMatrix()[1][2];
    double az = transform.getRotation().getMatrix()[2][2];

    double px = transform.getTranslation()[0];
    double py = transform.getTranslation()[1];
    double pz = transform.getTranslation()[2];

    double s1 = sin(solution[0].theta[0]);
    double c1 = cos(solution[0].theta[0]);

    double s6 = sin(solution[0].theta[5]);
    double c6 = cos(solution[0].theta[5]);

    double x = s1*(d5*(s6*ny+c6*oy)-ay*d6+py)+c1*(d5*(s6*nx+c6*ox)-ax*d6+px);
    double y = d5*(nz*s6+oz*c6)-az*d6-d1+pz;

    double theta2;
    theta2 = atan2(-y,-x) + acos((pow(a3,2.0)-pow(a2,2.0)-pow(x,2.0)-pow(y,2.0))/(2*a2*hypot(x,y)));
    theta2 = offset2 + saturate(theta2 - offset2);
    if (!isnan(theta2) && !isinf(theta2) && (low2 <= theta2 - offset2) && (theta2 - offset2 <= high2)) {
        solution[0].theta[1] = theta2;
        find_theta3(transform,solution);
    }
    theta2 = atan2(-y,-x) - acos((pow(a3,2.0)-pow(a2,2.0)-pow(x,2.0)-pow(y,2.0))/(2*a2*hypot(x,y)));
    theta2 = offset2 + saturate(theta2 - offset2);
    if (!isnan(theta2) && !isinf(theta2) && (low2 <= theta2 - offset2) && (theta2 - offset2 <= high2)) {
        solution[1].theta[1] = theta2;
        find_theta3(transform,&(solution[1]));
    }
}


void find_theta3(Transform transform, Solution *solution) {
    double nx = transform.getRotation().getMatrix()[0][0];
    double ny = transform.getRotation().getMatrix()[1][0];
    double nz = transform.getRotation().getMatrix()[2][0];

    double ox = transform.getRotation().getMatrix()[0][1];
    double oy = transform.getRotation().getMatrix()[1][1];
    double oz = transform.getRotation().getMatrix()[2][1];

    double ax = transform.getRotation().getMatrix()[0][2];
    double ay = transform.getRotation().getMatrix()[1][2];
    double az = transform.getRotation().getMatrix()[2][2];

    double px = transform.getTranslation()[0];
    double py = transform.getTranslation()[1];
    double pz = transform.getTranslation()[2];

    double s1 = sin(solution[0].theta[0]);
    double c1 = cos(solution[0].theta[0]);

    double s2 = sin(solution[0].theta[1]);
    double c2 = cos(solution[0].theta[1]);

    double s6 = sin(solution[0].theta[5]);
    double c6 = cos(solution[0].theta[5]);

    double x = s1*(d5*(s6*ny+c6*oy)-ay*d6+py)+c1*(d5*(s6*nx+c6*ox)-ax*d6+px);
    double y = d5*(nz*s6+oz*c6)-az*d6-d1+pz;

    double theta2 = solution[0].theta[1];

    double theta3;
    theta3 = atan2(y-a2*s2,x-a2*c2)-theta2;
    theta3 = offset3 + saturate(theta3 - offset3);
    if (!isnan(theta3) && !isinf(theta3) && (low3 <= theta3 - offset3) && (theta3 - offset3 <= high3)) {
        solution[0].theta[2] = theta3;
        find_theta4(transform,solution);
    }
}


void find_theta4(Transform transform, Solution *solution) {
    double nz = transform.getRotation().getMatrix()[2][0];

    double oz = transform.getRotation().getMatrix()[2][1];

    double az = transform.getRotation().getMatrix()[2][2];

    double s5 = sin(solution[0].theta[4]);
    double c5 = cos(solution[0].theta[4]);

    double s6 = sin(solution[0].theta[5]);
    double c6 = cos(solution[0].theta[5]);

    double theta2 = solution[0].theta[1];

    double theta3 = solution[0].theta[2];

    double theta4;
    theta4 = atan2(c5*(c6*nz-s6*oz)-s5*az,nz*s6+oz*c6) - theta2 - theta3;
    theta4 = offset4 + saturate(theta4 - offset4);
    if (!isnan(theta4) && !isinf(theta4) && (low4 <= theta4 - offset4) && (theta4 - offset4 <= high4)) {
        solution[0].theta[3] = theta4;

        solution[0].valid = true;
    }
}


void find_theta5(Transform transform, Solution *solution, double default_theta6) {
    double px = transform.getTranslation()[0];
    double py = transform.getTranslation()[1];

    double s1 = sin(solution[0].theta[0]);
    double c1 = cos(solution[0].theta[0]);

    double theta5;
    theta5 = acos((px*s1-py*c1-d4)/d6);
    theta5 = offset5 + saturate(theta5 - offset5);
    if (!isnan(theta5) && !isinf(theta5) && (low5 <= theta5 - offset5) && (theta5 - offset5 <= high5)) {
        for (int i = 0; i < 2; i++) {
            solution[i].theta[4] = theta5;
        }
        find_theta6(transform,solution,default_theta6);
    }
    theta5 = -acos((px*s1-py*c1-d4)/d6);
    theta5 = offset5 + saturate(theta5 - offset5);
    if (!isnan(theta5) && !isinf(theta5) && (low5 <= theta5 - offset5) && (theta5 - offset5 <= high5)) {
        for (int i = 2; i < 4; i++) {
            solution[i].theta[4] = theta5;
        }
        find_theta6(transform,&(solution[2]),default_theta6);
    }
}


void find_theta6(Transform transform, Solution *solution, double default_theta6) {
    double nx = transform.getRotation().getMatrix()[0][0];
    double ny = transform.getRotation().getMatrix()[1][0];

    double ox = transform.getRotation().getMatrix()[0][1];
    double oy = transform.getRotation().getMatrix()[1][1];

    double s1 = sin(solution[0].theta[0]);
    double c1 = cos(solution[0].theta[0]);

    double s5 = sin(solution[0].theta[4]);

    double theta6;
    if (s5 != 0.0) {
        theta6 = atan2((-s1*ox+c1*oy)/s5,(s1*nx-c1*ny)/s5);
    } else {
        theta6 = default_theta6;
    }
    theta6 = offset6 + saturate(theta6 - offset6);
    if (!isnan(theta6) && !isinf(theta6) && (low6 <= theta6 - offset6) && (theta6 - offset6 <= high6)) {
        for (int i = 0; i < 2; i++) {
            solution[i].theta[5] = theta6;
        }
        find_theta2(transform,solution);
    }
}


void adjust(double *theta, double *theta_ref) {
    double offset[] = {offset1, offset2, offset3};
    double tmp_theta;

    for (int i = 0; i < 3; i++) {
        //calculate the other physically achievable joint value
        if (theta[i] != offset[i]) {
            tmp_theta = theta[i] - sign(theta[i] - offset[i])*2*PI;
        } else {
            tmp_theta = theta[i] + sign(theta_ref[i] - offset[i])*2*PI;
        }

        //update joint value if necessary
        if (pow(theta_ref[i]-theta[i],2.0) > pow(theta_ref[i]-tmp_theta,2.0)) {
            theta[i] = tmp_theta;
        }
    }
}


void nearest (Solution_set solution_set, double *theta, double *theta_ref) {
    int i_min = -1;
    double distance_min = 100.0;
    double distance_i;
    for (int i = 0; i < 8; i++) {
        if (solution_set.solution[i].valid) {
            adjust(solution_set.solution[i].theta,theta_ref);

            distance_i = distance(solution_set.solution[i].theta,theta_ref);
            if (distance_i < distance_min) {
                i_min = i;
                distance_min = distance_i;
            }
        }
    }

    for (int i = 0; i < 6; i++) {
        theta[i] = solution_set.solution[i_min].theta[i];
    }
}


void print_transform(Transform transform) {
    for (int i = 0; i < 3; i++) {
        cout << transform.getRotation().getMatrix()[i][0] << "\t\t"
             << transform.getRotation().getMatrix()[i][1] << "\t\t"
             << transform.getRotation().getMatrix()[i][2] << "\t\t"
             << transform.getTranslation()[i] << endl;
    }
    cout << 0 << "\t\t" << 0 << "\t\t" << 0 << "\t\t" << 1 << endl;
    cout << endl;
}


double sign(double x) {
    if (x > 0.0) {
        return (1.0);
    } else if (x < 0.0) {
        return (-1.0);
    } else {
        return (0.0);
    }
}


double saturate(double theta) {
    return(theta - sign(theta)*2.0*PI*floor(abs(theta)/(2.0*PI) + 0.5));
}


double distance(double *theta_a, double *theta_b) {
    double sq_sum = 0.0;
    for (int i = 0; i < 6; i++) {
        sq_sum += pow(theta_a[i]-theta_b[i],2.0);
    }
    return (sqrt(sq_sum));
}
