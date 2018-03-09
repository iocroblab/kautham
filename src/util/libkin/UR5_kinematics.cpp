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


#include <math.h>
#include <utility>
#include <kautham/util/libkin/UR5_kinematics.h>


using namespace std;
using namespace mt;


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
    double offset[] = {offset1, offset2, offset3, offset4, offset5, offset6};
    double alpha[] = {alpha0, alpha1, alpha2, alpha3, alpha4, alpha5};
    double a[] = {a0, a1, a2, a3, a4, a5};
    double d[] = {d1, d2, d3, d4, d5, d6};

    Transform transform;
    for (int i = 0; i < 6; i++) {
        transform *= DH_transform(alpha[i],a[i],theta[i]+offset[i],d[i]);
    }

    return(transform);
}

Transform UR5_dir_kin(double *theta, const unsigned int joint) {
    double offset[] = {offset1, offset2, offset3, offset4, offset5, offset6};
    double alpha[] = {alpha0, alpha1, alpha2, alpha3, alpha4, alpha5};
    double a[] = {a0, a1, a2, a3, a4, a5};
    double d[] = {d1, d2, d3, d4, d5, d6};

    Transform transform;
    for (unsigned int i = 0; i < joint+1; i++) {
        transform *= DH_transform(alpha[i],a[i],theta[i]+offset[i],d[i]);
    }

    return(transform);
}


int UR5_inv_kin(Transform transform, bool shoulder_positive, bool wrist_positive,
                bool elbow_positive, double *theta, double *theta_ref) {

    double theta_tmp[6];

    mt::Point3 pos = transform.getTranslation();
//    std::cout << " UR5 IK library ------------------------------- " <<std::endl;
//    std::cout << " Solving IK ....." <<std::endl;
//    std::cout << "UR5: pos = " <<pos[0]<<" " <<pos[1]<<" " <<pos[2]<<" " <<std::endl;
//    std::cout << "UR5: shoulder_positive / wrist_positive / elbow_positive = "
//              << shoulder_positive<<" / "<< wrist_positive<<" / "<< elbow_positive<< std::endl;
//    std::cout << "UR5: errs = " << UR5_NO_ERROR << " "
//                                << UR5_JOINT_1  << " "
//                                << UR5_JOINT_2  << " "
//                                << UR5_JOINT_3  << " "
//                                << UR5_JOINT_4  << " "
//                                << UR5_JOINT_5  << " "
//                                << UR5_JOINT_6 << std::endl;

    //if no reference was specified, set the default values
    if (theta_ref == NULL) {
        theta_ref = new double[6];
        fill_n(theta_ref,6,0);
    }

    double ax = transform.getRotation().getMatrix()[0][2];
    double ay = transform.getRotation().getMatrix()[1][2];

    double px = transform.getTranslation()[0];
    double py = transform.getTranslation()[1];

    double c_alpha = px - d6*ax;
    double s_alpha = py - d6*ay;

    double alpha;
    double beta;

    double c_beta = d4 / hypot(c_alpha,s_alpha);

    if (in_interval(&c_beta,-1.0,1.0)) {
        alpha = atan2(s_alpha,c_alpha);
        beta = acos(c_beta);

        theta_tmp[0] = alpha + (2.0*shoulder_positive -1.0)*beta + PI/2.0 - offset1;

        adjust(&theta_tmp[0],theta_ref[0]);

        if (!in_interval(&theta_tmp[0],low1,high1)) {
            //theta1 is out of range
            return (UR5_JOINT_1);
        }
    } else {
        //theta1 has no solution
        return (UR5_JOINT_1);
    }

    double s1 = sin(theta_tmp[0]+offset1);
    double c1 = cos(theta_tmp[0]+offset1);

    c_alpha = (-px*s1 + py*c1 + d4) / d6;

    if (in_interval(&c_alpha,-1.0,1.0)) {
        alpha = acos(c_alpha);

        theta_tmp[4] = (2.0*wrist_positive-1.0)*alpha -offset5;

        adjust(&theta_tmp[4],theta_ref[4]);

        if (!in_interval(&theta_tmp[4],low5,high5)) {
            //theta5 is out of range
            return (UR5_JOINT_5);
        }
    } else {
        //theta5 has no solution
        return (UR5_JOINT_5);
    }

    double nx = transform.getRotation().getMatrix()[0][0];
    double ny = transform.getRotation().getMatrix()[1][0];

    double ox = transform.getRotation().getMatrix()[0][1];
    double oy = transform.getRotation().getMatrix()[1][1];

    double s5 = sin(theta_tmp[4]+offset5);

    if (in_interval(&s5,0.0,0.0)) {
        theta_tmp[5] = theta_ref[5];
    } else {
        theta_tmp[5] = atan2((s1*ox-c1*oy)/s5,(-s1*nx+c1*ny)/s5) - offset6;
    }
    adjust(&theta_tmp[5],theta_ref[5]);

    if (!in_interval(&theta_tmp[5],low6,high6)) {
        //theta6 is out of range
        return (UR5_JOINT_6);
    }

    double nz = transform.getRotation().getMatrix()[2][0];

    double oz = transform.getRotation().getMatrix()[2][1];

    double az = transform.getRotation().getMatrix()[2][2];

    double pz = transform.getTranslation()[2];

    double c6 = cos(theta_tmp[5]+offset6);
    double s6 = sin(theta_tmp[5]+offset6);

    double x = s1*(d5*(s6*ny+c6*oy)-ay*d6+py)+c1*(d5*(s6*nx+c6*ox)-ax*d6+px);
    double y = d5*(nz*s6+oz*c6)-az*d6-d1+pz;

    c_beta = (a3*a3 - a2*a2 - x*x - y*y) / (2*a2*hypot(x,y));

    if (in_interval(&c_beta,-1.0,1.0)) {
        alpha = atan2(-y,-x);
        beta = acos(c_beta);

        theta_tmp[1] = alpha +(2.0*elbow_positive-1.0)*beta - offset2;

        adjust(&theta_tmp[1],theta_ref[1]);

        if (!in_interval(&theta_tmp[1],low2,high2)) {
            //theta2 is out of range
            return (UR5_JOINT_2);
        }
    } else {
        //theta2 has no solution
        return (UR5_JOINT_2);
    }

    double c2 = cos(theta_tmp[1]+offset2);
    double s2 = sin(theta_tmp[1]+offset2);

    theta_tmp[2] = atan2(y-a2*s2,x-a2*c2) - (theta_tmp[1]+offset2) - offset3;
    adjust(&theta_tmp[2],theta_ref[2]);
    if (!in_interval(&theta_tmp[2],low3,high3)) {
        //theta3 is out of range
        return (UR5_JOINT_3);
    }

    double c5 = cos(theta_tmp[4]+offset5);

    alpha = atan2(c5*(c6*nz-s6*oz)-s5*az,-nz*s6-oz*c6);

    theta_tmp[3] = alpha - (theta_tmp[1]+offset2) - (theta_tmp[2]+offset3) - offset4;
    adjust(&theta_tmp[3],theta_ref[3]);

    if (!in_interval(&theta_tmp[3],low4,high4)) {
        //theta4 is out of range
        return (UR5_JOINT_4);
    }

    //only if a solution was found, update theta values
    for (int i = 0; i < 6; i++) {
        theta[i] = theta_tmp[i];
    }
    return (UR5_NO_ERROR);
}


bool UR5_inv_kin(Transform transform, Solution_set *solution_set,
                 double *theta_ref) {
    //if no reference was specified, set the default values
    if (theta_ref == NULL) {
        theta_ref = new double[6];
        fill_n(theta_ref,6,0);
    }

    solution_set->solution = new Solution[8];

    find_theta1(transform,solution_set->solution,theta_ref);

    for (int i = 0; i < 8; i++) {
        if (solution_set->solution[i].valid) {
            solution_set->num_solutions += 1;
        }
    }

    return(solution_set->num_solutions > 0);
}


bool UR5_inv_kin(Transform transform, double *theta, double *theta_ref) {
    Solution_set solution_set;
    //if UR5 inverse kinematics suceed
    if (UR5_inv_kin(transform,&solution_set,theta_ref)) {
        //find nearest solution next to the reference configuration
        nearest(solution_set,theta,theta_ref);

        return true;
    } else {
        return false;
    }
}


void UR5_controls(double *control, double *theta) {
    double low[] = {low1, low2, low3, low4, low5, low6};
    double high[] = {high1, high2, high3, high4, high5, high6};

    for (int i = 0; i < 6; i++) {
        control[i] = (theta[i] - low[i])/(high[i] - low[i]);

        //ensure control is physically achievable
        in_interval(&control[i],0.0,1.0);
    }
}


void UR5_thetas(double *control, double *theta) {
    double low[] = {low1, low2, low3, low4, low5, low6};
    double high[] = {high1, high2, high3, high4, high5, high6};

    for (int i = 0; i < 6; i++) {
        theta[i] = low[i] + control[i]*(high[i]-low[i]);

        //ensure theta is physically achievable
        in_interval(&theta[i],low[i],high[i]);
    }
}


void find_theta1(Transform transform, Solution *solution,
                 double *theta_ref) {
    double ax = transform.getRotation().getMatrix()[0][2];
    double ay = transform.getRotation().getMatrix()[1][2];

    double px = transform.getTranslation()[0];
    double py = transform.getTranslation()[1];

    double c_alpha = px - d6*ax;
    double s_alpha = py - d6*ay;

    double c_beta = d4 / hypot(c_alpha,s_alpha);

    if (in_interval(&c_beta,-1.0, 1.0)) {
        double alpha = atan2(s_alpha,c_alpha);
        double beta = acos(c_beta);

        double theta1;

        theta1 = alpha + beta + PI/2.0 - offset1;
        adjust(&theta1,theta_ref[0]);

        if (in_interval(&theta1,low1,high1)) {
            for (int i = 0; i < 4; i++) {
                solution[i].theta[0] = theta1;
            }

            find_theta5(transform,solution,theta_ref);
        } else {
            //theta1 is out of range
        }

        theta1 = alpha - beta + PI/2.0 - offset1;
        adjust(&theta1,theta_ref[0]);

        if (in_interval(&theta1,low1,high1)) {
            for (int i = 4; i < 8; i++) {
                solution[i].theta[0] = theta1;
            }

            find_theta5(transform,&solution[4],theta_ref);
        } else {
            //theta1 is out of range
        }
    } else {
        //theta1 has no solution
    }
}


void find_theta2(Transform transform, Solution *solution,
                 double *theta_ref) {
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

    double theta1 = solution[0].theta[0]+offset1;
    double s1 = sin(theta1);
    double c1 = cos(theta1);

    double theta6 = solution[0].theta[5]+offset6;
    double s6 = sin(theta6);
    double c6 = cos(theta6);

    double x = s1*(d5*(s6*ny+c6*oy)-ay*d6+py)+c1*(d5*(s6*nx+c6*ox)-ax*d6+px);
    double y = d5*(nz*s6+oz*c6)-az*d6-d1+pz;

    double c_beta = (a3*a3 - a2*a2 - x*x - y*y) / (2*a2*hypot(x,y));

    if (in_interval(&c_beta,-1.0,1.0)) {
        double alpha = atan2(-y,-x);
        double beta = acos(c_beta);

        double theta2;

        theta2 = alpha + beta - offset2;
        adjust(&theta2,theta_ref[1]);
        if (in_interval(&theta2,low2,high2)) {
            solution[0].theta[1] = theta2;

            find_theta3(transform,solution,theta_ref);
        } else {
            //theta2 is out of range
        }

        theta2 = alpha - beta - offset2;
        adjust(&theta2,theta_ref[1]);
        if (in_interval(&theta2,low2,high2)) {
            solution[1].theta[1] = theta2;

            find_theta3(transform,&solution[1],theta_ref);
        } else {
            //theta2 is out of range
        }
    } else {
        //theta2 has no solution
    }
}


void find_theta3(Transform transform, Solution *solution,
                 double *theta_ref) {
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

    double theta1 = solution[0].theta[0]+offset1;
    double s1 = sin(theta1);
    double c1 = cos(theta1);

    double theta2 = solution[0].theta[1]+offset2;
    double s2 = sin(theta2);
    double c2 = cos(theta2);

    double theta6 = solution[0].theta[5]+offset6;
    double s6 = sin(theta6);
    double c6 = cos(theta6);

    double x = s1*(d5*(s6*ny+c6*oy)-ay*d6+py)+c1*(d5*(s6*nx+c6*ox)-ax*d6+px);
    double y = d5*(nz*s6+oz*c6)-az*d6-d1+pz;

    double alpha = atan2(y-a2*s2,x-a2*c2);

    double theta3;

    theta3 = alpha - theta2 - offset3;
    adjust(&theta3,theta_ref[2]);
    if (in_interval(&theta3,low3,high3)) {
        solution[0].theta[2] = theta3;

        find_theta4(transform,solution,theta_ref);
    } else {
        //theta3 is out of range
    }
}


void find_theta4(Transform transform, Solution *solution,
                 double *theta_ref) {
    double nz = transform.getRotation().getMatrix()[2][0];

    double oz = transform.getRotation().getMatrix()[2][1];

    double az = transform.getRotation().getMatrix()[2][2];

    double theta5 = solution[0].theta[4]+offset5;
    double s5 = sin(theta5);
    double c5 = cos(theta5);

    double theta6 = solution[0].theta[5]+offset6;
    double s6 = sin(theta6);
    double c6 = cos(theta6);

    double alpha = atan2(c5*(c6*nz-s6*oz)-s5*az,-nz*s6-oz*c6);

    double theta2 = solution[0].theta[1]+offset2;

    double theta3 = solution[0].theta[2]+offset3;

    double theta4;

    theta4 = alpha - theta2 - theta3 - offset4;
    adjust(&theta4,theta_ref[3]);
    if (in_interval(&theta4,low4,high4)) {
        solution[0].theta[3] = theta4;

        solution[0].valid = true;
    } else {
        //theta4 is out of range
    }
}


void find_theta5(Transform transform, Solution *solution,
                 double* theta_ref) {
    double px = transform.getTranslation()[0];
    double py = transform.getTranslation()[1];

    double theta1 = solution[0].theta[0]+offset1;
    double s1 = sin(theta1);
    double c1 = cos(theta1);

    double c_alpha = (-px*s1+py*c1+d4) / d6;

    if (in_interval(&c_alpha,-1.0,1.0)) {
        double alpha = acos(c_alpha);

        double theta5;

        theta5 = alpha - offset5;
        adjust(&theta5,theta_ref[4]);
        if (in_interval(&theta5,low5,high5)) {
            for (int i = 0; i < 2; i++) {
                solution[i].theta[4] = theta5;
            }

            find_theta6(transform,solution,theta_ref);
        } else {
            //theta5 is out of range
        }

        theta5 = -alpha - offset5;
        adjust(&theta5,theta_ref[4]);
        if (in_interval(&theta5,low5,high5)) {
            for (int i = 2; i < 4; i++) {
                solution[i].theta[4] = theta5;
            }

            find_theta6(transform,&solution[2],theta_ref);
        } else {
            //theta5 is out of range
        }
    } else {
        //theta5 has no solution
    }
}


void find_theta6(Transform transform, Solution *solution,
                 double *theta_ref) {
    double nx = transform.getRotation().getMatrix()[0][0];
    double ny = transform.getRotation().getMatrix()[1][0];

    double ox = transform.getRotation().getMatrix()[0][1];
    double oy = transform.getRotation().getMatrix()[1][1];

    double theta1 = solution[0].theta[0]+offset1;
    double s1 = sin(theta1);
    double c1 = cos(theta1);

    double s5 = sin(solution[0].theta[4]+offset5);

    double theta6;
    if (in_interval(&s5,0.0,0.0,0.002)) {
        theta6 = theta_ref[5];
    } else {
        theta6 = atan2((s1*ox-c1*oy)/s5,(-s1*nx+c1*ny)/s5) - offset6;
        adjust(&theta6,theta_ref[5]);
    }
    if (in_interval(&theta6,low6,high6)) {
        for (int i = 0; i < 2; i++) {
            solution[i].theta[5] = theta6;
        }

        find_theta2(transform,solution,theta_ref);
    } else {
        //theta6 is out of range
    }
}


void adjust(double *theta, double theta_ref) {
    double tmp_theta;

    *theta = saturate(*theta);

    //calculate the other physically achievable joint value
    //if theta is aproximately equal to zero
    if (in_interval(theta,0,0)) {
        tmp_theta = *theta + sign(theta_ref)*2*PI;
    } else {
        tmp_theta = *theta - sign(*theta)*2*PI;
    }

    //update joint value if necessary
    if (abs(theta_ref-*theta) > abs(theta_ref-tmp_theta)) {
        *theta = tmp_theta;
    }

    //theta = theta_ref if |theta-theta_ref| < TOL
    in_interval(theta,theta_ref,theta_ref);
}


void nearest (Solution_set solution_set, double *theta, double *theta_ref) {
    int i_min = -1;
    double distance_min = MAX_DIST;
    double distance_i;
    for (int i = 0; i < 8; i++) {
        if (solution_set.solution[i].valid) {
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


double sign(double x) {
    return ((x>0)-(x<0));
}


double saturate(double theta) {
    if (in_interval(&theta,-PI,PI)) {
        return (theta);
    } else {
        return(theta - sign(theta)*2.0*PI*floor(abs(theta)/(2.0*PI) + 0.5));
    }
}


double distance(double *theta_a, double *theta_b) {
    double sq_sum = 0.0;
    for (int i = 0; i < 6; i++) {
        sq_sum += pow(theta_a[i]-theta_b[i],2.0);
    }
    return (sqrt(sq_sum));
}


bool in_interval(double *x, double xmin, double xmax, double tol) {
    //if interval was incorrectly defined
    if (xmin > xmax) {
        //swap limits
        swap(xmin,xmax);
    }

    //if x is in the interval
    if ((xmin-tol <= *x) && (*x <= xmax+tol)) {
        //adjust x if necessary
        if (*x < xmin) {
            *x = xmin;
        } else if (xmax < *x) {
            *x = xmax;
        }

        return true;
    } else {
        return false;
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
