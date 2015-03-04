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


#include "pcaresult.h"
#include <algorithm>
#include <boost/math/distributions/fisher_f.hpp>
#include <boost/math/distributions/normal.hpp>

PCAResult::PCAResult(const arma::vec barycenter,
                     const arma::vec eigenvalues,
                     const arma::mat eigenvectors) :
    dim(barycenter.n_elem),b(barycenter),a(eigenvalues),U(eigenvectors) {
    //check dimension
    if (eigenvalues.n_elem != dim || eigenvectors.n_cols != dim ||
            eigenvectors.n_rows != dim) {
        throw std::invalid_argument("Wrong dimension values");
    }

    //check eigenvalues
    if (a[0] < 0.) {
        throw std::invalid_argument("Eigenvalues can not be negative");
    }
    for (unsigned int i =  1; i < dim; i++) {
        if (a[i] < 0.) {
            throw std::invalid_argument("Eigenvalues can not be negative");
        }
        if (a[i] > a[i-1]) {
            throw std::invalid_argument("Eigenvalues must be sorted in decreasing order");
        }
    }

    //check eigenvectors
    for (unsigned int i =  0; i < dim; i++) {
        if (std::abs(1.-std::abs(norm(U.col(i)))) > 1e-6) {
            throw std::invalid_argument("Eigenvectors must be unitary");
        }
        for (unsigned int j = i+1; j < dim; j++) {
            if (std::abs(dot(U.col(i),U.col(j))) > 1e-6) {
                throw std::invalid_argument("Eigenvector matrix should be orthogonal");
            }
        }
    }
    /*if (std::abs(arma::det(U)-1.) > 1e-6) {
        throw std::invalid_argument("Determinant of eigenvector matrix should be 1");
    }*/
}


PCAResult::~PCAResult() {

}


double PCAResult::distance(const arma::vec x) const {
    if (x.n_elem == dim) {
        arma::vec y(dim,0.);
        arma::vec z(x-b);
        for (unsigned int i = 0; i < dim; i++) {
            y += std::max(std::min(dot(z,U.col(i)),a[i]),-a[i])*U.col(i);
        }

        return norm(z-y);
    } else {
        throw std::invalid_argument("Wrong dimension");
        return -1.;
    }
}


double PCAResult::distance(const PCAResult *x, double cTrans, double cRot) const {
    if (cTrans >= 0. && cRot >= 0. && (cTrans+cRot) > 0. &&
            x->dim == dim && type() != UNKNOWN && x && type() == x->type()) {
        return (cTrans*dTrans(x->b)+cRot*dRot(x->a,x->U))/(cTrans+cRot);
    } else {
        throw std::invalid_argument("Both PCAResult must be of the same type and weights should be positive");
        return -1.;
    }
}


/*double PCAResult::distance(const PCAResult *x, double c1, double c2, double c3) const {
    if (c1 >= 0. && c2 >= 0. && c3 >= 0. && (c1+c2+c3) > 0. &&
            x->dim == dim && type() != UNKNOWN && x && type() == x->type()) {
        return (c1*d1(x->b)+c2*d2(x->a)+c3*d3(x->U))/(c1+c2+c3);
    } else {
        throw std::invalid_argument("PCAResult must be of the type and weights should be positive");
        return -1.;
    }
}*/


double PCAResult::quality(unsigned int samples) const {
    if (dim == 0) return 0.;

    if (dim == 1) return 1.;

    boost::math::fisher_f F(samples-1,samples-1);
    double e(1.), tmp;
    for (unsigned int i = 0; i < (dim-1); i++) {
        if (a[i] == 0.) return 0.;
        for (unsigned int j = i+1; j < dim; j++) {
            tmp = a[i]/a[j];
            e *= boost::math::cdf(F,tmp*tmp);
        }
    }

    return e;
}


PCAResultType PCAResult::type() const {
    return UNKNOWN;
}


double PCAResult::dTrans(const arma::vec xb) const {
    return 0.;
}


double PCAResult::dRot(const arma::vec xa, const arma::mat xU) const {
    arma::mat A(dim,dim,arma::fill::zeros);
    arma::mat xA(dim,dim,arma::fill::zeros);
    double pMax = 1.;
    double pMin = pow(2.,-double(dim)/2.);
    for (unsigned int i = 0; i < dim; ++i) {
        A(i,i) = a(i)*a(i);
        xA(i,i) = xa(i)*xa(i);
        pMax *= a(i)+xa(dim-i-1);
        pMin *= a(i)+xa(i);
    }
    double p = sqrt(arma::det(U*A*U.t()+xU*xA*xU.t()));

    if (pMax <= pMin || p > pMax || p < pMin) {
        return 1.;

        std::cout << pMin << " " << p << " " << pMax << std::endl;
        std::cout << a << std::endl;
        std::cout << xa << std::endl;
        std::cout << U << std::endl;
        std::cout << xU << std::endl;

        throw "error";
    }

    return (pMax/p)*(p-pMin)/(pMax-pMin);
}


/*double PCAResult::d1(const arma::vec xb) const {
    return 0.;
}


double PCAResult::d2(const arma::vec xa) const {
    double d(0.), tmp;
    for (unsigned int i = 0; i < dim; i++) {
        tmp = std::max(1.-xa[i]/a[i],1.-a[i]/xa[i]);
        d += tmp*tmp;
    }

    return sqrt(d/double(dim));
}


double PCAResult::d3(const arma::vec xU) const {
    double d(0.), tmp;
    for (unsigned int i = 0; i < dim; i++) {
        tmp = dot(xU.col(i),U.col(i));
        tmp = 1.-tmp*tmp;
        d += tmp*tmp;
    }

    return sqrt(d/double(dim));
}*/


PositionPCAResult::PositionPCAResult(const arma::vec barycenter,
                                     const arma::vec eigenvalues,
                                     const arma::mat eigenvectors) :
    PCAResult(barycenter,eigenvalues,eigenvectors) {
    //check barycenter
    for (unsigned int i = 0; i < dim; i++) {
        if (b[i] < 0. || b[i] > 1.) {
            throw std::invalid_argument("Barycenter values must be in the range [0, 1]");
        }
    }
}


PositionPCAResult::~PositionPCAResult() {

}


PCAResultType PositionPCAResult::type() const {
    return POSITION;
}


double PositionPCAResult::dTrans(const arma::vec xb) const {
    double d(0.), tmp;
    for (unsigned int i = 0; i < dim; i++) {
        tmp = 1.-std::abs(1.-std::abs(xb[i]-b[i]));
        d += tmp*tmp;
    }

    return sqrt(d/double(dim));
}


/*double PositionPCAResult::d1(const arma::vec xb) const {
    double d(0.), tmp;
    for (unsigned int i = 0; i < dim; i++) {
        tmp = 1.-std::abs(1.-std::abs(xb[i]-b[i]));
        d += tmp*tmp;
    }

    return sqrt(d/double(dim));
}*/


VelocityPCAResult::VelocityPCAResult(const arma::vec barycenter,
                                     const arma::vec eigenvalues,
                                     const arma::mat eigenvectors) :
    PCAResult(barycenter,eigenvalues,eigenvectors) {
    //check barycenter
    for (unsigned int i = 0; i < dim; i++) {
        if (b[i] < -1. || b[i] > 1.) {
            throw std::invalid_argument("Barycenter values must be in the range [-1, 1]");
        }
    }
}


VelocityPCAResult::~VelocityPCAResult() {

}


PCAResultType VelocityPCAResult::type() const {
    return VELOCITY;
}


double VelocityPCAResult::dTrans(const arma::vec xb) const {
    //return norm(xb-b)/2./sqrt(double(dim));
    if (norm(b) == 0. || norm(xb) == 0.) {
        return norm(xb-b)/3./sqrt(double(dim));
    } else {
        return (2.-dot(xb,b)/norm(xb)/norm(b))*norm(xb-b)/6./sqrt(double(dim));
    }
}


/*double VelocityPCAResult::d1(const arma::vec xb) const {
    //return norm(xb-b)/2./sqrt(double(dim));
    if (norm(b) == 0. || norm(xb) == 0.) {
        return norm(xb-b)/3./sqrt(double(dim));
    } else {
        return (2.-dot(xb,b)/norm(xb)/norm(b))*norm(xb-b)/6./sqrt(double(dim));
    }
}*/


double distance(const PCAResult *PMDset, const arma::vec x) {
    return PMDset->distance(x);
}


double distance(const PCAResult *x, const PCAResult *y,
                double cTran, double cRot) {
    if (x && y) {
        return x->distance(y,cTran,cRot);
    } else {
        throw std::invalid_argument("NULL pointers");
        return -1;
    }
}


/*double distance(const PCAResult *x, const PCAResult *y,
                double c1, double c2, double c3) {
    if (x && y) {
        return x->distance(y,c1,c2,c3);
    } else {
        throw std::invalid_argument("NULL pointers");
        return -1;
    }
}*/


PCAResult *PCA(const arma::mat M, PCAResultType type, double alfa) {
    //Check the arguments
    unsigned int n = M.n_rows;
    unsigned int m = M.n_cols;
    if (n <= m) throw std::invalid_argument("Not enough samples");

    //Compute PCA
    arma::mat U;
    arma::mat S;
    arma::vec a;
    if (princomp(U,S,a,M)) {

        if (arma::det(U) < 0.) U.col(m-1) = -U.col(m-1);

        arma::vec b(trans(mean(M)));

        if (alfa < 0. || alfa > 1.) alfa = 0.05;
        boost::math::normal N(0.,1.);
        double lambda = boost::math::quantile(N,(1+pow(1-alfa,1./double(m)))/2.);
        a = lambda*sqrt(a);

        PCAResult *PMDset;
        switch (type) {
        case POSITION:
            PMDset = new PositionPCAResult(b,a,U);
            break;
        case VELOCITY:
            PMDset = new VelocityPCAResult(b,a,U);
            break;
        default:
            PMDset = new PCAResult(b,a,U);
            break;
        }

        return PMDset;
    } else {
        throw std::runtime_error("PCA failed");
    }
}


PCAResult *xml2PMD(const pugi::xml_node *node, PCAResultType type) {
    if (!node) return NULL;

    unsigned int dim = 0;
    pugi::xml_node tmp = node->child("Offset").child("DOF");
    while (tmp) {
        dim++;
        tmp = tmp.next_sibling("DOF");
    }

    arma::vec b(dim);
    arma::vec a(dim);
    arma::mat U(dim,dim);

    pugi::xml_node bNode = node->child("Offset").child("DOF");
    pugi::xml_node aNode = node->child("Control");
    pugi::xml_node UNode;

    for (unsigned int i = 0; i < dim; i++) {
        if (!bNode.attribute("value")) return NULL;
        b[i] = bNode.attribute("value").as_double();

        if (!aNode.attribute("eigValue")) return NULL;
        a[i] = aNode.attribute("eigValue").as_double();

        UNode = aNode.child("DOF");
        for (unsigned int j = 0; j < dim; j++) {
            if (!UNode.attribute("value")) return NULL;
            U.at(j,i) = UNode.attribute("value").as_double();

            UNode = UNode.next_sibling("DOF");
        }

        bNode = bNode.next_sibling("DOF");
        aNode = aNode.next_sibling("Control");
    }

    PCAResult *PMDset;
    switch (type) {
    case POSITION:
        PMDset = new PositionPCAResult(b,a,U);
        break;
    case VELOCITY:
        PMDset = new VelocityPCAResult(b,a,U);
        break;
    default:
        PMDset = new PCAResult(b,a,U);
        break;
    }

    return PMDset;
}


void PMD2xml(pugi::xml_node *node, const PCAResult *PMDset) {
    if (!node || !PMDset) return;

    pugi::xml_node offset = node->append_child("Offset");
    pugi::xml_node control;
    pugi::xml_node dof;
    for (unsigned int i = 0; i < PMDset->dim; i++) {
        dof = offset.append_child("DOF");
        dof.append_attribute("value").set_value(PMDset->b[i]);

        control = node->append_child("Control");
        control.append_attribute("eigValue").set_value(PMDset->a[i]);

        for (unsigned int j = 0; j < PMDset->dim; j++) {
            dof = control.append_child("DOF");
            dof.append_attribute("value").set_value(PMDset->U(j,i));
        }
    }
}
