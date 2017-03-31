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


#include <kautham/planner/omplg/synergy.h>
#include <algorithm>
#include <math.h>
#include <boost/math/distributions/fisher_f.hpp>
#include <boost/math/distributions/normal.hpp>


#define SIGN(x) ((x>0.0)-(x<0.0))


Synergy::Synergy(const arma::vec barycenter,
                 const arma::vec eigenvalues,
                 const arma::mat eigenvectors) :
    dim(barycenter.n_elem),b(barycenter),a(eigenvalues),U(eigenvectors),
    cov(U*diagmat(a%a)*U.t()),covInv(U*solve(diagmat(a%a),U.t())),bb(dot(b,b)),
    prob(1-erf(bb/sqrt(DBL_EPSILON+2.0*as_scalar(b.t()*cov*b)))) {

    std::cout << std::endl << "Prob " << prob << std::endl << std::endl;

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
            throw std::invalid_argument("Eigenvalues must sorted in decreasing order");
        }
    }

    //check eigenvectors
    for (unsigned int i =  0; i < dim; i++) {
        if (fabs(1.-fabs(norm(U.col(i)))) > 1e-6) {
            throw std::invalid_argument("Eigenvectors must be unitary");
        }
        for (unsigned int j = i+1; j < dim; j++) {
            if (fabs(dot(U.col(i),U.col(j))) > 2e-6) {
                throw std::invalid_argument("Eigenvector matrix should be orthogonal");
            }
        }
    }

    nr = 0;
    double accum = 0.;
    double total = arma::sum(a);
    while (accum < 0.95*total) {
        accum += a[nr];
        nr++;
    }
    std::cout << "Reduced number of FOS synergies " << nr << std::endl;
}


Synergy::~Synergy() {

}


double Synergy::distance(const arma::vec x) const {
    if (x.n_elem == dim) {
        //The distance between x and the synergy box is the distance between x
        //and the porjection of x in the synergy box

        /*arma::vec y(dim,arma::fill::zeros);
        arma::vec z(x-b);
        for (unsigned int i = 0; i < dim; i++) {
            y += std::max(std::min(dot(z,U.col(i)),a[i]),-a[i])*U.col(i);
        }

        return norm(z-y);*/

        arma::vec u(U.t()*(x-b));
        arma::vec w(u);
        for (unsigned int i = 0; i < dim; ++i) {
            w[i] = std::max(std::min(w[i],a[i]),-a[i]);
        }

        return norm(w-u);
    } else {
        throw std::invalid_argument("Wrong dimension");
        return -1.;
    }
}


double Synergy::distance(const Synergy *x, double cTrans, double cRot) const {
    if (cTrans >= 0. && cRot >= 0. && (cTrans+cRot) > 0. &&
            x->dim == dim && order() != UNKNOWN && x && order() == x->order()) {
        double d = (cTrans*dTrans(x->b)+cRot*dRot(x->a,x->U))/(cTrans+cRot);
        if (std::isnan(d) || d < -DBL_EPSILON || d > 1 + DBL_EPSILON) throw;

        return std::min(std::max(d,0.),1.);
    } else {
        throw std::invalid_argument("Both synergies must be of the same order "
                                    "and the weights should be positive");
        return -1.;
    }
}


double Synergy::quality(unsigned int samples) const {
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


SynergyOrder Synergy::order() const {
    return UNKNOWN;
}


double Synergy::dTrans(const arma::vec /*xb*/) const {
    return 0.;
}


double Synergy::dRot(const arma::vec xa, const arma::mat xU) const {
    //Compute pMax and pMin
    arma::mat A(diagmat(a%a));
    arma::mat xA(diagmat(xa%xa));
    double pMax = 1.;
    double pMin = pow(2.,-double(dim)/2.);
    for (unsigned int i = 0; i < dim; ++i) {
        pMax *= a(i)+xa(dim-i-1);
        pMin *= a(i)+xa(i);
    }
    if (fabs(pMax-pMin) < DBL_EPSILON) {//pMax == pMin
        return 0.5;//Undefined
    } else {//pMax != pMin
        if (pMax < pMin) throw;//Always it must be pMax >= pMin
    }

    //Compute p
    double p = arma::det(U*A*U.t()+xU*xA*xU.t());
    if (fabs(p) < DBL_EPSILON) {
        p = 0.;
    } else {
        if (p < 0.) {
            throw;//Always it must be p >= 0.
        } else {
            p = sqrt(p);
        }
    }
    if (fabs(p-pMin) < DBL_EPSILON) {//pMin == p < pMax
        return 0.;
    } else{
        if (p < pMin) throw;//Always it must be p >= pMin
    }

    //Compute d
    double d = (pMax*(p-pMin))/(p*(pMax-pMin)+DBL_EPSILON);
    if (std::isnan(d) || d < -DBL_EPSILON || d > 1. + DBL_EPSILON) {
        throw;//Always it must be 0. <= d <= 1.
    } else if (d < 0.) {
        d = 0.;
    } else if (d > 1.) {
        d = 1.;
    }

    return d;
}


double Synergy::alignment(arma::vec x) const {
    double xb(dot(x,b));
    double c1(2.0*sqrt(as_scalar(x.t()*cov*x))/norm(x)/a[0]-1.0);
    double c0;
    if (fabs(xb) < DBL_EPSILON) {
        c0 = 0;
    } else {
        arma::vec y(x*bb/xb-b);
       c0 = SIGN(xb)*exp(-0.5*as_scalar(y.t()*covInv*y));
    }
    double c(prob*c1+(1.0-prob)*c0);
    if (c < -1.0+DBL_EPSILON) return DBL_MAX;

    double f(prob+(1.0-prob)*(acos(xb/norm(b)/norm(x))/M_PI*2.0));
    return sqrt((1.0-c)/(1.0+c))*f;
    //return sqrt((1.0-c)/(1.0+c))*acos(xb/norm(b)/norm(x))/M_PI;
}


/*double Synergy::alignment(arma::vec x) {
    arma::vec y(arma::zeros(dim));
    arma::vec z;
    double xz;
    for (unsigned int i = 0; i < dim; ++i) {
        z = b+a[i]*U.col(i);+
        xz = dot(x,z);
        if (xz > 0.0) {
            y += (a[i]*xz/dot(z,z))*z;
        }
    }
    for (unsigned int i = 0; i < dim; ++i) {
        z = b-a[i]*U.col(i);
        xz = dot(x,z);
        if (xz > 0.0) {
            y += (a[i]*xz/dot(z,z))*z;
        }
    }

    return norm(z)/norm(x)/a[1];
}*/



ZeroOrderSynergy::ZeroOrderSynergy(const arma::vec barycenter,
                                   const arma::vec eigenvalues,
                                   const arma::mat eigenvectors) :
    Synergy(barycenter,eigenvalues,eigenvectors) {
    //check barycenter
    for (unsigned int i = 0; i < dim; i++) {
        if (b[i] < 0. || b[i] > 1.) {
            throw std::invalid_argument("Barycenter values must be in the range [0, 1]");
        }
    }
}


ZeroOrderSynergy::~ZeroOrderSynergy() {

}


SynergyOrder ZeroOrderSynergy::order() const {
    return ZERO;
}


double ZeroOrderSynergy::dTrans(const arma::vec xb) const {
    double d(0.), tmp;
    for (unsigned int i = 0; i < dim; i++) {
        tmp = 1.-fabs(1.-fabs(xb[i]-b[i]));
        d += tmp*tmp;
    }

    return sqrt(d/double(dim));
}


FirstOrderSynergy::FirstOrderSynergy(const arma::vec barycenter,
                                     const arma::vec eigenvalues,
                                     const arma::mat eigenvectors) :
    Synergy(barycenter,eigenvalues,eigenvectors) {
    //check barycenter
    for (unsigned int i = 0; i < dim; i++) {
        if (b[i] < -1. || b[i] > 1.) {
            throw std::invalid_argument("Barycenter values must be in the range [-1, 1]");
        }
    }
}


FirstOrderSynergy::~FirstOrderSynergy() {

}


SynergyOrder FirstOrderSynergy::order() const {
    return FIRST;
}


double FirstOrderSynergy::dTrans(const arma::vec xb) const {
    if (norm(b) == 0. || norm(xb) == 0.) {
        return norm(xb-b)/3./sqrt(double(dim));
    } else {
        return (2.-dot(xb,b)/norm(xb)/norm(b))*norm(xb-b)/6./sqrt(double(dim));
    }
}


double distance(const Synergy *synergy, const arma::vec x) {
    return synergy->distance(x);
}


double distance(const Synergy *x, const Synergy *y,
                double cTran, double cRot) {
    if (x && y) {
        return x->distance(y,cTran,cRot);
    } else {
        throw std::invalid_argument("NULL pointers");
        return -1;
    }
}


Synergy *PCA(const arma::mat M, SynergyOrder order, double alfa) {
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

        Synergy *synergy;
        switch (order) {
            case ZERO:
                synergy = new ZeroOrderSynergy(b,a,U);
            break;
            case FIRST:
                synergy = new FirstOrderSynergy(b,a,U);
            break;
            default:
                synergy = new Synergy(b,a,U);
            break;
        }

        return synergy;
    } else {
        throw std::runtime_error("PCA failed");
    }
}


Synergy *xml2synergy(const pugi::xml_node *node, SynergyOrder order) {
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

    Synergy *synergy;
    switch (order) {
        case ZERO:
            synergy = new ZeroOrderSynergy(b,a,U);
        break;
        case FIRST:
            synergy = new FirstOrderSynergy(b,a,U);
        break;
        default:
            synergy = new Synergy(b,a,U);
        break;
    }

    return synergy;
}


void synergy2xml(pugi::xml_node *node, const Synergy *synergy) {
    if (!node || !synergy) return;

    pugi::xml_node offset = node->append_child("Offset");
    pugi::xml_node control;
    pugi::xml_node dof;
    for (unsigned int i = 0; i < synergy->dim; i++) {
        dof = offset.append_child("DOF");
        dof.append_attribute("value").set_value(synergy->b[i]);

        control = node->append_child("Control");
        control.append_attribute("eigValue").set_value(synergy->a[i]);

        for (unsigned int j = 0; j < synergy->dim; j++) {
            dof = control.append_child("DOF");
            dof.append_attribute("value").set_value(synergy->U(j,i));
        }
    }
}
