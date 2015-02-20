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


PCAResult::PCAResult(arma::vec barycenter, arma::vec eigenvalues,
                     arma::mat eigenvectors) :
    dim(barycenter.n_elem),b(barycenter),a(eigenvalues),U(eigenvectors) {
    //check dimension
    if (eigenvalues.n_elem != dim || eigenvectors.n_cols != dim ||
            eigenvectors.n_rows != dim) {
        throw std::runtime_error("Wrong dimension values");
    }

    //check eigenvalues
    if (a[0] <= 0.) {
        throw std::runtime_error("First eigenvalue must be positve");
    }
    for (unsigned int i =  1; i < dim; i++) {
        if (a[i] < 0.) {
            throw std::runtime_error("Eigenvalues can not be negative");
        }
        if (a[i] > a[i-1]) {
            throw std::runtime_error("Eigenvalues must sorted in decreasing order");
        }
    }

    //check eigenvectors
    for (unsigned int i =  0; i < dim; i++) {
        if (std::abs(1.-std::abs(norm(U.col(i)))) > 1e-6) {
            throw std::runtime_error("Eigenvectors must be unitary");
        }
        for (unsigned int j = i+1; j < dim; j++) {
            if (std::abs(dot(U.col(i),U.col(j))) > 1e-6) {
                throw std::runtime_error("Eigenvector matrix should be orthogonal");
            }
        }
    }
}


double PCAResult::distance(arma::vec x) {
    if (x.n_elem == dim) {
        arma::vec y(dim,0.);
        arma::vec z(x-b);
        for (unsigned int i = 0; i < dim; i++) {
            y += std::max(std::min(dot(z,U.col(i)),a[i]),-a[i])*U.col(i);
        }

        return norm(z-y);
    } else {
        return -1.;
    }
}


double PCAResult::distance(PCAResult *x, double c1, double c2, double c3) {
    if (c1 >= 0. && c2 >= 0. && c3 >= 0. && (c1+c2+c3) > 0. && x->dim == dim) {
        try {
            if ((dynamic_cast<PositionPCAResult*>(this) &&
                 dynamic_cast<PositionPCAResult*>(x)) ||
                    (dynamic_cast<VelocityPCAResult*>(this) &&
                     dynamic_cast<VelocityPCAResult*>(x))) {
                return (c1*d1(x->b)+c2*d2(x->a)+c3*d3(x->U))/(c1+c2+c3);
            } else {
                return -1;
            }
        } catch(...) {
            return -1.;
        }
    } else {
        return -1.;
    }
}


double PCAResult::quality(unsigned int samples) {
    if (dim == 0) return 0.;

    if (dim == 1) return 1.;

    boost::math::fisher_f_distribution<double> F(samples-1,samples-1);
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


double PCAResult::d1(arma::vec xb) {
    return 0.;
}


double PCAResult::d2(arma::vec xa) {
    double d(0.), tmp;
    for (unsigned int i = 0; i < dim; i++) {
        tmp = std::max(1.-xa[i]/a[i],1.-a[i]/xa[i]);
        d += tmp*tmp;
    }

    return sqrt(d/double(dim));
}


double PCAResult::d3(arma::vec xU) {
    double d(0.), tmp;
    for (unsigned int i = 0; i < dim; i++) {
        tmp = dot(xU.col(i),U.col(i));
        tmp = 1.-tmp*tmp;
        d += tmp*tmp;
    }

    return sqrt(d/double(dim));
}


PositionPCAResult::PositionPCAResult(arma::vec barycenter, arma::vec eigenvalues,
                                     arma::mat eigenvectors) :
    PCAResult(barycenter,eigenvalues,eigenvectors) {
    //check barycenter
    for (unsigned int i = 0; i < dim; i++) {
        if (b[i] < 0. || b[i] > 1.) {
            throw std::runtime_error("Barycenter values must be normalized in the range [0, 1]");
        }
    }
}


double PositionPCAResult::d1(arma::vec xb) {
    double d(0.), tmp;
    for (unsigned int i = 0; i < dim; i++) {
        tmp = 1.-std::abs(1.-std::abs(xb[i]-b[i]));
        d += tmp*tmp;
    }

    return sqrt(d/double(dim));
}


VelocityPCAResult::VelocityPCAResult(arma::vec barycenter, arma::vec eigenvalues,
                                     arma::mat eigenvectors) :
    PCAResult(barycenter,eigenvalues,eigenvectors),avgVel(setAvgVel()),
    maxVel(setMaxVel()),minVel(setMinVel()) {
}


double VelocityPCAResult::d1(arma::vec xb) {
    return norm(xb-b)/2./sqrt(double(dim));
}


double VelocityPCAResult::setAvgVel() {
    return sqrt(dot(b,b)+dot(a,a)/3.);
}


double VelocityPCAResult::setMaxVel() {
    arma::vec v(b);
    double tmp;
    for (unsigned int i = 0; i < dim; i++) {
        tmp = dot(b,U.col(i));
        if (tmp >= 0.) {
            tmp = a[i];
        } else {
            tmp = -a[i];
        }
        v += tmp*U.col(i);
    }

    return norm(v);
}


double VelocityPCAResult::setMinVel() {
    arma::vec v(b);
    double tmp;
    for (unsigned int i = 0; i < dim; i++) {
        tmp = dot(b,U.col(i));
        if (tmp >= 0.) {
            tmp = -a[i];
        } else {
            tmp = a[i];
        }
        v += tmp*U.col(i);
    }

    return norm(v);
}


double distance(PCAResult *pmdSet, arma::vec x) {
    return pmdSet->distance(x);
}


double distance(PCAResult *x, PCAResult *y, double c1, double c2, double c3) {
    return x->distance(y,c1,c2,c3);
}
