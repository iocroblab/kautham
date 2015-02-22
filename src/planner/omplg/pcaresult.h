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


#ifndef PCARESULT_H
#define PCARESULT_H


#include <armadillo>


//! Class containing all the results from the PCA of a sample set
class PCAResult {
public:
    //! Dimension
    const unsigned int dim;

    //! Barycenter vector
    const arma::vec b;

    //! Eigenvalues vector
    const arma::vec a;

    //! Eigenvectors matrix (vectors in columns)
    const arma::mat U;


    //! Constructor
    PCAResult(arma::vec barycenter, arma::vec eigenvalues,
              arma::mat eigenvectors);

    //! Distance to point
    double distance(arma::vec x);

    //! Distance to another PCAResult (must be of the same type)
    double distance(PCAResult *x, double c1 = 1., double c2 = 4.,
                    double c3 = 8.);

    //! Quality of the sample set
    double quality(unsigned int samples);

protected:
    //! Distance between barycenters
    virtual double d1(arma::vec xb);

    //! Distance between eigenvalues
    virtual double d2(arma::vec xa);

    //! Distance between eigenvectors
    virtual double d3(arma::vec xU);
};


//! Class containing all the results from the PCA of a positions sample set
class PositionPCAResult : public PCAResult {
public:
    //! Constructor
    PositionPCAResult(arma::vec barycenter, arma::vec eigenvalues,
                      arma::mat eigenvectors);

protected:
    //! Distance between barycenters
    double d1(arma::vec xb);
};


//! Class containing all the results from the PCA of a velocities sample set
class VelocityPCAResult : public PCAResult {
public:
    //! Average velocity modulus of the samples
    const double avgVel;

    //! Maximum velocity modulus of the samples
    const double maxVel;

    //! Minimum velocity modulus of the samples
    const double minVel;


    //! Constructor
    VelocityPCAResult(arma::vec barycenter, arma::vec eigenvalues,
                      arma::mat eigenvectors);

protected:
    //! Distance between barycenters
    double d1(arma::vec xb);

    //!
    double setAvgVel();

    //!
    double setMaxVel();

    //!
    double setMinVel();
};


//! Distance between a PMD set and a point
double distance(PCAResult *pmdSet, arma::vec x);

//! Distance between two PMDs set (must be of the same type)
double distance(PCAResult *a, PCAResult *b, double c1 = 1., double c2 = 4.,
                double c3 = 8.);

#endif // PCARESULT_H
