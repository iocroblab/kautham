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
#include <pugixml.hpp>

//! Types of PCA results
enum PCAResultType {UNKNOWN, POSITION, VELOCITY};


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
    PCAResult(const arma::vec barycenter, const arma::vec eigenvalues,
              const arma::mat eigenvectors);

    //! Destructor
    virtual ~PCAResult();

    //! Distance to point
    double distance(const arma::vec x) const;

    //! Distance to another PCAResult (must be of the same type)
    double distance(const PCAResult *x, double cTrans = 1., double cRot = 4.) const;

    /*//! Distance to another PCAResult (must be of the same type)
    double distance(const PCAResult *x, double c1 = 1., double c2 = 4.,
                    double c3 = 8.) const;*/

    //! Quality of the sample set
    double quality(unsigned int samples) const;

    //! Returns PCAResult type
    virtual PCAResultType type() const;

protected:
    //! Distance between barycenters
    virtual double dTrans(const arma::vec xb) const;

    //! Distance between covariance matrices
    virtual double dRot(const arma::vec xa, const arma::mat Ua) const;

    /*//! Distance between barycenters
    virtual double d1(const arma::vec xb) const;

    //! Distance between eigenvalues
    virtual double d2(const arma::vec xa) const;

    //! Distance between eigenvectors
    virtual double d3(const arma::vec xU) const;*/
};


//! Class containing all the results from the PCA of a positions sample set
class PositionPCAResult : public PCAResult {
public:
    //! Constructor
    PositionPCAResult(const arma::vec barycenter, const arma::vec eigenvalues,
                      const arma::mat eigenvectors);

    //! Destructor
    ~PositionPCAResult();

    //! Returns PCAResult type
    PCAResultType type() const;

protected:
    //! Distance between barycenters
    virtual double dTrans(const arma::vec xb) const;

    /*//! Distance between barycenters
    double d1(const arma::vec xb) const;*/
};


//! Class containing all the results from the PCA of a velocities sample set
class VelocityPCAResult : public PCAResult {
public:
    //! Constructor
    VelocityPCAResult(const arma::vec barycenter, const arma::vec eigenvalues,
                      const arma::mat eigenvectors);

    //! Destructor
    ~VelocityPCAResult();

    //! Returns PCAResult type
    PCAResultType type() const;

protected:
    //! Distance between barycenters
    virtual double dTrans(const arma::vec xb) const;

    /*//! Distance between barycenters
    double d1(const arma::vec xb) const;*/
};


//! Distance between a PMD set and a point
double distance(const PCAResult *PMDset, const arma::vec x);

//! Distance between two PMDs set (must be of the same type)
double distance(const PCAResult *a, const PCAResult *b, double cTrans = 1.,
                double cRot = 4.);

/*//! Distance between two PMDs set (must be of the same type)
double distance(const PCAResult *a, const PCAResult *b, double c1 = 1.,
                double c2 = 4., double c3 = 8.);*/

//! Computes PCA
//! M should be [n,m]
//! if type is POSITION, limits should be [m,2] and contain minimum and
//! maximum values of x_j to normalize every variable in [0,1]
//! if type is VELOCITY, limits should be [m,1] and contain the maximum
//! absolute value of x_j to normalize every variable in [-1,1]
PCAResult *PCA(const arma::mat M, PCAResultType type, double alfa = 0.05);

//! Returns a PCAresult read from the given xml node
PCAResult *xml2PMD(const pugi::xml_node *node, PCAResultType type);

//! Writes a PCAresult into the given xml node
void PMD2xml(pugi::xml_node *node, const PCAResult *PMDset);

#endif // PCARESULT_H
