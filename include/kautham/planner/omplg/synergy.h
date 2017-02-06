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


#ifndef SYNERGY_H
#define SYNERGY_H


#include <armadillo>
#include <pugixml.hpp>


//! Synergy orders
enum SynergyOrder {UNKNOWN, ZERO, FIRST};


//! Class containing all the results from the PCA of a sample set
class Synergy {
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
    Synergy(const arma::vec barycenter, const arma::vec eigenvalues,
              const arma::mat eigenvectors);

    //! Destructor
    virtual ~Synergy();

    //! Distance to point
    double distance(const arma::vec x) const;

    //! Distance to another Synergy (must be of the same order)
    double distance(const Synergy *x, double cTrans = 1., double cRot = 4.) const;

    //! Quality of the sample set
    double quality(unsigned int samples) const;

    //! Returns Synergy order
    virtual SynergyOrder order() const;

    //! Returns a measure of the alignment between x and the synergies
    double alignment(arma::vec x) const;

    unsigned int getReducedDimension() const {return nr;}
protected:
    //! Distance between barycenters
    virtual double dTrans(const arma::vec xb) const;

    //! Distance between covariance matrices
    virtual double dRot(const arma::vec xa, const arma::mat Ua) const;

    //! Covariance matrix
    const arma::mat cov;

    //! Inverse of the covariance matrix
    const arma::mat covInv;

    //! Square of the norm of b
    const double bb;

    //! Weight parameter for the alignment computation
    const double prob;

    unsigned int nr;
};


//! Class containing all the results from the PCA of a positions sample set
class ZeroOrderSynergy : public Synergy {
public:
    //! Constructor
    ZeroOrderSynergy(const arma::vec barycenter, const arma::vec eigenvalues,
                      const arma::mat eigenvectors);

    //! Destructor
    ~ZeroOrderSynergy();

    //! Returns Synergy order
    SynergyOrder order() const;

protected:
    //! Distance between barycenters
    virtual double dTrans(const arma::vec xb) const;
};


//! Class containing all the results from the PCA of a velocities sample set
class FirstOrderSynergy : public Synergy {
public:
    //! Constructor
    FirstOrderSynergy(const arma::vec barycenter, const arma::vec eigenvalues,
                      const arma::mat eigenvectors);

    //! Destructor
    ~FirstOrderSynergy();

    //! Returns Synergy order
    SynergyOrder order() const;

protected:
    //! Distance between barycenters
    virtual double dTrans(const arma::vec xb) const;
};


//! Distance between a PMD set and a point
double distance(const Synergy *synergy, const arma::vec x);

//! Distance between two PMDs set (must be of the same order)
double distance(const Synergy *a, const Synergy *b, double cTrans = 1.,
                double cRot = 4.);

//! Computes PCA
//! M should be [n,m]
//! if order is ZERO, limits should be [m,2] in size and contain minimum and
//! maximum values of x_j to normalize every variable in [0,1]
//! if order is FIRST, limits should be [m,1] in size and contain the maximum
//! absolute value of x_j to normalize every variable in [-1,1]
Synergy *PCA(const arma::mat M, SynergyOrder order, double alfa = 0.05);

//! Returns a Synergy read from the given xml node
Synergy *xml2synergy(const pugi::xml_node *node, SynergyOrder order);

//! Writes a Synergy into the given xml node
void synergy2xml(pugi::xml_node *node, const Synergy *synergy);


#endif // SYNERGY_H
