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


#ifndef PCAKDTREE_H
#define PCAKDTREE_H

#include "pcaresult.h"
#include <armadillo>


//!
typedef struct {
    PCAResult *childL;
    PCAResult *childR;
    double fV;
    double fD;
    double f;
    unsigned int x;
    unsigned int axis;
} OptiData;


//! Class that represents a node of a PCA kd tree
class PCAkdtree_node {
private:
    friend class PCAkdtree;

    //! Constructor
    PCAkdtree_node(const PCAkdtree_node *parentNode, PCAResult *PMD,
                   const arma::mat Mv, const arma::mat Cp, const arma::mat limits,
                   double thV, double thD, double tol, double alfa);

    //! Constructor
    PCAkdtree_node(const pugi::xml_node *node, const std::vector<PCAResult*> *PMD,
                   const PCAkdtree_node *parentNode = NULL);

    //! Destructor
    ~PCAkdtree_node();

    //! Returns the PMD set of the cell of the sample c
    PCAResult *getPMD(const arma::vec c);

    //! Returns the color of the cell of the sample c
    unsigned int getColor(const arma::vec c);

    //! Returns true if the node is a leaf
    bool isLeaf();

    //! Finds the best location to divide the cell
    OptiData * minimize(const std::vector<std::vector<unsigned int> > *X,
                        const arma::mat M, double thV, double thD,
                        double tol, double alfa);

    //! Writes node data in the specified xml node and
    //! copies the PMD set of the cell into the given vector
    void xml(pugi::xml_node *node, std::vector<PCAResult*> *PMDv);

    //!
    std::vector<std::vector<unsigned int> > *sortByAxis(const arma::mat Cp,
                                                        unsigned int j);



    //! Parent node
    const PCAkdtree_node *parent;

    //! Velocity PMD set of the node
    PCAResult *PMDv;

    //! Direction of the division
    unsigned int axis;

    //! Where the cell is divided
    double location;

    //! Limits of the cell
    arma::mat lim;

    //! Left child node
    PCAkdtree_node *childL;

    //! Right child node
    PCAkdtree_node *childR;

    //! Color of the cell
    unsigned int color;
};


//! Class that represents a PCA kd tree
class PCAkdtree {
public:
    //! Constructor
    PCAkdtree(const arma::mat Mp, const arma::mat Mv, const arma::mat posLimits,
              const arma::vec velLimits, double tol = 0.001, double alfa = 0.05);

    //! Constructor
    PCAkdtree(const std::string filename);

    //! Destructor
    ~PCAkdtree();

    //! Returns the PMD set of the cell of the sample c
    PCAResult *getPMD(const arma::vec x);

    //! Loads the tree structure from a file
    bool load(const std::string filename);

    //! Saves the tree structure into a file
    bool save(const std::string filename);

    /*//! Plots the cells of the tree and the given samples (only for 2D)
    void plot(const arma::mat Cp = arma::mat(0,0));*/

    const arma::vec getVelocityLimits() {return Lv;}

private:
    //! Root node of the tree
    PCAkdtree_node *root;

    //! Position PMD set of the tree
    PCAResult *PMDp;

    //! Position limits
    arma::mat Lp;

    //! Velocity limits
    arma::vec Lv;
};

//! Constructs a PCA kd tree
PCAkdtree *makePCAkdtree(const arma::mat M, const arma::vec t,
                         const arma::mat posLimits = arma::mat(0,0),
                         const arma::vec velLimits = arma::vec(0u));

#endif // PCAKDTREE_H
