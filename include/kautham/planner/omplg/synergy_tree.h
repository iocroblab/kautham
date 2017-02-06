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


#ifndef SYNERGY_TREE_H
#define SYNERGY_TREE_H


#include "synergy.h"


//! Data structure representing an optimum cell division
typedef struct {
    Synergy *childL;
    Synergy *childR;
    double fV;
    double fD;
    double f;
    unsigned int x;
    unsigned int axis;
} OptiData;


//! Class that represents a node of a PCA kd tree
class SynergyTree_node {
private:
    friend class SynergyTree;

    //! Constructor
    SynergyTree_node(const SynergyTree_node *parentNode, Synergy *synergy,
                     const arma::mat Mv, const arma::mat Cp, const arma::mat limits,
                     double thV, double thD, double tol, double alfa);

    //! Constructor
    SynergyTree_node(const pugi::xml_node *node, const std::vector<Synergy*> *synergy,
                     const SynergyTree_node *parentNode = NULL);

    //! Destructor
    ~SynergyTree_node();

    //! Returns the synergy of the cell of the sample c
    Synergy *getSynergy(const arma::vec c);

    //! Returns true if the node is a leaf
    bool isLeaf();

    //! Finds the best location to divide the cell
    OptiData * minimize(const std::vector<std::vector<unsigned int> > *X,
                        const arma::mat M, double thV, double thD,
                        double tol, double alfa);

    //! Writes node data in the specified xml node and
    //! copies the synergy of the cell into the given vector
    void xml(pugi::xml_node *node, std::vector<Synergy*> *fos);

    //! Sorts the samples by the position value in the j-th component
    std::vector<std::vector<unsigned int> > *sortByAxis(const arma::mat Cp,
                                                        unsigned int j);



    //! Parent node
    const SynergyTree_node *parent;

    //! First order synergy of the node
    Synergy *fos;

    //! Direction of the division
    unsigned int axis;

    //! Where the cell is divided
    double location;

    //! Limits of the cell
    arma::mat lim;

    //! Left child node
    SynergyTree_node *childL;

    //! Right child node
    SynergyTree_node *childR;
};


//! Class that represents a PCA kd tree
class SynergyTree {
public:
    //! Constructor
    SynergyTree(const arma::mat &Mp, const arma::mat &Mv, const arma::mat &posLimits,
                const arma::vec &velLimits, double tol = 0.001, double alfa = 0.05);

    //! Constructor
    SynergyTree(const std::string &filename);

    //! Destructor
    virtual ~SynergyTree();

    //! Returns the first order synergy of the cell of the sample c
    const Synergy *getSynergy(const arma::vec &x, bool clamp = false) const;

    const Synergy *getZOS() const {return  zos;}

    //! Loads the tree structure from a file
    bool load(const std::string &filename);

    //! Saves the tree structure into a file
    bool save(const std::string &filename);

    //! Returns the position limits
    arma::mat getPositionLimits() {return Lp;}

    //! Returns the position limits
    arma::mat getVelocityLimits() {return Lv;}

    //! Returns the distance between q and the zero-order synergy box
    double distance(const arma::vec &x);

    virtual arma::vec vectorField(const arma::vec &x) const;

protected:
    SynergyTree() {}

    //! Root node of the tree
    SynergyTree_node *root;

    //! Zero order synergy of the tree
    Synergy *zos;

    //! Position limits
    arma::mat Lp;

    //! Velocity limits
    arma::vec Lv;
};

//! Constructs a PCA kd tree
SynergyTree *makeSynergyTree(const arma::mat &M, const arma::vec &t,
                             const arma::mat posLimits = arma::mat(),
                             const arma::vec velLimits = arma::vec());

#endif // SYNERGY_TREE_H
