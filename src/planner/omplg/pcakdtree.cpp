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


#include "pcakdtree.h"
#include <algorithm>
#include <cmath>
#include <vector>
#include <map>
#include <pugixml.hpp>
/*#include <qt4/QtGui/QImage>
#include <qt4/QtGui/QColor>
#include <qt4/QtGui/QLabel>
#include <qt4/QtGui/QPixmap>
#include <qt4/QtGui/QApplication>*/


//Golden ratio
#define CGOLD 0.3819660

//Small number that protects against trying to achieve fractional
//accuracy for a minimum that happens to be exactly zero
#define ZEPS 1e-10


unsigned int numTotalEvals;


bool brent(double ax, double bx, double cx, double (*f)(double),
           double tol, unsigned int maxIt, double *xmin,
           double *fmin = NULL, unsigned int *iter = NULL) {
    /*Given a function f and a bracketing triplet ax < bx < cx with
     * f(bx) < f(ax) and f(bx) < f(cx), this routine isolates the
     * minimum to a fractional precision of about tol using Brent’s method.*/

    //Initializations...
    if (ax > cx) std::swap(ax,cx);
    if (bx < ax || bx > cx) bx = 0.5*(ax+cx);
    tol = std::abs(tol);
    double a = ax;
    double b = cx;
    double v = bx;
    double w = v;
    double x = v;
    double e = 0.; //This will be the distance moved on the step before last
    double fv = f(v);
    double fw = fv;
    double fx = fv;
    unsigned int it = 1;
    bool found = false;
    bool parabOK;
    double xm,tol1,tol2,r,q,p,etemp,d,u,fu;

    //Main program loop
    while (it < maxIt && !found) {
        xm = 0.5*(a+b);
        tol1 = tol*std::abs(x)+ZEPS;
        tol2 = 2.*tol1;
        if (std::abs(x-xm) <= (tol2-0.5*(b-a))) {
            found = true;
        } else {
            it++;
            parabOK = false;
            if (std::abs(e) > tol1) {
                //Construct a trial parabolic fit
                r = (x-w)*(fx-fv);
                q = (x-v)*(fx-fw);
                p = (x-v)*q-(x-w)*r;
                q = 2.*(q-r);
                if (q > 0.) p = -p;
                q = std::abs(q);
                etemp = e;
                e = d;

                //Determine the acceptability of the parabolic fit
                if (std::abs(p) < std::abs(0.5*q*etemp) &&
                        p > (q*(a-x)) && p < (q*(b-x))) {
                    //Take the parabolic step
                    parabOK = true;
                    d = p/q;
                    u = x+d;
                    if ((u-a) < tol2 || (b-u) < tol2) {
                        if ((xm-x) >= 0.) {
                            d = tol1;
                        } else {
                            d = -tol1;
                        }
                    }
                }
            }
            if (!parabOK) {
                //Take a golden section step into the larger of the two segments
                if (x >= xm) {
                    e = a-x;
                } else {
                    e = b-x;
                }
                d = CGOLD*e;
            }
            if (std::abs(d) >= tol1) {//d computed from parabolic fit
                u = x+d;
            } else { //d computed from golden section
                if (d >= 0.) {
                    u = x+tol1;
                } else {
                    u = x-tol1;
                }
            }
            fu = f(u); //This is the one and only function evaluation per iteration
            if (fu <= fx) {
                if (u >= x) {
                    a = x;
                } else {
                    b = x;
                }
                v = w;
                fv = fw;
                w = x;
                fw = fx;
                x = u;
                fx = fu;
            } else {
                if (u < x) {
                    a = u;
                } else {
                    b = u;
                }
                if (fu <= fw || w == x) {
                    v = w;
                    fv = fw;
                    w = u;
                    fw = fu;
                } else {
                    if (fu <= fv || v == x || v == w) {
                        v = u;
                        fv = fu;
                    }
                }
            }
        }
    }

    if (xmin) *xmin = x;
    if (fmin) *fmin = fx;
    if (iter) *iter = it;

    return found;
}


double interpolate(double x, double x1, double y1, double x2, double y2) {
    return y1+(y2-y1)*(x-x1)/(x2-x1);
}


double normalize(double x, double xMin, double xMax) {
    return (x-xMin)/(xMax-xMin);
}


double normalize(double v, double vMax) {
    return v/vMax;
}


PCAkdtree_node::PCAkdtree_node(const PCAkdtree_node *parentNode, PCAResult *PMD,
                               const arma::mat Mv, const arma::mat Cp,
                               const arma::mat limits, double thV,
                               double thD, double tol, double alfa) :
    parent(parentNode),PMDv(PMD),lim(limits) {
    std::cout << "PMDv:" << std::endl << PMDv->U << std::endl
              << PMDv->b << std::endl << PMDv->a;

    thV *= 1;
    thD *= 1;


    unsigned int m = Mv.n_cols;
    unsigned int n = Mv.n_rows;
    double maxfV = 0.;
    double maxfD = 0.;
    OptiData *optiData;
    std::map<double,OptiData*> optiSet;
    std::vector<std::vector<std::vector<unsigned int> >*> X;
    for (unsigned int j = 0; j < m; ++j) {
        X.push_back(sortByAxis(Cp,j));

        if (X.at(j)->size() == 0) break;

        optiData = minimize(X.at(j),Mv,thV,thD,tol,alfa);
        if (optiData) {//If it converged
            optiData->axis = j;
            optiSet.insert(std::pair<double,OptiData*>(optiData->f,optiData));
            maxfV = std::max(maxfV,optiData->fV);
            maxfD = std::max(maxfD,optiData->fD);
        }
    }

    bool divided = false;
    for (std::map<double,OptiData*>::const_iterator it = optiSet.begin();
         it != optiSet.end(); ++it) {
        //Get optiData
        optiData = it->second;

        //Check partition values value
        if (optiData->fV < thV && optiData->fD < thD && optiData->childL && optiData->childR) {
            //Set axis
            axis = optiData->axis;

            //Set location
            unsigned int kL = floor(0.5*(optiData->x-1.));
            unsigned int kR = ceil(0.5*(optiData->x-1.));
            double cL = Cp(X.at(axis)->at(kL).at(0),axis);
            double cR = Cp(X.at(axis)->at(kR).at(0),axis);
            location = 0.5*(cL+cR);

            //Check children narrowness
            if ((location-lim(axis,0)) > 0.05 && (lim(axis,1)-location) > 0.05) {
                std::cout << "Divide cell at x" << axis << " = " << location
                          << " (" << optiData->f << " <= " << 1 << ")" << std::endl;

                //Set PMDv
                delete PMDv;
                PMDv = NULL;

                //Get children PMDv
                PCAResult *PMDvL = new VelocityPCAResult(optiData->childL->b,
                                                         optiData->childL->a,
                                                         optiData->childL->U);
                PCAResult *PMDvR = new VelocityPCAResult(optiData->childR->b,
                                                         optiData->childR->a,
                                                         optiData->childR->U);

                //Divide matrices
                unsigned int nL = 0, nR;
                for (unsigned int k = 0; k <= kL; ++k) {
                    nL += X.at(axis)->at(k).size();
                }
                if (kL != kR) {
                    nR = n-nL;
                } else {
                    nR = n-nL+X.at(axis)->at(kR).size();
                }
                arma::uvec indicesL(nL), indicesR(nR);
                unsigned int iL = 0, iR = 0;
                for (unsigned int k = 0; k < X.at(axis)->size(); ++k) {
                    for (unsigned int l = 0; l < X.at(axis)->at(k).size(); ++l) {
                        if (k <= kL) {
                            indicesL[iL] = X.at(axis)->at(k).at(l);
                            iL++;
                        }
                        if (k >= kR) {
                            indicesR[iR] = X.at(axis)->at(k).at(l);
                            iR++;
                        }
                    }
                }
                arma::mat MvL(Mv.rows(indicesL));
                arma::mat MvR(Mv.rows(indicesR));
                arma::mat CpL(Cp.rows(indicesL));
                arma::mat CpR(Cp.rows(indicesR));
                arma::mat limL(lim);
                arma::mat limR(lim);
                limL.at(axis,1) = location;
                limR.at(axis,0) = location;

                //Update thresholds
                thV = std::min(thV,maxfV);
                thD = std::min(thD,maxfD);

                //Set children
                childL = new PCAkdtree_node(this,PMDvL,MvL,CpL,limL,thV,thD,tol,alfa);
                childR = new PCAkdtree_node(this,PMDvR,MvR,CpR,limR,thV,thD,tol,alfa);

                divided = true;
                break;
            }
        }
    }

    if (!divided) {
        std::cout << "Don't divide cell" << std::endl;

        //Set color
        //color = qRgb(rand()%256,rand()%256,rand()%256);

        //Set children
        childL = NULL;
        childR = NULL;
    }

    //Delete variables used
    for (unsigned int j = 0; j < X.size(); ++j) {
        delete X.at(j);
    }
    for (std::map<double,OptiData*>::const_iterator it = optiSet.begin();
         it != optiSet.end(); ++it) {
        delete it->second->childL;
        delete it->second->childR;
        delete it->second;
    }
}


PCAkdtree_node::PCAkdtree_node(const pugi::xml_node *node, const std::vector<PCAResult*> *PMD,
                               const PCAkdtree_node *parentNode) : parent(parentNode) {
    if (node->attribute("pmdSet")) {
        //Set PMDv
        PMDv = PMD->at(node->attribute("pmdSet").as_uint());

        //Set color
        //color = qRgb(rand()%256,rand()%256,rand()%256);

        //Set children
        childL = NULL;
        childR = NULL;
    } else {
        //Set PMDv
        PMDv = NULL;

        //Set axis
        if (node->attribute("axis")) {
            axis = node->attribute("axis").as_uint();
        } else {
            throw std::invalid_argument("XML node without axis attribute");
        }

        //Set location
        if (node->attribute("location")) {
            location = node->attribute("location").as_double();
        } else {
            throw std::invalid_argument("XML node without location attribute");
        }

        //Set children
        pugi::xml_node tmp = node->child("Left");
        if (tmp) {
            childL = new PCAkdtree_node(&tmp,PMD,this);
        } else {
            throw std::invalid_argument("Non-leaf XML node without children");
        }
        tmp = node->child("Right");
        if (tmp) {
            childR = new PCAkdtree_node(&tmp,PMD,this);
        } else {
            throw std::invalid_argument("Non-leaf XML node without children");
        }
    }
}


PCAkdtree_node::~PCAkdtree_node() {
    delete PMDv;
    delete childL;
    delete childR;
}


std::vector<std::vector<unsigned int> > *PCAkdtree_node::sortByAxis(const arma::mat Cp,
                                                                    unsigned int j) {
    std::vector<std::vector<unsigned int> > *X = new std::vector<std::vector<unsigned int> >;
    std::multimap<double,unsigned int> index;
    for (unsigned int i = 0; i < Cp.n_rows; ++i) {
        index.insert(std::pair<double,unsigned int>(Cp(i,j),i));
    }
    double c = index.begin()->first;
    std::vector<unsigned int> v;
    for (std::multimap<double,unsigned int>::iterator it = index.begin();
         it != index.end(); ++it) {
        if (c == it->first) {
            v.push_back(it->second);
        } else {
            X->push_back(v);
            v = std::vector<unsigned int>();
            c = it->second;
            v.push_back(it->second);
        }
    }
    X->push_back(v);

    return X;
}

class divisionCriteria {
public:
    divisionCriteria(const std::vector<std::vector<unsigned int> > *sorted,
                     const arma::mat samples, const PCAResult *PMD,
                     double thV, double thD, double alfa) :
        indices(sorted->size()),M(samples.n_rows,samples.n_cols),father(PMD) {
        unsigned int i = 0;
        for (unsigned int k = 0; k < sorted->size(); ++k) {
            if (k == 0) {
                indices.at(k) = sorted->at(k).size()-1;
            } else {
                indices.at(k) = sorted->at(k).size()+indices.at(k-1);
            }
            for (unsigned int p = 0; p < sorted->at(k).size(); ++p) {
                for (unsigned int j = 0; j < M.n_cols; ++j) {
                    M(i,j) = samples(sorted->at(k).at(p),j);
                }
                i++;
            }
        }

        cV = 1./thV;
        cD = 1./thD;
        a = alfa;
        vFather = arma::prod(father->a);
    }

    ~divisionCriteria() {
        for (std::map<unsigned int,OptiData*>::iterator it = F.begin();
             it != F.end(); ++it) {
            delete it->second->childL;
            delete it->second->childR;
            delete it->second;
        }
        F.clear();
        indices.clear();
        M.clear();
    }

    double eval(double value) {
        unsigned int x = std::max(std::min(round(value),2.*indices.size()),0.);

        if (x == 0 || x == 2*indices.size()) return 1.;

        std::map<unsigned int,OptiData*>::iterator it = F.find(x);
        if (it == F.end()) {
            double k = 0.5*(x-1.);
            OptiData *optiData = new OptiData;
            optiData->x = x;
            if (indices.at(floor(k))+1 <= 10*M.n_cols) {
                optiData->childL = NULL;
                optiData->childR = new VelocityPCAResult(father->b,father->a,father->U);
                optiData->fV = 1.;
                optiData->fD = 1.;
                optiData->f = 1.;
            } else if (M.n_rows-indices.at(ceil(k)) <= 10*M.n_cols) {
                optiData->childL = new VelocityPCAResult(father->b,father->a,father->U);
                optiData->childR = NULL;
                optiData->fV = 1.;
                optiData->fD = 1.;
                optiData->f = 1.;
            } else {
                optiData->childL = PCA(M.rows(0,indices.at(floor(k))),VELOCITY,a);
                optiData->childR = PCA(M.rows(indices.at(ceil(k)),M.n_rows-1),VELOCITY,a);
                optiData->fV = std::max(arma::prod(optiData->childL->a),
                                        arma::prod(optiData->childR->a))/vFather;
                optiData->fD = 1.-std::min(father->distance(optiData->childL),
                                           father->distance(optiData->childR));
                optiData->f = (cV*optiData->fV+cD*optiData->fD)/2.;
            }

            if (optiData->f < 0.) throw;

            F.insert(std::pair<unsigned int,OptiData*>(x,optiData));

            return optiData->f;
        } else {
            return it->second->f;
        }
    }

    unsigned int getNumEvals() {return F.size();}

    OptiData* getOptiData(double value) {
        unsigned int x = std::max(std::min(round(value),2.*indices.size()),0.);
        OptiData *optiData = new OptiData;
        optiData->x = x;
        std::map<unsigned int,OptiData*>::iterator it = F.find(x);
        if (it == F.end()) {
            if (x == 0) {
                optiData->childL = NULL;
                optiData->childR = new VelocityPCAResult(father->b,father->a,father->U);
                optiData->f = 1.;
                optiData->fV = 1.;
                optiData->fD = 1.;

                return optiData;
            } else if (x == 2*indices.size()) {
                optiData->childL = new VelocityPCAResult(father->b,father->a,father->U);
                optiData->childR = NULL;
                optiData->f = 1.;
                optiData->fV = 1.;
                optiData->fD = 1.;

                return optiData;
            } else {
                eval(x);
                it = F.find(x);
            }
        }

        if (it->second->childL) {
            optiData->childL = new VelocityPCAResult(it->second->childL->b,
                                                     it->second->childL->a,
                                                     it->second->childL->U);
        } else {
            optiData->childL = NULL;
        }
        if (it->second->childR) {
            optiData->childR = new VelocityPCAResult(it->second->childR->b,
                                                     it->second->childR->a,
                                                     it->second->childR->U);
        } else {
            optiData->childR = NULL;
        }
        optiData->f = it->second->f;
        optiData->fV = it->second->fV;
        optiData->fD = it->second->fD;

        return optiData;
    }

private:
    std::vector<unsigned int> indices;
    arma::mat M;
    const PCAResult *father;
    std::map<unsigned int,OptiData*> F;
    double a;
    double cV;
    double cD;
    double vFather;
};

divisionCriteria *DC;

double f(double x) {
    return DC->eval(x);
}

OptiData *PCAkdtree_node::minimize(const std::vector<std::vector<unsigned int> > *X,
                                   const arma::mat M, double thV, double thD, double tol,
                                   double alfa) {

    DC = new divisionCriteria(X,M,PMDv,thV,thD,alfa);

    unsigned int iter;
    double xMin, fMin;
    if (brent(0.,X->size(),2.*X->size(),&f,tol,50,&xMin,&fMin,&iter)) {

        std::cout << "Succeed iter: " << iter << " evals: " << DC->getNumEvals()
                  << " fMin: " << fMin << " xMin: " << int(round(xMin)) << std::endl;

        OptiData *optiData = DC->getOptiData(xMin);

        delete DC;

        return optiData;
    } else {
        std::cout << "Failed iter: " << iter << " evals: " << DC->getNumEvals()
                  << " fMin: " << fMin << " xMin: " << int(round(xMin)) << std::endl;

        delete DC;

        return NULL;
    }
}


PCAResult *PCAkdtree_node::getPMD(const arma::vec c) {
    if (isLeaf()) {
        return PMDv;
    } else {
        if (c[axis] < location || (c[axis] == location && rand()%2)) {
            return childL->getPMD(c);
        } else {
            return childR->getPMD(c);
        }
    }
}


unsigned int PCAkdtree_node::getColor(const arma::vec c) {
    if (isLeaf()) {
        return color;
    } else {
        if (c[axis] < location || (c[axis] == location && rand()%2)) {
            return childL->getColor(c);
        } else {
            return childR->getColor(c);
        }
    }
}


bool PCAkdtree_node::isLeaf() {
    return PMDv;
}


void PCAkdtree_node::xml(pugi::xml_node *node, std::vector<PCAResult*> *PMD) {
    if (isLeaf()) {
        node->append_attribute("pmdSet").set_value(int(PMD->size()));
        PMD->push_back(PMDv);
    } else {
        node->append_attribute("axis").set_value(axis);
        node->append_attribute("location").set_value(location);
        pugi::xml_node child = node->append_child("Left");
        childL->xml(&child,PMD);
        child = node->append_child("Right");
        childR->xml(&child,PMD);
    }
}


PCAkdtree::PCAkdtree(const arma::mat Mp, const arma::mat Mv, const arma::mat posLimits,
                     const arma::vec velLimits, double tol, double alfa) :
    PMDp(PCA(Mp,POSITION)),Lp(posLimits),Lv(velLimits) {
    std::cout << "pLim: " << std::endl << Lp << "PMDp:" << std::endl
              << PMDp->U << std::endl << PMDp->b << std::endl << PMDp->a
              << "vLim: " << std::endl << Lv;

    //Check the arguments
    unsigned int n = Mp.n_rows;
    unsigned int m = Mp.n_cols;
    if (Mv.n_rows != n || Mv.n_cols != m || Lp.n_cols != 2 ||
            Lp.n_rows != m || Lv.n_elem != m) {
        throw std::invalid_argument("Wrong dimensions");
    }

    //Transform position samples into PMDp base
    arma::mat Cp(n,m);
    for (unsigned int i = 0; i < n; ++i) {
        arma::vec x(arma::trans(Mp.row(i))-PMDp->b);
        for (unsigned int j = 0; j < m; ++j) {
            if (PMDp->a[j] == 0) {
                Cp(i,j) = 0.5;
            } else {
                Cp(i,j) = std::max(std::min(arma::dot(x,PMDp->U.col(j))/
                                            (2.*PMDp->a[j])+0.5,1.),0.);
            }
        }
    }

    //Populate the tree
    arma::mat limits(m,2);
    for (unsigned j = 0; j < m; ++j) {
        limits(j,0) = 0.;
        limits(j,1) = 1.;
    }
    root = new PCAkdtree_node(NULL,PCA(Mv,VELOCITY,alfa),Mv,Cp,limits,1.,1.,tol,alfa);

    //plot(Cp);
}


PCAkdtree::PCAkdtree(const std::string filename) {
    if (!load(filename)) throw std::invalid_argument("Tree construction failed");
}


PCAkdtree::~PCAkdtree() {
    Lp.clear();
    Lv.clear();
    delete root;
    delete PMDp;
}


PCAResult *PCAkdtree::getPMD(const arma::vec x) {
    //Check parameters
    if (x.n_elem != PMDp->dim) return NULL;

    //Normalize sample
    arma::vec xn(PMDp->dim);
    for (unsigned int j = 0; j < PMDp->dim; ++j) {
        xn[j] = normalize(x[j],Lp(j,0),Lp(j,1))-PMDp->b[j];
    }

    //Transform sample into PMDp base
    arma::vec c(PMDp->dim);
    for (unsigned int j = 0; j < PMDp->dim; ++j) {
        if (PMDp->a[j] == 0) {
            c[j] = 0.5;
        } else {
            c[j] = dot(xn,PMDp->U.col(j))/(2*PMDp->a[j])+0.5;
            if (c[j] < 0 || c[j] > 1) return NULL;
        }
    }

    return root->getPMD(c);
}


bool PCAkdtree::load(const std::string filename) {
    try {
        pugi::xml_document doc;
        pugi::xml_parse_result result = doc.load_file(filename.c_str());
        if (result) {
            //Read the dimension
            unsigned int n = 0;
            pugi::xml_node node = doc.child("Position").child("Limits");
            while (node) {
                n++;
                node = node.next_sibling("Limits");
            }

            //Read the position limits
            Lp.resize(n,2);
            node = doc.child("Position").child("Limits");
            for (unsigned int i = 0; i < n; ++i) {
                if (!node) return false;
                Lp(i,0) = node.attribute("min").as_float(0.);
                Lp(i,1) = node.attribute("max").as_float(0.);
                node = node.next_sibling("Limits");
            }

            //Read the position PMD set
            node = doc.child("Position").child("ControlSet");
            PMDp = xml2PMD(&node,POSITION);
            if (!PMDp) return false;

            //Read the velocity limits
            Lv.resize(n);
            node = doc.child("Velocity").child("Limits");
            for (unsigned int i = 0; i < n; ++i) {
                if (!node) return false;
                Lv[i] = node.attribute("max").as_float(0.);
                node = node.next_sibling("Limits");
            }

            //Read the velocity PMD sets
            node = doc.child("Velocity").child("ControlSet");
            std::vector<PCAResult*> PMDv;
            while (node) {
                PMDv.push_back(xml2PMD(&node,VELOCITY));
                node = node.next_sibling("ControlSet");
            }

            //Read the kd tree
            node = doc.child("Velocity").child("Tree").child("Root");
            root = new PCAkdtree_node(&node,&PMDv);

            return true;
        } else {
            std::cout << filename << " " << result.description() << std::endl;

            return false;
        }
    } catch(...) {
        return false;
    }
}


bool PCAkdtree::save(const std::string filename) {
    try {
        pugi::xml_document doc;

        //Write the position limits values
        pugi::xml_node node = doc.append_child("Position");
        pugi::xml_node tmp;
        for (unsigned int i = 0; i < PMDp->dim; ++i) {
            tmp = node.append_child("Limits");
            tmp.append_attribute("min").set_value(Lp(i,0));
            tmp.append_attribute("max").set_value(Lp(i,1));
        }

        //Write the position PMD set
        tmp = node.append_child("ControlSet");
        PMD2xml(&tmp,PMDp);

        //Write the velocity limits
        node = doc.append_child("Velocity");
        for (unsigned int i = 0; i < PMDp->dim; ++i) {
            node.append_child("Limits").append_attribute("max").
                    set_value(Lv[i]);
        }

        //Write the kd tree
        std::vector<PCAResult*> PMDv;
        tmp = node.append_child("Tree").append_child("Root");
        root->xml(&tmp,&PMDv);

        //Write the velocity PMD sets
        for (unsigned int i = 0; i < PMDv.size(); ++i) {
            tmp = node.append_child("ControlSet");
            PMD2xml(&tmp,PMDv.at(i));
        }

        return doc.save_file(filename.c_str());
    } catch(...) {
        return false;
    }
}


/*void PCAkdtree::plot(const arma::mat Cp) {
    if (PMDp->dim == 2) {
        arma::vec c(2u);
        unsigned int side = 800;
        QImage tree(side,side,QImage::Format_RGB16);
        for (unsigned int i = 0; i < side; ++i) {
            c[0] = 1.-(i+0.5)/double(side);
            for (unsigned int j = 0; j < side; ++j) {
                c[1] = 1.-(j+0.5)/double(side);
                tree.setPixel(i,j,root->getColor(c));
            }
        }

        unsigned int i, j;
        for (unsigned int k = 0; k < Cp.n_rows; ++k) {
            c[0] = Cp(k,0);
            c[1] = Cp(k,1);
            i = std::max(std::min(round(double(side)*(1.-Cp(k,0))-0.5),side-1.),0.);
            j = std::max(std::min(round(double(side)*(1.-Cp(k,1))-0.5),side-1.),0.);
            QColor color(root->getColor(c));
            QColor color2(255-color.red(),255-color.green(),255-color.blue());
            tree.setPixel(i,j,color2.rgb());
        }


        QApplication app(0,NULL);
        QLabel lbl;
        lbl.setPixmap(QPixmap::fromImage(tree));
        lbl.resize(side,side);
        lbl.show();
        app.exec();
        return;
    }
}*/


PCAkdtree *makePCAkdtree(const arma::mat M, const arma::vec t,
                         const arma::mat posLimits, const arma::vec velLimits) {
    //Check arguments
    unsigned int n = M.n_rows;
    unsigned int m = M.n_cols;

    arma::mat pLim(posLimits);
    if (posLimits.n_elem == 0) {
        pLim.set_size(m,2);
        for (unsigned int j = 0; j < m; ++j) {
            pLim.at(j,0) = arma::min(M.col(j));
            pLim.at(j,1) = arma::max(M.col(j));
        }
    }

    arma::vec vLim(velLimits);
    if (velLimits.n_elem == 0) {
        vLim.ones(m);
    }

    if (t.n_elem != n || pLim.n_cols != 2 || pLim.n_rows != m ||
            vLim.n_elem != m) {
        throw std::invalid_argument("Wrong dimension values");
    }

    //Compute normalized position and velocities samples
    arma::mat Mp(n,m);
    arma::mat Mv(n,m);
    unsigned int k;
    for (unsigned int j = 0; j < m; ++j) {
        k = 0;
        for (unsigned int i = 0; i < n; ++i) {
            if (i == 0 || t[i] <= t[i-1]) {//First sample
                if ((i+2) < n && t[i+1] > t[i] && t[i+2] > t[i+1]) {
                    Mp(k,j) = normalize(M(i,j),pLim.at(j,0),pLim.at(j,1));

                    Mv(k,j) = normalize(interpolate(i,i+0.5,(M(i+1,j)-M(i,j))/(t[i+1]-t[i]),
                                        i+1.5,(M(i+2,j)-M(i+1,j))/(t[i+2]-t[i+1])),vLim[j]);

                    k++;
                } else if ((i+1) < n && t[i+1] > t[i]) {
                    Mp(k,j) = normalize(M(i,j),pLim.at(j,0),pLim.at(j,1));

                    Mv(k,j) = normalize((M(i+1,j)-M(i,j))/(t[i+1]-t[i]),vLim[j]);

                    k++;
                }
            } else if (i == n-1 || t[i+1] <= t[i]) {//Last sample
                if (i > 1 && t[i-1] > t[i-2] && t[i] > t[i-1]) {
                    Mp(k,j) = normalize(M(i,j),pLim.at(j,0),pLim.at(j,1));

                    Mv(k,j) = normalize(interpolate(i,i-1.5,(M(i-2,j)-M(i-1,j))/(t[i-2]-t[i-1]),
                            i-0.5,(M(i-1,j)-M(i,j))/(t[i-1]-t[i])),vLim[j]);

                    k++;
                } else if (i > 0 && t[i] > t[i-1]) {
                    Mp(k,j) = normalize(M(i,j),pLim.at(j,0),pLim.at(j,1));

                    Mv(k,j) = normalize((M(i-1,j)-M(i,j))/(t[i-1]-t[i]),vLim[j]);

                    k++;
                }
            } else {//Intermedium samples
                Mp(k,j) = normalize(M(i,j),pLim.at(j,0),pLim.at(j,1));

                Mv(k,j) = normalize(interpolate(i,i-0.5,(M(i,j)-M(i-1,j))/(t[i]-t[i-1]),
                                    i+0.5,(M(i+1,j)-M(i,j))/(t[i+1]-t[i])),vLim[j]);

                k++;
            }
        }
    }
    n = k;
    Mp.resize(n,m);
    Mv.resize(n,m);

    if (velLimits.n_elem == 0) {
        for (unsigned int j = 0; j < m; ++j) {
            vLim[j] = std::max(arma::max(Mv.col(j)),-arma::min(Mv.col(j)));
            Mv.col(j) /= vLim[j];
        }
    }

    //Create the tree
    numTotalEvals = 0;
    PCAkdtree *tree = new PCAkdtree(Mp,Mv,pLim,vLim);
    std::cout << "numTotalEvals: " << numTotalEvals << std::endl;
    srand(time(NULL));
    return tree;
}
