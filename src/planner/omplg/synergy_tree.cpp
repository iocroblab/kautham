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


#include <kautham/planner/omplg/synergy_tree.h>
#include <algorithm>
#include <cmath>
#include <vector>
#include <map>
#include <pugixml.hpp>
#include <assert.h>

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


SynergyTree_node::SynergyTree_node(const SynergyTree_node *parentNode, Synergy *synergy,
                                   const arma::mat Mv, const arma::mat Cp,
                                   const arma::mat limits, double thV,
                                   double thD, double tol, double alfa) :
    parent(parentNode),fos(synergy),lim(limits) {
    std::cout << "fos:" << std::endl << fos->U << std::endl
              << fos->b << std::endl << fos->a;

    thV *= 1.;
    thD *= 1.;


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

        std::cout << "Checking axis " << j << " ... ";
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

                //Set fos
                delete fos;
                fos = NULL;

                //Get children fos
                Synergy *fosL = new FirstOrderSynergy(optiData->childL->b,
                                                      optiData->childL->a,
                                                      optiData->childL->U);
                Synergy *fosR = new FirstOrderSynergy(optiData->childR->b,
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
                childL = new SynergyTree_node(this,fosL,MvL,CpL,limL,thV,thD,tol,alfa);
                childR = new SynergyTree_node(this,fosR,MvR,CpR,limR,thV,thD,tol,alfa);

                divided = true;
                break;
            }
        }
    }

    if (!divided) {
        std::cout << "Don't divide cell" << std::endl;

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


SynergyTree_node::SynergyTree_node(const pugi::xml_node *node, const std::vector<Synergy*> *Synergy,
                                   const SynergyTree_node *parentNode) : parent(parentNode) {
    if (node->attribute("synergy")) {
        //Set fos
        fos = Synergy->at(node->attribute("synergy").as_uint());

        //Set children
        childL = NULL;
        childR = NULL;
    } else {
        //Set fos
        fos = NULL;

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
            childL = new SynergyTree_node(&tmp,Synergy,this);
        } else {
            throw std::invalid_argument("Non-leaf XML node without children");
        }
        tmp = node->child("Right");
        if (tmp) {
            childR = new SynergyTree_node(&tmp,Synergy,this);
        } else {
            throw std::invalid_argument("Non-leaf XML node without children");
        }
    }
}


SynergyTree_node::~SynergyTree_node() {
    delete fos;
    delete childL;
    delete childR;
}


std::vector<std::vector<unsigned int> > *SynergyTree_node::sortByAxis(const arma::mat Cp,
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
                     const arma::mat samples, const Synergy *Synergy,
                     double thV, double thD, double alfa) :
        indices(sorted->size()),M(samples.n_rows,samples.n_cols),father(Synergy) {
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
            double k = 0.5*double(x-1);
            OptiData *optiData = new OptiData;
            optiData->x = x;
            if (indices.at(floor(k))+1 <= 10*M.n_cols) {
                optiData->childL = NULL;
                optiData->childR = new FirstOrderSynergy(father->b,father->a,father->U);
                optiData->fV = 1.;
                optiData->fD = 1.;
                optiData->f = 1.;
            } else if (M.n_rows-indices.at(ceil(k)) <= 10*M.n_cols) {
                optiData->childL = new FirstOrderSynergy(father->b,father->a,father->U);
                optiData->childR = NULL;
                optiData->fV = 1.;
                optiData->fD = 1.;
                optiData->f = 1.;
            } else {
                optiData->childL = PCA(M.rows(0,indices.at(floor(k))),FIRST,a);
                optiData->childR = PCA(M.rows(indices.at(ceil(k)),M.n_rows-1),FIRST,a);

                double vL = arma::prod(optiData->childL->a);
                double vR = arma::prod(optiData->childR->a);
                optiData->fV = std::max(vL,vR)/vFather;

                if (std::isnan(vL) || std::isnan(vR) || std::isnan(vFather)) throw;

                double dfL = father->distance(optiData->childL);
                double dfR = father->distance(optiData->childR);
                optiData->fD = 1.-std::min(dfL,dfR);

                if (std::isnan(dfL) || std::isnan(dfR)) throw;

                optiData->f = (cV*optiData->fV+cD*optiData->fD)/2.;
                if (optiData->f < 0.) throw;
            }

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
                optiData->childR = new FirstOrderSynergy(father->b,father->a,father->U);
                optiData->f = 1.;
                optiData->fV = 1.;
                optiData->fD = 1.;

                return optiData;
            } else if (x == 2*indices.size()) {
                optiData->childL = new FirstOrderSynergy(father->b,father->a,father->U);
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
            optiData->childL = new FirstOrderSynergy(it->second->childL->b,
                                                     it->second->childL->a,
                                                     it->second->childL->U);
        } else {
            optiData->childL = NULL;
        }
        if (it->second->childR) {
            optiData->childR = new FirstOrderSynergy(it->second->childR->b,
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
    const Synergy *father;
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

OptiData *SynergyTree_node::minimize(const std::vector<std::vector<unsigned int> > *X,
                                     const arma::mat M, double thV, double thD, double tol,
                                     double alfa) {

    DC = new divisionCriteria(X,M,fos,thV,thD,alfa);

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


Synergy *SynergyTree_node::getSynergy(const arma::vec c) {
    if (isLeaf()) {
        return fos;
    } else {
        if (c[axis] < location || (c[axis] == location && rand()%2)) {
            return childL->getSynergy(c);
        } else {
            return childR->getSynergy(c);
        }
    }
}


bool SynergyTree_node::isLeaf() {
    return fos;
}


void SynergyTree_node::xml(pugi::xml_node *node, std::vector<Synergy*> *Synergy) {
    if (isLeaf()) {
        node->append_attribute("synergy").set_value(int(Synergy->size()));
        Synergy->push_back(fos);
    } else {
        node->append_attribute("axis").set_value(axis);
        node->append_attribute("location").set_value(location);
        pugi::xml_node child = node->append_child("Left");
        childL->xml(&child,Synergy);
        child = node->append_child("Right");
        childR->xml(&child,Synergy);
    }
}


SynergyTree::SynergyTree(const arma::mat &Mp, const arma::mat &Mv, const arma::mat &posLimits,
                         const arma::vec &velLimits, double tol, double alfa) :
    zos(PCA(Mp,ZERO)),Lp(posLimits),Lv(velLimits) {
    std::cout << "pLim: " << std::endl << Lp << "zos:" << std::endl
              << zos->U << std::endl << zos->b << std::endl << zos->a
              << "vLim: " << std::endl << Lv;

    //Check the arguments
    unsigned int n = Mp.n_rows;
    unsigned int m = Mp.n_cols;
    if (Mv.n_rows != n || Mv.n_cols != m || Lp.n_cols != 2 ||
            Lp.n_rows != m || Lv.n_elem != m) {
        throw std::invalid_argument("Wrong dimensions");
    }

    //Transform position samples into zos base
    arma::mat Cp(n,m);
    for (unsigned int i = 0; i < n; ++i) {
        arma::vec x(arma::trans(Mp.row(i))-zos->b);
        for (unsigned int j = 0; j < m; ++j) {
            if (zos->a[j] == 0) {
                Cp(i,j) = 0.5;
            } else {
                Cp(i,j) = std::max(std::min(arma::dot(x,zos->U.col(j))/
                                            (2.*zos->a[j])+0.5,1.),0.);
            }
        }
    }

    //Populate the tree
    arma::mat limits(m,2);
    for (unsigned j = 0; j < m; ++j) {
        limits(j,0) = 0.;
        limits(j,1) = 1.;
    }
    root = new SynergyTree_node(NULL,PCA(Mv,FIRST,alfa),Mv,Cp,limits,1.,1.,tol,alfa);
}


SynergyTree::SynergyTree(const std::string &filename) {
    if (!load(filename)) throw std::invalid_argument("Synergytree construction failed");
}


SynergyTree::~SynergyTree() {
    Lp.clear();
    Lv.clear();
    delete root;
    delete zos;
}


const Synergy *SynergyTree::getSynergy(const arma::vec &x, bool clamp) const {
    //Check parameters
    if (x.n_elem != zos->dim) return NULL;

    //Normalize sample
    arma::vec xn(zos->dim);
    for (unsigned int j = 0; j < zos->dim; ++j) {
        xn[j] = normalize(x[j],Lp(j,0),Lp(j,1));
    }

    //Transform sample into zos base
    arma::vec c(zos->dim);
    xn = (xn-zos->b);
    for (unsigned int j = 0; j < zos->dim; ++j) {
        if (zos->a[j] == 0) {
            c[j] = 0.5;
        } else {
            c[j] = dot(xn,zos->U.col(j))/(2*zos->a[j])+0.5;
            if (c[j] < 0 || c[j] > 1) {
                if (clamp) {
                    c[j] = std::min(std::max(c[j],0.),1.);
                } else {
                    return NULL;
                }
            }
        }
    }

    return root->getSynergy(c);
}


bool SynergyTree::load(const std::string &filename) {
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
                Lp(i,0) = node.attribute("min").as_double(0.);
                Lp(i,1) = node.attribute("max").as_double(0.);
                node = node.next_sibling("Limits");
            }

            //Read the position synergy
            node = doc.child("Position").child("ControlSet");
            zos = xml2synergy(&node,ZERO);
            if (!zos) return false;

            //Read the velocity limits
            Lv.resize(n);
            node = doc.child("Velocity").child("Limits");
            for (unsigned int i = 0; i < n; ++i) {
                if (!node) return false;
                Lv[i] = node.attribute("max").as_double(0.);
                node = node.next_sibling("Limits");
            }

            //Read the velocity synergies
            node = doc.child("Velocity").child("ControlSet");
            std::vector<Synergy*> fos;
            Synergy *synergy;
            while (node) {
                synergy = xml2synergy(&node,FIRST);
                if (!synergy) return false;
                fos.push_back(synergy);
                node = node.next_sibling("ControlSet");
            }

            //Read the kd tree
            node = doc.child("Velocity").child("Tree").child("Root");
            root = new SynergyTree_node(&node,&fos);

            return true;
        } else {
            std::cout << result.description() << std::endl;

            return false;
        }
    } catch(std::exception &e) {
        std::cout << e.what() << std::endl;
        return false;
    }
}


bool SynergyTree::save(const std::string &filename) {
    try {
        pugi::xml_document doc;

        //Write the position limits values
        pugi::xml_node node = doc.append_child("Position");
        pugi::xml_node tmp;
        for (unsigned int i = 0; i < zos->dim; ++i) {
            tmp = node.append_child("Limits");
            tmp.append_attribute("min").set_value(Lp(i,0));
            tmp.append_attribute("max").set_value(Lp(i,1));
        }

        //Write the position synergy
        tmp = node.append_child("ControlSet");
        synergy2xml(&tmp,zos);

        //Write the velocity limits
        node = doc.append_child("Velocity");
        for (unsigned int i = 0; i < zos->dim; ++i) {
            node.append_child("Limits").append_attribute("max").
                    set_value(Lv[i]);
        }

        //Write the kd tree
        std::vector<Synergy*> fos;
        tmp = node.append_child("Tree").append_child("Root");
        root->xml(&tmp,&fos);

        //Write the velocity synergies
        for (unsigned int i = 0; i < fos.size(); ++i) {
            tmp = node.append_child("ControlSet");
            synergy2xml(&tmp,fos.at(i));
        }

        return doc.save_file(filename.c_str());
    } catch(...) {
        return false;
    }
}


double SynergyTree::distance(const arma::vec &x) {
    return zos->distance(x);
}


arma::vec SynergyTree::vectorField(const arma::vec &x) const {
    const Synergy *s = getSynergy(x,true);
    assert(s);
    //const unsigned int n = s->getReducedDimension();
    arma::vec vf = s->b/*+s->U.cols(0,n-1)*(s->a.subvec(0,n-1)%arma::randn(n))*/;

    if (arma::norm(vf) > std::numeric_limits<double>::epsilon()) {
        return arma::normalise(vf);
    } else {
        return arma::zeros(s->dim);
    }
}


SynergyTree *makeSynergyTree(const arma::mat &M, const arma::vec &t,
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
    SynergyTree *tree = new SynergyTree(Mp,Mv,pLim,vLim);
    std::cout << "numTotalEvals: " << numTotalEvals << std::endl;
    srand(time(NULL));
    return tree;
}
