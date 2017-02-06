#ifndef VECTORFIELD_H
#define VECTORFIELD_H

#endif // VECTORFIELD_H



#include <kautham/planner/omplg/synergy_tree.h>

#define fb -1.
#define fa 0.5
#define yb 0.4


class VectorField : public SynergyTree {
public:
    VectorField() : problem(0) {}

    arma::vec vectorField(const arma::vec &x) const {
        arma::vec v(2,arma::fill::zeros);
        switch(problem) {
            case 0: {
                //River
                v[0] = (abs(x[1]-0.5) < 0.5*yb)? fb : fa;
            } break;
            case 1: {
                //Vortex
                double m = hypot(x[0]-0.5,x[1]-0.5);
                if (m > std::numeric_limits<double>::epsilon()) {
                    v[0] = (0.5-x[1])/m;
                    v[1] = (x[0]-0.5)/m;
                }
            } break;
            case 2: {
                //Function
                v[0] = cos(pow(M_PI*(2*x[0]-1.),2.)+(M_PI*(2.*x[1]-1.)));
                v[1] = 1.+(M_PI*(2*x[0]-1.))-pow(M_PI*(2.*x[1]-1.),2.);
            } break;
            case 3: {
                //Mountains
                double a = 1.5;
                double b = 5.;
                arma::vec y = x;
                y[0] += -0.9;
                y[1] += -0.9;
                v = a*b*exp(-b*arma::dot(y,y))*y;
                a = 1.5;
                b = 10.;
                y = x;
                y[0] += -0.2;
                y[1] += -0.2;
                v += a*b*exp(-b*arma::dot(y,y))*y;
                a = 1.2;
                b = 15.;
                y = x;
                y[0] += -0.3;
                y[1] += -0.7;
                v += a*b*exp(-b*arma::dot(y,y))*y;
                a = 1.;
                b = 20.;
                y = x;
                y[0] += -0.8;
                y[1] += -0.3;
                v += a*b*exp(-b*arma::dot(y,y))*y;
            } break;
        }

        return v;
    }

    void setProblem(unsigned int prob) {
        problem = prob;
    }

    unsigned int getProblem() {
        return problem;
    }

protected:
    unsigned int problem;
};
