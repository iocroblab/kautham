cost function:

r = getWeightNF1()*rNF1[i]+getWeightDist()*rDist[i]+getWeightAlpha()*rAlpha[i];

rNF1[i] = (NF1cost[i]-minNF1cost)/(maxNF1cost-minNF1cost);

//The new distance costs measures if the direction of motion of the tip goes in a positive direction
//of the gradient of the distance

rDist[i] = 1-((KthReal)(dcost[i]-mindcost)/(maxdcost-mindcost));		

//we want small alphas when beta is small AND we do not want grat increments of alpha
						
rAlpha[i] = 0.5*(1-abs(beta[i])*abs(alpha[i])+0.5*abs(alpha[i]-alpha0));

//Additionally and optionally a cost has been added if either beta or alpha are negative
if(beta[i]<0 || alpha[i]<0) rAlpha[i]+=0.1;

//files wiht the results using this additional cost have an "a" added to its name


