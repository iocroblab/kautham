%evalresults
clear v
%compare different executions using random
i=1;
v(:,:,i)=r442;
i=i+1;
v(:,:,i)=r442rotat;
evalpaths(v)
return 

%compare different executions using random
i=1;
v(:,:,i)=r1000;
i=i+1;
v(:,:,i)=r622;
i=i+1;
v(:,:,i)=r442;
evalpaths(v)
return 


%compare different executions using random
i=1;
v(:,:,i)=r1000;
i=i+1;
v(:,:,i)=r1000_n2;
i=i+1;
v(:,:,i)=r1000_n3;
i=i+1;
v(:,:,i)=r1000_n4;
i=i+1;
v(:,:,i)=r1000_n5;
evalpaths(v)
return 

%compare different executions using random
i=1;
v(:,:,i)=r442;
i=i+1;
v(:,:,i)=r442_n2;
i=i+1;
v(:,:,i)=r442_n3;
i=i+1;
v(:,:,i)=r442_n4;
i=i+1;
v(:,:,i)=r442_n5;
evalpaths(v)
return 

%compare the use of random
i=1;
v(:,:,i)=r604;
i=i+1;
v(:,:,i)=r604r;
i=i+1;
v(:,:,i)=r604r2;
i=i+1;
v(:,:,i)=r604r3;
evalpaths(v)
return 

%eval the use of alpha weight
i=1;
v(:,:,i)=r1000;
i=i+1;
v(:,:,i)=r802;
i=i+1;
v(:,:,i)=r604;
evalpaths(v)
return 
%CONCLUSIONS:
%Va be el 604 amb cost alpha 
%rAlpha[i] = 0.5*(1-abs(beta[i])*abs(alpha[i])+0.5*abs(alpha[i]-alpha0));


%compare the use of different alpha weights
i=1;
v(:,:,i)=r604;
i=i+1;
v(:,:,i)=r604a;
i=i+1;
v(:,:,i)=r604b;
evalpaths(v)
return 



%compare the use of different alpha weights
i=1;
v(:,:,i)=r802;
i=i+1;
v(:,:,i)=r802a;
evalpaths(v)
return 


%compare the use of different alpha weights
i=1;
v(:,:,i)=r802;
i=i+1;
v(:,:,i)=r604;
i=i+1;
v(:,:,i)=r622;
evalpaths(v)
return 
%compare the use of different alpha weights
i=1;
v(:,:,i)=r1000;
i=i+1;
v(:,:,i)=r820;
evalpaths(v)
return 







