% This function plots the data collected form the teleoperation procedure.
% The data were stored in the following order: Fx Fy Fz Xd Yd Zd Xr Yr Zr
S=load('Data_of_teleoperationT3_assisted.txt');
D=1;
k=3;
scale = 10;
fx=scale*S(1:k:end,1);
fy=scale*S(1:k:end,2);
fz=scale*S(1:k:end,3);
xd=S(1:k:end,4);
yd=S(1:k:end,5);
zd=S(1:k:end,6);
xr=S(1:k:end,7);
yr=S(1:k:end,8);
zr=S(1:k:end,9);
if D == 0
  plot(xd,yd,'r');
  hold on;
  plot(xr,yr,'ob');
 quiver(xr,yr,fx,fy,'m')
else
  plot3(xd,yd,zd,'r');
  hold on;
  plot3(xr,yr,zr);
  quiver3(xr,yr,zr,fx,fy,fz)
end
hold off;