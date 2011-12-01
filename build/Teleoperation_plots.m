% This function plots the data collected form the teleoperation procedure.
% The data were stored in the following order: Fx Fy Fz Xd Yd Zd Xr Yr Zr
S=load('Data_of_teleoperation.txt');
scale = 10;
fx=scale*S(:,1);
fy=scale*S(:,2);
fz=scale*S(:,3);
xd=S(:,4);
yd=S(:,5);
zd=S(:,6);
xr=S(:,7);
yr=S(:,8);
zr=S(:,9);
plot3(xd,yd,zd,'r');
hold on;
plot3(xr,yr,zr);
%quiver3(xr,yr,zr,fx,fy,fz)