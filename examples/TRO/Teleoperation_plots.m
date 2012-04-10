% This function plots the data collected form the teleoperation procedure.
% The data were stored in the following order: Fx Fy Fz Xd Yd Zd Xr Yr Zr
ejem='3D_simple';
S=load(strcat(ejem ,'_assisted.txt'));
D=3;
k=10;
scale = 5;

fx=scale*S(1:k:end,1);
fy=scale*S(1:k:end,2);
fz=scale*S(1:k:end,3);
xd=S(1:k:end,4);
yd=S(1:k:end,5);
zd=S(1:k:end,6);
xr=S(1:k:end,7);
yr=S(1:k:end,8);
zr=S(1:k:end,9);
h=figure;
if D == 2
  plot(xd,yd,'k');
  hold on;
  plot(xr,yr,'b');
  quiver(xr,yr,fx,fy,0,'g')
else
  plot3(xd,yd,zd,'k');
  hold on;
  plot3(xr,yr,zr);
  quiver3(xr,yr,zr,fx,fy,fz, 'color', 'g')
end

S=load(strcat(ejem ,'_unassisted.txt'));
xd=S(1:k:end,1);
yd=S(1:k:end,2);
zd=S(1:k:end,3);
xr=S(1:k:end,4);
yr=S(1:k:end,5);
zr=S(1:k:end,6);
if D == 2
  plot(xr,yr,'r');
else
  plot3(xr,yr,zr,'r');
end

axis equal;
set(gca,'YTick',[]);
set(gca,'XTick',[]);
set(gca,'ZTick',[]);


