%function evalpath evaluates a GUIBRO path:
%The length, the mean distance,
function evalpaths(v)

for k = 1: size(v,3)
    %first compute the length
    length(k)=0;
    alpha(1,1,k)=0.0;
    alpha(1,2,k)=v(1,2,k);
    beta(1,1,k)=0.0;
    beta(1,2,k)=v(1,3,k);
    NF1(1,1,k)=0.0;
    NF1(1,2,k)=v(1,7,k);
    Dist(1,1,k)=0.0;
    Dist(1,2,k)=v(1,8,k);
    for i = 2: size(v,1)
        deltax=v(i,4,k)-v(i-1,4,k);
        deltay=v(i,5,k)-v(i-1,5,k);
        deltaz=v(i,6,k)-v(i-1,6,k);
        delta=sqrt(deltax*deltax+deltay*deltay+deltaz*deltaz);
        length(k) = length(k)+delta;
        alpha(i,1,k)=length(k);
        alpha(i,2,k)=v(i,2,k);
        beta(i,1,k)=length(k);
        beta(i,2,k)=v(i,3,k);
        NF1(i,1,k)=length(k);
        NF1(i,2,k)=v(i,7,k);
        Dist(i,1,k)=length(k);
        Dist(i,2,k)=v(i,8,k);
    end
    length(k)
    %Compute the mean distance
    distance(k)=0;
    for i = 1: size(v,1)
        distance(k)=distance(k)+v(i,8,k);
    end
    distance(k)/size(v,1)
end

c=[[0.8, 0.0 0.0],
    [0.0 0.8 0.0],
    [0.0 0.0 0.8],
    [0.5 0.5 0.0],
    [0.0 0.5 0.5],
    [0.5 0.0 0.5],
    [0.0 0.0 0.0]]

figure
plot(alpha(:,1,1),alpha(:,2,1),'Color',c(1,:))
title('Alpha');
for k = 2: size(v,3)
    hold on
    plot(alpha(:,1,k),alpha(:,2,k),'Color',c(k,:))
end

figure
plot(beta(:,1,1),beta(:,2,1),'Color',c(1,:))
title('Beta');
for k = 2: size(v,3)
    hold on
    plot(beta(:,1,k),beta(:,2,k),'Color',c(k,:))
end

figure
plot(NF1(:,1,1),NF1(:,2,1),'Color',c(1,:))
title('NF1');
for k = 2: size(v,3)
    hold on
    plot(NF1(:,1,k),NF1(:,2,k),'Color',c(k,:))
end

figure
plot(Dist(:,1,1),Dist(:,2,1),'Color',c(1,:))
title('Distance');
for k = 2: size(v,3)
    hold on
    plot(Dist(:,1,k),Dist(:,2,k),'Color',c(k,:))
end


