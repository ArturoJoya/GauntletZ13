close all

%Script to identify multiple lines in a scan
%load playpensample
load myGauntletMap.mat
%eliminate zeros and sample farther than 3m (range of LIDAR)
%index=find(r_all~=0 & r_all<3);
%r_clean=r_all(index);
%theta_clean=theta_all(index);

%plot the polar data as verification
%figure(1)
%polarplot(deg2rad(theta_clean),r_clean,'ks','MarkerSize',6,'MarkerFaceColor','m')
%title('Visualization of Polar Data')

%convert to Cartesian and plot again for verification
%[x,y]=pol2cart(deg2rad(theta_clean),r_clean);
points=[GFramePoints(1,:)',GFramePoints(2,:)'];
x = points(:,1);
y = points(:,2);
figure(2)
plot(points(:,1),points(:,2),'ks')
title('Scan Data- Clean')
xlabel('[m]')
ylabel('[m]')

bestOutlierSet=points;
nn=1;
visualize=0;
d=0.018;
n=6000;

f = 0;
fx = 0;
fy = 0;
[xG,yG]=meshgrid(-1:0.01:2,-3:0.01:1);
while size(bestOutlierSet,1) > 30
    [fitline_coefs(nn,:),bestInlierSet,bestOutlierSet,bestEndPoints(:,:,nn)]= robustLineFit(x,y,d,n,visualize);
    
    if isnan(fitline_coefs(nn,1))
        disp('All Lines Identified')
        break;
    end
    
    for i = 1:length(bestInlierSet)
    f = f - log(sqrt((xG-bestInlierSet(i,1)).^2 +(yG-bestInlierSet(i,2)).^2));
    fx = fx - (xG-bestInlierSet(i,1))./((xG-bestInlierSet(i,1)).^2 +(yG-bestInlierSet(i,2)).^2);
    fy = fy - (yG-bestInlierSet(i,2))./((xG-bestInlierSet(i,1)).^2 +(yG-bestInlierSet(i,2)).^2);
    end
    %[theta_clean,r_clean]=cart2pol(bestOutlierSet(:,1),bestOutlierSet(:,2));
    %theta_clean=rad2deg(theta_clean);
    x = bestOutlierSet(:,1);
    y = bestOutlierSet(:,2);
    
    nn=nn+1;
end
figure;
n=9001;
R = 0.25;
%plot(bestOutlierSet(:,1), bestOutlierSet(:,2))
[fit_params,bestInlierSet,bestOutlierSet]= CircFitKnownR(bestOutlierSet,d,n,R);

% t = linspace(0,2*pi,100);
% a = fit_params(1) + R*cos(t);
% b = fit_params(2) + R*sin(t);

for t = linspace(0,2*pi,450)
    a = fit_params(1) + R*cos(t);
    b = fit_params(2) + R*sin(t);
    f = f + log(sqrt((xG-a).^2 +(yG-b).^2));
    fx = fx + (xG-a)./((xG-a).^2 +(yG-b).^2);
    fy = fy + (yG-b)./((xG-a).^2 +(yG-b).^2);
end

t = linspace(0,2*pi,100);
x_cir = fit_params(1) + R*cos(t);
y_cir = fit_params(2) + R*sin(t);

figure(1);
plot(x,y,'ks')
hold on
for kk=1:size(bestEndPoints,3)
    plot(bestEndPoints(:,1,kk), bestEndPoints(:,2,kk), 'r')
end
plot(x_cir,y_cir)
title(['RANSAC with d=' num2str(d) ' and n=' num2str(n)])
xlabel('[m]')
ylabel('[m]')

%figure(2)
contour(xG,yG,f, 'k', 'ShowText', 'On')
quiver(xG,yG,fx,fy)
%plot(x_cir, y_cir);
axis equal