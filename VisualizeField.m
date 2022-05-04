%Gauntlet Challenge Visualizations

close all
load myGauntletMap

%define Neato parameters
base = 0.235; %m
lambda = 0.05;
%define map
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
%initialize potential field and gradient
f = 0;
fx = 0;
fy = 0;
%global points
[xG,yG]=meshgrid(-1:0.01:2,-3:0.01:1);

while size(bestOutlierSet,1) > 30
    [fitline_coefs(nn,:),bestInlierSet,bestOutlierSet,bestEndPoints(:,:,nn)]= robustLineFit(x,y,d,n,visualize);
    
    if isnan(fitline_coefs(nn,1))
        disp('All Lines Identified')
        break;
    end
    
    %create the sources from lines defined by inliers
    for i = 1:length(bestInlierSet)
    f = f - log(sqrt((xG-bestInlierSet(i,1)).^2 +(yG-bestInlierSet(i,2)).^2));
    fx = fx - (xG-bestInlierSet(i,1))./((xG-bestInlierSet(i,1)).^2 +(yG-bestInlierSet(i,2)).^2);
    fy = fy - (yG-bestInlierSet(i,2))./((xG-bestInlierSet(i,1)).^2 +(yG-bestInlierSet(i,2)).^2);
    end
    
    %generate the points to feed back into the robustLineFit function
    x = bestOutlierSet(:,1);
    y = bestOutlierSet(:,2);
    
    nn=nn+1;
end

%initialize and fit circle to outlier set
figure;
n=9001;
R = 0.25;
[fit_params,bestInlierSet,bestOutlierSet]= CircFitKnownR(bestOutlierSet,d,n,R);

%generate sinks from circle fit
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
%plot Gauntlet map with all visualizations
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

contour(xG,yG,f, 'k', 'ShowText', 'On')
quiver(xG,yG,fx,fy)
axis equal