function [fit_params,bestInlierSet,bestOutlierSet]= CircFitKnownR(points,d,n,R)
% %eliminate zeros
% index=find(r~=0 & r<3);
% r_clean=r(index);
% theta_clean=theta(index);
% 
% %convert to Cartesian and plot again for verification
% [x,y]=pol2cart(deg2rad(theta_clean),r_clean);
% points=[x,y];
 candpts = [0,0];
 %bestcandidates = [];
 bestInlierSet = zeros(0,2);
 bestOutlierSet = zeros(0,2);
 bestEndPoints = zeros(0,2);
 fit_params = zeros(0,3);

 
 for l=1:n
     %obtain points to find a candidate circle
     %uncomment if Statistics and Machine Learning Toolbox
     candpts = datasample(points, 3, 'Replace', false);
     %indeces = randperm(length(points),3);
     %for i = 1:length(indeces)
     %    candpts(i,:) = points(indeces(i),:);
     %end
     dists = [];
     
     %generate candidate circle parameters from candidate circle points
     A = [ones(length(candpts),1), 2*candpts(:,1), 2*candpts(:,2)];
     p = (A'*A)\(A'*[candpts(:,1).^2, candpts(:,2).^2, zeros(length(candpts),1)-(R^2)]);
     h = p(2,1)+p(2,2);
     k = p(3,1)+p(3,2);
     rad = R;
     center = [-h;-k];
     %generate candidate circle from parameters
     t = linspace(0,2*pi,100);
     x_cir = center(1) + rad*cos(t);
     y_cir = center(2) + rad*sin(t);
     cand_cir = [x_cir; y_cir];
     plot(x_cir, y_cir);
     
     for m = 1:length(points(:,1))
         dirCir2Pt = [];
         dirCen2Pt = [];
         closestdist = 1;
         for o = 1:length(x_cir)
             dirCir2Pt = points(m,:)'-cand_cir(:,o);
             if norm(dirCir2Pt) < closestdist
                 closestdist = norm(dirCir2Pt);
             end
         end
         dists(end+1) = closestdist;
     end
     
     inliers = abs(dists) < d;
     if sum(inliers) > size(bestInlierSet,1);
        bestInlierSet=points(inliers, :); %points where logical array is true
        bestOutlierSet = points(~inliers, :); %points where logical array is not true
        %bestcandidates=candpts;
        fit_params = [center(1) center(2) rad];
     end
 end

%plot the polar data as verification
%figure(1)
%polarplot(deg2rad(theta_clean),r_clean,'ks','MarkerSize',6,'MarkerFaceColor','m')
%title('Visualization of Polar Data')

%figure(2)
%plot(points(:,1),points(:,2),'ks')
%title('Scan Data- Clean')
%xlabel('[m]')
%ylabel('[m]')

%Now we need to plot our results
clf
figure(2)
t = linspace(0,2*pi,100);
x_cir = fit_params(1) + R*cos(t);
y_cir = fit_params(2) + R*sin(t);
plot(bestInlierSet(:,1), bestInlierSet(:,2), 'ks')
hold on
plot(bestOutlierSet(:,1),bestOutlierSet(:,2),'bs')
plot(x_cir, y_cir);
%plot(bestEndPoints(:,1), bestEndPoints(:,2), 'r')
legend('Inliers','Outliers','Best Fit','location','northwest')
title(['RANSAC with d=' num2str(d) ' and n=' num2str(n)])
xlabel('[m]')
ylabel('[m]')
% Create textbox
annotation(figure(2),'textbox',...
    [0.167071428571429 0.152380952380952 0.25 0.1],...
    'String',{'Number of Inliers:' num2str(size(bestInlierSet,1))},...
    'FitBoxToText','off');

end