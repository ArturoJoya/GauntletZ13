load('myGauntletScans.mat','r_all','theta_all');

%name key - G (global) N (neato) L (LIDAR)

%Neato origin with relation to Global
O_GN = [1.9, -2.7; 0, 0; 0, -2; 1.9, -1];
%Neato Orientation with relation to Global
oris_GN = [0; -pi/4; 0; pi];

%Lidar origin with relation to Neato
O_NL = [-0.084; 0];
%Translation matrix from Lidar to Neato Frame
T_NL = [1, 0, O_NL(1); 0, 1, O_NL(2); 0, 0, 1];

%Map = figure;
GFramePoints=[0;0;1];

% for each scan
for i = 1:length(oris_GN)
    LFramePoints = [r_all(:,i).*cos(theta_all(:,i)), r_all(:,i).*sin(theta_all(:,i))]';
    LFramePoints(3,:) = 1;
    
%     figure;
%     scatter(LFramePoints(1,:), LFramePoints(2,:));
%     title(["Scan", num2str(i), "in LIDAR Frame"]);
    
    NFramePoints = T_NL*LFramePoints;
    
%     figure;
%     scatter(NFramePoints(1,:), NFramePoints(2,:));
%     title(["Scan", num2str(i), "in Neato Frame"]);
    
    GFramePoints = horzcat(GFramePoints,[1, 0, O_GN(i,1); 0, 1, O_GN(i,2); 0, 0, 1]*[cos(oris_GN(i)), -sin(oris_GN(i)), 0; sin(oris_GN(i)), cos(oris_GN(i)), 0; 0, 0, 1]*NFramePoints);
    
    %figure(Map);
    %scatter(GFramePoints(1,:), GFramePoints(2,:));
    %hold on
    %title("All Scans in Global Frame")
end
clf
figure(1)
GFramePoints = GFramePoints(:, 2:end);
scatter(GFramePoints(1,:), GFramePoints(2,:));
%figure(Map);
    