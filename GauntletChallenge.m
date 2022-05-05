function NeatoPath = GauntletChallenge()
%Function used to complete Level 2 of the Gauntlet Challenge
%The steps involve - CollectScans (or load Scans), combine scans(onto one
%conjunction of points), fit the BOB to the data set to get sink, use other
%points to create sources. Generate potential field from these points,
%calculate gradient descent / ascent. Drive and calculate gradient ascent /
%decent (refer to flatland assignment. Stop

%define Neato parameters
base = 0.235; %m
lambda = 0.05;

%define Circle fitting parameters
d = 0.005; %m
n = 1000; %iterations for circle fit
R = 0.25; %m
%GFramePoints = [];

%establish potential field equations
%syms x y a b
%poteq = log(sqrt((x-a).^2 +(y-b).^2));
%gradxeq = (x-a)./((x-a).^2 +(y-b).^2);
%gradyeq = (y-b)./((x-a).^2 +(y-b).^2);
fNeato = 0;

%Visualize field and create syms equations
VisualizeField
%generate gradient for Neato to use
gradNeato = gradient(fNeato, [xN, yN]);

% 
% %Create Gauntlet Map from points scanned in real time
% collectScans
% makeGauntletMap
% GFramePoints = GFramePoints(1:2,:)';
% 
% [xGlobal,yGlobal]=meshgrid(-3:0.05:3,-3:0.05:3);
% %TODO fix function to be more accurate
% [fit_params,bestInlierSet,bestOutlierSet]= CircFitKnownR(GFramePoints,d,n,R);
% %TODO, condense f so that subbing in values takes less time 
% for i = 1:length(bestInlierSet)
%     f = f + subs(poteq,[a,b], [bestInlierSet(i,1), bestInlierSet(i,2)]);
% end
% for j = 1:length(bestOutlierSet)
%     f = f - subs(poteq,[a,b], [bestOutlierSet(j,1), bestOutlierSet(j,2)]);
% end
% grad = gradient(f, [x, y]);
% 
% for k = 1:length(xGlobal)
%     for l = 1:length(yGlobal)
%         visf(end+1) = subs(f, [x,y], [xGlobal(k), yGlobal(l)]);
%         visgrad(:,end+1) = subs(grad, [x,y], [xGlobal(k), yGlobal(l)]);
%     end
% end
% 
% %define BoB
% t = linspace(0,2*pi,100);
% x_cir = fit_params(1) + R*cos(t);
% y_cir = fit_params(2) + R*sin(t);
% 
% clf
% figure();
% contour(xGlobal,yGlobal,visf, 'k', 'ShowText', 'On')
% hold on
% quiver(xGlobal,yGloval,visgrad(1,:),visgrad(2,:))
% plot(x_cir, y_cir);
% axis equal
% hold off

head = [0;1];
pos = [0;2];

angSpeed = 0.2;  %rad/s (set higher than real to help with testing)
linSpeed = 0.75;  %m/s

%setup publisher
pub = rospublisher('/raw_vel');
msg = rosmessage(pub);
%stop robot
msg.Data = [0, 0];
send(pub, msg);
pause(2);

%place Neato at starting location
placeNeato(pos(1), pos(2), head(1), head(2));
pause(2);

reachedbob = false;
while ~reachedbob
    %get gradient of position (for ascent
    gradVal = double(subs(grad, {x, y}, {pos(1), pos(2)}));
    crossProd = cross([head; 0], [gradVal; 0]);
    turnDir = sign(crossProd(3));
    turnAngle = asin(norm(crossProd)/(norm(head)*norm(gradVal)));
    turnTime = double(turnAngle) / angSpeed;
    msg.Data = [-turnDir*angSpeed*base/2,
                turnDir*angSpeed*base/2];
    send(pub, msg);
    startTurn = rostic;
    while rostoc(startTurn) < turnTime
        pause(0.01);
    end
    head = gradVal;
    
    %act on robot
    fwdDist = norm(gradVal*lambda);
    fwdTime = fwdDist / linSpeed;
    msg.Data = [linSpeed, linSpeed];
    send(pub, msg);
    startFwd = rostic;
    while rostoc(startFwd) < fwdTime
        nmsg = receive(sub_states);
        for j = 1 : length(nmsg.Name)
            if strcmp(nmsg.Name{j}, 'neato_standalone')
                posX = nmsg.Pose(j).Position.X;
                posY = nmsg.Pose(j).Position.Y;
            end
        end
        NeatoPath(end+1,:) = [posX, posY];
        pause(0.01)
    end
    % update position for the next iteration
    pos = pos + gradVal*lambda;
    reachedbob = fwdDist < 0.01;
end
msg.Data = [0, 0];
send(pub, msg);
end