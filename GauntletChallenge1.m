function NeatoPath = GauntletChallenge1()

load myGauntletmap.mat
%Function for completing level 1 of the gauntlet challenge
clf

%define map
points=[GFramePoints(1,:)',GFramePoints(2,:)'];
x = points(:,1);
y = points(:,2);
figure(2)
plot(x,y,'ks')
title('Scan Data- Clean')
xlabel('[m]')
ylabel('[m]')


%define Neato parameters
base = 0.235; %m
lambda = 0.05;

%Generate map potentials from given coordinates

%initialize potential field and gradient
f = 0;
fx = 0;
fy = 0;

%initialize syms for the Neato to use based on the points fitted
%establish potential field equations
syms xN yN a b
poteq = log(sqrt((xN-a).^2 +(yN-b).^2));
fNeato = 0;
%global points
[xG,yG]=meshgrid(-1.5:0.01:2.5,-3.37:0.01:1);
%[xG,yG]=meshgrid(-10:0.01:10,-10:0.01:10);

%Outline
for aO = -1.2:0.05:2.2
    f = f + log(sqrt((xG-aO).^2 + (yG-0.7).^2)) + log(sqrt((xG-aO).^2 + (yG+3.07).^2));
    fx = fx + (xG-aO)./((xG-aO).^2 +(yG-0.7).^2) + (xG-aO)./((xG-aO).^2 +(yG+3.07).^2);
    fy = fy + (yG-0.7)./((xG-aO).^2 +(yG-0.7).^2) + (yG+3.37)./((xG-aO).^2 +(yG+3.07).^2);
    fNeato = fNeato + subs(poteq,[a,b], [aO,0.7]) + subs(poteq,[a,b], [aO,-3.07]);
    figure(2)
    hold on
    plot(aO,-3.07, 'r.')
    plot(aO,0.7,'r.')
end
for bO = -3.07:0.05:0.7
    f = f + log(sqrt((xG+1.2).^2 + (yG-bO).^2)) + log(sqrt((xG-2.2).^2 + (yG-bO).^2));
    fx = fx + (xG+1.2)./((xG+1.2).^2 +(yG-bO).^2) + (xG-2.2)./((xG-2.2).^2 +(yG-bO).^2);
    fy = fy + (yG-bO)./((xG+1.2).^2 +(yG-bO).^2) + (yG-bO)./((xG-2.2).^2 +(yG-bO).^2);
    fNeato = fNeato + subs(poteq,[a,b], [-1.5,bO]) + subs(poteq,[a,b], [2.5,bO]);
    figure(2)
    hold on
    plot(-1.2,bO,'r.')
    plot(2.2,bO,'r.')
end
%Squares as circles
square_centers = [-0.25, -1; 1, -0.7; 1.41, -2]; 
arbitrary_constant = 0.05;
for i = 1:length(square_centers)
    for t = 0:0.4:2*pi
        aC = square_centers(i,1) + 0.25*cos(t);
        bC = square_centers(i,2) + 0.25*sin(t);
        f = f - arbitrary_constant.*log(sqrt((xG-aC).^2 +(yG-bC).^2));
        fx = fx - arbitrary_constant.*(xG-aC)./((xG-aC).^2 +(yG-bC).^2);
        fy = fy - arbitrary_constant.*(yG-bC)./((xG-aC).^2 +(yG-bC).^2);
        fNeato = fNeato - arbitrary_constant.*subs(poteq,[a,b], [aC,bC]);
        figure(2)
        hold on
        plot(aC,bC,'r.')
    end
end
%BoB
for t = 0:0.4:2*pi
    aC = 0.75 + 0.25*cos(t);
    bC = -2.5 + 0.25*sin(t);
    f = f + log(sqrt((xG-aC).^2 +(yG-bC).^2));
    fx = fx + (xG-aC)./((xG-aC).^2 +(yG-bC).^2);
    fy = fy + (yG-bC)./((xG-aC).^2 +(yG-bC).^2);
    fNeato = fNeato + subs(poteq,[a,b], [aC,bC]);
    figure(2)
    hold on
    plot(aC,bC,'b.')
end
gradNeato = gradient(fNeato, [xN, yN]);

contour(xG,yG,f, 'k', 'ShowText', 'On')
quiver(xG,yG,fx,fy)
contour3(xG, yG, f, 100)
view([0 90])
axis equal

%Run gradient ascent
r_i = [0; 0];
delta = 0.9;% delta > 1 causes stepsize to grow; this prevents convergence
lam = 0.05;
tol = 1e-3;
n = 0;
n_max = 25;
grad_i = double(subs(gradNeato, {xN, yN}, {r_i(1), r_i(2)}));
R = [r_i];

% Run the algorithm
%R = gradient_ascend(gradNeato, r_0, delta, lambda_0, tolerance, n_max);
while (n < n_max) %&& (norm(grad_i)> tol)
    r_i = r_i - lam.*grad_i;
    grad_i = (double(subs(gradNeato, {xN, yN}, {r_i(1), r_i(2)})));
    lam = delta * lam;
    n = n + 1;
    R(:, end+1) = r_i;
end

figure(2);
plot(R(:, 1), R(:, 2), "r")
hold on
plot(R(:, 1), R(:, 2), "r.", "MarkerSize", 10)
plot(R(1, 1), R(1, 2), "k.", "MarkerSize", 20)
plot(R(end, 1), R(end, 2), "r.", "MarkerSize", 20)

%set up Neato
head = [1;0];
pos = [0;0];
NeatoPath = [0,0];

angSpeed = 0.2;  %rad/s (set higher than real to help with testing)
linSpeed = 0.5;  %m/s

%setup publisher
pub = rospublisher('/raw_vel');
sub_states = rossubscriber('/gazebo/model_states', 'gazebo_msgs/ModelStates');
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
    %get gradient of position (for descent
    gradVal = -double(subs(gradNeato, {xN, yN}, {pos(1), pos(2)}));
    crossProd = cross([head; 0], [gradVal; 0]);
    turnDir = sign(crossProd(3));
    turnAngle = asin(norm(crossProd)/(norm(head)*norm(gradVal)));
    turnTime = double(turnAngle) / angSpeed;
    msg.Data = [-turnDir*angSpeed*base/2,
                turnDir*angSpeed*base/2];
    send(pub, msg);
    startTurn = rostic;
    while rostoc(startTurn) < 1.18*turnTime
        pause(0.01);
    end
    head = gradVal;
    
    %act on robot
    fwdDist = norm(gradVal*lambda);
    fwdTime = fwdDist / linSpeed;
    msg.Data = [linSpeed, linSpeed];
    send(pub, msg);
    startFwd = rostic;
    while rostoc(startFwd) < 0.91*fwdTime
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