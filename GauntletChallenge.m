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
fNeato = 0;

%Visualize field and create syms equations
VisualizeField

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
    %get gradient of position (for ascent
    gradVal = (double(subs(gradNeato, {xN, yN}, {pos(1), pos(2)})));
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