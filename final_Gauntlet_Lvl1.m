function NeatoPath = final_Gauntlet_Lvl1()

syms xN yN a b
poteq = log(sqrt((xN-a).^2 +(yN-b).^2));
fNeato = 0;
%global points
%[xG,yG]=meshgrid(-1.5:0.01:2.5,-3.37:0.01:1);
%[xG,yG]=meshgrid(-10:0.01:10,-10:0.01:10);
arbitrary_constant = 0.06;
%Outline
for aO = -1.2:0.05:2.2
    fNeato = fNeato + subs(poteq,[a,b], [aO,0.7]) + subs(poteq,[a,b], [aO,-3.07]);
end
for bO = -3.07:0.05:0.7
    fNeato = fNeato + subs(poteq,[a,b], [-1.5,bO]) + subs(poteq,[a,b], [2.5,bO]);
end
% %Squares as circles
square_centers = [-0.25, -1; 1, -0.7; 1.41, -2]; 
for i = 1:length(square_centers)
    for t = 0:0.4:2*pi
        aC = square_centers(i,1) + 0.353*cos(t);
        bC = square_centers(i,2) + 0.353*sin(t);
        fNeato = fNeato - arbitrary_constant.*subs(poteq,[a,b], [aC,bC]);
    end
end
%BoB
for t = 0:0.4:2*pi
    aC = 0.75 + 0.25*cos(t);
    bC = -2.5 + 0.25*sin(t);
    fNeato = fNeato + subs(poteq,[a,b], [aC,bC]);
end
gradNeato = gradient(fNeato, [xN, yN]);

%set up Neato
base = 0.235; %m
lambda = 0.05;
head = [1;0];
pos = [-0.5;0];
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

n=0;
%reachedbob = false;
while n < 10
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
    while rostoc(startTurn) < 1*turnTime
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
    %reachedbob = fwdDist < 0.01;
    n = n+1;
end
msg.Data = [0, 0];
send(pub, msg);

end