clear all; clf;
sub = rossubscriber('/scan');

% place Neato at the origin pointing in the ihat_G direction
pub = rospublisher('/raw_vel');
sub_states = rossubscriber('/gazebo/model_states', 'gazebo_msgs/ModelStates');
msg = rosmessage(pub);
%stop robot
msg.Data = [0, 0];
send(pub, msg);
placeNeato(1.9,-2.7,1,0)
pause(2);

% Collect data at the room origin
scan_message = receive(sub);
r_1 = scan_message.Ranges(1:end-1);
theta_1 = deg2rad([0:359]');

% place Neato at the origin pointing in a different direction
placeNeato(0,0,cos(pi/4),sin(pi/4))
pause(2);

% Collect data for the second location
scan_message = receive(sub);
r_2 = scan_message.Ranges(1:end-1);
theta_2 = deg2rad([0:359]');

% Place Neato at the third location
placeNeato(0,-2,1,0)
pause(2);

% Collect data for the third location
scan_message = receive(sub);
r_3 = scan_message.Ranges(1:end-1);
theta_3 = deg2rad([0:359]');

% Place Neato at the fourth location
placeNeato(1.9,-1,cos(pi),sin(pi))
pause(2);

% Collect data for the fourth location
scan_message = receive(sub);
r_4 = scan_message.Ranges(1:end-1);
theta_4 = deg2rad([0:359]');

% Collect all data into matrices
r_all = [r_1 r_2 r_3 r_4];
theta_all = [theta_1 theta_2 theta_3 theta_4];

% Save scans in a .mat file
save gauntletScans.mat theta_all r_all