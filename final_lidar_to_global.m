clear all; clf; hold on;
load gauntletScans.mat theta_all r_all

% Define Neato scan locations and angles
global_locations = [1.9, -2.7; 0, 0; 0, -2; 1.9, -1];
global_angles = [0; -pi/4; 0; pi];

% Define vector offset for the LiDAR sensor with relation to the Neato
lidar_offset = [-0.084, 0];

[L_frame_points N_frame_points G_frame_points] = deal(zeros(360,8));

% Convert LiDAR scans to cartesian
for i = 1:length(global_angles) 
    L_frame_points(:,[2*i-1 2*i]) = [r_all(:,i).*cos(theta_all(:,i)) r_all(:,i).*sin(theta_all(:,i))];
    N_frame_points(:,[2*i-1 2*i]) = L_frame_points(:,[2*i-1 2*i]) + lidar_offset;
    G_frame_points(:,[2*i-1 2*i]) = N_frame_points(:,[2*i-1 2*i])*[cos(global_angles(i)) -sin(global_angles(i)); sin(global_angles(i)) cos(global_angles(i))] + global_locations(i, :);
    plot(G_frame_points(:,2*i-1), G_frame_points(:,2*i), "*")
end

title("Neato LiDAR Scans")
xlabel("Distance (m)"); ylabel("Distance (m)")
legend({"Scan 1", "Scan 2", "Scan 3", "Scan 4"})