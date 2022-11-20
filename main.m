clc;
clear;
close all;

init;

%% scan loop
% history of poses
poses = zeros(slam_cfg.iterations,3);

% get initial pose
if(default)
    pose = [60,30, atan2(30,-10)];  
else
    pose = e.getValidPose();
end

a1 = agent(pose, agent_cfg);

for i = 1 : slam_cfg.iterations
i
poses(i,:) = a1.pose;
r = a1.sense(e.Coccupied);
addScan(slamAlg, r);
a1 = a1.step(r);
end


[scans, optimizedPoses]  = scansAndPoses(slamAlg);

% this rotation is just for plotting purposes: in this way both occupancy
% maps are represented rotated by the same initial quantity
rotMat = [cos(poses(1,3)), -sin(poses(1,3)); sin(poses(1,3)), cos(poses(1,3))];
optimizedPoses(:,1:2) = (rotMat*optimizedPoses(:,1:2)')' + poses(1,1:2);
optimizedPoses(:,3) = optimizedPoses(:,3) + poses(1,3);

map_slam = buildMap(scans, optimizedPoses, map_cfg.mapResolution, lidar_cfg.maxRange);
map_ground_truth = buildMap(scans, poses, map_cfg.mapResolution, lidar_cfg.maxRange);

%% plotting
% figure 1
figure
plot(map)
hold on;
scatter(optimizedPoses(:,1), optimizedPoses(:,2), 1, 'r')
scatter(poses(:,1), poses(:,2),1,'b')
title("Comparison of SLAM with ground truth")
hold off;

% figure 2
figure
t = tiledlayout(1,2);
title(t,'Occupancy maps')

nexttile
show(map_slam)
title("Optimized poses")

nexttile
show(map_ground_truth)
title("Ground truth")

% figure 3
figure
show(slamAlg)
title("SLAM algorithm result","Poses + scans")
