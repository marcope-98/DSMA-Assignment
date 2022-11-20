clc;
clear;
close all;

init;

%% scan loop
% pose = e.getValidPose();
iterations = 300;
poses = zeros(iterations,3);
deltaYaw = 0;
deltaX   = 0.2;
pose = [60,30, atan2(30,-10)];
for i = 1 : iterations
i
poses(i,:) = pose;

r = l.scan(pose, e.Coccupied);

[isScanAccepted, loopClosureInfo, optimizationInfo] = addScan(slamAlg, r);

steeringDir = vfh(r,0);
rotMat = [cos(pose(3)), -sin(pose(3));...
          sin(pose(3)), +cos(pose(3))];
pose(1:2) = pose(1:2) + (deltaX*rotMat*[cos(deg2rad(deltaYaw)); sin(deg2rad(deltaYaw))])';
pose(3) = pose(3) + steeringDir;

end


[scans, optimizedPoses]  = scansAndPoses(slamAlg);

% this rotation is just for plotting purposes: in this way both occupancy
% maps are represented rotated by the same initial quantity
rotMat = [cos(poses(1,3)), -sin(poses(1,3)); sin(poses(1,3)), cos(poses(1,3))];
optimizedPoses(:,1:2) = (rotMat*optimizedPoses(:,1:2)')' + poses(1,1:2);
optimizedPoses(:,3) = optimizedPoses(:,3) + poses(1,3);

map_slam = buildMap(scans, optimizedPoses, s.mapResolution, l.maxRange);
map_ground_truth = buildMap(scans, poses, s.mapResolution, l.maxRange);

%% plotting
% figure 1
figure
plot(e.Coccupied)
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
