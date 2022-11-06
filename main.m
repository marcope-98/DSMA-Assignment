clc;
clear;
close all;

init;

%% scan loop
r = LidarScan([Point(1), Point(2), deg2rad(90)], Coccupied, lidar_settings);

[isScanAccepted, loopClosureInfo, optimizationInfo] = addScan(slamAlg, r);

show(slamAlg);

[scans, optimizedPoses]  = scansAndPoses(slamAlg);
map = buildMap(scans, optimizedPoses, map_settings.Resolution, lidar_settings.maxRange);
%% plotting

figure
show(map)
% mapshow(buildings)
hold on;
% rectangle('Position', boundary)
% xlim([boundary(1) - padding, boundary(1) + boundary(3) + padding])
% ylim([boundary(2) - padding, boundary(2) + boundary(4) + padding])
scatter(Point(1), Point(2))
% show(slamAlg.PoseGraph)
hold off;

