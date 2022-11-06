clc;
clear;
close all;

init;

%% scan loop
r = l.scan(e.getValidPose(), e.Coccupied);
[isScanAccepted, loopClosureInfo, optimizationInfo] = addScan(slamAlg, r);
show(slamAlg);
[scans, optimizedPoses]  = scansAndPoses(slamAlg);
map = buildMap(scans, optimizedPoses, s.mapResolution, l.maxRange);
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

