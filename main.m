clc;
clear;
close all;

init;

%% scan loop
% pose = e.getValidPose();
% pose = [Point(1),Point(2),deg2rad(90)];
iterations = 100;
poses = zeros(iterations,3);
% 60 30
% 50 60
% pose_list = [60, 30] + (1/iterations)*(sqrt(30^2 + 10^2))*(60-30)/(60-50);

% pose_list = [60, 30] + (1/iterations)*(sqrt(30^2 + 10^2))*(-0.2);
pose = [60,30, atan2(30,-10)];
for i = 1 : iterations
i
poses(i,:) = pose;

r = l.scan(pose, e.Coccupied);

[isScanAccepted, loopClosureInfo, optimizationInfo] = addScan(slamAlg, r);

% show(slamAlg);
% VFH = l.maxRange - r.Ranges;
% middle = (numel(VFH)-1)/2 + 1;
% [~, candidate] = min(VFH);
% if VFH(candidate) >= VFH(middle) 
%     candidate = middle;
% end
% rotMat = [cos(pose(3)), -sin(pose(3));...
%           sin(pose(3)), +cos(pose(3))];
% pose(1:2) = pose(1:2) + (s.deltaX*rotMat*[cos(l.angles(candidate)); sin(l.angles(candidate))])';

pose(1:2) = pose(1:2) + (0.01*[-10, 30]);
% pose(3) = pose(3) + l.angles(candidate);

end
[scans, optimizedPoses]  = scansAndPoses(slamAlg);
map = buildMap(scans, optimizedPoses, s.mapResolution, l.maxRange);
%% plotting

figure
% show(map)
plot(e.Coccupied)
% mapshow(buildings)
hold on;
scatter(optimizedPoses(:,1), optimizedPoses(:,2),1, 'r')
scatter(poses(:,1), poses(:,2),1,'b')
% rectangle('Position', boundary)
% xlim([boundary(1) - padding, boundary(1) + boundary(3) + padding])
% ylim([boundary(2) - padding, boundary(2) + boundary(4) + padding])
% scatter(Point(1), Point(2))
% show(slamAlg.PoseGraph)
hold off;

