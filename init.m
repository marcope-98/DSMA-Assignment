%% includes
addpath(genpath('include'));

%% load constants
common;

%% initialize environment
% load environment
boundary = readmatrix(boundary_path);
buildings = shaperead(buildings_path);

%% initialize SLAM algo
slamAlg = lidarSLAM(mapResolution, maxLidarRange);
slamAlg.LoopClosureThreshold = 210;  
slamAlg.LoopClosureSearchRadius = 8;

%% define C-free (in this way we can assign a legal initial position to the agents)
% ll, lr, ur, ul
Cfree = polyshape([boundary(1), boundary(1) + boundary(3), boundary(1) + boundary(3), boundary(1)],...
                  [boundary(2), boundary(2), boundary(2) + boundary(4), boundary(2) + boundary(4)]);
for i=1:numel(buildings)
    Cfree = subtract(Cfree, polyshape(buildings(i,:).X, buildings(i,:).Y));
end

Point = [0,0];
in = false;
while ~in
 Point(1)=unifrnd(boundary(1), boundary(3));
 Point(2)=unifrnd(boundary(2), boundary(4));
 in = isinterior(Cfree, Point(1), Point(2));
end


clear boundary_path buildings_path
