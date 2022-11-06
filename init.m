%% includes
addpath(genpath('include'));

%% load constants
common;

%% initialize environment
% load environment
boundary     = readmatrix(boundary_path);
world_limits = boundary + [-padding, -padding, 2*padding, 2*padding] ;
buildings    = shaperead(buildings_path);

%% initialize SLAM algo
slamAlg = lidarSLAM(map_settings.Resolution, lidar_settings.maxRange);
slamAlg.LoopClosureThreshold = 200;  
slamAlg.LoopClosureSearchRadius = 3;

%% define C-free (in this way we can assign a legal initial position to the agents)
% ll, lr, ur, ul
Cfree = polyshape([boundary(1), boundary(1) + boundary(3), boundary(1) + boundary(3), boundary(1)],...
                  [boundary(2), boundary(2), boundary(2) + boundary(4), boundary(2) + boundary(4)]);
Coccupied = polyshape([world_limits(1), world_limits(1) + world_limits(3), world_limits(1) + world_limits(3), world_limits(1)],...
                      [world_limits(2), world_limits(2), world_limits(2) + world_limits(4), world_limits(2) + world_limits(4)]);
                  
for i=1:numel(buildings)
    Cfree = subtract(Cfree, polyshape(buildings(i,:).X, buildings(i,:).Y));
end
Coccupied = subtract(Coccupied, Cfree);

Point = [0,0];
in = false;
while ~in
 Point(1)=unifrnd(boundary(1), boundary(3));
 Point(2)=unifrnd(boundary(2), boundary(4));
 in = isinterior(Cfree, Point(1), Point(2));
end
Point = [527,425]; % This is a known viable point

clear boundary_path buildings_path
