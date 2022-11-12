%% includes
addpath(genpath('include'));

%% load constants and models
s = common;
l = lidar; % move constructor into agent class after

%% initialize environment
% load environment
boundary     = readmatrix(s.boundary_path);
world_limits = boundary + s.padding * [-1, -1, 2, 2] ;
buildings    = shaperead(s.buildings_path);

e = env(world_limits, boundary, buildings);

%% initialize SLAM algo
slamAlg = lidarSLAM(s.mapResolution, l.maxRange);
slamAlg.LoopClosureThreshold = 210;  
slamAlg.LoopClosureSearchRadius = 8;

clear boundary_path buildings_path
