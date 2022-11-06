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
slamAlg.LoopClosureThreshold = 200;  
slamAlg.LoopClosureSearchRadius = 3;

Point = [527,425]; % This is a known viable point

clear boundary_path buildings_path
