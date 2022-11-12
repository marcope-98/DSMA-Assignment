%% includes
addpath(genpath('include'));

%% load constants and models
s = common; % settings/constants
l = lidar;  % TODO: move constructor into agent class after

%% initialize environment
% load environment
boundary     = readmatrix(s.boundary_path);
world_limits = boundary + s.padding * [-1, -1, 2, 2] ;
buildings    = shaperead(s.buildings_path);
% initialize environment
e = env(world_limits, boundary, buildings);

%% initialize SLAM algo
% I have no idea how to tune these hyperparameters
slamAlg = lidarSLAM(s.mapResolution, l.maxRange);
slamAlg.LoopClosureThreshold = 210;  
slamAlg.LoopClosureSearchRadius = 8;

clear boundary world_limits buildings