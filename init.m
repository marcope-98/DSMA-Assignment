%% includes
addpath(genpath('include'));

%% load config, variables and constants
config;

%% initialize environment
% load environment
boundary     = readmatrix(boundary_path);
world_limits = boundary + padding * [-1, -1, 2, 2] ;
buildings    = shaperead(buildings_path);
% initialize environment
e = env(world_limits, boundary, buildings, map_cfg.Scale);
map = e.getMap();

%% initialize SLAM algo
slamAlg = lidarSLAM(map_cfg.mapResolution, lidar_cfg.maxRange, 20);
slamAlg.LoopClosureThreshold    = slam_cfg.LoopClosureThreshold;  
slamAlg.LoopClosureSearchRadius = slam_cfg.LoopClosureSearchRadius;

clear boundary  boundary_path ...
      buildings buildings_path ...
      padding world_limits
      
