%% General constants and assets
padding = 100;
default = true;
buildings_path = 'assets/buildings/buildings.shp';
boundary_path  = 'assets/boundary.csv';

%% Map  config
map_cfg = struct;
map_cfg.mapResolution = 5;
map_cfg.Scale = 0.1;

%% Lidar config
lidar_cfg = struct;
lidar_cfg.scanSize  = 541;      % [-] // make this always an odd number so zero is a value in the angles91
lidar_cfg.minRange  = 0.1;
lidar_cfg.maxRange  = 20;       % [m]
lidar_cfg.FOV       = 270;      % [deg]

%% SLAM config
slam_cfg = struct;
slam_cfg.iterations = 300;
slam_cfg.LoopClosureThreshold = 210;
slam_cfg.LoopClosureSearchRadius = 8;

%% Agent constants
agent_cfg = struct;
agent_cfg.velocity  = 1.0; % [m/s] constant velocity of the agent
agent_cfg.tick      = 5;   % [Hz] update of position per second
agent_cfg.lidar_cfg = lidar_cfg;
