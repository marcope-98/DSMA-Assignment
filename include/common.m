padding = 100;  % [meters]
buildings_path = 'assets/buildings/buildings.shp';
boundary_path = 'assets/boundary.csv';

%% LIDAR parameters
LidarFOVdeg = 90;                           % [deg]
maxLidarRange  = 8;                         % [meters]
LidarFOV    = abs(deg2rad(lidarFOVdeg));    % [radians]

%% MAP parameters
mapResolution = 20;

%% AGENTS parameters
n_agents = 1000;