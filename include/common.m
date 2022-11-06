%% misc
padding        = 100;  % [meters]
buildings_path = 'assets/buildings/buildings.shp';
boundary_path  = 'assets/boundary.csv';



%% LIDAR parameters
lidar_settings = struct;
lidar_settings.ScanSize  = 91; % make this always an odd number so zero is a value in the angles
lidar_settings.maxRange  = 40;
lidar_settings.FOVdeg    = 90;
lidar_settings.Stepdeg   = lidar_settings.FOVdeg/(lidar_settings.ScanSize-1);
lidar_settings.FOVrad    = abs(deg2rad(lidar_settings.FOVdeg));
lidar_settings.Angles    = deg2rad(-lidar_settings.FOVdeg/2 : ...
                                    lidar_settings.Stepdeg : ...
                                    lidar_settings.FOVdeg/2);
%% MAP parameters
map_settings = struct;
map_settings.Resolution  = 20;

%% AGENTS parameters
nAgents       = 1;
