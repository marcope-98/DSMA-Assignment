classdef lidar
    properties
        angles    = [];                  % [rad]
        minRange;
        maxRange;
        scanSize;
    end
    

    
    methods
        function obj = lidar(cfg)
            % Constructor
            obj.angles = deg2rad(-cfg.FOV/2 : ...
                                  cfg.FOV/(cfg.scanSize-1) : ...
                                  cfg.FOV/2);
            obj.minRange = cfg.minRange;
            obj.maxRange = cfg.maxRange;
            obj.scanSize = cfg.scanSize;
        end
        
        function scans = scan(obj, pose, map)
            % this is basically a naive raytracing algorithm
            ranges = zeros(obj.scanSize,1);                        
            pos = pose(1:2);
            yaw = pose(3);
            rotMat = [cos(yaw), -sin(yaw);...
                      sin(yaw), +cos(yaw)];
            for i = 1 : obj.scanSize
                % linesegment is a 2x2 matrix:
                %   first  row is starting point of line segment (i.e. pos)
                %   second row is the final point of the line segment (i.e. the ray with maxRange length)
                %       the ray needs to be rotated according to the yaw of the agent
                lineseg = [pos; (pos' + obj.maxRange*rotMat*[cos(obj.angles(i)); sin(obj.angles(i))])'];
                % check for intersection with the map
                [in,~] = intersect(map, lineseg);
                % if there is at least a point inside Coccupied and the
                %   intersection is less then the maxRange of the lidar add
                %   the norm to the ranges
                if numel(in) ~= 0 && norm(in(1,:) - pos) < obj.maxRange
                    ranges(i) = norm(in(1,:) - pos);
                end
            end
            % create lidarScans and return
            scans = lidarScan(ranges, obj.angles);
            end
        

    end
    
end