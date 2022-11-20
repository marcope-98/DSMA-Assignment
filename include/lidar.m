classdef lidar
    properties (Constant)
        ScanSize  = 541;      % [-] // make this always an odd number so zero is a value in the angles91
        minRange  = 0.1;
        maxRange  = 20;       % [m]
        FOV       = 270;      % [deg]
    end
    
    properties
        angles    = [];                  % [rad]
    end
    

    
    methods
        function obj = lidar()
            % Constructor
            obj.angles = deg2rad(-obj.FOV/2 : ...
                                  obj.FOV/(obj.ScanSize-1) : ...
                                  obj.FOV/2);
        end
        
        function scans = scan(obj, pose, map)
            % this is basically a naive raytracing algorithm
            
%             ranges = obj.maxRange*ones(obj.ScanSize,1);
            ranges = zeros(obj.ScanSize,1);                        
            pos = pose(1:2);
            yaw = pose(3);
            rotMat = [cos(yaw), -sin(yaw);...
                      sin(yaw), +cos(yaw)];
            for i = 1 : obj.ScanSize
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