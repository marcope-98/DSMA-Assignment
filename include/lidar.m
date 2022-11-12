classdef lidar
    properties (Constant)
        ScanSize  = 541;      % [-] // make this always an odd number so zero is a value in the angles91
        maxRange  = 20;      % [m]
        FOV       = 270;      % [deg]
    end
    
    properties
        angles    = [];                  % [rad]
    end
    

    
    methods
        function obj = lidar()
            obj.angles = deg2rad(-obj.FOV/2 : ...
                                  obj.FOV/(obj.ScanSize-1) : ...
                                  obj.FOV/2);
        end
        
        function scans = scan(obj, pose, map)
            ranges = obj.maxRange*ones(obj.ScanSize,1);
            pos = pose(1:2);
            yaw = pose(3);
            rotMat = [cos(yaw), -sin(yaw);...
                      sin(yaw), +cos(yaw)];
            %     figure
            %     plot(map)
            %     hold on;
            for i = 1 : obj.ScanSize
                lineseg = [pos; (pos' + obj.maxRange*rotMat*[cos(obj.angles(i)); sin(obj.angles(i))])'];
                [in,~] = intersect(map, lineseg);

                % [in,out] = intersect(map, lineseg);
                % plot(in(:,1), in(:,2), 'r',out(:,1), out(:,2), 'b')
                if numel(in) ~= 0 && norm(in(1,:) - pos) < obj.maxRange
                    ranges(i) = norm(in(1,:) - pos);
                end
            end
%             hold off;

            scans = lidarScan(ranges, obj.angles);
        
        end
        

    end
    
end