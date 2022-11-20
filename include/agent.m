classdef agent

    properties
        pose = [];
        vfh;
        sensor;
        deltaX;
    end
        
    methods
        %% Constructor
        function obj = agent(input_pose, cfg)
            obj.pose = input_pose;
            obj.vfh = controllerVFH;
            obj.vfh.UseLidarScan = true;
            obj.sensor = lidar(cfg.lidar_cfg);
            obj.deltaX = cfg.velocity / cfg.tick;
        end
            
        %% Getters        
        function res = pos(obj)
            res = obj.pose(1:2);
        end
    
        function res = yaw(obj)
            res = obj.pose(3);
        end
            
        %% Methods
        function communicate(obj)
            % To implement: communication client-server
        end
        
        function res = sense(obj, map)
            res = obj.sensor.scan(obj.pose,map);
        end

        
        function res = step(obj, scans)
            steeringDir = obj.vfh(scans,0);
            % get rotation matrix
            rotMat = obj.rotationMatrix();
            % update the pose of the robot
            obj.pose(1:2) = obj.pose(1:2) + ...
                           (obj.deltaX * rotMat * [cos(steeringDir); sin(steeringDir)])';
            obj.pose(3) = obj.pose(3) + steeringDir;
            res = obj;
        end
        
        function res = rotationMatrix(obj)
            res = [cos(obj.pose(3)), -sin(obj.pose(3));...
                   sin(obj.pose(3)), +cos(obj.pose(3))];
        end
            
            
    end
end