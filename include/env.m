classdef env < handle

    properties
        boundary;
        Cfree;
        Coccupied;
    end

    
    methods
        function  obj = env(world, boundary, obstacles, Scale)
            obj.boundary  = boundary;
            obj.Cfree     = polyshape([boundary(1), boundary(1) + boundary(3), boundary(1) + boundary(3), boundary(1)],...
                                      [boundary(2), boundary(2),               boundary(2) + boundary(4), boundary(2) + boundary(4)]);
            obj.Coccupied = polyshape([world(1), world(1) + world(3), world(1) + world(3), world(1)],...
                                      [world(2), world(2),            world(2) + world(4), world(2) + world(4)]);
                  
            for i=1:numel(obstacles)
                obj.Cfree = subtract(obj.Cfree, polyshape(obstacles(i,:).X, obstacles(i,:).Y));
            end
            obj.Coccupied = subtract(obj.Coccupied, obj.Cfree);
            
            obj.boundary  = obj.boundary * Scale;
            obj.Coccupied = scale(obj.Coccupied, Scale);
            obj.Cfree     = scale(obj.Cfree, Scale);
        end
        
        function pose = getValidPose(obj)
            pose = [0,0,0];
            in = false;
            while ~in
                % sample random x and y coordinates in map boundary
                pose(1)=unifrnd(obj.boundary(1), obj.boundary(3));
                pose(2)=unifrnd(obj.boundary(2), obj.boundary(4));
                % check if sampled coordinates are inside Cfree
                in = isinterior(obj.Cfree, pose(1), pose(2));
            end
            % assign random yaw (in radiants)
            pose(3) = deg2rad(360*rand());
        end
        
        function map = getMap(obj)
            map = obj.Coccupied;
        end
    end

end
