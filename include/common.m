classdef common
    properties(Constant)
        padding = 100;
        buildings_path = 'assets/buildings/buildings.shp';
        boundary_path  = 'assets/boundary.csv';
        mapResolution  = 5; % original value: 20
        deltaX = 1; % take one meter step 
    end
end