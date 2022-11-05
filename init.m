%% includes
addpath(genpath('include'));

%% load constants
common;

%% initialize environment
% load environment
boundary = readmatrix(boundary_path);
buildings = shaperead(buildings_path);