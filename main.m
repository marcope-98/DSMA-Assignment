clc;
clear;
close all;

init;

%% scan loop
% create ranges n_scans from 0 to maxrange;
% pass by reference and change the values of each of the range based on the
% raytracing
% need absolute pose of the robot
% maybe add noise

%% plotting

mapshow(buildings)
hold on;
rectangle('Position', boundary)
xlim([boundary(1) - padding, boundary(1) + boundary(3) + padding])
ylim([boundary(2) - padding, boundary(2) + boundary(4) + padding])
scatter(Point(1), Point(2))
hold off;

