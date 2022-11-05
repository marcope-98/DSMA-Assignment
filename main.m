clc;
clear;
close all;

init;

% plotting
mapshow(buildings)
hold on;
rectangle('Position', boundary)
xlim([boundary(1) - padding, boundary(1) + boundary(3) + padding])
ylim([boundary(2) - padding, boundary(2) + boundary(4) + padding])
hold off;

