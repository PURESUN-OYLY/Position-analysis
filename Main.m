
%% Test GeoEntity class

% Clear the workspace
clear;      % Clear the workspace
clc;        % Clear the command window
close all;  % Close all the figure
axis equal; % Set the axis equal, to show the entity in the same size
axis vis3d;

view(45, 30);

geoEnt = GeoEntity();

obj1 = geoEnt.Create('frustum');
obj1.setPos([1, 2, 3]);
obj1.setRad(3);
obj1.setHeight(4)
% obj1.setCen([1, 2, 3, 1, 2, 0]);
obj1.setCen(1, 2, 3);
obj1.setRotTb(0, 1, 0, 0, 0, 0);

% disp(obj1.pos)
% disp(geoEnt.count)

geoEnt.renderAll();
