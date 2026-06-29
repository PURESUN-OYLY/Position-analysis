
%% Test GeoEntity class

% Clear the workspace
clear; clc; close all;

% obj1 = GeoEntity(GeoEntity.sphere)

% disp(['Create ', GeoEntity.sphere{2}])

% Test the constructor
obj1 = GeoEntity(GeoEntity.sphere);
disp(obj1.type{2})



% disp(GeoEntity.getAll())