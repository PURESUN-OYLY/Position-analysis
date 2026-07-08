
%% Test GeoEntity class

% Clear the workspace
clear;      % Clear the workspace
clc;        % Clear the command window
close all;  % Close all the figure
axis equal; % Set the axis equal, to show the entity in the same size
axis vis3d;

%% Base config
lidarFOV = [120 90];
lidarRange = [0 70];

view(45, 30);

geoEnt = GeoEntity();

obj1 = geoEnt.Create('frustum');
obj1.setPos([0, 0, 0]);
obj1.setRad(3);
obj1.setHeight(10)
obj1.setAlignMode('bottom_cen')
% obj1.setCen([1, 2, 3, 1, 2, 0]);

obj2 = geoEnt.Create('frustum');
obj2.setPos([0, 0, 10]);
obj2.setRad(1.5, 3);
obj2.setHeight(2)
obj2.setAlignMode('bottom_cen')
% obj2.setCen(1, 2, 3);

obj3 = geoEnt.Create('frustum');
obj3.setPos([0, 0, 12]);
obj3.setRad(1.5);
obj3.setHeight(1.5)
obj3.setAlignMode('bottom_cen')
% obj3.setCen(1, 2, 3);


% disp(obj1.pos)
disp(['Total entity: ', num2str(geoEnt.count)])

geoEnt.renderAll();

obj2.toTriangles();

info = rendererinfo(gca);   % 需要 figure 句柄
disp(info.GraphicsRenderer);

% Config the lidar point cloud generator as AC1_lidar
lidar = lidarPointCloudGenerator(...
    'AzimuthResolution', 0.625, ...                         % Horizontal resolution (degree)
    'ElevationResolution', 0.625, ...                       % Vertical resolution (degree)
    'AzimuthLimits', [-lidarFOV(1)/2 lidarFOV(1)/2], ...    % Horizontal field of view (degree)
    'ElevationLimits', [-lidarFOV(2)/2 lidarFOV(2)/2], ...  % Vertical field of view (degree)
    'MaxRange', lidarRange(2), ...                          % Maximum range (meter)
    'HasNoise', false, ...                                  % Enable noise (more realistic)
    'HasOrganizedOutput', true, ...                         % Organized output (similar to image grid)
    'HasEgoVehicle', false);                                % No ego vehicle point cloud
