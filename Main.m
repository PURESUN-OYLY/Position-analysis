% Clear the workspace
clear;      % Clear the workspace
clc;        % Clear the command window
close all;  % Close all the figure
axis equal; % Set the axis equal, to show the entity in the same size
axis vis3d;

%% Default unit: meter

%% Base config
lidarFOV = [120, 90];
lidarRange = [0.1, 1];
lidarpos = [-1, 0, 0.6];
lidarYPR = [45, 30, 0];
lidarYPR = deg2rad(lidarYPR);

n = 32;

view(45, 30);

geoEnt = GeoEntity();

obj1 = geoEnt.Create('frustum');
obj1.setPos([0, 0, 0]);
obj1.setRad(0.3);
obj1.setHeight(1)
obj1.setAlignMode('bottom_cen')
obj1.setN(n);
obj1.setRenderElement(false, true, true);

% obj1.setCen([1, 2, 3, 1, 2, 0]);

obj2 = geoEnt.Create('frustum');
obj2.setPos([0, 0, 1]);
obj2.setRad(0.15, 0.3);
obj2.setHeight(0.2)
obj2.setAlignMode('bottom_cen')
obj2.setN(n);
obj2.setRenderElement(false, false, true);

% obj2.setCen(1, 2, 3);

obj3 = geoEnt.Create('frustum');
obj3.setPos([0, 0, 1.2]);
obj3.setRad(0.15);
obj3.setHeight(0.15)
obj3.setAlignMode('bottom_cen')
obj3.setN(n);
obj3.setRenderElement(true, false, true);
% obj3.setCen(1, 2, 3);


% disp(obj1.pos)
disp(['Total entity: ', num2str(geoEnt.count)])

geoEnt.renderAll();

% Clip triangles for frustum
obj1.toTriangles();
obj2.toTriangles();
obj3.toTriangles();

% Concatenate all triangles for frustum
tris_all = cat(3, obj1.tris, obj2.tris, obj3.tris);

% % Render all triangles for frustum
% for i = 1:size(tris_all, 3)
%     X = tris_all(1, :, i);
%     Y = tris_all(2, :, i);
%     Z = tris_all(3, :, i);

%     fill3(X, Y, Z, [mod(i, 20) * 0.05, 0.5, 1 - mod(i, 20) * 0.05], 'FaceAlpha', 0.9);
% end

% obj3.drawAABB();



lidar = AC1(lidarpos);

lidar.setRange(lidarRange);
lidar.setYawPitchRoll(lidarYPR);
lidar.showScanRange(6, [0.3 0.8 1]);
lidar.togView_ScanRangegrid(true);
lidar.renderBody();
lidar.showAxis();

[points, ranges, triIdxs] = lidar.scan(geoEnt.entityList);
scatter3(points(1,:), points(2,:), points(3,:), 5, 'red', 'filled');


% disp(length(points))
