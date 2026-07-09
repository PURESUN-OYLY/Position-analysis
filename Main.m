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

pts = [0, 0, 0, 1, 0, 1, 2, 3, 4, 5, 4, 5, 5, 5, 4, 3, 2, 2, 1;
       1, 2, 3, 4, 5, 6, 7, 7, 6, 5, 4, 3, 2, 1, 0, 0, 0, 1, 1;
       0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];

X = pts(1, :);
Y = pts(2, :);
Z = pts(3, :);

hold on;
fill3(X, Y, Z, [0 0.5 0], 'FaceAlpha', 0.9);

% Earcut animation animation
tris = EarClip.clip(pts);

for i = 1:size(tris, 3)
    X = tris(1, :, i);
    Y = tris(2, :, i);
    Z = tris(3, :, i);

    fill3(X, Y, Z, [mod(i, 10) * 0.1, 0.5, 0], 'FaceAlpha', 0.9);
end
