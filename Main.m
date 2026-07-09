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

n = 10;

view(45, 30);

geoEnt = GeoEntity();

obj1 = geoEnt.Create('frustum');
obj1.setPos([0, 0, 0]);
obj1.setRad(3);
obj1.setHeight(10)
obj1.setAlignMode('bottom_cen')
obj1.setN(n);
obj1.setRenderElement(false, true, true);

% obj1.setCen([1, 2, 3, 1, 2, 0]);

obj2 = geoEnt.Create('frustum');
obj2.setPos([0, 0, 10]);
obj2.setRad(1.5, 3);
obj2.setHeight(2)
obj2.setAlignMode('bottom_cen')
obj2.setN(n);
obj2.setRenderElement(false, false, true);
% obj2.setCen(1, 2, 3);

obj3 = geoEnt.Create('frustum');
obj3.setPos([0, 0, 12]);
obj3.setRad(1.5);
obj3.setHeight(1.5)
obj3.setAlignMode('bottom_cen')
obj3.setN(n);
obj3.setRenderElement(true, false, true);
% obj3.setCen(1, 2, 3);


% disp(obj1.pos)
disp(['Total entity: ', num2str(geoEnt.count)])

geoEnt.renderAll();

% obj2.toTriangles();

% % Get the renderer info
% info = rendererinfo(gca);
% disp(info.GraphicsRenderer);

% pts = [0, 0, 0, 1, 0, 1, 2, 3, 4, 5, 4, 5, 5, 5, 4, 3, 2, 2, 1;
%        1, 2, 3, 4, 5, 6, 7, 7, 6, 5, 4, 3, 2, 1, 0, 0, 0, 1, 1;
%        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
% pts = obj1.pts;
pts = obj1.pts;
pts = pts + [10; 0; 0];
% disp(pts)
X = pts(1, :);
Y = pts(2, :);
Z = pts(3, :);

hold on;
% fill3(X(1:obj1.n-1), Y(1:obj1.n-1), Z(1:obj1.n-1), [0 0.5 0], 'FaceAlpha', 0.9);

% fill3(X(obj1.n + 1:2*obj1.n-1), Y(obj1.n + 1:2*obj1.n-1), Z(obj1.n + 1:2*obj1.n-1), [0 0.5 0], 'FaceAlpha', 0.9);

% Earcut animation for top
tris = EarClip.clip(pts(:, 1:obj1.n - 1));

% Earcut animation for bottom
tris_b = EarClip.clip(pts(:, obj1.n + 1:2 * obj1.n - 1));

for i = 1:size(tris, 3)
    X = tris(1, :, i);
    Y = tris(2, :, i);
    Z = tris(3, :, i);

    fill3(X, Y, Z, [mod(i, 20) * 0.05, 0.5, 1 - mod(i, 20) * 0.05], 'FaceAlpha', 0.9);
end

for i = 1:size(tris_b, 3)
    X = tris_b(1, :, i);
    Y = tris_b(2, :, i);
    Z = tris_b(3, :, i);

    fill3(X, Y, Z, [mod(i, 20) * 0.05, 0.5, 1 - mod(i, 20) * 0.05], 'FaceAlpha', 0.9);
end

% X = pts(1, :);
% Y = pts(2, :);
% Z = pts(3, :);

% hold on;
% fill3(X, Y, Z, [0 0.5 0], 'FaceAlpha', 0.9);

% % Earcut animation animation
% tris = EarClip.clip(pts);

% for i = 1:size(tris, 3)
%     X = tris(1, :, i);
%     Y = tris(2, :, i);
%     Z = tris(3, :, i);

%     fill3(X, Y, Z, [mod(i, 10) * 0.1, 0.5, 0], 'FaceAlpha', 0.9);
% end