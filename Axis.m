% Clear the workspace
clear;      % Clear the workspace
clc;        % Clear the command window
close all;  % Close all the figure
axis equal; % Set the axis equal, to show the entity in the same size
axis vis3d;
axis on;
xlabel('X'); ylabel('Y'); zlabel('Z');
camproj('orthographic');
view(45, 30);   % 方位角45°，仰角30°

n = 32;
ax_size = 10;


% Build sphere
[X, Y, Z] = sphere(n);
Xo = X * 0.25;
Yo = Y * 0.25;
Zo = Z * 0.25;

% Build X axis
[Xz, Yz, Zz] = cylinder(0.15, n);
Zz = Zz * ax_size;

[Xx, Yx, Zx] = rotateMesh(Xz, Yz, Zz, [0, 0, 0], 'y', pi / 2);
[Xy, Yy, Zy] = rotateMesh(Xz, Yz, Zz, [0, 0, 0], 'x', - pi / 2);

[Xzh, Yzh, Zzh] = cylinder([0.25, 0], n);
Zzh = Zzh * 1;
Zzh = Zzh + ax_size;

[Xxh, Yxh, Zxh] = rotateMesh(Xzh, Yzh, Zzh, [0, 0, 0], 'y', pi / 2);
[Xyh, Yyh, Zyh] = rotateMesh(Xzh, Yzh, Zzh, [0, 0, 0], 'x', - pi / 2);

hold on;
surf(Xo, Yo, Zo, 'FaceColor', 'w', 'EdgeColor', 'none', 'FaceAlpha', 0.9);
surf(Xz, Yz, Zz, 'FaceColor', [0.3, 0.4, 1], 'EdgeColor', 'none', 'FaceAlpha', 0.9);
surf(Xx, Yx, Zx, 'FaceColor', [1, 0.3, 0.3], 'EdgeColor', 'none', 'FaceAlpha', 0.9);
surf(Xy, Yy, Zy, 'FaceColor', [0.2, 1, 0.3], 'EdgeColor', 'none', 'FaceAlpha', 0.9);
surf(Xxh, Yxh, Zxh, 'FaceColor', [1, 0.3, 0.3], 'EdgeColor', 'none', 'FaceAlpha', 0.9);
surf(Xyh, Yyh, Zyh, 'FaceColor', [0.3, 1, 0.3], 'EdgeColor', 'none', 'FaceAlpha', 0.9);
surf(Xzh, Yzh, Zzh, 'FaceColor', [0.3, 0.4, 1], 'EdgeColor', 'none', 'FaceAlpha', 0.9);
hold off;

lighting gouraud; camlight; axis equal;


function [Xr, Yr, Zr] = rotateMesh(X, Y, Z, center, axis, angle)
% rotateMesh: rotate mesh around a center
% center: [cx, cy, cz] the center of rotation
% axis: 'x', 'y', 'z' or 3×1 unit vector
% angle: radians

P = [X(:), Y(:), Z(:)];
P0 = P - center;

if ischar(axis) || isstring(axis)
    switch axis
        case 'x', R = [1 0 0; 0 cos(angle) -sin(angle); 0 sin(angle) cos(angle)];
        case 'y', R = [cos(angle) 0 sin(angle); 0 1 0; -sin(angle) 0 cos(angle)];
        case 'z', R = [cos(angle) -sin(angle) 0; sin(angle) cos(angle) 0; 0 0 1];
    end
else
    k = axis / norm(axis);
    K = [0 -k(3) k(2); k(3) 0 -k(1); -k(2) k(1) 0];
    R = eye(3) + sin(angle)*K + (1-cos(angle))*(K*K);
end

Pr = (R * P0')' + center;

Xr = reshape(Pr(:,1), size(X));
Yr = reshape(Pr(:,2), size(Y));
Zr = reshape(Pr(:,3), size(Z));
end