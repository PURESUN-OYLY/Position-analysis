classdef Axis < handle
    properties
        n = 32;
        ax_size = 1;
        origin_pt = [0, 0, 0];
        rotate_ang = [0, 0, 0];

        visiable = true;
        rendered = false;

        h_surf = [];
        h_fill = [];
    end

    methods
        function obj = Axis(origin_pt, rotate_ang, ax_size)
            if nargin >= 1
                obj.origin_pt = origin_pt;
            end
            if nargin >= 2
                obj.rotate_ang = rotate_ang;
            end
            if nargin >= 3
                obj.ax_size = ax_size;
            end
        end

        function render(obj)
            % Build sphere
            [X, Y, Z] = sphere(obj.n);
            Xo = X * 0.25 * obj.ax_size / 10 + obj.origin_pt(1);
            Yo = Y * 0.25 * obj.ax_size / 10 + obj.origin_pt(2);
            Zo = Z * 0.25 * obj.ax_size / 10 + obj.origin_pt(3);

            % Build axis
            [Xz, Yz, Zz] = cylinder(0.15 * obj.ax_size / 10, obj.n);
            Zz = Zz * obj.ax_size;
            Xz = Xz + obj.origin_pt(1);
            Yz = Yz + obj.origin_pt(2);
            Zz = Zz + obj.origin_pt(3);
            [Xx, Yx, Zx] = Axis.rotateMesh(Xz, Yz, Zz, obj.origin_pt, 'y', pi / 2);
            [Xy, Yy, Zy] = Axis.rotateMesh(Xz, Yz, Zz, obj.origin_pt, 'x', - pi / 2);

            % Build axis head
            [Xzh, Yzh, Zzh] = cylinder([0.25 * obj.ax_size / 10, 0], obj.n);
            Zzh = Zzh * obj.ax_size / 10;
            Zzh = Zzh + obj.ax_size;
            Xzh = Xzh + obj.origin_pt(1);
            Yzh = Yzh + obj.origin_pt(2);
            Zzh = Zzh + obj.origin_pt(3);
            [Xxh, Yxh, Zxh] = Axis.rotateMesh(Xzh, Yzh, Zzh, obj.origin_pt, 'y', pi / 2);
            [Xyh, Yyh, Zyh] = Axis.rotateMesh(Xzh, Yzh, Zzh, obj.origin_pt, 'x', - pi / 2);


            % Rotate axis's elements
            % disp(obj.rotate_ang)
            [Xx, Yx, Zx] = Axis.rotateMesh(Xx, Yx, Zx, obj.origin_pt, 'x', obj.rotate_ang(1));
            [Xy, Yy, Zy] = Axis.rotateMesh(Xy, Yy, Zy, obj.origin_pt, 'y', obj.rotate_ang(2));
            % disp(obj.origin_pt)
            % disp(Xz)
            [Xz, Yz, Zz] = Axis.rotateMesh(Xz, Yz, Zz, obj.origin_pt, 'z', obj.rotate_ang(3));
            % disp(Xz)
            [Xxh, Yxh, Zxh] = Axis.rotateMesh(Xxh, Yxh, Zxh, obj.origin_pt, 'x', obj.rotate_ang(1));
            [Xyh, Yyh, Zyh] = Axis.rotateMesh(Xyh, Yyh, Zyh, obj.origin_pt, 'y', obj.rotate_ang(2));
            [Xzh, Yzh, Zzh] = Axis.rotateMesh(Xzh, Yzh, Zzh, obj.origin_pt, 'z', obj.rotate_ang(3));


            % Render parameters, color and alpha
            clr_x = [1, 0.3, 0.3];
            clr_y = [0.2, 1, 0.3];
            clr_z = [0.3, 0.4, 1];
            axis_alpha = 0.9;

            hold on;

            % surf origin point
            obj.h_surf = [obj.h_surf; surf(Xo, Yo, Zo, 'FaceColor', 'w', 'EdgeColor', 'none', 'FaceAlpha', axis_alpha)];
            % surf body of axises
            obj.h_surf = [obj.h_surf; surf(Xx, Yx, Zx, 'FaceColor', clr_x, 'EdgeColor', 'none', 'FaceAlpha', axis_alpha)];
            obj.h_surf = [obj.h_surf; surf(Xy, Yy, Zy, 'FaceColor', clr_y, 'EdgeColor', 'none', 'FaceAlpha', axis_alpha)];
            obj.h_surf = [obj.h_surf; surf(Xz, Yz, Zz, 'FaceColor', clr_z, 'EdgeColor', 'none', 'FaceAlpha', axis_alpha)];
            % surf head of axises
            obj.h_surf = [obj.h_surf; surf(Xxh, Yxh, Zxh, 'FaceColor', clr_x, 'EdgeColor', 'none', 'FaceAlpha', axis_alpha)];
            obj.h_fill = [obj.h_fill; fill3(Xxh(1, :), Yxh(1, :), Zxh(1, :), clr_x, 'EdgeColor', 'none', 'FaceAlpha', axis_alpha)];
            obj.h_surf = [obj.h_surf; surf(Xyh, Yyh, Zyh, 'FaceColor', clr_y, 'EdgeColor', 'none', 'FaceAlpha', axis_alpha)];
            obj.h_fill = [obj.h_fill; fill3(Xyh(1, :), Yyh(1, :), Zyh(1, :), clr_y, 'EdgeColor', 'none', 'FaceAlpha', axis_alpha)];
            obj.h_surf = [obj.h_surf; surf(Xzh, Yzh, Zzh, 'FaceColor', clr_z, 'EdgeColor', 'none', 'FaceAlpha', axis_alpha)];
            obj.h_fill = [obj.h_fill; fill3(Xzh(1, :), Yzh(1, :), Zzh(1, :), clr_z, 'EdgeColor', 'none', 'FaceAlpha', axis_alpha)];

            obj.rendered = true;
        end

        % Toggle the visibility of the axis
        function togView(obj, visiable)
            if nargin < 2
                obj.visiable = ~obj.visiable;
            else
                obj.visiable = visiable;
            end

            if obj.rendered
                if obj.visiable
                    set(obj.h_surf, 'Visible', 'on');
                    set(obj.h_fill, 'Visible', 'on');
                else
                    set(obj.h_surf, 'Visible', 'off');
                    set(obj.h_fill, 'Visible', 'off');
                end
            else
                obj.render();
            end
        end

    end
    methods (Static)

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
    end
end
