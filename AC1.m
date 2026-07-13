classdef AC1 < handle
    properties
        % The default parameters
        n = 32;

        % The position of the AC1 (world coordinates)
        pos = [0; 0; 0]         % 3×1 the position of the AC1 (world coordinates)
        yaw = 0                 % yaw angle (rad, around Z axis)
        pitch = 0               % pitch angle (rad, around Y axis)
        roll = 0                % roll angle (rad, around X axis)

        % The default parameters of the AC1
        fov_h = deg2rad(120)    % horizontal FOV (rad)
        fov_v = deg2rad(90)     % vertical FOV (rad)
        res_h = 192             % horizontal resolution
        res_v = 144             % vertical resolution
        range = [1, 70]

        % AC1 body size [w, h, d]
        body_size = [0.08, 0.095, 0.04];

        % The local directions of the AC1, default to -Z
        dirs_local              % 3×N local directions (default to -Z)
        origins                 % 3×N ray origins (repeat pos)

        % body display handle
        h_surf = [];
        h_fill = [];
        h_patch = [];

        h_range_grid = [];
        range_grid_visible = true;
    end

    methods
        % Constructor
        function obj = AC1(pos, yaw, pitch, roll)
            if nargin > 0, obj.pos = pos(:); end
            if nargin > 1, obj.yaw = yaw; end
            if nargin > 2, obj.pitch = pitch; end
            if nargin > 3, obj.roll = roll; end

            obj.dirs_local = obj.precomputeDirs();
            obj.updateOrigins();
        end

        % Precompute local directions (default to -Z)
        function dirs = precomputeDirs(obj)
            % From Left to Right, from top to bottom
            % The sequence of the matlab is anti-clockwise, so need to reverse the order
            theta_h = linspace(obj.fov_h/2, -obj.fov_h/2, obj.res_h);
            theta_v = linspace(obj.fov_v/2, -obj.fov_v/2, obj.res_v);

            [TH, TV] = ndgrid(theta_h, theta_v);
            % disp(rad2deg(TH));
            % disp(rad2deg(TV));

            % Total number of rays
            N = obj.res_h * obj.res_v;

            phi = TH(:);
            theta = TV(:);

            % Lidar coordinates: X - Forward, Y - Left, Z - Up
            dirs = zeros(3, N);
            dirs(1, :) = cos(phi) .* cos(theta);   % X - Forward
            dirs(2, :) = sin(phi) .* cos(theta);   % Y - Left
            dirs(3, :) = sin(theta);               % Z - Up

            % disp(dirs);
            dirs = dirs ./ sqrt(sum(dirs.^2, 1));
        end

        % Update ray origins to the position of the AC1
        function updateOrigins(obj)
            N = obj.res_h * obj.res_v;
            obj.origins = repmat(obj.pos, 1, N);
        end

        % Get rays in world coordinates system
        function [opts, dirs] = getRays(obj)
            opts = obj.origins;

            % Rotation matrix: yaw(Z) -> pitch(Y) -> roll(X)
            R = obj.eulerRotation(obj.yaw, obj.pitch, obj.roll);

            % Rotate local directions to world coordinates system
            dirs = R * obj.dirs_local;
        end

        function [hitPoints, ranges, triIdxs] = scan(obj, entities)
            [opts, dirs] = obj.getRays();
            N = size(opts, 2);

            ranges = inf(1, N);
            hitPoints = zeros(3, N);
            triIdxs = zeros(1, N);
            hit = false(1, N);

            for e = 1:length(entities)
                entity = entities(e);

                aabbHit = AC1.rayAABBIntersect(opts, dirs, entity.aabb);
                if ~any(aabbHit), continue; end

                active = find(aabbHit);
                [entityHit, t, ~, ~, eTriIdx] = ...
                    AC1.intersectBatch(opts(:, active), dirs(:, active), entity.tris);

                for i = 1:length(active)
                    r = active(i);
                    if entityHit(i) && t(i) < ranges(r)
                        ranges(r) = t(i);
                        hitPoints(:, r) = opts(:, r) + dirs(:, r) * t(i);
                        triIdxs(r) = eTriIdx(i);
                        hit(r) = true;
                    end
                end
            end

            hit = hit & (ranges > obj.range(1)) & (ranges < obj.range(2));

            hitPoints = hitPoints(:, hit);
            ranges = ranges(hit);
            triIdxs = triIdxs(hit);
        end

        function R = eulerRotation(~, yaw, pitch, roll)
            Rz = [cos(yaw) -sin(yaw) 0; sin(yaw) cos(yaw) 0; 0 0 1];
            Rx = [1 0 0; 0 cos(pitch) -sin(pitch); 0 sin(pitch) cos(pitch)];
            Ry = [cos(roll) 0 sin(roll); 0 1 0; -sin(roll) 0 cos(roll)];
            R = Rz * Rx * Ry;
        end

        % Set the yaw, pitch, roll of the AC1
        function setYawPitchRoll(obj, yaw, pitch, roll)
            obj.yaw = yaw;
            obj.pitch = pitch;
            obj.roll = roll;
        end

        % Set the scan range
        function setRange(obj, range)
            obj.range = range;

            % After setting the range, precompute the local directions
            obj.dirs_local = obj.precomputeDirs();
            obj.updateOrigins();
        end

        % Show lidar body
        function renderBody(obj, color)
            if nargin < 2
                color = 'w';
            end
            % draw lazer ball
            b_size = obj.body_size(3) * 0.5;

            [X, Y, Z] = sphere(obj.n);
            X = X * b_size + obj.pos(1);
            Y = Y * b_size + obj.pos(2);
            Z = Z * b_size + obj.pos(3);

            obj.h_surf = surf(X, Y, Z, 'FaceColor', [0.4 0.4 0.4], 'EdgeColor', 'none');

            % draw the lidar body
            verts =[
                % obj.body_size: [length, height, width]
                0, obj.body_size(2) / 2, obj.body_size(1) / 2;
                0, -obj.body_size(2) / 2, obj.body_size(1) / 2;
                0, -obj.body_size(2) / 2, -obj.body_size(1) / 2;
                0, obj.body_size(2) / 2, -obj.body_size(1) / 2;
                -obj.body_size(3), obj.body_size(2) / 2, obj.body_size(1) / 2;
                -obj.body_size(3), -obj.body_size(2) / 2, obj.body_size(1) / 2;
                -obj.body_size(3), -obj.body_size(2) / 2, -obj.body_size(1) / 2;
                -obj.body_size(3), obj.body_size(2) / 2, -obj.body_size(1) / 2;
                ];

            % Rotation matrix: yaw(Z) -> pitch(Y) -> roll(X)
            R = obj.eulerRotation(obj.yaw, obj.pitch, obj.roll);
            verts = R * verts';

            % Translate to the position of the AC1
            % form local coordinates system to world coordinates system
            verts = verts' + obj.pos';

            faces =[
                1, 2, 3, 4;
                5, 6, 7, 8;
                1, 4, 8, 5;
                2, 3, 7, 6;
                1, 2, 6, 5;
                3, 4, 8, 7;
                ];

            obj.h_patch = patch('Vertices', verts, 'Faces', faces, ...
                'FaceColor', color, 'EdgeColor', 'none', 'LineWidth', 1);
        end

        % Show the scan range
        function showScanRange(obj, grid, color)
            if nargin < 3
                color = [0.3 0.8 1];
            end
            % color: Scan range color
            [opts, dirs] = obj.getRays();

            % Total number of rays
            N = obj.res_h * obj.res_v;

            % Corner indices
            idx = [1, ...               % Left top
                obj.res_h, ...          % Right top
                N, ...                  % Right bottom
                N - obj.res_h + 1, ...  % Left bottom
                1];                     % Close to left top

            % Ray endpoints at the scan range
            spts = opts(:, idx) + dirs(:, idx) * obj.range(1);
            epts = opts(:, idx) + dirs(:, idx) * obj.range(2);


            % Draw the scan range, main boundary lines
            hold on;

            for i = 1:4
                hd = plot3([spts(1, i), epts(1, i)], ...
                    [spts(2, i), epts(2, i)], ...
                    [spts(3, i), epts(3, i)], ...
                    'Color', color, 'LineWidth', 1);

                obj.h_range_grid = [obj.h_range_grid; hd];
            end

            % Draw the scan range
            for i = 0:grid
                if i == 0
                    % First column
                    colIdx = 1:obj.res_h:N;
                elseif i == grid
                    % Last column
                    colIdx = obj.res_h:obj.res_h:N;
                else
                    colIdx = (floor(obj.res_h / grid) * i):obj.res_h:N;
                end

                colEnd = opts(:, colIdx) + dirs(:, colIdx) * obj.range(2);
                hd = plot3(colEnd(1,:), colEnd(2,:), colEnd(3,:), 'Color', color, 'LineWidth', 1);

                obj.h_range_grid = [obj.h_range_grid; hd];
            end

            for i = 0:grid
                if i == 0
                    rowIdx = 1:(obj.res_h * (i + 1));
                elseif i == grid
                    rowIdx = N - obj.res_h + 1:N;
                else
                    rowIdx = (floor(i / grid * obj.res_v) - 1) * obj.res_h + 1:floor(i / grid * obj.res_v) * obj.res_h;
                end
                rowEnd = opts(:, rowIdx) + dirs(:, rowIdx) * obj.range(2);
                hd = plot3(rowEnd(1,:), rowEnd(2,:), rowEnd(3,:), 'Color', color, 'LineWidth', 1);

                obj.h_range_grid = [obj.h_range_grid; hd];
            end
        end

        % Toggle the visibility of the scan range grid
        function togView_ScanRangegrid(obj, visiable)
            if nargin < 2
                obj.range_grid_visible = ~obj.range_grid_visible;
            else
                obj.range_grid_visible = visiable;
            end

            if obj.range_grid_visible
                set(obj.h_range_grid, 'Visible', 'on');
            else
                set(obj.h_range_grid, 'Visible', 'off');
            end
        end
    end

    methods (Static, Access = private)
        % AABB intersection
        function hit = rayAABBIntersect(origins, dirs, aabb)
            % N = size(origins, 2);
            minB = aabb(1:3);
            maxB = aabb(4:6);

            inv_dir = 1 ./ dirs;
            inv_dir(abs(dirs) < 1e-15) = inf;

            t1 = (minB - origins) .* inv_dir;
            t2 = (maxB - origins) .* inv_dir;

            tmin = min(t1, t2);
            tmax = max(t1, t2);

            t_enter = max(tmin, [], 1);
            t_exit = min(tmax, [], 1);

            hit = (t_enter < t_exit) & (t_exit > 0);
        end

        % Triangle batch intersection
        function [hit, t, u, v, tri_idx] = intersectBatch(origins, dirs, tris)
            N = size(origins, 2);
            T = size(tris, 3);
            eps = 1e-7;

            hit = false(1, N);
            t = inf(1, N);
            u = zeros(1, N);
            v = zeros(1, N);
            tri_idx = zeros(1, N);

            for i = 1:T
                v0 = tris(:, 1, i);
                v1 = tris(:, 2, i);
                v2 = tris(:, 3, i);

                e1 = v1 - v0;
                e2 = v2 - v0;

                for r = 1:N
                    if t(r) < 0
                        continue;
                    end

                    rd = dirs(:, r);
                    ro = origins(:, r);

                    h = cross(rd, e2);
                    a = dot(e1, h);

                    if abs(a) < eps
                        continue;
                    end

                    f = 1.0 / a;
                    s = ro - v0;
                    u_val = f * dot(s, h);

                    if u_val < 0.0 || u_val > 1.0
                        continue;
                    end

                    q = cross(s, e1);
                    v_val = f * dot(rd, q);

                    if v_val < 0.0 || u_val + v_val > 1.0
                        continue;
                    end

                    t_val = f * dot(e2, q);

                    if t_val > eps && t_val < t(r)
                        t(r) = t_val;
                        hit(r) = true;
                        u(r) = u_val;
                        v(r) = v_val;
                        tri_idx(r) = i;
                    end
                end
            end
        end
    end
end
