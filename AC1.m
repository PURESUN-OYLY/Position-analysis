classdef AC1 < handle
    properties
        pos = [0; 0; 0]         % 3×1 the position of the AC1 (world coordinates)
        yaw = 0                 % yaw angle (rad, around Z axis)
        pitch = 0               % pitch angle (rad, around Y axis)
        roll = 0                % roll angle (rad, around X axis)

        % The default parameters of the AC1
        fov_h = deg2rad(120)    % horizontal FOV (rad)
        fov_v = deg2rad(90)     % vertical FOV (rad)
        res_h = 384             % horizontal resolution
        res_v = 288             % vertical resolution
        range = [1, 70]

        % The local directions of the AC1, default to -Z
        dirs_local              % 3×N local directions (default to -Z)
        origins                 % 3×N ray origins (repeat pos)

        % display handle
        h_surf = [];
        h_fill = [];
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
            theta_h = linspace(-obj.fov_h/2, obj.fov_h/2, obj.res_h);
            theta_v = linspace(-obj.fov_v/2, obj.fov_v/2, obj.res_v);

            [TH, TV] = meshgrid(theta_h, theta_v);

            % Total number of rays
            N = obj.res_h * obj.res_v;

            phi = TH(:);
            theta = TV(:);

            % Lidar coordinates: X - Forward, Y - Left, Z - Up
            dirs = zeros(3, N);
            dirs(1, :) = cos(phi) .* cos(theta);   % X - Forward
            dirs(2, :) = sin(phi) .* cos(theta);   % Y - Left
            dirs(3, :) = sin(theta);               % Z - Up

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

        function showScanRange(obj, color)
            % color: Scan range color

            if nargin >= 2
                disp(['User specified color: ', num2str(color)])
            else
                color = [0.3 0.8 1];
            end

            [opts, dirs] = obj.getRays();

            % Total number of rays
            N = obj.res_h * obj.res_v;

            % Corner indices
            idx = [1, ...               % Left top
                obj.res_v, ...          % Right top
                N, ...                  % Right bottom
                N - obj.res_v + 1, ...  % Left bottom
                1];                     % Close to left top

            disp(idx)

            % Ray endpoints at the scan range
            spts = opts(:, idx) + dirs(:, idx) * obj.range(1);
            epts = opts(:, idx) + dirs(:, idx) * obj.range(2);

            % Draw the scan range, main boundary lines
            hold on;
            
            for i = 1:4
                plot3([spts(1, i), epts(1, i)], ...
                    [spts(2, i), epts(2, i)], ...
                    [spts(3, i), epts(3, i)], ...
                    'Color', [1 - (i - 1) * 0.25, (i - 1) * 0.25, 0], 'LineWidth', 1);
            end

            % 画远边界四条弧线（连接相邻角点）
            % 上边界（第一行）
            % topIdx = 1:obj.res_h;
            % topEnd = origins(:, topIdx) + dirs(:, topIdx) * obj.range(2);
            % plot3(topEnd(1,:), topEnd(2,:), topEnd(3,:), 'Color', color, 'LineWidth', 1);

            % % 下边界（最后一行）
            % botIdx = N-obj.res_h+1:N;
            % botEnd = origins(:, botIdx) + dirs(:, botIdx) * obj.range(2);
            % plot3(botEnd(1,:), botEnd(2,:), botEnd(3,:), 'Color', color, 'LineWidth', 1);

            % % 左边界（第一列）
            % leftIdx = 1:obj.res_h:N;
            % leftEnd = origins(:, leftIdx) + dirs(:, leftIdx) * obj.range(2);
            % plot3(leftEnd(1,:), leftEnd(2,:), leftEnd(3,:), 'Color', color, 'LineWidth', 1);

            % % 右边界（最后一列）
            % rightIdx = obj.res_h:obj.res_h:N;
            % rightEnd = origins(:, rightIdx) + dirs(:, rightIdx) * obj.range(2);
            % plot3(rightEnd(1,:), rightEnd(2,:), rightEnd(3,:), 'Color', color, 'LineWidth', 1);
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
