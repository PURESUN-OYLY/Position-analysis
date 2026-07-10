classdef AC1 < handle
    properties
        pos = [0; 0; 0]         % 3×1 the position of the AC1 (world coordinates)
        yaw = 0                 % yaw angle (rad, around Z axis)
        pitch = 0               % pitch angle (rad, around Y axis)
        roll = 0                % roll angle (rad, around X axis)

        % The default parameters of the AC1
        fov_h = deg2rad(120)    % horizontal FOV (rad)
        fov_v = deg2rad(90)     % vertical FOV (rad)
        res_h = 192             % horizontal resolution
        res_v = 144             % vertical resolution

        % The local directions of the AC1, default to -Z
        dirs_local              % 3×N local directions (default to -Z)
        origins                 % 3×N ray origins (repeat pos)
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
            theta_h = linspace(-obj.fov_h/2, obj.fov_h/2, obj.res_h);
            theta_v = linspace(-obj.fov_v/2, obj.fov_v/2, obj.res_v);

            [TH, TV] = meshgrid(theta_h, theta_v);
            N = obj.res_h * obj.res_v;

            phi = TH(:)';
            theta = TV(:)';

            % Lidar coordinates: X - Right, Y - Forward, Z - Up
            dirs = zeros(3, N);
            dirs(1, :) = sin(phi) .* cos(theta);   % X - Right
            dirs(2, :) = cos(phi) .* cos(theta);   % Y - Forward
            dirs(3, :) = sin(theta);               % Z - Up

            dirs = dirs ./ sqrt(sum(dirs.^2, 1));
        end

        % Update ray origins to the position of the AC1
        function updateOrigins(obj)
            N = obj.res_h * obj.res_v;
            obj.origins = repmat(obj.pos, 1, N);
        end

        % Get rays in world coordinates system
        function [origins, dirs] = getRays(obj)
            origins = obj.origins;

            % Rotation matrix: yaw(Z) -> pitch(Y) -> roll(X)
            R = obj.eulerRotation(obj.yaw, obj.pitch, obj.roll);

            % Rotate local directions to world coordinates system
            dirs = R * obj.dirs_local;
        end

        function [hitPoints, ranges, triIdxs] = scan(obj, entities)
            [origins, dirs] = obj.getRays();
            N = size(origins, 2);

            ranges = inf(1, N);
            hitPoints = zeros(3, N);
            triIdxs = zeros(1, N);
            hit = false(1, N);

            for e = 1:length(entities)
                entity = entities(e);

                aabbHit = RayIntersector.rayAABBIntersect(origins, dirs, entity.aabb);
                if ~any(aabbHit), continue; end

                active = find(aabbHit);
                [entityHit, t, ~, ~, eTriIdx] = ...
                    RayIntersector.intersectBatch(origins(:, active), dirs(:, active), entity.tris);

                for i = 1:length(active)
                    r = active(i);
                    if entityHit(i) && t(i) < ranges(r)
                        ranges(r) = t(i);
                        hitPoints(:, r) = origins(:, r) + dirs(:, r) * t(i);
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
    end
end