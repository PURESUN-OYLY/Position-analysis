classdef Entity < handle
    properties
        % Default type is undefined, which means the entity is not initialized
        id = 0;
        type (1,:) = 'undefined';
        n = 32;

        % r: radius, h:height, t:top, b:bottom
        % x:axis-x, y:axis-y, z:axis-z
        pos(1, 3) double = [0, 0, 0];               % The center of mass of entity
        cen(1, 6) double = [0, 0, 0, 0, 0, 0];      % Top and bottom face center of entity
        rad(1, 4) double = [0, 0, 0, 0];            % Top and bottom radius of entity
        rot(1, 3) double = [0, 0, 0];               % Posture of entity[x, y, z]
        rottb(1, 6) double = [0, 0, 0, 0, 0, 0];    % Top and bottom face tilt of entity
        size(1, 3) double = [0, 0, 0];              % Size of entity[x, y, z]
        color(1, 3) double = [0.4, 0.6, 0.8];       % The color of entity
        alignMode = 'midaxis';

        % render element
        render_top = true;
        render_bottom = true;
        render_side = true;

        %% Surface handle
        face_h = [];    % Surface handle of entity
        pts = [];       % Lidar point cloud of top and bottom
        aabb = [];      % Axis-aligned bounding box of entity
        tris = [];      % Triangle indices of entity
    end

    methods
        % Constructor, initialize the entity with default values
        function obj = Entity(type, id)
            if ~Entity.supportChk(type)
                error('Type %s is not supported', type);
            else
                obj.id = id;
                obj.type = type;
            end
        end

        % Set the number of points of the entity
        function setN(obj, n)
            obj.n = n;
        end

        % Set render element of the entity
        % render_top: boolean, true = render top face, false = not render top face
        % render_bottom: boolean, true = render bottom face, false = not render bottom face
        % render_side: boolean, true = render side face, false = not render side face
        function setRenderElement(obj, render_top, render_bottom, render_side)
            obj.render_top = render_top;
            obj.render_bottom = render_bottom;
            obj.render_side = render_side;
        end

        function setPos(obj, pos)
            obj.pos = pos;
        end

        % Set the top and bottom face center of the entity
        % rad: [4, 1) double
        % rad(rta, rtb, rba, rbt)
        function setRad(obj, varargin)
            % frustum use 4 radius, top and bottom radius
            if obj.typeChk('frustum')
                % disp(varargin);
                if nargin == 2
                    obj.rad = [varargin{1}, varargin{1}, varargin{1}, varargin{1}];
                elseif nargin == 3
                    obj.rad = [varargin{1}, varargin{1}, varargin{2}, varargin{2}];
                elseif nargin == 5
                    obj.rad = [varargin{:}];
                end
                % disp(obj.rad);
            end
        end

        % Set height of the entity
        % h: double
        function setHeight(obj, h)
            % frustum use 1 height, it is z-axis size
            if obj.typeChk('frustum')
                obj.size(3) = h;
                % Auto set top and bottom face center
                obj.cen(1:3) = obj.pos;
                obj.cen(4:6) = obj.pos + [0, 0, h];
            end
        end

        % Set the size of the entity
        % size: [3, 1) double
        function setSize(obj, varargin)
            if obj.typeChk('frustum')
                if nargin == 3
                    obj.size = [varargin{1}, varargin{1}, varargin{2}];
                elseif nargin == 4
                    obj.size = [varargin{1}, varargin{2}, varargin{3}];
                end
            end
        end

        % Set align mode of the entity
        % alignMode: string, default is 'midaxis', 'centroid', 'bottom_cen', 'top_cen'
        function setAlignMode(obj, alignMode)
            if strcmpi(alignMode, 'midaxis') || strcmpi(alignMode, 'centroid') || strcmpi(alignMode, 'bottom_cen') || strcmpi(alignMode, 'top_cen')
                obj.alignMode = alignMode;
            else
                error('Align mode %s is not supported', alignMode);
            end
        end

        % Set the top and bottom face center of the entity
        % cen: [6, 1) double
        function setCen(obj, varargin)
            if obj.typeChk('frustum')
                if nargin == 4
                    obj.cen = [varargin{1}, varargin{2}, varargin{3}, varargin{1}, varargin{2}, varargin{3} + obj.size(3)];
                elseif nargin == 7
                    obj.cen = [varargin{1}, varargin{2}, varargin{3}, varargin{4}, varargin{5}, varargin{6}];
                    % Adjust the height of the frustum, to make the top and bottom face center aligned
                    obj.size(3) = varargin{7} - varargin{3};
                end
            end
        end

        % Set the top and bottom face tilt of the entity
        % rottb: [6, 1) double, unit is rad
        function setRotTb(obj, varargin)
            if obj.typeChk('frustum')
                if nargin == 7
                    obj.rottb = [varargin{:}];
                else
                    error('Number of input arguments must be 6');
                end
            end
        end

        % Check the type of the entity
        function res = typeChk(obj, type)
            res = strcmpi(obj.type, type);
            if ~res
                error('Type of the entity is not %s', type);
            end
        end

        % Get the size of the entity
        % res: [3, 1) double
        function res = getSize(obj)
            res = obj.size;
        end

        % Show the size of the entity
        function res = showSize(obj)
            res = obj.size;
            disp(['Size of the entity ', obj.type, ' is: ', num2str(res)]);
        end

        % Render the entity in the figure
        function render(obj)
            if strcmp(obj.type, 'frustum')
                % Render the frustum in the figure
                obj.drawFrustum();
            end
        end

        function drawFrustum(obj)
            % obj: Frustum entity object

            a_top = obj.rad(1);      % Top face major axis radius
            b_top = obj.rad(2);      % Top face minor axis radius
            a_bottom = obj.rad(3);   % Bottom face major axis radius
            b_bottom = obj.rad(4);   % Bottom face minor axis radius

            tilt_bottom = obj.rottb(1:3);  % Bottom face local tilt [theta, phi, psi]
            tilt_top = obj.rottb(4:6);     % Top face local tilt [theta, phi, psi]

            % Calculate bottom and top face center position
            % Get the center position from cen(1:3) and cen(4:6) respectively
            pos_bottom = obj.cen(1:3);
            pos_top = obj.cen(4:6);

            % Generate ellipse cross-section
            theta = linspace(0, 2*pi, obj.n);
            % theta = linspace(0, 2*pi, obj.n + 1);
            % remove the last point, to avoid duplicate points
            % theta(end) = [];
            % disp(size(theta))

            % Bottom face ellipse (local coordinates)
            x_b = a_bottom * cos(theta);
            y_b = b_bottom * sin(theta);
            z_b = zeros(1, obj.n);

            % Top face ellipse (local coordinates)
            x_t = a_top * cos(theta);
            y_t = b_top * sin(theta);
            z_t = zeros(1, obj.n);

            % Local rotation
            R_b = obj.eulerRotation(tilt_bottom);
            R_t = obj.eulerRotation(tilt_top);

            pts_b = R_b * [x_b; y_b; z_b] + pos_bottom(:);
            pts_t = R_t * [x_t; y_t; z_t] + pos_top(:);

            % Align the entity to the position of the center of the frustum
            switch obj.alignMode
                case 'centroid'
                    allPts = [pts_b, pts_t];
                    center = mean(allPts, 2)';
                case 'midaxis'
                    center = (pos_bottom + pos_top) / 2;
                case 'bottom_cen'
                    center = pos_bottom;
                case 'top_cen'
                    center = pos_top;
                otherwise
                    center = (pos_bottom + pos_top) / 2;  % default midaxis
            end

            offset = obj.pos(:) - center(:);
            pts_b = pts_b + offset;
            pts_t = pts_t + offset;

            % Global rotation
            R_global = obj.eulerRotation(obj.rot);

            pts_b_shifted = pts_b - obj.pos(:);
            pts_t_shifted = pts_t - obj.pos(:);

            pts_b = R_global * pts_b_shifted + obj.pos(:);
            pts_t = R_global * pts_t_shifted + obj.pos(:);

            % save the point cloud of top and bottom face
            % the point cloud is in local coordinates and has been aligned to the position of the entity
            obj.pts = [pts_t, pts_b];
            obj.aabb = [min(obj.pts, [], 2); max(obj.pts, [], 2)];

            % disp(['Total point cloud size: ', num2str(size(obj.pts, 2))])

            % Bottom face
            X_bottom = pts_b(1, :);
            Y_bottom = pts_b(2, :);
            Z_bottom = pts_b(3, :);

            % Top face
            X_top = pts_t(1, :);
            Y_top = pts_t(2, :);
            Z_top = pts_t(3, :);

            % Side face
            X = [X_bottom; X_top];
            Y = [Y_bottom; Y_top];
            Z = [Z_bottom; Z_top];

            % Render on canvas
            hold on;

            if obj.render_side
                obj.face_h =[obj.face_h; surf(X, Y, Z, 'FaceColor', obj.color, 'EdgeColor', 'none', 'FaceAlpha', 0.8)];
            end

            if obj.render_top
                obj.face_h =[obj.face_h; fill3(X_top, Y_top, Z_top, obj.color, 'FaceAlpha', 0.9)];
            end

            if obj.render_bottom
                obj.face_h =[obj.face_h; fill3(X_bottom, Y_bottom, Z_bottom, obj.color, 'FaceAlpha', 0.9)];
            end

            % hold off;

        end

        function hp = drawAABB(obj)
            hp = Pub.drawAABB(obj.aabb);
        end

        % Euler rotation matrix
        function R = eulerRotation(~, angles)
            % angles = [theta, phi, psi] around Z, Y, X axis
            theta = angles(1);  % Around Z axis
            phi = angles(2);    % Around Y axis
            psi = angles(3);    % Around X axis

            Rz = [cos(theta)  -sin(theta)  0;
                sin(theta)   cos(theta)  0;
                0            0           1];

            Ry = [cos(phi)  0  sin(phi);
                0         1  0;
                -sin(phi)  0  cos(phi)];

            Rx = [1  0         0;
                0  cos(psi) -sin(psi);
                0  sin(psi)  cos(psi)];

            R = Rz * Ry * Rx;  % Order: Rx * Ry * Rz
        end

        function toTriangles(obj)
            % Convert the entity to triangles
            if strcmp(obj.type, 'frustum')
                % Initialize the triangles matrix
                obj.tris = [];

                % Earcut animation for top
                if obj.render_top
                    obj.tris = cat(3, obj.tris, EarClip.clip(obj.pts(:, 1:obj.n - 1)));
                end

                % Earcut animation for bottom
                if obj.render_bottom
                    obj.tris = cat(3, obj.tris, EarClip.clip(obj.pts(:, obj.n + 1:2 * obj.n - 1)));
                end

                % Earcut animation for side
                if obj.render_side
                    tris_s = [];

                    pts_t = obj.pts(:, 1:obj.n);
                    pts_b = obj.pts(:, obj.n + 1:2 * obj.n);

                    for i = 1:obj.n - 1
                        tris_s = cat(3, tris_s, [pts_t(:, i), pts_b(:, i), pts_b(:, i+1)]);
                        tris_s = cat(3, tris_s, [pts_t(:, i), pts_t(:, i+1), pts_b(:, i+1)]);
                    end
                    obj.tris = cat(3, obj.tris, tris_s);
                end
                % Clip triangles for frustum face end
            end
        end
    end

    methods (Static)
        % Check the type of the entity
        function res = typeList()
            types ={'frustum';
                'sphere';
                'cube';
                'cycle';
                'triangle';
                'tetragonum'};

            res = types;
        end

        % Check support of the type
        function res = supportChk(type)
            types = Entity.typeList();
            if any(strcmpi(types, type))
                res = true;
            else
                res = false;
            end
        end

    end
end