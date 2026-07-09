classdef EarClip < handle
    methods (Static)
        % Earcut animation function
        function tris = clip(poly)
            % Check if the polygon is valid
            if ~EarClip.polyDirection3D(poly)
                poly = poly(:,end:-1:1);
                disp('Reverse the polygon order to anti-clockwise order');
            end

            [poly2D, frame] = EarClip.toLocal2D(poly);
            % EarClip.fill2D(poly2D, [6, 0], [0.5 0 0]);

            % Earcut
            tris2D = EarClip.earClip2D(poly2D);

            % Project to 3D
            tris = EarClip.toWorld3D(tris2D, frame);
        end
    end

    methods (Static, Access = private)

        % function fill2D(pts, offset, color)
        %     X = pts(1, :);
        %     Y = pts(2, :);
        %     X = X + offset(1);
        %     Y = Y + offset(2);
        %     fill(X, Y, color, 'FaceAlpha', 0.9);
        % end

        function tris = toWorld3D(tris2D, frame)
            T = size(tris2D, 3);
            tris = zeros(3, 3, T);

            for i = 1:T
                for j = 1:3
                    p2 = tris2D(:, j, i);      % 2×1 [x; y]
                    x = p2(1);
                    y = p2(2);

                    p3 = frame.origin + x * frame.u_axis + y * frame.v_axis;
                    tris(:, j, i) = p3;
                end
            end
        end

        function [poly2D, frame] = toLocal2D(poly3D)
            n = size(poly3D, 2);
            origin = poly3D(:, 1);

            % Calculate normal vector
            normal = [0; 0; 0];
            for i = 1:n
                v1 = poly3D(:, i);
                v2 = poly3D(:, mod(i, n) + 1);
                v3 = poly3D(:, mod(i+1, n) + 1);
                e1 = v2 - v1;
                e2 = v3 - v2;
                normal = normal + cross(e1, e2);
            end
            normal = normal / norm(normal);

            % Calculate u_axis (longest edge direction)
            max_len = 0;
            u_axis = [1; 0; 0];
            for i = 1:n
                j = mod(i, n) + 1;
                edge = poly3D(:, j) - poly3D(:, i);
                len = norm(edge);
                if len > max_len
                    max_len = len;
                    u_axis = edge / len;
                end
            end

            % Calculate v_axis (orthogonal direction)
            v_axis = cross(normal, u_axis);
            v_axis = v_axis / norm(v_axis);
            u_axis = cross(v_axis, normal);
            u_axis = u_axis / norm(u_axis);

            % Project to 2D
            poly2D = zeros(2, n);
            for i = 1:n
                diff = poly3D(:, i) - origin;
                poly2D(1, i) = dot(diff, u_axis);
                poly2D(2, i) = dot(diff, v_axis);
            end

            % Pack the frame
            frame.u_axis = u_axis;
            frame.v_axis = v_axis;
            frame.normal = normal;
            frame.origin = origin;
        end

        function tris2D = earClip2D(poly2D)
            % poly2D: 2×N local 2D vertices (assumed counter-clockwise, no-intersecting)
            % tris2D: 2×3×T output triangles (2D vertices)

            % h_pts = [];  % Point display handles, for testing

            n = size(poly2D, 2);

            if n == 3
                tris2D = reshape(poly2D, 2, 3, 1);
                return;
            end

            % Active vertex list (circular linked list)
            vertices = 1:n;
            numV = n;

            % Preallocate triangle storage
            tris_list = cell(n - 2, 1);
            triCount = 0;

            V = poly2D;  % Local reference, to avoid repeated indexing

            while numV > 3
                earFound = false;

                % Traverse all active vertices
                for i = 1:numV
                    % Circular neighbor indices
                    prev_idx = mod(i - 2 + numV, numV) + 1;
                    next_idx = mod(i, numV) + 1;

                    v_prev = vertices(prev_idx);
                    v_curr = vertices(i);
                    v_next = vertices(next_idx);
                    
                    % % Update point display
                    % if ~isempty(h_pts)
                    %     delete(h_pts);
                    % end

                    % h_pts(1) = scatter(V(1, v_prev) + 6, V(2, v_prev), 'o');
                    % h_pts(2) = scatter(V(1, v_curr) + 6, V(2, v_curr), 'o');
                    % h_pts(3) = scatter(V(1, v_next) + 6, V(2, v_next), 'o');

                    % Check if the triangle is an ear triangle
                    if EarClip.isEar2D(V, v_prev, v_curr, v_next, vertices(1:numV))
                        % Store triangle coordinates
                        triCount = triCount + 1;
                        tris_list{triCount} = [V(:, v_prev), V(:, v_curr), V(:, v_next)];

                        % Remove ear vertex
                        vertices(i) = [];
                        numV = numV - 1;
                        earFound = true;
                        break;  % Restart search
                    end
                end

                if ~earFound
                    error('Earcut failed: No ear triangle found, polygon may be self-intersecting or not simple');
                end
            end

            % Last triangle formed by the remaining 3 vertices
            triCount = triCount + 1;
            tris_list{triCount} = [V(:, vertices(1)), V(:, vertices(2)), V(:, vertices(3))];

            % Assemble into 2×3×T array
            tris2D = zeros(2, 3, triCount);
            for i = 1:triCount
                tris2D(:, :, i) = tris_list{i};
            end
        end

        function flag = isEar2D(V, v_prev, v_curr, v_next, activeVerts)
            % V: 2xN, all vertices coordinates (local 2D)
            % v_prev, v_curr, v_next: three vertex indices
            % activeVerts: current active vertex indices list

            p_prev = V(:, v_prev);
            p_curr = V(:, v_curr);
            p_next = V(:, v_next);

            % Convex angle check (2D cross product)
            e1 = p_curr - p_prev;
            e2 = p_next - p_curr;
            cross_z = e1(1) * e2(2) - e1(2) * e2(1);

            if cross_z <= 1e-12
                flag = false;
                return;
            end

            % Triangle internal check
            tri = [p_prev, p_curr, p_next];

            for i = 1:length(activeVerts)
                v = activeVerts(i);
                if v == v_prev || v == v_curr || v == v_next
                    continue;
                end

                p = V(:, v);
                if EarClip.pointInTriangle2D(p, tri)
                    flag = false;
                    return;
                end
            end

            flag = true;
        end

        function flag = pointInTriangle2D(p, tri)
            % p: 2×1 point in [x, y]
            % tri: 2×3 triangle vertices [v1, v2, v3]
            % flag: true = point p in triangle tri

            % get the coordinates of the point and the triangle vertices
            px = p(1);  py = p(2);
            x1 = tri(1, 1);  y1 = tri(2, 1);
            x2 = tri(1, 2);  y2 = tri(2, 2);
            x3 = tri(1, 3);  y3 = tri(2, 3);

            % Calculate the denominator, use 'Cramer's Rule'
            denom = (y2 - y3) * (x1 - x3) + (x3 - x2) * (y1 - y3);

            % Degenerate triangle check
            if abs(denom) < 1e-10
                flag = false;
                return;
            end

            % Calculate the barycentric coordinates
            a = ((y2 - y3) * (px - x3) + (x3 - x2) * (py - y3)) / denom;
            b = ((y3 - y1) * (px - x3) + (x1 - x3) * (py - y3)) / denom;
            c = 1 - a - b;

            % Strict internal check (excluding boundary, to avoid colinear vertex problem)
            eps = -1e-10;
            flag = (a > eps) && (b > eps) && (c > eps);
        end

        function dir = polyDirection(pts)
            n = size(pts, 2);

            area = sum(pts(1, 1:n) .* pts(2, [2:n, 1]) - pts(1, [2:n, 1]) .* pts(2, 1:n));

            dir = sign(area);
        end

        function dir = polyDirection3D(pts)
            % Find the best direction of the polygon
            normal = cross(pts(:, 2) - pts(:, 1), pts(:, 3) - pts(:, 1));
            [~, maxDim] = max(abs(normal));

            % Project on other two dimensions
            dims = setdiff([1, 2, 3], maxDim);
            ptsProj = pts(dims, :);

            dir = EarClip.polyDirection(ptsProj);
        end
    end
end
