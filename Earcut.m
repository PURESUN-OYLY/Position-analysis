% Clear the workspace
clear;      % Clear the workspace
clc;        % Clear the command window
close all;  % Close all the figure
axis equal; % Set the axis equal, to show the entity in the same size
axis vis3d;


pts = [0, 0, 0, 1, 0, 1, 2, 3, 4, 5, 4, 5, 5, 5, 4, 3, 2, 2, 1;
       1, 2, 3, 4, 5, 6, 7, 7, 6, 5, 4, 3, 2, 1, 0, 0, 0, 1, 1;
       0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];

X = pts(1, :);
Y = pts(2, :);
Z = pts(3, :);

hold on;
fill3(X, Y, Z, [0 0.5 0], 'FaceAlpha', 0.9);


%% Earcut

% Draw polygon outline
plot([pts(1,:), pts(1,1)], [pts(2,:), pts(2,1)], 'k-', 'LineWidth', 1.5);

% Earcut animation animation
earClip(pts);

% Earcut animation function
function earClip(poly)
    % Initialize handles
    h_verts = [];   % Vertex handles
    h_ear = [];     % Ear highlight handles
    h_tris = [];    % Triangle handles
    
    % Check if the polygon is valid
    if ~polyDirection3D(poly)
        poly = poly(:,end:-1:1);
        disp('Reverse the polygon order to anti-clockwise order');
    end

    n = size(poly, 2);
    vertices = 1:n;
    numV = n;
    V = poly;
    
    while numV > 3
        earFound = false;
        
        drawnow;
        for i = 1:numV
            prev_idx = mod(i - 2 + numV, numV) + 1;
            next_idx = mod(i, numV) + 1;
            
            v_prev = vertices(prev_idx);
            v_curr = vertices(i);
            v_next = vertices(next_idx);
            
            % showPts([v_prev, v_curr, v_next], [0, 0]);
            if isEar2D(V, v_prev, v_curr, v_next, vertices(1:numV))
                
                % Clear old ear highlight handle
                if ~isempty(h_ear) && isvalid(h_ear)
                    delete(h_ear);
                end
                
                % Highlight current ear
                tri_x = [V(1,v_prev), V(1,v_curr), V(1,v_next), V(1,v_prev)];
                tri_y = [V(2,v_prev), V(2,v_curr), V(2,v_next), V(2,v_prev)];
                h_ear = fill(tri_x, tri_y, 'c', 'FaceAlpha', 0.5, 'EdgeColor', 'c');
                
                % Update vertex display
                active_pts = V(:, vertices(1:numV));
                h_verts = updateScatter(h_verts, active_pts(1,:), active_pts(2,:));
                                
                % Store clipped triangles
                h_tris = addTri(h_tris, V(:, [v_prev, v_curr, v_next]));
                
                % Delete ear vertex point
                vertices(i) = [];
                numV = numV - 1;
                earFound = true;
                drawnow;
                break;
            end
        end
        
        if ~earFound
            error('Earcut failed');
        end
    end
    
    % Add the last triangle
    addTri(h_tris, V(:, vertices(1:3)));
    
    % Clear ear highlight
    if ~isempty(h_ear) && isvalid(h_ear)
        delete(h_ear);
    end
end

% Update scatter plot
% h_old: Old scatter handle
% x, y: New data points
function h = updateScatter(h_old, x, y)
    if ~isempty(h_old) && isvalid(h_old)
        delete(h_old);
    end
    h = scatter(x, y, 50, 'r', 'filled');
end

% Add triangle
% h_old: Old triangle handle
% tri: New triangle points
function h = addTri(h_old, tri)
    if isempty(h_old)
        h_old = [];
    end
    h_new = fill([tri(1,:), tri(1,1)], [tri(2,:), tri(2,1)], ...
                 'b', 'FaceAlpha', 0.3, 'EdgeColor', 'none');
    h = [h_old, h_new];   % Accumulate triangle handles
end

function showPts(pts, offset)
    persistent h_pts;
    if ~isempty(h_pts)
        delete(h_pts);
    end
    
    for i = 1:size(pts, 2)
        h_pts(i) = scatter(pts(1, i) + offset(1), pts(2, i) + offset(2), 'o');
    end
end

function fill2D(pts, offset, color)
    X = pts(1, :);
    Y = pts(2, :);
    X = X + offset(1);
    Y = Y + offset(2);
    fill(X, Y, color, 'FaceAlpha', 0.9);
end

function tris = triangulate(poly3D)
    % poly3D: 3×N Polygon points (clockwise order and no self-intersection)
    % tris:   3×3×T Triangle mesh in world coordinates
    
    disp(poly3D)

    [poly2D, frame] = toLocal2D(poly3D);

    fill2D(poly2D, [6, 0], [0.5 0 0]);

    % Earcut
    tris2D = earClip2D(poly2D);
    
    % Project to 3D
    tris = toWorld3D(tris2D, frame);
end

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
            
            % Check if the triangle is an ear triangle
            if isEar2D(V, v_prev, v_curr, v_next, vertices(1:numV))
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
    
    % fill2D(tri, [6, 0], [0 0 0.5]);
    % showPts(tri, [0, 0]);
    
    for i = 1:length(activeVerts)
        v = activeVerts(i);
        if v == v_prev || v == v_curr || v == v_next
            continue;
        end
        
        p = V(:, v);
        if pointInTriangle2D(p, tri)
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
    eps = 1e-10;
    flag = (a > eps) && (b > eps) && (c > eps);
end

% function dir = polyDirection(pts)
%     n = size(pts, 2);

%     area = sum(pts(1, 1:n) .* pts(2, [2:n, 1]) - pts(1, [2:n, 1]) .* pts(2, 1:n));

%     dir = sign(area);
% end

% function dir = polyDirection3D(pts)
%     % Find the best direction of the polygon
%     normal = cross(pts(:, 2) - pts(:, 1), pts(:, 3) - pts(:, 1));
%     [~, maxDim] = max(abs(normal));

%     % Project on other two dimensions
%     dims = setdiff([1, 2, 3], maxDim);
%     ptsProj = pts(dims, :);

%     dir = polyDirection(ptsProj);
% end