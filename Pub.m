classdef Pub < handle
    properties

    end
    methods (Static)
        function hp = drawAABB(aabb, faceColor, faceAlpha, edgeColor, lineWidth)
            % Check the input arguments
            if nargin == 1
                faceColor = [0.2 0.8 0.2];
                faceAlpha = 0.3;
                edgeColor = 'g';
                lineWidth = 1;
            elseif nargin == 2
                faceAlpha = 0.3;
                edgeColor = 'g';
                lineWidth = 1;
            elseif nargin == 3
                edgeColor = 'g';
                lineWidth = 1;
            elseif nargin == 4
                lineWidth = 1;
            elseif nargin == 5
            else
                error('Invalid number of input arguments.');
            end

            % Check the aabb box is valid
            if length(aabb) ~= 6
                error('Invalid aabb box.');
            end
            disp(aabb);
            
            if any(~isfinite(aabb)) || any(aabb(1:3) >= aabb(4:6))
                error('Invalid AABB: [%f %f %f %f %f %f]', aabb);
            end
            minX = aabb(1); minY = aabb(2); minZ = aabb(3);
            maxX = aabb(4); maxY = aabb(5); maxZ = aabb(6);

            % 8 points of the frustum's aabb box
            verts = [
                minX, minY, minZ;  % 1
                maxX, minY, minZ;  % 2
                maxX, maxY, minZ;  % 3
                minX, maxY, minZ;  % 4
                minX, minY, maxZ;  % 5
                maxX, minY, maxZ;  % 6
                maxX, maxY, maxZ;  % 7
                minX, maxY, maxZ;  % 8
                ];

            % 6 faces of the frustum's aabb box
            faces = [
                1, 2, 3, 4;   % Bottom (Z=min)
                5, 6, 7, 8;   % Top (Z=max)
                2, 6, 7, 3;   % Right (X=max)
                1, 5, 8, 4;   % Left (X=min)
                4, 3, 7, 8;   % Front (Y=max)
                1, 2, 6, 5;   % Back (Y=min)
                ];

            % Render all faces for frustum's aabb box
            hp = patch('Vertices', verts, 'Faces', faces, ...
                'FaceColor', faceColor, 'FaceAlpha', faceAlpha, ...
                'EdgeColor', edgeColor, 'LineWidth', lineWidth);
        end
    end
end