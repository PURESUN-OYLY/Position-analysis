classdef GeoEntity < handle
    enumeration
        % r: radius, h:height, t:top, b:bottom
        % x:axis-x, y:axis-y, z:axis-z
        cylinder;   % [r]
        sphere;     
        cube;
        cycle;
        triangle;
        tetragonum;
    end

    properties
        pos(1, 3) double = [0, 0, 0];               % The center of mass of entity
        cen(1, 6) double = [0, 0, 0, 0, 1, 1, 1];   % Top and bottom face center of entity
        rad(1, 2) double = [1, 1];                  % Top and bottom radius of entity
        rot(1, 3) double = [0, 0, 0];               % Posture of entity[x, y, z]
        color(1, 3) double = [0,4, 0,6, 0.8];       % The color of entity
    end

    methods
        function obj = GeoEntity(type)
            if ~obj.isValid(type)
                disp([type 'is not allowed'])
                disp(['Only support: ', obj.getAll()])
            end
        end
    end

    methods (Static)
        function flag = isValid(name)
            try
                ShapeType(name);
                flag = true;
            catch
                flag = false;
            end
        end
        
        function list = getAll()
            e = enumeration('ShapeType');
            list = string(e);
        end
    end
end