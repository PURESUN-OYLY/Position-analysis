classdef GeoEntity < handle
    enumeration
        notdefined({0, 'undefined'})    % [id, name]
        cylinder({1, 'cylinder'})       % [id, name]
        sphere({2, 'sphere'})           % [id, name]
        cube({3, 'cube'})               % [id, name]
        cycle({4, 'cycle'})             % [id, name]
        triangle({5, 'triangle'})       % [id, name]
        tetragonum({6, 'tetragonum'})   % [id, name]
    end

    properties
        % Default type is undefined, which means the entity is not initialized
        type = {};

        % r: radius, h:height, t:top, b:bottom
        % x:axis-x, y:axis-y, z:axis-z
        pos(1, 3) double = [0, 0, 0];               % The center of mass of entity
        cen(1, 6) double = [0, 0, 0, 1, 1, 1];      % Top and bottom face center of entity
        rad(1, 2) double = [1, 1];                  % Top and bottom radius of entity
        rot(1, 3) double = [0, 0, 0];               % Posture of entity[x, y, z]
        color(1, 3) double = [0.4, 0.6, 0.8];       % The color of entity
    end

    methods
        % Constructor, initialize the entity with default values
        function obj = GeoEntity(type)
            obj.type = type;
        end
    end

    methods (Static)
        function list = getAll()
            e = enumeration('ShapeType');
            list = string(e);
        end
    end
end