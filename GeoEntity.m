% This is a class to manage the entity in the figure
classdef GeoEntity < handle
    properties
        % Count of the entity in the list
        % Default value is 0
        count = 0;

        % List of the entity in the list
        % Default value is empty matrix
        entityList = [];
    end
    methods
        function obj = GeoEntity()
        end

        function ent = Create(obj, type)
            ent = Entity(type, obj.count);
            obj.count = obj.count + 1;
            obj.entityList = [obj.entityList; ent];
        end

        function renderAll(obj)
            for i = 1:obj.count
                obj.entityList(i).render();
            end
        end
    end
end
