classdef PannelUI < handle
    properties
        fig;
        col = 3;
        row = 2;
        size = [300 400];
        posw = [400 400];       % Window position
        cell_size = [100 50];
        name = 'Lidar Controller';
        gap = 5;
    end
    methods
        function obj = PannelUI(col, row, size, pos, name)
            if nargin < 5
                name = 'Lidar Controller';
            end
            if nargin < 4
                pos = [400 400];
            end
            if nargin < 3
                size = [300 400];
            end
            obj.col = col;
            obj.row = row;
            obj.size = size;
            obj.posw = pos;
            obj.name = name;

            % Calculate the size of each cell
            obj.cell_size = [floor((obj.size(1) - obj.gap) / obj.col) - obj.gap, ...
                floor((obj.size(2) - obj.gap) / obj.row) - obj.gap];

            % Make a grid of buttons, confirm the size of each button is equal
            obj.size(1) = (obj.cell_size(1) + obj.gap) * obj.col + obj.gap;
            obj.size(2) = (obj.cell_size(2) + obj.gap) * obj.row + obj.gap;

            obj.fig = uifigure('Position', [obj.posw(1) obj.posw(2) obj.size(1) obj.size(2)], ...
                'Color', [0.1 0.1 0.1], 'Name', obj.name);
        end

        function button(obj, col, row, text, callback, color)
            if nargin < 6
                color = [0.3 0.8 1];
            end

            % Calculate the position of the button
            pos = [(obj.cell_size(1) + obj.gap) * (col - 1) + obj.gap, ...
                (obj.cell_size(2) + obj.gap) * (obj.row - row) + obj.gap];
            % pos = pos + obj.posw;
            pos = [pos, obj.cell_size];

            disp(pos)
            disp(callback)

            uibutton(obj.fig, 'push', 'Position', pos, 'Text', text, ...
                'FontColor', 'white', 'FontSize', 14, ...
                'BackgroundColor', color, 'ButtonPushedFcn', callback);

        end
    end
end