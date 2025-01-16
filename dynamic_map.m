function [map, cost_map] = dynamic_map()
clc
warning off
%obstacles = 0;
% Path dimensions
map_width = 100; % meters
map_height = 100; % meters
resolution = 1; % cell size (1 meter)

% Create binary occupancy map
map = robotics.BinaryOccupancyGrid(map_width, map_height, 1/resolution);

% Initialize cost map
cost_map = zeros(map_width, map_height);

% Create a rectangle in the center of the map with 1-meter borders
rect_width = 100; % rectangle width
rect_height = 16; % rectangle height
y_start = (map_height - rect_height) / 2; % start on the Y-axis
y_end = y_start + rect_height; % end on the Y-axis
border_thickness = 1; % border thickness (1 meter)

for x = 1:rect_width
    for y = round(y_start):round(y_end)
        % Top and bottom borders
        if y < round(y_start) + border_thickness || y > round(y_end) - border_thickness
            cost_map(x, y) = 10000; % Assign cost to borders
            setOccupancy(map, [x, y], 1); % Mark occupancy in the map
        end
    end
end

for y = round(y_start):round(y_end)
    for x = 1:rect_width
        % Left and right borders
        if x <= border_thickness || x > rect_width - border_thickness
            cost_map(x, y) = 3000; % Assign cost to borders
            setOccupancy(map, [x, y], 1); % Mark occupancy in the map
        end
    end
end

% Generate 20 random points inside the rectangle with a cost of 10000
num_obstacles = 5;
obstacle_radius = 1; % 1 meter
obstacles = zeros(num_obstacles, 2);
for i = 1:num_obstacles
    while true
        % Generate a random position inside the rectangle (excluding borders)
        x = randi([15, rect_width - 10]);
        y = randi([round(y_start) + border_thickness + 1, round(y_end) - border_thickness]);

        % Verify no overlap with existing obstacles
        overlap = false;
        for j = 1:i-1
            if sqrt((obstacles(j, 1) - x)^2 + (obstacles(j, 2) - y)^2) < obstacle_radius
                overlap = true;
                break;
            end
        end

        if ~overlap
            obstacles(i, :) = [x, y];
            break;
        end
    end

    % Assign a cost of 10000 to the cells around the obstacle
    for dx = -obstacle_radius:obstacle_radius
        for dy = -obstacle_radius:obstacle_radius
            if sqrt(dx^2 + dy^2) <= obstacle_radius
                cx = x + dx;
                cy = y + dy;
                if cx > 0 && cy > 0 && cx <= map_width && cy <= map_height
                    cost_map(cx, cy) = 10000;
                    setOccupancy(map, [cx, cy], 1); % Mark occupancy in the map
                end
            end
        end
    end
end

% % Display occupancy map
% figure;
% show(map);
% title('Occupancy Map');
% grid on;
% set(gca, 'XTick', 0:10:map_width, 'YTick', 0:2:map_height);
% 
% % Display cost map
% figure;
% imagesc(cost_map);
% set(gca, 'YDir', 'normal');
% colorbar;
% title('Cost Map');
% xlabel('X [m]');
% ylabel('Y [m]');
% grid on;
% set(gca, 'XTick', 0:10:map_width, 'YTick', 0:2:map_height);

end


