function [map, cost_map] = dynamic_map()
clc;

% Path dimensions
map_width = 100; % meters
map_height = 100; % meters
resolution = 1; % cell size 

% Create binary occupancy map
map = robotics.BinaryOccupancyGrid(map_width, map_height, 1/resolution);

% Initialize cost map
cost_map = zeros(map_width, map_height);

% Create a rectangle in the center of the map with 1-meter borders
rect_width = map_width-2; % rectangle width
rect_height = 16; % rectangle height
y_start = (map_height - rect_height) / 2; % start on the Y-axis
y_end = y_start + rect_height; % end on the Y-axis
border_thickness = 1; % border thickness (1 meter)

% Add borders to the map
for x = (map_width-rect_width):rect_width
    for y = round(y_start):round(y_end)
        % Top and bottom borders
        if y < round(y_start) + border_thickness || y > round(y_end) - border_thickness
            setOccupancy(map, [x, y], 1); % Mark occupancy in the map
        end
    end
end

for y = round(y_start):round(y_end)
    for x = 1:rect_width
        % Left and right borders
        if x <= border_thickness || x > rect_width - border_thickness
            setOccupancy(map, [x + (map_width - rect_width) / 2, y], 1); % Mark occupancy in the map left
            %setOccupancy(map, [x + map_width - ((map_width - rect_width) ), y], 1); % Mark occupancy in the map right 
        end
    end
end

% Generate random obstacles inside the rectangle
num_obstacles = 10;
obstacle_radius = 0.25; % 0.25 meter
obstacles = zeros(num_obstacles, 2);
for i = 1:num_obstacles
    while true
        % Generate a random position inside the rectangle (excluding borders)
        x = randi([6 + border_thickness, rect_width - border_thickness]);
        y = randi([round(y_start) + border_thickness + 1, round(y_end) - border_thickness - 1]);
        
        % Check if the position is free
        if ~checkOccupancy(map, [x, y])
            obstacles(i, :) = [x, y];
            setOccupancy(map, [x, y], 1); % Mark the position as occupied
            cost_map(x, y) = 10000; % High cost for obstacle
            break;
        end
    end
end

% Generate the cost map based on proximity to obstacles
obstacle_matrix = getOccupancy(map);
map_width_real = map_width;
map_height_real = map_height;

for x = 1:map_width_real
    for y = 1:map_height_real
        if obstacle_matrix(y, x) == 0 % Only for free cells
            % Define proximity ranges
            for dist = 1:5
                x_min = max(x - dist, 1);
                x_max = min(x + dist, map_width_real);
                y_min = max(y - dist, 1);
                y_max = min(y + dist, map_height_real);
                
                if any(any(obstacle_matrix(y_min:y_max, x_min:x_max)))
                    cost_map(y, x) = 10^(5 - dist); % Assign cost based on proximity
                    break;
                end
            end
        end
    end
end

% % Visualizations for debugging/checking
% figure;
% show(map);
% title('Map with Obstacles');
% 
%     % Display the cost map
%     figure;
%     imagesc(cost_map);
%     colorbar;

end



