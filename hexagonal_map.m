function [map, cost_map, obstacles] = hexagonal_map()
    %% Parameters
    % Map dimensions in meters
    size_grid_x = 100; % Width of the map
    size_grid_y = 100; % Height of the map
    size_square = 1;   % Size of each square on the grid (in meters)

    % Hexagonal boundary settings
    hex_radius = 50; % Radius of the hexagon in grid units
    center_x = size_grid_x / 2; % Center X-coordinate of the hexagon
    center_y = size_grid_y / 2; % Center Y-coordinate of the hexagon
    thickness = 2; % Thickness of the hexagonal boundary

    % Internal circular obstacles settings
    cir_radius = 4; % Radius of the circular obstacles

    %% Create the occupancy map
    map = robotics.BinaryOccupancyGrid(size_grid_x, size_grid_y, size_square);

    % Initialize the obstacles list
    obstacles = [];

    %% Define hexagonal boundary
    % Generate hexagonal boundary using polar coordinates
    for theta = 0:60:360
        x1 = center_x + hex_radius * cosd(theta);
        y1 = center_y + hex_radius * sind(theta);
        x2 = center_x + hex_radius * cosd(theta + 60);
        y2 = center_y + hex_radius * sind(theta + 60);

        % Add points along the edges with specified thickness
        for t = 0:0.01:1
            base_x = round(x1 * (1 - t) + x2 * t);
            base_y = round(y1 * (1 - t) + y2 * t);
            for dx = -thickness:thickness
                for dy = -thickness:thickness
                    if sqrt(dx^2 + dy^2) <= thickness
                        obstacles = [obstacles; base_x + dx, base_y + dy];
                    end
                end
            end
        end
    end

    %% Add internal circular obstacles
    % Define the centers of the circular obstacles
    internal_points = [
        center_x - 20, center_y - 20;
        center_x, center_y - 20;
        center_x + 20, center_y - 20;
        center_x - 20, center_y;
        center_x, center_y;
        center_x + 20, center_y;
        center_x - 20, center_y + 20;
        center_x, center_y + 20;
        center_x + 20, center_y + 20
    ];

    % Generate circular obstacles around each center
    for i = 1:size(internal_points, 1)
        cx = internal_points(i, 1);
        cy = internal_points(i, 2);
        for dx = -cir_radius:cir_radius
            for dy = -cir_radius:cir_radius
                if sqrt(dx^2 + dy^2) <= cir_radius
                    obstacles = [obstacles; cx + dx, cy + dy];
                end
            end
        end
    end

    %% Set obstacles in the occupancy map
    setOccupancy(map, obstacles, 1);

    %% Display the occupancy map
    figure;
    show(map);
    title('Occupancy Map');
    grid on;
    set(gca, 'XTick', 0:2:size_grid_x, 'YTick', 0:2:size_grid_y);

    %% Initialize cost map
    cost_map = zeros(size_grid_x, size_grid_y);

    %% Generate cost map
    % Obtain the obstacle matrix
    obstacle_matrix = getOccupancy(map);

    % Iterate through the grid to assign cost values
    for x = 1:size_grid_x
        for y = 1:size_grid_y
            % Check if the current cell is free
            if obstacle_matrix(x, y) == 0
                % Define boundaries for neighbors
                x_min1 = max(x-1, 1); x_max1 = min(x+1, size_grid_x);
                y_min1 = max(y-1, 1); y_max1 = min(y+1, size_grid_y);

                x_min2 = max(x-2, 1); x_max2 = min(x+2, size_grid_x);
                y_min2 = max(y-2, 1); y_max2 = min(y+2, size_grid_y);

                x_min3 = max(x-3, 1); x_max3 = min(x+3, size_grid_x);
                y_min3 = max(y-3, 1); y_max3 = min(y+3, size_grid_y);

                % Assign cost based on proximity to obstacles
                if any(any(obstacle_matrix(x_min1:x_max1, y_min1:y_max1))) % Immediate neighbors
                    cost_map(x, y) = 10000; % First layer
                elseif any(any(obstacle_matrix(x_min2:x_max2, y_min2:y_max2))) % Second layer
                    cost_map(x, y) = 1000; % Second layer
                elseif any(any(obstacle_matrix(x_min3:x_max3, y_min3:y_max3))) % Third layer
                    cost_map(x, y) = 100; % Third layer
                end
            end
        end
    end

    %% Display the cost map
    figure;
    imagesc(cost_map);
    set(gca, 'YDir', 'normal');
    colorbar;
    title('Cost Map');
    xlabel('X');
    ylabel('Y');
    set(gca, 'XTick', 0:2:size_grid_x, 'YTick', 0:2:size_grid_y);
end

