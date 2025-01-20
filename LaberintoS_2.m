function [map, cost_map] = LaberintoS_2()
    clc;
    close all;
    warning off;

    %% Map parameters
    map_width = 100;  % Map width in meters
    map_height = 100; % Map height in meters
    resolution = 1;   % 1 cell = 1 meter

    %% Create occupancy map and cost map
    map = robotics.BinaryOccupancyGrid(map_width, map_height, 1 / resolution);
    cost_map = zeros(map_width, map_height);

    %% Add obstacles (e.g., borders, lines, and corners)
    obstacles = [];

    % Borders
    for x = 1:map_width
        obstacles = [obstacles; x, 1; x, map_height]; % Top and bottom borders
    end
    for y = 1:map_height
        obstacles = [obstacles; 1, y; map_width, y]; % Left and right borders
    end

    % Corners 
    corner_size = 4;
    for x = 1:corner_size
        for y = 1:corner_size
            obstacles = [obstacles; x, y]; % Bottom-left
            obstacles = [obstacles; map_width - x + 1, y]; % Bottom-right
            obstacles = [obstacles; x, map_height - y + 1]; % Top-left
            obstacles = [obstacles; map_width - x + 1, map_height - y + 1]; % Top-right

            %20 to 50
            obstacles = [obstacles; x, y+20-1]; % Bottom-left
            obstacles = [obstacles; map_width - x + 1, y+40-1]; % Bottom-right
            obstacles = [obstacles; x, y+(50-corner_size)]; % Top-left
            obstacles = [obstacles; map_width - x + 1, y+(30-corner_size)]; % Top-right

            %60 to 90
            obstacles = [obstacles; x, y+60-1]; % Bottom-left
            obstacles = [obstacles; map_width - x + 1, y+80-1]; % Bottom-right
            obstacles = [obstacles; x, y+(90-corner_size)]; % Top-left
            obstacles = [obstacles; map_width - x + 1, y+(70-corner_size)]; % Top-right



        end
    end

    % Vertical lines on the left and right borders
    left_centers = 10:20:90;
    for y_c = left_centers
        for yy = y_c - 2:y_c + 2
            if yy >= 1 && yy <= map_height
                obstacles = [obstacles; 1, yy]; % Left border
                obstacles = [obstacles; map_width, yy]; % Right border
            end
        end
    end

    % Horizontal lines
    for y_val = [10, 20, 50, 60, 90]
        % Line starting at x=1 and ending at x=85
        for x_val = 1:85
            obstacles = [obstacles; x_val, y_val];
        end
    end
    for y_val = [30, 40, 70, 80]
        % Line starting at x=15 and ending at x=100
        for x_val = 15:100
            obstacles = [obstacles; x_val, y_val];
        end
    end

    % Set obstacles in the occupancy map
    setOccupancy(map, obstacles, 1);

    %% Assign cost map values
    % Get the binary matrix for obstacles
    obstacle_matrix = getOccupancy(map);

    % Iterate through the grid to assign costs
    for x = 1:map_width
        for y = 1:map_height
            if obstacle_matrix(x, y) == 0 % Only for free cells
                % Define boundaries for neighbor checks
                x_min1 = max(x - 1, 1); x_max1 = min(x + 1, map_width);
                y_min1 = max(y - 1, 1); y_max1 = min(y + 1, map_height);

                x_min2 = max(x - 2, 1); x_max2 = min(x + 2, map_width);
                y_min2 = max(y - 2, 1); y_max2 = min(y + 2, map_height);

                x_min3 = max(x - 3, 1); x_max3 = min(x + 3, map_width);
                y_min3 = max(y - 3, 1); y_max3 = min(y + 3, map_height);

                x_min4 = max(x - 4, 1); x_max4 = min(x + 4, map_width);
                y_min4 = max(y - 4, 1); y_max4 = min(y + 4, map_height);

                x_min5 = max(x - 5, 1); x_max5 = min(x + 5, map_width);
                y_min5 = max(y - 5, 1); y_max5 = min(y + 5, map_height);

                % Assign costs based on proximity to obstacles
                if any(any(obstacle_matrix(x_min1:x_max1, y_min1:y_max1))) % Immediate neighbors
                    cost_map(x, y) = 10000; % First layer
                elseif any(any(obstacle_matrix(x_min2:x_max2, y_min2:y_max2))) % Second layer
                    cost_map(x, y) = 1000; % Second layer
                elseif any(any(obstacle_matrix(x_min3:x_max3, y_min3:y_max3))) % Third layer
                    cost_map(x, y) = 100; % Third layer
                elseif any(any(obstacle_matrix(x_min4:x_max4, y_min4:y_max4))) % Forth layer
                    cost_map(x, y) = 50; % Forth layer
                elseif any(any(obstacle_matrix(x_min4:x_max5, y_min5:y_max5))) % Fifth layer
                    cost_map(x, y) = 10; % Fifth layer
                end
            end
        end
    end

    %% Visualize the maps
    % Display the occupancy map
    figure;
    show(map);
    title('Occupancy Map');
    grid on;
    set(gca, 'XTick', 0:10:map_width, 'YTick', 0:10:map_height);

    % Display the cost map
    figure;
    imagesc(cost_map);
    set(gca, 'YDir', 'normal');
    colorbar;
    title('Cost Map');
    xlabel('X [m]');
    ylabel('Y [m]');
    grid on;
    set(gca, 'XTick', 0:10:map_width, 'YTick', 0:10:map_height);
end

