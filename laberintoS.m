function [map, cost_map] = laberintoS()
    clc
    warning off
    
    % Map dimensions
    map_width  = 100;  % meters
    map_height = 100;  % meters
    resolution = 1;    % 1 cell = 1 meter
    % Costs
    lineEnds = 10000;
    lineStarts = 1000;
    lineMid = 500;
    sideWalls = 7000;
    corners = 1000;

    % Create binary occupancy map
    map = robotics.BinaryOccupancyGrid(map_width, map_height, 1/resolution);

    % Initialize cost map
    cost_map = zeros(map_width, map_height);

    %%----------------------------------------------------------------------
    % 1. Borders around the map (cost = 5000)
    %%----------------------------------------------------------------------
    border_thickness = 1;  % Border thickness
    
    % Top and bottom borders
    for x = 1:map_width
        for y = 1:border_thickness  % Bottom border
            cost_map(x, y) = 500;
            setOccupancy(map, [x, y], 1);
        end
        for y = map_height - border_thickness + 1 : map_height  % Top border
            cost_map(x, y) = 500;
            setOccupancy(map, [x, y], 1);
        end
    end

    % Left and right borders
    for y = 1:map_height
        for x = 1:border_thickness  % Left border
            cost_map(x, y) = sideWalls;
            setOccupancy(map, [x, y], 1);
        end
        for x = map_width - border_thickness + 1 : map_width  % Right border
            cost_map(x, y) = sideWalls;
            setOccupancy(map, [x, y], 1);
        end
    end

    %%----------------------------------------------------------------------
    % 2. Corners (3x3 meters) with cost = 10000 (according to your code)
    %%----------------------------------------------------------------------
    corner_size = 3;

    % A) Bottom-left corner
    for x = 1:corner_size
        for y = 1:corner_size
            cost_map(x, y) = corners;
            %setOccupancy(map, [x, y], 1);
        end
    end

    % B) Bottom-right corner
    for x = map_width - corner_size + 1 : map_width
        for y = 1:corner_size
            cost_map(x, y) = corners;
            %setOccupancy(map, [x, y], 1);
        end
    end

    % C) Top-left corner
    for x = 1:corner_size
        for y = map_height - corner_size + 1 : map_height
            cost_map(x, y) = corners;
            %setOccupancy(map, [x, y], 1);
        end
    end

    % D) Top-right corner
    for x = map_width - corner_size + 1 : map_width
        for y = map_height - corner_size + 1 : map_height
            cost_map(x, y) = corners;
            %setOccupancy(map, [x, y], 1);
        end
    end

    %%----------------------------------------------------------------------
    % 2bis. Vertical "lines" of 5 m on the left and right edges
    %       with cost = 1000
    %
    %  - Left: every 20 m starting at y=10 (10, 30, 50, 70, 90)
    %  - Right: every 20 m starting at y=20 (20, 40, 60, 80, 100)
    %%----------------------------------------------------------------------

    % Left edge (x=1), centers at y=10, 30, 50, 70, 90
    left_centers = 10:20:90;
    for y_c = left_centers
        y_start = max(1,  y_c - 2);
        y_end   = min(map_height, y_c + 2);
        for yy = y_start : y_end
            cost_map(1, yy) = lineStarts;
            setOccupancy(map, [1, yy], 1);
        end
    end

    % Right edge (x=100), centers at y=20, 40, 60, 80, 100
    right_centers = 20:20:100;
    for y_c = right_centers
        y_start = max(1,  y_c - 2);
        y_end   = min(map_height, y_c + 2);
        for yy = y_start : y_end
            cost_map(map_width, yy) = lineStarts;
            setOccupancy(map, [map_width, yy], 1);
        end
    end

    %%----------------------------------------------------------------------
    % 3. Horizontal lines of 85 m along X (adapted from your code)
    %    - a) y = [10,30,50,70,90], x = 0..84
    %         (x= 0..4   -> 10000, x=5..79  -> 5000, x=80..84 -> 10000)
    %    - b) y = [20,40,60,80],    x = 15..99
    %         (x=15..19 -> 10000, x=20..94 -> 5000, x=95..99 -> 10000)
    %%----------------------------------------------------------------------

    % a) Lines: y = [10, 30, 50, 70, 90]
    line_y_start_0 = [10, 30, 50, 70, 90];
    for y_val = line_y_start_0
        % Segment 10000 (first 5 m: x=0..4)
        for x_val = 0 : 4
            cost_map(x_val+1, y_val) = lineStarts;
            setOccupancy(map, [x_val+1, y_val], 1);
        end
        % Segment (x=5..79)
        for x_val = 5 : 79
            cost_map(x_val+1, y_val) = lineMid;
            setOccupancy(map, [x_val+1, y_val], 1);
        end
        % Segment 10000 (last 5 m: x=80..84)
        for x_val = 80 : 84
            cost_map(x_val+1, y_val) = lineEnds;
            setOccupancy(map, [x_val+1, y_val], 1);
        end
    end

    % b) Lines: y = [20, 40, 60, 80]
    line_y_start_15 = [20, 40, 60, 80];
    for y_val = line_y_start_15
        % Segment 10000 (first 5 m: x=15..19)
        for x_val = 15 : 19
            cost_map(x_val+1, y_val) = lineEnds;
            setOccupancy(map, [x_val+1, y_val], 1);
        end
        % Remaining segment
        for x_val = 20 : 94
            cost_map(x_val, y_val) = lineMid;
            setOccupancy(map, [x_val, y_val], 1);
        end
        % Segment 10000 (last 5 m: x=95..99)
        for x_val = 95 : 99
            cost_map(x_val, y_val) = lineStarts;
            setOccupancy(map, [x_val, y_val], 1);
        end
    end

    %%----------------------------------------------------------------------
    % 4. Random obstacles (single cell point) with cost = 10000
    %%----------------------------------------------------------------------
    num_obstacles = 1;  
    obstacles = zeros(num_obstacles, 2);

    for i = 1:num_obstacles
        while true
            % Generate random position within the map
            % Avoid the 1-cell border, 3x3 corners, and lines,
            % (or any cell where cost_map > 0)
            x_rand = randi([2, map_width - 1]);
            y_rand = randi([2, map_height - 1]);

            % Check it does not overlap with previous obstacles
            overlap = any(obstacles(:,1) == x_rand & obstacles(:,2) == y_rand);

            % Check it does not fall in already occupied zones (borders, corners, lines)
            if cost_map(x_rand, y_rand) > 0
                overlap = true;
            end

            if ~overlap
                obstacles(i, :) = [x_rand, y_rand];
                break;
            end
        end

        % Assign cost 10000 to that cell
        cost_map(x_rand, y_rand) = 2000;
        setOccupancy(map, [x_rand, y_rand], 1);
    end

    %----------------------------------------------------------------------
    % (Optional) Visualize occupancy map and cost map
    %----------------------------------------------------------------------
%     figure;
%     show(map);
%     title('Occupancy Map');
%     grid on;
%     set(gca, 'XTick', 0:10:map_width, 'YTick', 0:10:map_height);
%     
%     figure;
%     imagesc(cost_map);
%     set(gca, 'YDir', 'normal');
%     colorbar;
%     title('Cost Map');
%     xlabel('X [m]');
%     ylabel('Y [m]');
%     grid on;
%     set(gca, 'XTick', 0:10:map_width, 'YTick', 0:10:map_height);

end

