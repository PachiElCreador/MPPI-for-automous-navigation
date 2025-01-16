function [map, cost_map, obstacles] = hexagonal_map()

% Set the size of the map in meters
size_grid_x = 100; % width of the map
size_grid_y = 100; % height of the map
% Size of each square on the grid in meters
size_square = 1;

% Create the map with help of the Robotics toolbox
map = robotics.BinaryOccupancyGrid(size_grid_x, size_grid_y, size_square);

% Define the hexagonal boundary
hex_radius = 50; % Radius of the hexagon in grid units
center_x = size_grid_x / 2;
center_y = size_grid_y / 2;

obstacles = [];

% Define the hexagonal boundary points using polar coordinates
thickness = 2; % Thickness of the hexagonal boundary
for theta = 0:60:360
    x1 = center_x + hex_radius * cosd(theta);
    y1 = center_y + hex_radius * sind(theta);
    x2 = center_x + hex_radius * cosd(theta + 60);
    y2 = center_y + hex_radius * sind(theta + 60);
    
    % Add points along the edges with thickness
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

% Add internal circular obstacles
cir_radius = 4; % Radius of the circular obstacles
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

% Set the obstacles on the map
setOccupancy(map, obstacles, 1);

% Display the map
figure
show(map)

%Set a grid for the map
grid on
set(gca,'XTick',0:2:size_grid_x,'YTick',0:2:size_grid_y) 

% Initialize cost map
cost_map = zeros(size_grid_x, size_grid_y);

%% Set high cost for hexagon boundaries
for theta = 0:60:360
    x1 = center_x + hex_radius * cosd(theta);
    y1 = center_y + hex_radius * sind(theta);
    x2 = center_x + hex_radius * cosd(theta + 60);
    y2 = center_y + hex_radius * sind(theta + 60);

    % Add high cost along the edges with thickness
    for t = 0:0.01:1
        base_x = round(x1 * (1 - t) + x2 * t);
        base_y = round(y1 * (1 - t) + y2 * t);
        for dx = -thickness:thickness
            for dy = -thickness:thickness
                if sqrt(dx^2 + dy^2) <= thickness
                    if base_x + dx > 0 && base_y + dy > 0 && base_x + dx <= size_grid_x && base_y + dy <= size_grid_y
                        cost_map(base_x + dx, base_y + dy) = 10000; % High cost for hexagon boundary
                    end
                end
            end
        end
    end
end

% Set costs for internal circular obstacles
for i = 1:size(internal_points, 1)
    cx = internal_points(i, 1);
    cy = internal_points(i, 2);
    for dx = -cir_radius:cir_radius
        for dy = -cir_radius:cir_radius
            if sqrt(dx^2 + dy^2) <= cir_radius
                x = round(cx + dx);
                y = round(cy + dy);
                if x > 0 && y > 0 && x <= size_grid_x && y <= size_grid_y
                    cost_map(x, y) = 5000; % Medium cost for circular obstacles
                end
            end
        end
    end
end

% % Display the cost map for debugging
% figure;
% imagesc(cost_map);
% set(gca, 'YDir', 'normal'); % Corrige la dirección del eje Y
% colorbar;
% title('Cost Map');
% xlabel('X [grid units]');
% ylabel('Y [grid units]');
% 
% %Set a grid for the map
% grid on
% set(gca,'XTick',0:2:size_grid_x,'YTick',0:2:size_grid_y) 

end

