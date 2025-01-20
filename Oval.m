function [map, cost_map, obstacles] = Oval()
    close all;
    %% Parámetros del mapa
    map_width  = 100;  % Tamaño en X (en celdas)
    map_height = 100;  % Tamaño en Y (en celdas)
    cellSize   = 1;    % Tamaño de cada celda (en metros)

    %% Crear el mapa (BinaryOccupancyGrid de Robotics Toolbox)
    map = robotics.BinaryOccupancyGrid(map_width, map_height, cellSize);

    %% Construcción de obstáculos (óvalo en diagonal)
    obstacles = [];

    % Parámetros del óvalo en diagonal
    center_x = map_width / 2;
    center_y = map_height / 2;
    outer_a = 50; % Semi-eje mayor del óvalo exterior
    outer_b = 35; % Semi-eje menor del óvalo exterior
    inner_a = 40; % Semi-eje mayor del óvalo interior
    inner_b = 25; % Semi-eje menor del óvalo interior
    angle = -pi / 4; % Ángulo de rotación en radianes (45 grados)

    % Función para rotar puntos
    rotate = @(x, y) [cos(angle), -sin(angle); sin(angle), cos(angle)] * [x; y];

    % Generar obstáculos para las áreas libres invertidas
    for x = 1:map_width
        for y = 1:map_height
            % Transformar las coordenadas al marco del óvalo rotado
            rotated_coords = rotate(x - center_x, y - center_y);
            x_rot = rotated_coords(1);
            y_rot = rotated_coords(2);

            % Ecuación del óvalo exterior
            if ((x_rot / outer_a)^2 + (y_rot / outer_b)^2) > 1
                % Ecuación del óvalo interior
                obstacles = [obstacles; x, y];
            elseif ((x_rot / inner_a)^2 + (y_rot / inner_b)^2) <= 1
                obstacles = [obstacles; x, y];
            end
        end
    end

    %% Marcar ocupación de los obstáculos en el mapa
    setOccupancy(map, obstacles, 1);

    %% Inicializar el cost_map
    cost_map = zeros(map_width, map_height);

    %% Generar la matriz de ocupación a partir del mapa
    obstacle_matrix = getOccupancy(map);

    %% Asignar costos basados en cercanía a obstáculos
    for x = 1:map_width
        for y = 1:map_height
            if obstacle_matrix(x, y) == 0 % Solo para celdas libres
                % Delimitar la vecindad para cada capa
                x_min1 = max(x - 1, 1);   x_max1 = min(x + 1, map_width);
                y_min1 = max(y - 1, 1);   y_max1 = min(y + 1, map_height);

                x_min2 = max(x - 2, 1);   x_max2 = min(x + 2, map_width);
                y_min2 = max(y - 2, 1);   y_max2 = min(y + 2, map_height);

                x_min3 = max(x - 3, 1);   x_max3 = min(x + 3, map_width);
                y_min3 = max(y - 3, 1);   y_max3 = min(y + 3, map_height);

                x_min4 = max(x - 4, 1);   x_max4 = min(x + 4, map_width);
                y_min4 = max(y - 4, 1);   y_max4 = min(y + 4, map_height);

                x_min5 = max(x - 5, 1);   x_max5 = min(x + 5, map_width);
                y_min5 = max(y - 5, 1);   y_max5 = min(y + 5, map_height);

                % Capas de costo según vecindario
                if any(any(obstacle_matrix(x_min1:x_max1, y_min1:y_max1)))
                    cost_map(x, y) = 10000;  % Capa 1 (adyacencia inmediata)
                elseif any(any(obstacle_matrix(x_min2:x_max2, y_min2:y_max2)))
                    cost_map(x, y) = 1000;   % Capa 2
                elseif any(any(obstacle_matrix(x_min3:x_max3, y_min3:y_max3)))
                    cost_map(x, y) = 100;    % Capa 3
                elseif any(any(obstacle_matrix(x_min4:x_max4, y_min4:y_max4)))
                    cost_map(x, y) = 10;     % Capa 4
                elseif any(any(obstacle_matrix(x_min5:x_max5, y_min5:y_max5)))
                    cost_map(x, y) = 5;      % Capa 5
                end
            end
        end
    end

    %% (Opcional) Visualizaciones para depurar/checar
%     figure;
%     show(map);
%     title('Circuito óvalo en diagonal invertido');
% 
%    figure;
%    imagesc(cost_map');
%    axis xy; colorbar; title('Cost Map');
end
