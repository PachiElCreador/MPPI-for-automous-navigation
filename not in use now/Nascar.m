function [map, cost_map, obstacles] = Nascar()
    clc;
    clear all;
    close all;

    %% Parámetros del mapa
    map_width  = 800; % Tamaño en X (en celdas)
    map_height = 200; % Tamaño en Y (en celdas)
    cellSize   = 2; % Esto hace que la celda mida 1/cellSize metros

                shifts = [ 0.5,  0.0;
                      -0.5,  0.0;
                       0.0, -0.5;
                       0.5, -0.5;
                       0.5,  0.5;
                       0.0,  0.5;
                      -0.5, -0.5;
                      -0.5,  0.5;

                      -0.5, -1.0;
                       0.0, -1.0;
                       0.5, -1.0;

                       1.0, -0.5;
                       1.0,  0.0;
                       1.0,  0.5;

                       0.5,  1.0;
                       0.0,  1.0;
                      -0.5,  1.0;

                      -1.0,  0.5;
                      -1.0,  0.0;
                      -1.0, -0.5];

    %% Crear el mapa (BinaryOccupancyGrid de Robotics Toolbox)
    map = robotics.BinaryOccupancyGrid(map_width, map_height, cellSize);

    %% Construcción de obstáculos (óvalo con rectas largas y medios círculos)
    obstacles = [];

    % Parámetros generales
    straight_length = map_width - 2 * (map_height / 2); % Longitud de las rectas
    radius = ((map_height - 15) / 2) - 9; % Radio de los medios círculos

    % Centros de los medios círculos
    left_circle_center = [radius + 1, map_height / 2];
    right_circle_center = [map_width - radius - 1, map_height / 2];

    % Generar obstáculos para las rectas largas
    for x = radius + 10:map_width - radius - 10 
        for offset = 0:11 % ancho de linea
            obstacles = [obstacles; x, map_height / 2 - radius - offset ]; % Recta inferior
            obstacles = [obstacles; x+0.5, map_height / 2 - radius - offset ]; % Recta inferior
            obstacles = [obstacles; x, map_height / 2 - radius - offset - 0.5]; % Recta inferior
            obstacles = [obstacles; x+0.5, map_height / 2 - radius - offset - 0.5]; % Recta inferior

            obstacles = [obstacles; x, map_height / 2 + radius + offset ]; % Recta superior

            obstacles = [obstacles; x+0.5, map_height / 2 + radius + offset ]; % Recta inferior
            obstacles = [obstacles; x, map_height / 2 + radius + offset + 0.5]; % Recta inferior
            obstacles = [obstacles; x+0.5, map_height / 2 + radius + offset + 0.5]; % Recta inferior
        end
    end
    %% Expandir los obstáculos para cubrir áreas vacías
    %inflate(map, 1 / cellSize); % Asegura que todas las celdas se llenen completamente

    % Generar obstáculos para los medios círculos
    for angle = 0:180
        theta = deg2rad(angle + 90);
        for offset = 0:11 % Línea de 12 metros de ancho
            % Medio círculo izquierdo
            x_left = round(left_circle_center(1) + (radius + offset) * cos(theta)) + 16;
            y_left = round(left_circle_center(2) + (radius + offset) * sin(theta));
            obstacles = [obstacles; x_left, y_left];

            % Medio círculo derecho
            x_right = round(right_circle_center(1) - (radius + offset) * cos(theta)) - 16;
            y_right = round(right_circle_center(2) + (radius + offset) * sin(theta));
            obstacles = [obstacles; x_right, y_right];
            
            
            obstacles = [obstacles; x_left+1, y_left];

            if x_left - 1 > 0, obstacles = [obstacles; abs(x_left - 1), y_left]; end

            %obstacles = [obstacles; x_left, y_left-1];
            %obstacles = [obstacles; x_left+(-1), y_left];
            %obstacles = [obstacles; x_left+(-2), y_left];
            
%             obstacles = [obstacles; x_right+.5, y_right];
%             obstacles = [obstacles; x_right-.5, y_right];
%             obstacles = [obstacles; x_right, y_right-.5];
%             obstacles = [obstacles; x_right+.5, y_right-.5];
%             obstacles = [obstacles; x_right+.5, y_right+.5];
% 
%             obstacles = [obstacles; x_right, y_right+.5];
%             obstacles = [obstacles; x_right-.5, y_right-.5];
%             obstacles = [obstacles; x_right-.5, y_right+.5];
            %obstacles = [obstacles; x_right+(1), y_right];
            %obstacles = [obstacles; x_right+(-1), y_right];
            % Definir los desplazamientos relativos para los obstáculos
% Definir los desplazamientos relativos para los obstáculos

            
            % Generar los nuevos obstáculos
            new_obstacles = [x_right, y_right] + shifts;
            obstacles = [obstacles; new_obstacles];
            new_obstacles = [x_left, y_left] + shifts;
            
            % Agregar los nuevos obstáculos a la lista existente
            obstacles = [obstacles; new_obstacles];


        end
    end

    %% Marcar ocupación de los obstáculos en el mapa
    setOccupancy(map, obstacles, 1);



    %% Invertir el mapa directamente
    % Crear una matriz binaria del mapa actual
    occupancy_matrix = occupancyMatrix(map);

    % Invertir los valores (1 -> 0, 0 -> 1)
    inverted_occupancy_matrix = ~occupancy_matrix;

    % Crear un nuevo mapa con la matriz invertida
    map = robotics.BinaryOccupancyGrid(inverted_occupancy_matrix, cellSize);
    %map = robotics.BinaryOccupancyGrid(map, cellSize);

    
    % Crear una matriz binaria del mapa actual
    obstacle_matrix = occupancyMatrix(map);
% Obtener las dimensiones reales del mapa
[map_height_real, map_width_real] = size(obstacle_matrix);

% Inicializar cost_map con dimensiones reales
cost_map = zeros(map_height_real, map_width_real);

% Asignar costos basados en cercanía a obstáculos
for x = 1:map_width_real
    for y = 1:map_height_real
            if obstacle_matrix(y, x) == 0 % Solo para celdas libres
            % Delimitar la vecindad para cada capa
            x_min1 = max(x - 1, 1);   x_max1 = min(x + 1, map_width_real);
            y_min1 = max(y - 1, 1);   y_max1 = min(y + 1, map_height_real);

            x_min2 = max(x - 2, 1);   x_max2 = min(x + 2, map_width_real);
            y_min2 = max(y - 2, 1);   y_max2 = min(y + 2, map_height_real);

            x_min3 = max(x - 3, 1);   x_max3 = min(x + 3, map_width_real);
            y_min3 = max(y - 3, 1);   y_max3 = min(y + 3, map_height_real);

            x_min4 = max(x - 4, 1);   x_max4 = min(x + 4, map_width_real);
            y_min4 = max(y - 4, 1);   y_max4 = min(y + 4, map_height_real);

            x_min5 = max(x - 5, 1);   x_max5 = min(x + 5, map_width_real);
            y_min5 = max(y - 5, 1);   y_max5 = min(y + 5, map_height_real);

            x_min6 = max(x - 6, 1);   x_max6 = min(x + 6, map_width_real);
            y_min6 = max(y - 6, 1);   y_max6 = min(y + 6, map_height_real);

            x_min7 = max(x - 7, 1);   x_max7 = min(x + 7, map_width_real);
            y_min7 = max(y - 7, 1);   y_max7 = min(y + 7, map_height_real);

            x_min8 = max(x - 8, 1);   x_max8 = min(x + 8, map_width_real);
            y_min8 = max(y - 8, 1);   y_max8 = min(y + 8, map_height_real);

            x_min9 = max(x - 9, 1);   x_max9 = min(x + 9, map_width_real);
            y_min9 = max(y - 9, 1);   y_max9 = min(y + 9, map_height_real);

            

            % Capas de costo según vecindario
            if any(any(obstacle_matrix(y_min1:y_max1, x_min1:x_max1)))
                cost_map(y, x) = 10000;  % Capa 1 (adyacencia inmediata)
            elseif any(any(obstacle_matrix(y_min2:y_max2, x_min2:x_max2)))
                cost_map(y, x) = 8000;   % Capa 2
            elseif any(any(obstacle_matrix(y_min3:y_max3, x_min3:x_max3)))
                cost_map(y, x) = 6000;   % Capa 3
            elseif any(any(obstacle_matrix(y_min4:y_max4, x_min4:x_max4)))
                cost_map(y, x) = 4000;   % Capa 4
            elseif any(any(obstacle_matrix(y_min5:y_max5, x_min5:x_max5)))
                cost_map(y, x) = 2000;    % Capa 5
            elseif any(any(obstacle_matrix(y_min6:y_max6, x_min6:x_max6)))
                cost_map(y, x) = 1000;    % Capa 6
            elseif any(any(obstacle_matrix(y_min7:y_max7, x_min7:x_max7)))
                cost_map(y, x) = 500;    % Capa 7
            elseif any(any(obstacle_matrix(y_min8:y_max8, x_min8:x_max8)))
                cost_map(y, x) = 100;     % Capa 8
            elseif any(any(obstacle_matrix(y_min9:y_max9, x_min9:x_max9)))
                cost_map(y, x) = 10;     % Capa 9
            end
            end
    end
end



    %% Visualizaciones para depurar/checar
%     figure;
%     show(map);
%     title('Circuito invertido (ocupación invertida)');
% 
%     figure;
%     imagesc(cost_map');
%     axis xy; colorbar; title('Cost Map');
end


