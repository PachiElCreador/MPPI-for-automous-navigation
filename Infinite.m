
function [map, cost_map, obstacles] = Infinite()
close all;
    %% Parámetros del mapa
    map_width  = 100;  % Tamaño en X (en celdas)
    map_height = 100;  % Tamaño en Y (en celdas)
    cellSize   = 1;    % Tamaño de cada celda (en metros)

    %% Crear el mapa (BinaryOccupancyGrid de Robotics Toolbox)
    map = robotics.BinaryOccupancyGrid(map_width, map_height, cellSize);

    %% Construcción de obstáculos (4 cuadrados y bordes)
    obstacles = [];

    % 1) Añadir borde superior e inferior
    for x = 1:map_width
        obstacles = [obstacles; x, 1];       % Borde inferior
        obstacles = [obstacles; x, map_height]; % Borde superior
    end

    % 2) Añadir borde izquierdo y derecho
    for y = 1:map_height
        obstacles = [obstacles; 1, y];       % Borde izquierdo
        obstacles = [obstacles; map_width, y]; % Borde derecho
    end

    % 1) Primer cuadrado: (x=5..45, y=5..45)
    for x = 11:44
        for y = 11:44
            obstacles = [obstacles; x, y];
        end
    end

    % 2) Segundo cuadrado: (x=55..95, y=5..45)
    for x = 55:100
        for y = 0:45
            obstacles = [obstacles; x, y];
        end
    end

    % 3) Tercer cuadrado: (x=0..45, y=55..95)
    for x = 0:45
        for y = 55:100
            obstacles = [obstacles; x, y];
        end
    end

    % 4) Cuarto cuadrado: (x=55..90, y=55..95)
    for x = 56:89
        for y = 56:89
            obstacles = [obstacles; x, y];
        end
    end

   %% Suavizar inferior izquierda
    for x = 10:20
        for y = 10:20
            if y <= 10 - (x - 20)
                obstacles(obstacles(:, 1) == x & obstacles(:, 2) == y, :) = [];
                obstacles = [obstacles; x-9, y-9];
            end
        end
    end

   for x =10:20
        for y = 35:45
            if y >= 35 + (x - 10)
                obstacles(obstacles(:, 1) == x & obstacles(:, 2) == y, :) = [];
                obstacles = [obstacles; x-9, y+9];
            end
        end
    end

    for x = 35:45
        for y = 20:-1:10 % Decreciendo en y
            if y <= 10 + (x - 35) % Ajustar para apuntar al sureste hacia la derecha
                obstacles(obstacles(:, 1) == x & obstacles(:, 2) == y, :) = [];
                obstacles = [obstacles; x+9, y-9];
            end
        end
    end


   %% Suavizar superior derecha   
    for x = 80:90
        for y = 80:90
            if y >= 90 - (x - 80) % Ajustar la lógica para el nuevo rango
                obstacles(obstacles(:, 1) == x & obstacles(:, 2) == y, :) = [];
                obstacles = [obstacles; x+9, y+9];
            end
        end
    end

    for x = 55:65
        for y = 80:90
            if y >= 80 + (x - 55)
                obstacles(obstacles(:, 1) == x & obstacles(:, 2) == y, :) = [];
                obstacles = [obstacles; x-9, y+9];
            end
        end
    end
    
    for x = 80:90
        for y = 65:-1:55 % Decreciendo en y
            if y <= 55 + (x - 80) % Ajustar para apuntar al sureste hacia la derecha
                obstacles(obstacles(:, 1) == x & obstacles(:, 2) == y, :) = [];
                obstacles = [obstacles; x+9, y-9];
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

%     %% (Opcional) Visualizaciones para depurar/checar
%     figure;
%     show(map);
%     title('Circuito infinito');
%     
%     figure;
%     imagesc(cost_map');
%     axis xy; colorbar; title('Cost Map');
end

