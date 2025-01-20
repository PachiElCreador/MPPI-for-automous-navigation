function [map, costMap, obstacles] = createMap() 
    %% Parámetros del mapa
    gridSizeX = 100;     % Tamaño en X (en celdas)
    gridSizeY = 100;     % Tamaño en Y (en celdas)
    cellSize  = 1;       % Tamaño de cada celda (en metros)

    %% Crear el mapa (BinaryOccupancyGrid de Robotics Toolbox)
    map = robotics.BinaryOccupancyGrid(gridSizeX, gridSizeY, cellSize);

    %% Construcción de obstáculos
    obstacles = [];
    for x = 0:gridSizeX
        for y = 0:gridSizeY
            % Borde externo
            if x == 0 || x == gridSizeX || y == 0 || y == gridSizeY
                obstacles = [obstacles; x y];  
            end
            % Cuadrado interno
            if x > 12 && x <= 88 && y > 12 && y <= 88
                obstacles = [obstacles; x y];  
            end
        end
    end

    % Marcar ocupación de los obstáculos en el mapa
    setOccupancy(map, obstacles, 1);

%     %% Mostrar mapa
%     figure;
%     show(map);
%     grid on;
%     set(gca, 'XTick', 0:2:gridSizeX, 'YTick', 0:2:gridSizeY);

    %% Inicializar el costMap
    costMap = zeros(gridSizeX, gridSizeY);

    %% Calcular los costos basados en adyacencia a obstáculos
    % Generar una matriz binaria para los obstáculos
    obstacleMatrix = getOccupancy(map);

    % Determinar celdas adyacentes a los obstáculos
    for x = 2:gridSizeX-1
        for y = 2:gridSizeY-1
            % Verificar si la celda actual es adyacente a un obstáculo
            if obstacleMatrix(x, y) == 0 % Solo celdas libres
                % Verificar vecinos
                if any(any(obstacleMatrix(x-1:x+1, y-1:y+1))) % Si algún vecino es un obstáculo
                    costMap(x, y) = 10000; % Adyacente inmediato
                elseif any(any(obstacleMatrix(x-2:x+2, y-2:y+2))) % Segundo nivel de vecinos
                    costMap(x, y) = 1000; % Segunda capa
                elseif any(any(obstacleMatrix(x-3:x+3, y-3:y+3))) % Tercer nivel de vecinos
                    costMap(x, y) = 100; % Tercera capa
                elseif any(any(obstacleMatrix(x-4:x+4, y-4:y+4))) % Tercer nivel de vecinos
                    costMap(x, y) = 10; % Tercera capa
                end
            end
        end
    end

%     %%Opcional: visualización del costMap
%     figure;
%     imagesc(costMap);
%     set(gca, 'YDir', 'normal');
%     colorbar;
%     title('Cost Map');
%     xlabel('X [grid units]');
%     ylabel('Y [grid units]');
%     set(gca, 'XTick', 0:2:gridSizeX, 'YTick', 0:2:gridSizeY);
end

