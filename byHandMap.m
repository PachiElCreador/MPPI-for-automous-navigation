function [map, costMap, obstacles] = createMap()
    %% Parámetros del mapa
    gridSizeX  = 100;     % Tamaño en X (en celdas)
    gridSizeY  = 100;     % Tamaño en Y (en celdas)
    cellSize   = 1;       % Tamaño de cada celda (en metros)

    %% Crear el mapa (BinaryOccupancyGrid de Robotics Toolbox)
    map = robotics.BinaryOccupancyGrid(gridSizeX, gridSizeY, cellSize);

    %% Construcción de obstáculos
    obstacles = [];
    for x = 0 : gridSizeX
        for y = 0 : gridSizeY
            % Borde externo
            if x == 0 || x == gridSizeX || y == 0 || y == gridSizeY
                obstacles = [obstacles; x y];  
            end
            % “Cuadro” interno
            if x > 12 && x <= 88 && y > 12 && y <= 88
                obstacles = [obstacles; x y];  
            end
        end
    end

    % Marcar ocupación de los obstáculos en el mapa
    setOccupancy(map, obstacles, 1);

    %% Mostrar mapa
    figure;
    show(map);
    grid on;
    set(gca, ...
        'XTick', 0:2:gridSizeX, ...
        'YTick', 0:2:gridSizeY);

    %% Inicializar el costMap
    costMap = zeros(gridSizeX, gridSizeY);

    %% Rellenar el costMap
    % Aquí definimos los costos para diversas líneas 
    for x = 1 : gridSizeX
        for y = 1 : gridSizeY
            
            % Zonas de alto costo 
            if (y == 12 || x == 12 || y == 89 || x == 89 || y == 2  || ...
                x == 2  || x == 99)                             
                costMap(x, y) = 10000; 
            elseif (y == 11 || x == 11 || y == 98 || x == 98 || ...
                    y == 3  || x == 3)
                costMap(x, y) = 1000;
            elseif (y == 10 || x == 10 || y == 97 || x == 97 || ...
                    y == 4  || x == 4)
                costMap(x, y) = 100;
            elseif (y == 9  || x == 9  || y == 96 || x == 96 || ...
                    y == 5)
                costMap(x, y) = 10;
            elseif (y == 8  || x == 8  || y == 93 || x == 93)
                costMap(x, y) = 5;
            elseif (y == 7  || x == 7  || y == 94 || x == 94)
                costMap(x, y) = 0;
            end

            % Líneas adicionales con costo medio/bajo
            if (x == 6 && y >= 6 && y <= 95) || (x == 95 && y >= 6 && y <= 95) ...
               || (y == 6 && x > 6  && x < 96) || (y == 95 && x > 5  && x < 96)
                costMap(x, y) = 5;
            end
        end
    end

    %% Ajustes en “esquinas” u otras celdas puntuales
    % Aquí se conservan las asignaciones que ‘sobreescriben’ celdas específicas:
    cornerAdjustments = [
        % Esquina superior izquierda
        8 8 0;    9 9 0;    10 10 0;   9 8 0;    10 8 0;   10 9 0; ...
        11 9 0;   11 8 0;   8 9 0;     8 10 0;   8 11 0;   8 12 0; ...
        8 13 0;   9 10 0;   9 11 0;    9 12 0;   12 9 0;   12 8 0; ...
        13 9 0;   13 8 0;   8 14 0;    8 15 0;   8 16 0;   9 13 0; ...
        9 14 0;   7 7 5;    6 6 5;     7 8 5;    8 7 5;    12 8 0; ...
        13 8 0;   14 8 0;   15 8 0;    16 8 0;   12 9 0;   13 9 0; 14 9 0;
        
        % Esquina inferior derecha
        94 94 0;  93 93 0;  92 92 0;   93 94 0;   92 94 0;   92 93 0; ...
        91 93 0;  91 94 0;  94 93 0;   94 92 0;   94 91 0;   94 90 0; ...
        94 89 0;  93 92 0;  93 91 0;   93 90 0;   90 93 0;   90 94 0; ...
        89 93 0;  89 94 0;  94 88 0;   94 87 0;   94 86 0;   93 89 0; ...
        93 88 0;  94 94 5;  96 96 5;   93 94 0;   94 93 0;   93 88 0; ...
        90 94 0;  89 94 0;  88 94 0;   87 94 0;   86 94 0;   90 93 0; ...
        89 93 0;  88 93 0;

        % Esquina inferior izquierda
        94 8 0;   93 9 0;   92 10 0;   93 8 0;    92 8 0;    92 9 0; ...
        91 9 0;   91 8 0;   94 9 0;    94 10 0;   94 11 0;   94 12 0; ...
        94 13 0;  93 10 0;  93 11 0;   93 12 0;   90 9 0;    90 8 0; ...
        89 9 0;   89 8 0;   94 14 0;   94 15 0;   94 16 0;   93 13 0; ...
        93 14 0;  95 7 5;   96 6 5;    95 8 5;    94 7 5;    93 14 0; ...
        90 8 0;   89 8 0;   88 8 0;    87 8 0;    86 8 0;    90 9 0;  ...
        89 9 0;   88 9 0;   91 92 0;   90 92 5;   90 93 0;
        
        % Esquina superior derecha
        8 94 0;   9 93 0;   10 92 0;   9 94 0;    10 94 0;   10 93 0; ...
        11 93 0;  11 94 0;  8 93 0;    8 92 0;    8 91 0;    8 90 0; ...
        8 89 0;   9 92 0;   9 91 0;    9 90 0;    12 93 0;   12 94 0; ...
        13 93 0;  13 94 0;  8 88 0;    8 87 0;    8 86 0;    9 89 0; ...
        9 88 0;   7 95 5;   6 96 5;    7 94 5;    8 95 5;    9 88 0; ...
        12 94 0;  13 94 0;  14 94 0;   15 94 0;   16 94 0;   12 93 0; ...
        13 93 0;  14 93 0;  9 87 0;    8 85 0;    10 91 0;   11 92 0; ...
        12 91 0;  13 91 0;
    ];

    % Aplicar ajustes (sobreescritura) en costMap
    for i = 1 : size(cornerAdjustments, 1)
        x = cornerAdjustments(i, 1);
        y = cornerAdjustments(i, 2);
        c = cornerAdjustments(i, 3);
        
        % Validar que (x, y) esté dentro de los límites
        if x >= 1 && x <= gridSizeX && y >= 1 && y <= gridSizeY
            costMap(x, y) = c;
        end
    end

    % Opcional: visualización del costMap
    {
    figure;
    imagesc(costMap);
    set(gca, 'YDir', 'normal');
    colorbar;
    title('Cost Map');
    xlabel('X [grid units]');
    ylabel('Y [grid units]');
    set(gca, 'XTick', 0:2:gridSizeX, 'YTick', 0:2:gridSizeY);
    }

end
