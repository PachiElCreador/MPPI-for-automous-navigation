%% Model Predictive Path Integral Control
% Implements Model Predictive Path Integral Control for a Single Track
% model navigating a grid map at high velocity.

function [tmeasure, states_history, u_opt] = MPPI_nav_controller(system_dynamics, param, running_cost, terminal_cost)

%% Creates the map and a matrix with the cost of each grid quare
%[map, cost_map, obstacles] = createMap();
%[map, cost_map, obstacles] = map_creation()
%[map, cost_map, obstacles] = hexagonal_map();
%[map, cost_map] = dynamic_map(); %uncomment for static obstacle line map
%[map, cost_map] = laberintoS();
%[map, cost_map] = LaberintoS_2();
%[map, cost_map, obstacles] = Infinite()
[map, cost_map, obstacles] = Oval()
%[map, cost_map, obstacles] = Nascar()

%%
% Opt Hyperparameters
timesteps = param.timesteps; %Horizon(timesteps)
samples = param.samples;  %N samples
iterations = param.iterations; % N iterations
xmeasure = param.xmeasure;    %Initial conditions
x_desired = param.x_desired; % desired state
tmeasure = param.tmeasure;    % Initial time
dT = param.dT;    % timestep
u = param.u; % Initial input sequence
u_UB = param.u_UB; %Control upper bound
u_LB = param.u_LB; %Control lower bound
system_noise = param.system_noise; %Noise of the system (dW)

% Control hyperparameters
lambda = param.lambda; %Inverse temperature
sigma = param.sigma; %Covariance
gamma = param.gamma; % Control cost parameter
alpha = param.alpha; % Base distribution parameters (0 < alpha < 1)


% Initialize matrices for storing simulation data
% Records the state trajectory of the system at each timestep
states_history = [];
% Stores the optimal control inputs computed during each iteration
u_opt = [];
% Keeps track of the system noise applied at each step
dW_history = [];
% Saves all sampled trajectories during the simulation
trajectory_history = [];
% Logs the costs associated with each sampled trajectory
S_save = [];



%%
% Inicializa el objeto VideoWriter
videoFile = VideoWriter('video.mp4', 'MPEG-4');
videoFile.FrameRate = 10; % frames
open(videoFile);
%%

% Save initial state in its historz matrix
states_history = [states_history; xmeasure];

% Begin MPC iterations
for loop = 0:1:iterations
    if mod(loop, 5) == 0
        %[map, cost_map] = dynamic_map();
        %[map, cost_map] = laberintoS();
        %[map, cost_map] = LaberintoS_2();
    end
  
    
    
    
    % Perform forward sampling to generate control trajectories and costs
    [u, save_trajectory, S] = gettrajectories(samples, xmeasure, tmeasure, sigma, timesteps, alpha, u_UB, u_LB, u, ...
        system_dynamics, gamma, dT, x_desired, running_cost, terminal_cost, loop, lambda, map, cost_map);
    
    % Store the first optimal control input for this iteration
    u_opt = [u_opt; u(1,:)];
    
    % Optionally add noise to the first control input 
    % u(1,:) = u(1,:) + normrnd(0, sigma, [1, 1]);
    
    % Generate system noise for all state dimensions
    dW = arrayfun(@(s) sqrt(dT) * normrnd(0, system_noise(s)), 1:size(xmeasure, 2));
    
    % Solve the system dynamics for the next state
    next_state = ode1(@(t, x) system_dynamics(t, x, u(1,:), dW), [tmeasure tmeasure + dT], xmeasure);
    
    % Update the current time step
    tmeasure = tmeasure + dT;

    
    
    % Check if an obstacle was hit during the current step
    hit_obstacle = 0; 
    if hit_obstacle == 0
        for i = 1:size(next_state, 1)
            % Determine if the current position intersects an obstacle
            hit_obstacle = getOccupancy(map, [next_state(i, 1), next_state(i, 2)]);
            
            if hit_obstacle == 1
                % Update the state to the point just before hitting the obstacle
                xmeasure(:, state) = next_state(1, state);
                break;
            end
            
            % Update the state trajectory with the next state values
            for state = 1:size(next_state, 2)
                xmeasure(:, state) = next_state(i, state);
            end
        end
    end

    
    % Record relevant information into history matrices
    dW_history = [dW_history; dW]; % Append the system noise history
    states_history = [states_history; xmeasure]; % Append the current state to the state history
    trajectory_history(:,:,:,loop+1) = save_trajectory; % Save the sampled trajectories for this iteration
    S_save = [S_save; S]; % Append the trajectory costs to the cost history
    
    % Shift the input sequence to the left, propagating the control inputs
    for t = 1:timesteps-1
        u(t, :) = u(t + 1, :); % Shift the control input at each timestep
    end
    
    % Initialize the last control input with zeros (optional)
    u(timesteps, :) = [0 0 0 0]; % Set the last control input to a default value
    
    % Plot 

    % Actualiza la gráfica
    figure(7); 

    % Captura el frame de la figura
    frame = getframe(gcf); % Captura la figura actual
    writeVideo(videoFile, frame); % Escribe el frame al archivo

    plot_trajectories(states_history,save_trajectory,map,S,u,timesteps,dT,system_dynamics);
    
end
% VideoWriter
close(videoFile);
disp('Video de la simulación guardado como simulation_output.mp4');


end


function weights = calculate_weights(S, lambda, samples)
% Calculate importance sampling weights
% - S: Costs associated with each trajectory sample
% - lambda: Inverse temperature parameter
% - samples: Total number of trajectory samples

% - weights: Importance sampling weights for each trajectory

% Initialize normalizing factor and weights
n = 0;
weights = zeros(1, samples);

% Compute the normalization factor using the minimum cost as a baseline
beta = min(S);
for k = 1:samples-1
    n = n + exp(-(1 / lambda) * (S(k) - beta));
end

% Calculate the weight for each sample
for k = 1:samples-1
    weights(k) = exp(-(1 / lambda) * (S(k) - beta)) / n;
end

end


function [u, save_trajectory, S] = gettrajectories(samples,xmeasure,tmeasure,sigma,timesteps,alpha,u_UB,u_LB,u...
    ,system_dynamics,gamma,dT,x_desired,running_cost,terminal_cost,loop,lambda,map,cost_map)

% Initialize the costs to zero
S = zeros(1, samples-1);

% Loop over each sample to calculate trajectories and costs
for k = 1:1:samples-1
    
    save_trajectory(1,:,k) = xmeasure(end,:); % Save initial state of the trajectory
    hit_obstacle = 0; % Flag to check if the model hits an obstacle
    
    % Set the initial state for every trajectory
    x_0 = xmeasure(end,:);
    t_0 = tmeasure(end,:);
    
    % Generate Gaussian noise for control inputs
    for i=1:1:size(u,2)
        noise(i,:,k) = normrnd(0,sigma(i),[1,timesteps]);
    end
    
    % Simulate trajectory over the defined time horizon
    for t = 2:1:timesteps
        
        % Apply control inputs with noise
        if k < (1 - alpha)*samples
            v(t-1,:) = u(t-1,:) + noise(:,t-1,k)';
        else
            % Use only noise for some samples
            v(t-1,:) = noise(:,t-1,k)';
        end
        
        % Ensure control inputs stay within bounds
        for i=1:1:size(u,2)
            if v(t-1,i) > u_UB(i)
                v(t-1,i) = u_UB(i);
            elseif v(t-1,i) < u_LB(i)
                v(t-1,i) = u_LB(i);
            end
        end

        % Integrate system dynamics
        l = t-1; % Current time step index
        dW = zeros(1, size(x_0,2)); % No system noise in sampling
        [x_] = ode1(@(t,x)system_dynamics(t, x ,v(l,:),dW),[t_0 t_0+dT], x_0); % Fast ODE solver
        t_0 = t_0 + dT; % Update time
        
        if isreal(x_)
            % Ensure state stays within map boundaries
            x_0_(:,1) = x_(:,1); % Temporary variables for position
            x_0_(:,2) = x_(:,2);
            
            % Clip positions to map bounds
            x_0_(:,1) = min(max(x_0_(:,1), 0), 100);
            x_0_(:,2) = min(max(x_0_(:,2), 0), 100);
            
            % Check if trajectory hits an obstacle
            if hit_obstacle == 0
                for i = 1:1:size(x_0_,1)
                    hit_obstacle = getOccupancy(map,[x_0_(i,1),x_0_(i,2)]);
                    x_0 = x_(i,:); % Update state
                    if hit_obstacle
                        break;
                    end
                end
            end
            
            % Calculate cost of the current grid square
            grid_idx = world2grid(map,[x_0_(end,1) x_0_(end,2)]);
            cost_grid_square = cost_map(grid_idx(1),grid_idx(2));
        else
            hit_obstacle = 1; % Mark trajectory as invalid
        end
        
        % Save the current state of the trajectory
        save_trajectory(t,:,k) = x_0;
        
        % Compute running cost
        cost = running_cost(x_0,x_desired);
        S(k) = S(k) + cost + cost_grid_square + gamma * u(t-1,:) * inv(diag(ones(1, size(u,2))) .* sigma) * v(t-1,:)' + hit_obstacle * 1e11;
    end
    
    % Add terminal cost
    S(k) = S(k) + terminal_cost(x_0,x_desired);
    if isinf(S(k))
        S(k) = 1e30; % Assign a very large cost to invalid trajectories
    end
    
    % Display iteration progress
    disp(['Smp: ', num2str(k), ' itr: ', num2str(loop), ' Sample Cost: ' num2str(S(k))]);
end

% Calculate importance weights for trajectories
weights = calculate_weights(S,lambda,samples);

% Adjust control inputs based on weighted noise
for t=1:1:timesteps
    weights_sum = zeros(1,size(u,2));
    for k=1:1:samples-1
        weights_sum = weights_sum + (weights(k) * noise(:,t,k))';
    end
    weights_vector(t,:) = weights_sum;
end

for t=1:1:timesteps
    % Update control inputs
    u(t,:) = u(t,:) + weights_vector(t,:);
    
    % Ensure control inputs satisfy constraints
    for i = 1:1:size(u,2)
        if u(t,i) < u_LB(i)
            u(t,i) = u_LB(i);
        elseif u(t,i) > u_UB(i)
            u(t,i) = u_UB(i);
        end
    end
end

end

function plot_trajectories(states_history, save_trajectory, map, S, u, timesteps, dT, system_dynamics)
    % Configura la figura y el mapa
    figure(7);
    hold off;
    show(map);
    hold on;
    grid on;
    set(gca, 'XTick', 0:1:100, 'YTick', 0:1:100);

    % Normaliza el costo S para asignar colores a las trayectorias
    cmap = winter;
    colorbar;
    norm_S = (S - min(S)) / (max(S) - min(S) + 1e-6); % Normalización con margen para evitar división por cero
    [~, minidx] = min(norm_S); % Índice de la trayectoria con menor costo
    colors = interp1(1:size(cmap, 1), cmap, linspace(1, size(cmap, 1), 100));

    % Asocia colores a cada trayectoria según el costo normalizado
    trajectory_colors = colors(round(norm_S * 99) + 1, :);

    % Traza cada trayectoria con su color asignado
    for k = 1:size(save_trajectory, 3)
        plot(save_trajectory(:, 1, k), save_trajectory(:, 2, k), 'Color', trajectory_colors(k, :));
    end

    % Obtiene la posición y velocidad actuales
    x_ = states_history(end, 1);
    y_ = states_history(end, 2);
    r = states_history(end, 3); % Magnitud de la velocidad
    u_ = r * cos(states_history(end, 5) - states_history(end, 4)); % Componente x de la velocidad
    v_ = r * sin(states_history(end, 5) - states_history(end, 4)); % Componente y de la velocidad

    % Dibuja el vector de velocidad
    quiver(x_, y_, u_, v_, 'LineWidth', 2, 'MaxHeadSize', 50, 'Color', 'r');

    % Traza la trayectoria recorrida hasta el momento
    plot(states_history(:, 1), states_history(:, 2), '-b', 'DisplayName', 'Current Trajectory');

    % Traza la trayectoria de menor costo
    plot(save_trajectory(:, 1, minidx), save_trajectory(:, 2, minidx), '.k');

    % Predice estados futuros usando la dinámica del sistema
    save_current_states = [];
    x_open = states_history(end, :);
    for j = 1:timesteps
        [x_next] = ode1(@(t, x) system_dynamics(t, x, u(j, :), zeros(1, size(states_history, 2))), [0 dT], x_open);
        save_current_states = [save_current_states; x_next];
        x_open = x_next(end, :);
    end
    plot(save_current_states(:, 1), save_current_states(:, 2), '.r');

    % Dibuja el modelo simplificado de un vehículo
    draw_vehicle(x_, y_, states_history(end, 5), u(1, 1));

    % Muestra la velocidad actual como texto
    text(x_ + 2, y_ + 10, num2str(states_history(end, 3)), 'Color', 'r');
    drawnow;
end

function draw_vehicle(x, y, yaw_angle, steering_angle)
    % Parámetros del vehículo
    l_f = 1.19; % Distancia al frente
    l_r = 1.37; % Distancia trasera
    R = 0.302; % Radio de las ruedas

    % Dibuja el cuerpo del vehículo
    car_outline = plot([x - l_r, x + l_f, x + l_f, x - l_r, x - l_r], ...
                       [y - 0.6, y - 0.6, y + 0.6, y + 0.6, y - 0.6], 'LineWidth', 2);
    rotate(car_outline, [0 0 1], rad2deg(yaw_angle), [x y 0]);

    % Dibuja las ruedas
    wheel_offsets = [l_f, -l_r];
    colors = ['r', 'b'];
    for i = 1:2
        for j = [-1, 1] % Superior e inferior
            wheel = plot([x + wheel_offsets(i) - R, x + wheel_offsets(i) + R, ...
                          x + wheel_offsets(i) + R, x + wheel_offsets(i) - R, ...
                          x + wheel_offsets(i) - R], ...
                         [y + j * 0.55, y + j * 0.55, y + j * 0.25, y + j * 0.25, y + j * 0.55], ...
                         'LineWidth', 2, 'Color', colors(i));
            rotate(wheel, [0 0 1], rad2deg(yaw_angle), [x y 0]);
            if i == 1 % Ruedas delanteras con ángulo de dirección
                rotate(wheel, [0 0 1], rad2deg(steering_angle), ...
                       [x + l_f * cos(yaw_angle), y + l_f * sin(yaw_angle), 0]);
            end
        end
    end
end

