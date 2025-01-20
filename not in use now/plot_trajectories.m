

function plot_trajectories(states_history,save_trajectory,map,S,u,timesteps,dT,system_dynamics)

figure(7)
hold off
show(map);
hold on
grid on

set(gca,'XTick',0:1:100,'YTick',0:1:100)


% Normalize cost for plotting the
colormap = winter;
colorbar
A = [1:size(colormap,1)/100:size(colormap,1)]';
B = [1:1:size(colormap,1)]';

newcolormap = interp1(B, colormap ,A);
S_norm = (S - min(S))/(max(S) - min(S)+0.000001);
[Svalue, minidx] = min(S_norm);
temperature_range = [0:1/98:1];
temperature = zeros(1, size(S_norm,2));
color = zeros(size(S_norm,2),3,1);

for i=1:1:size(S_norm,2)
    temperature(i) = find(temperature_range < round(S_norm(i),3)+0.01 & temperature_range > round(S_norm(i),3)-0.01,1);
    color(i,:,:) = newcolormap(temperature(i),:,:);
end


% Plot each trajectory with its respective color
for k=1:1:size(save_trajectory,3)
    plot(save_trajectory(:,1,k),save_trajectory(:,2,k),'Color', color(k,:,:))
end


x_ = states_history(end,1); y_ = states_history(end,2);

% Plots the velocity vector
r = states_history(end,3); % magnitude of the velocity vector
u_ = r * cos(states_history(end,5) - states_history(end,4)); % Direction here slip angle minus yaw angle
v_ = r * sin(states_history(end,5) - states_history(end,4));
h = quiver(x_,y_,u_,v_,'LineWidth',2,'MaxHeadSize',50,'Color','r');


% Plot the driven trajectory of the single track model until now
plot(states_history(:,1),states_history(:,2),'-b','DisplayName','Current Trajectory')

plot(save_trajectory(:,1,minidx),save_trajectory(:,2,minidx),'.k')

% Plot the current states given by the current input sequence
save_current_states = [];
x_open = states_history(end,:);
dW = zeros(1, size(states_history,2));
for j = 1:1:timesteps
    
    [ x_test_] = ode1(@(t,x)system_dynamics(t, x ,u(j,:),dW), [0 dT], x_open);
    save_current_states = [save_current_states; x_test_];
    x_open = x_test_(end,:);
    
end
plot(save_current_states(:,1),save_current_states(:,2),'.r')


% Draw thesingle track model as a car

l_f=1.19016; % front lenght
l_r=1.37484; % back lenght
l=l_f+l_r;  %total lenght
R=0.302;    % whell radius


car = plot([x_-l_r  x_+l_f  x_+l_f  x_-l_r  x_-l_r], [y_-0.6  y_-0.6  y_+0.6  y_+0.6  y_-0.6],'LineWidth',2);
rotate(car, [0 0 1], rad2deg(states_history(end,5)),[x_ y_ 0]); %Rotates the car yaw angle
wheel_1 = plot([x_+l_f-R  x_+l_f+R  x_+l_f+R  x_+l_f-R  x_+l_f-R], [y_+1.10/2  y_+1.10/2  y_+1.10/2-0.3  y_+1.10/2-0.3  y_+1.10/2],'LineWidth',2,'Color','r');
wheel_2 = plot([x_-l_r-R  x_-l_r+R  x_-l_r+R  x_-l_r-R  x_-l_r-R], [y_+1.10/2  y_+1.10/2  y_+1.10/2-0.3  y_+1.10/2-0.3  y_+1.10/2],'LineWidth',2,'Color','b');
wheel_3 = plot([x_+l_f-R  x_+l_f+R  x_+l_f+R  x_+l_f-R  x_+l_f-R], [y_-1.10/2  y_-1.10/2  y_-1.10/2+0.3  y_-1.10/2+0.3  y_-1.10/2],'LineWidth',2,'Color','r');
wheel_4 = plot([x_-l_r-R  x_-l_r+R  x_-l_r+R  x_-l_r-R  x_-l_r-R], [y_-1.10/2  y_-1.10/2  y_-1.10/2+0.3  y_-1.10/2+0.3  y_-1.10/2],'LineWidth',2,'Color','b');

% Rotates the front wheels with the yaw angle and the current input
% steering angle
rotate(wheel_1, [0 0 1], rad2deg(states_history(end,5)),[x_ y_ 0]);
rotate(wheel_2, [0 0 1], rad2deg(states_history(end,5)),[x_ y_ 0]);
rotate(wheel_1, [0 0 1], rad2deg(u(1,1)),[x_+l_f*cos(states_history(end,5)) y_+l_f*sin(states_history(end,5)) 0]);
rotate(wheel_3, [0 0 1], rad2deg(states_history(end,5)),[x_ y_ 0]);
rotate(wheel_4, [0 0 1], rad2deg(states_history(end,5)),[x_ y_ 0]);
rotate(wheel_3, [0 0 1], rad2deg(u(1,1)),[x_+l_f*cos(states_history(end,5)) y_+l_f*sin(states_history(end,5)) 0]);


text(x_ +2 ,y_ + 10,num2str(states_history(end,3)), 'Color','r');
drawnow

end

function [u, save_trajectory, S] = gettrajectories(samples, xmeasure, tmeasure, sigma, timesteps, alpha, u_UB, u_LB, u, ...
    system_dynamics, gamma, dT, x_desired, running_cost, terminal_cost, loop, lambda, map, cost_map)

% Initialize costs for each sample to zero
S = zeros(1, samples - 1);

% Generate samples
for k = 1:samples - 1
    % Save the initial state for the trajectory
    save_trajectory(1, :, k) = xmeasure(end, :);
    hit_obstacle = 0; % Flag to check if an obstacle is hit

    % Initialize state and time for the trajectory
    x_0 = xmeasure(end, :);
    t_0 = tmeasure(end, :);

% Generate noise for the control inputs
noise = zeros(size(u, 2), timesteps, samples - 1);
for k = 1:samples - 1
    for i = 1:size(u, 2)
        noise(i, :, k) = normrnd(0, sigma(i), [1, timesteps]);
    end
end



    % Simulate the system for the given horizon
    for t = 2:timesteps
        % Apply control with noise
        if k < (1 - alpha) * samples
            v = u(t - 1, :) + noise(:, t - 1)';
        else
            v = noise(:, t - 1)';
        end

        % Enforce control bounds
        v = max(min(v, u_UB), u_LB);

        % Simulate system dynamics
        dW = zeros(1, size(x_0, 2)); % No system noise during sampling
        x_ = ode1(@(t, x) system_dynamics(t, x, v, dW), [t_0 t_0 + dT], x_0);
        t_0 = t_0 + dT;

        if isreal(x_)
            % Ensure the state remains within map boundaries
            x_(:, 1) = max(min(x_(:, 1), 100), 0);
            x_(:, 2) = max(min(x_(:, 2), 100), 0);

            % Check for obstacles in the trajectory
            if ~hit_obstacle
                for i = 1:size(x_, 1)
                    hit_obstacle = getOccupancy(map, [x_(i, 1), x_(i, 2)]);
                    x_0 = x_(i, :);
                    if hit_obstacle
                        break;
                    end
                end
            end

            % Compute grid square cost
            grid_idx = world2grid(map, [x_(end, 1), x_(end, 2)]);
            cost_grid_square = cost_map(grid_idx(1), grid_idx(2));
        else
            hit_obstacle = 1;
        end

        % Save the trajectory and calculate costs
        save_trajectory(t, :, k) = x_0;
        S(k) = S(k) + running_cost(x_0, x_desired) + cost_grid_square + gamma * (v * diag(1 ./ sigma) * v') + hit_obstacle * 1e12;
    end

    % Add terminal cost
    S(k) = S(k) + terminal_cost(x_0, x_desired);

    % Ensure costs are finite
    if isinf(S(k))
        S(k) = 1e30;
    end

    % Display information for debugging
    disp(['Sample: ', num2str(k), ', Iteration: ', num2str(loop), ', Cost: ', num2str(S(k))]);
end

% Calculate weights for control distribution
weights = calculate_weights(S, lambda, samples);

% Update control inputs based on weighted noise
weights_vector = zeros(timesteps, size(u, 2));
for t = 1:timesteps
    for k = 1:samples - 1
        weights_vector(t, :) = weights_vector(t, :) + weights(k) * noise(:, t, k)';
    end
    u(t, :) = u(t, :) + weights_vector(t, :);

    % Enforce control bounds
    u(t, :) = max(min(u(t, :), u_UB), u_LB);
end

end

