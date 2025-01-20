% % function [tmeasure, states_history, u_opt] = SBMPC_target(system_dynamics, param, running_cost, terminal_cost)
% % % Optimization hyperparameters
% % timesteps = param.timesteps;
% % samples = param.samples;
% % iterations = param.iterations;
% % xmeasure = param.xmeasure;
% % x_desired = param.x_desired;
% % tmeasure = param.tmeasure;
% % dT = param.dT;
% % u = param.u;
% % u_UB = param.u_UB;
% % u_LB = param.u_LB;
% % system_noise = param.system_noise;
% % 
% % % Control hyperparameters
% % lambda = param.lambda;
% % sigma = param.sigma;
% % gamma = param.gamma;
% % alpha = param.alpha;
% % 
% % % Initialize matrices
% % states_history = [];
% % u_opt = [];
% % dW_history = [];
% % trajectory_history = [];
% % S_save = [];
% % 
% % % Create map using the hexagonal map function
% % [map, cost_map, obstacles] = map_creation();
% % 
% % % Save initial state
% % states_history = [states_history; xmeasure];
% % 
% % % Begin MPC iterations
% % for loop = 0:1:iterations
% %     hit_obstacle = 0;
% %     
% %     % Get trajectories using modified sampling that considers hexagonal boundaries
% %     [u, save_trajectory, S] = gettrajectories_hex(samples, xmeasure, tmeasure, sigma, timesteps, alpha, u_UB, u_LB, u, ...
% %         system_dynamics, gamma, dT, x_desired, running_cost, terminal_cost, loop, lambda, map, cost_map);
% %     
% %     % Save optimal input
% %     u_opt = [u_opt; u(1,:)];
% %     
% %     % Add system noise
% %     for state=1:1:size(xmeasure,2)
% %         dW(state) = sqrt(dT)*normrnd(0,system_noise(state),[1,1]);
% %     end
% %     
% %     % Solver step
% %     [next_state] = ode1(@(t,x)system_dynamics(t, x, u(1,:), dW), [tmeasure tmeasure+dT], xmeasure);
% %     tmeasure = tmeasure + dT;
% %     
% %     % Check collision with hexagonal boundary and obstacles
% %     if hit_obstacle == 0
% %         for i = 1:1:size(next_state,1)
% %             hit_obstacle = getOccupancy(map,[next_state(i,1),next_state(i,2)]);
% %             if hit_obstacle==1
% %                 xmeasure(:,state) = next_state(1,state);
% %                 break;
% %             end
% %             
% %             for state=1:1:size(next_state,2)
% %                 xmeasure(:,state) = next_state(i,state);
% %             end
% %         end
% %     end
% %     
% %     % Save history
% %     states_history = [states_history; xmeasure];
% %     trajectory_history(:,:,:,loop+1) = save_trajectory;
% %     S_save = [S_save;S];
% %     
% %     % Shift control sequence
% %     for t = 1:1:timesteps-1
% %         u(t,:) = u(t+1,:);
% %     end
% %     u(timesteps,:) = [0 0 0 0];
% %     
% %     % Plot current state with modified visualization
% %     plot_hex_trajectories(states_history, save_trajectory, map, S, u, timesteps, dT, system_dynamics);
% % end
% % end
% % 
% % function [u, save_trajectory, S] = gettrajectories_hex(samples, xmeasure, tmeasure, sigma, timesteps, alpha, u_UB, u_LB, u, ...
% %     system_dynamics, gamma, dT, x_desired, running_cost, terminal_cost, loop, lambda, map, cost_map)
% % 
% % % Initialize costs
% % S = zeros(1, samples-1);
% % 
% % % Sampling with hexagonal boundary consideration
% % for k = 1:1:samples-1
% %     save_trajectory(1,:,k) = xmeasure(end,:);
% %     hit_obstacle = 0;
% %     x_0 = xmeasure(end,:);
% %     t_0 = tmeasure(end,:);
% %     
% %     % Generate noise for control inputs
% %     for i=1:1:size(u,2)
% %         noise(i,:,k) = normrnd(0,sigma(i),[1,timesteps]);
% %     end
% %     
% %     % Apply stochastic control for horizon
% %     for t = 2:1:timesteps
% %         if k < (1 - alpha)*samples
% %             v(t-1,:) = u(t-1,:) + noise(:,t-1,k)';
% %         else
% %             v(t-1,:) = noise(:,t-1,k)';
% %         end
% %         
% %         % Apply control bounds
% %         for i=1:1:size(u,2)
% %             v(t-1,i) = min(max(v(t-1,i), u_LB(i)), u_UB(i));
% %         end
% %         
% %         l = t-1;
% %         dW = zeros(1, size(x_0,2));
% %         [x_] = ode1(@(t,x)system_dynamics(t, x, v(l,:), dW), [t_0 t_0+dT], x_0);
% %         t_0 = t_0+dT;
% %         
% %         if isreal(x_)
% %             % Check and correct position within map boundaries
% %             x_0_(:,1) = x_(:,1);
% %             x_0_(:,2) = x_(:,2);
% %             
% %             % Boundary checking
% %             for i = 1:1:size(x_0_,1)
% %                 x_0_(i,1) = min(max(x_0_(i,1), 0), 100);
% %                 x_0_(i,2) = min(max(x_0_(i,2), 0), 100);
% %             end
% %             
% %             % Check collision with obstacles and hex boundary
% %             if hit_obstacle == 0
% %                 for i = 1:1:size(x_0_,1)
% %                     hit_obstacle = getOccupancy(map,[x_0_(i,1),x_0_(i,2)]);
% %                     x_0 = x_(i,:);
% %                     if hit_obstacle
% %                         break;
% %                     end
% %                 end
% %             end
% %             
% %             % Get cost from cost map
% %             grid_idx = world2grid(map,[x_0_(end,1) x_0_(end,2)]);
% %             cost_grid_square = cost_map(grid_idx(1),grid_idx(2));
% %             
% %         else
% %             hit_obstacle = 1;
% %         end
% %         
% %         save_trajectory(t,:,k) = x_0;
% %         
% %         % Calculate total cost with hexagonal boundary consideration
% %         [cost] = running_cost(x_0,x_desired);
% %         S(k) = S(k) + cost + cost_grid_square + gamma*u(t-1,:)*inv(diag(ones(1, size(u,2))).*sigma)*v(t-1,:)' + hit_obstacle*1e8;
% %     end
% %     
% %     % Terminal cost
% %     S(k) = S(k) + terminal_cost(x_0,x_desired);
% %     if isinf(S(k))
% %         S(k) = 1e30;
% %     end
% %     
% %     disp(['Sample: ', num2str(k), ' Iteration: ', num2str(loop), ' Cost: ' num2str(S(k))]);
% % end
% % 
% % % Calculate weights and update control
% % weights = calculate_weights(S,lambda,samples);
% % 
% % for t=1:1:timesteps
% %     weights_sum = zeros(1,size(u,2));
% %     for k=1:1:samples-1
% %         weights_sum = weights_sum + (weights(k)*noise(:,t,k))';
% %     end
% %     weights_vector(t,:) = weights_sum;
% % end
% % 
% % % Update control sequence
% % for t=1:1:timesteps
% %     u(t,:) = u(t,:) + weights_vector(t,:);
% %     for i = 1:1:size(u,2)
% %         u(t,i) = min(max(u(t,i), u_LB(i)), u_UB(i));
% %     end
% % end
% % end
% % 
% % function plot_hex_trajectories(states_history, save_trajectory, map, S, u, timesteps, dT, system_dynamics)
% % figure(7)
% % hold off
% % show(map)
% % hold on
% % grid on
% % 
% % % Plot sampled trajectories
% % colormap = winter;
% % A = [1:size(colormap,1)/100:size(colormap,1)]';
% % B = [1:1:size(colormap,1)]';
% % newcolormap = interp1(B, colormap, A);
% % S_norm = (S - min(S))/(max(S) - min(S) + 1e-6);
% % [~, minidx] = min(S_norm);
% % 
% % temperature_range = [0:1/98:1];
% % temperature = zeros(1, size(S_norm,2));
% % color = zeros(size(S_norm,2),3,1);
% % 
% % for i=1:1:size(S_norm,2)
% %     temperature(i) = find(temperature_range < round(S_norm(i),3)+0.01 & temperature_range > round(S_norm(i),3)-0.01,1);
% %     color(i,:,:) = newcolormap(temperature(i),:,:);
% % end
% % 
% % % Plot trajectories with costs
% % for k=1:1:size(save_trajectory,3)
% %     plot(save_trajectory(:,1,k),save_trajectory(:,2,k),'Color', color(k,:,:))
% % end
% % 
% % % Plot current state and velocity vector
% % x_ = states_history(end,1); 
% % y_ = states_history(end,2);
% % r = states_history(end,3);
% % u_ = r * cos(states_history(end,5) - states_history(end,4));
% % v_ = r * sin(states_history(end,5) - states_history(end,4));
% % quiver(x_,y_,u_,v_,'LineWidth',2,'MaxHeadSize',50,'Color','r');
% % 
% % % Plot vehicle trajectory
% % plot(states_history(:,1),states_history(:,2),'-b','LineWidth',2)
% % plot(save_trajectory(:,1,minidx),save_trajectory(:,2,minidx),'.k')
% % 
% % % Draw vehicle representation
% % l_f=1.19016;
% % l_r=1.37484;
% % l=l_f+l_r;
% % R=0.302;
% % 
% % car = plot([x_-l_r x_+l_f x_+l_f x_-l_r x_-l_r], ...
% %            [y_-0.6 y_-0.6 y_+0.6 y_+0.6 y_-0.6],'LineWidth',2);
% % rotate(car, [0 0 1], rad2deg(states_history(end,5)),[x_ y_ 0]);
% % 
% % % Draw wheels
% % wheel_positions = {
% %     [x_+l_f, y_+1.10/2, 'r'];
% %     [x_-l_r, y_+1.10/2, 'b'];
% %     [x_+l_f, y_-1.10/2, 'r'];
% %     [x_-l_r, y_-1.10/2, 'b']
% % };
% % 
% % 
% % drawnow
% % end
% % 
% % function weights = calculate_weights(S,lambda,samples)
% % %     Calculate weights
% % n = 0;
% % weights = zeros(1,samples);
% % beta = min(S);
% % for k=1:1:samples-1
% %     n = n + exp(-(1/lambda)*(S(k) - beta));
% % end
% % for k=1:1:samples-1
% %     weights(k) = (exp(-(1/lambda)*(S(k)-beta)))/n;
% % end
% % 
% % end
