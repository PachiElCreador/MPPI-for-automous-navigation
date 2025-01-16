%% Model Predictive Path Integral Control
% Implements Model Predictive Path Integral Control for a Single Track
% model navigating a grid map at high velocity.

function [tmeasure, states_history, u_opt] = MPPI_nav_controller(system_dynamics, param, running_cost, terminal_cost)


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


% Initialize matrices
states_history = [];
u_opt = [];
dW_history = [];
trajectory_history = [];
S_save = [];


% Set seed for using always the same random numbers (for comparing simulations)
% rng('default');
% rng(2);

% Creates the map and a matrix with the cost of each grid quare
[map, cost_map, obstacles] = byHandMap();
%[map, cost_map, obstacles] = hexagonal_map();
%[map, cost_map] = dynamic_map(); %uncomment for static obstacle line map
%[map, cost_map] = laberintoS();

% Sets all cost to zero;
% cost_map = cost_map*0;

% Save initial state in its historz matrix
states_history = [states_history; xmeasure];

% Set extra obstacles at the beginning of the simulation
% obstacle = [obstacle;3 50;4 50; 5 50;2 50;1 50;3 51;4 51; 5 51;2 51;1 51;...
% 6 50;6 51; 7 50; 7 51; 8 50;8 51;3 52;4 52; 5 52;2 52;1 52;7 52;8 52;];
% setOccupancy(map,obstacle,1);
% obstacle = [obstacle;12 70;11 70;10 70;9 70;8 70;12 71;11 71;10 71;9 71;...
% 8 71;12 72;11 72;10 72;9 72;8 72];
% setOccupancy(map,obstacle,1);


% Begin MPC iterations
for loop = 0:1:iterations
    if mod(loop, 5) == 0
        %[map, cost_map] = dynamic_map();
        %[map, cost_map] = laberintoS();
        %[map, cost_map] = LaberintoS_2();
    end
  
    
    hit_obstacle = 0; % did not hit an obstacle
    
    
    % Initialize forward sampling
    [u, save_trajectory,S] = gettrajectories(samples,xmeasure,tmeasure,sigma,timesteps,alpha,u_UB,u_LB,u...
        ,system_dynamics,gamma,dT,x_desired,running_cost,terminal_cost,loop,lambda,map,cost_map);
    
    
    %  Save the first optimal input
    u_opt = [u_opt; u(1,:)];
   
    
    %   Add noise to the input (optional)
    %     u(1,:) = u(1,:) +  normrnd(0,sigma,[1,1]);
    
    % Set the system noise
    for state=1:1:size(xmeasure,2)
        dW(state) = sqrt(dT)*normrnd(0,system_noise(state),[1,1]);
    end
    
    % Solver
%     [t_f, next_state] = ode45(@(t,x)system_dynamics(t, x ,u(1,:),dW), [tmeasure tmeasure+dT], xmeasure);
        [ next_state] = ode1(@(t,x)system_dynamics(t, x ,u(1,:),dW), [tmeasure tmeasure+dT], xmeasure);
    
%     tmeasure = t_f(end,:);
    tmeasure = tmeasure+dT;
    
    
    % Checks if obstacle was hit
    if hit_obstacle == 0
        for i = 1:1:size(next_state,1)
            
            hit_obstacle = getOccupancy(map,[next_state(i,1),next_state(i,2)]);
            if hit_obstacle==1
                
                xmeasure(:,state) = next_state(1,state);
                break;
            end
            
            for state=1:1:size(next_state,2)
                xmeasure(:,state) = next_state(i,state);
            end
            
        end
        
    end
    
    
    
    % Save relevant information into its history matrices
    dW_history = [dW_history; dW];
    states_history = [states_history; xmeasure];
    trajectory_history(:,:,:,loop+1) = save_trajectory;
    S_save = [S_save;S];
    
    
    %     Shift the input vector to the left and leave the new last input value
    %     as the same as the last one
    for t = 1:1:timesteps-1
        u(t,:) = u(t+1,:);
    end
    
    %     Initialize the new last input with the desired values (optional)
    %         u(timesteps,:) = [0 2000 0.1 0.1];
    u(timesteps,:) = [0 0 0 0];
    
    % Plot the current stand
    plot_trajectories(states_history,save_trajectory,map,S,u,timesteps,dT,system_dynamics);
    
    
    
end


end

% calculates the weights for each sample of the system
%  input:
%        S: sample cost
%        lambda: inverse temperature
%        samples: number of samples (can be subs to size(S,*))
% Output: importance weights
function weights = calculate_weights(S,lambda,samples)
%     Calculate weights
n = 0;
weights = zeros(1,samples);
beta = min(S);
for k=1:1:samples-1
    n = n + exp(-(1/lambda)*(S(k) - beta));
end
for k=1:1:samples-1
    weights(k) = (exp(-(1/lambda)*(S(k)-beta)))/n;
end

end

%   Plot a map with the single track model and all its trajectories
%   input: states_history: all states trajectories
%          save_trajectory: current trajectory of the sample
%          map: grid map
%          S: cost of everey trajectory
%          u: control input sequence
%          timesteps: number of timesteps
%          dt: delta T
%          system_dynamics: single track model
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

% axis([min(min(save_trajectory(:,1,:)))-10 max(max(save_trajectory(:,1,:)))+10 min(min(save_trajectory(:,2,:)))-10 max(max(save_trajectory(:,2,:)))+10])

text(x_ +2 ,y_ + 10,num2str(states_history(end,3)), 'Color','r');
drawnow

end

function [u, save_trajectory, S] = gettrajectories(samples,xmeasure,tmeasure,sigma,timesteps,alpha,u_UB,u_LB,u...
    ,system_dynamics,gamma,dT,x_desired,running_cost,terminal_cost,loop,lambda,map,cost_map)

% Initialize the costs to zero
S = zeros(1, samples-1);

% Sampling
for k = 1:1:samples-1
    
    save_trajectory(1,:,k) = xmeasure(end,:); % Save all the calculate traejctories
    hit_obstacle = 0; % if the single track model hit an obstacle
    
    % Set the initial state for every trajectory
    x_0 = xmeasure(end,:);
    t_0 = tmeasure(end,:);
    
    % Get the normal distributed noise for the input control
    for i=1:1:size(u,2)
        noise(i,:,k) = normrnd(0,sigma(i),[1,timesteps]);
    end
    
    % Apply the stochastic input for the given horizon
    for t = 2:1:timesteps
        
        % Apply control with noise
        if k < (1 - alpha)*samples
            v(t-1,:) = u(t-1,:) + noise(:,t-1,k)';
        else
            % Aplly just the noise
            v(t-1,:) = noise(:,t-1,k)';
        end
        
        % Verifies if the control respects its lower and upper bounds
        for i=1:1:size(u,2)
            if  v(t-1,i) > u_UB(i)
                v(t-1,i) = u_UB(i);
            elseif v(t-1,i) < u_LB(i)
                v(t-1,i) = u_LB(i);
            end
        end

        %  Set it to the system
        l = t-1; %Actual time step
        
        dW = zeros(1, size(x_0,2)); % No system noise for sampling
        %[t_, x_] = ode45(@(t,x)system_dynamics(t, x ,v(l,:),dW), [t_0
        %t_0+dT],x_0); % ODE 45 does not work, dont know why
        [x_] = ode1(@(t,x)system_dynamics(t, x ,v(l,:),dW),[t_0 t_0+dT], x_0); % ODE 1 for fast sampling
        t_0 = t_0+dT; %Set new time
        
        if isreal(x_)
            % Checks if the position of the single track model is in the
            % boundaries of the map
            x_0_(:,1) = x_(:,1);    %Help variables
            x_0_(:,2) = x_(:,2);
            
            % Checks and corrects its position
            for i = 1:1:size(x_0_,1)
                if x_0_(i,1) > 100
                    x_0_(i,1) = 100;
                end
                if x_0_(i,1) < 0
                    x_0_(i,1) = 0;
                end
                if x_0_(i,2) > 100
                    x_0_(i,2) = 100;
                end
                
                if x_0_(i,2) < 0
                    x_0_(i,2) = 0;
                end
                
            end
            
            
            % Checks if an obstacle was hit, if it was hit before, then no
            % no need to check again
            if hit_obstacle == 0
                for i = 1:1:size(x_0_,1)
                    hit_obstacle = getOccupancy(map,[x_0_(i,1),x_0_(i,2)]);
                    x_0 = x_(i,:);  % Update state until it gets hit or not
                    if hit_obstacle
                        break;
                    end
                end
            end
            
            
            % Gets the cost of the current grid square
            grid_idx = world2grid(map,[x_0_(end,1) x_0_(end,2)]);
            cost_grid_square = cost_map(grid_idx(1),grid_idx(2));
            
        else
            
            hit_obstacle = 1;
            
        end
        % Save the calculated trajectory
        save_trajectory(t,:,k) = x_0;
        
        
        % Calculate the total cost
        [cost] = running_cost(x_0,x_desired);
        S(k) = S(k) +  cost  + 1*cost_grid_square  + 1*gamma*u(t-1,:)*inv(diag(ones(1, size(u,2))).*sigma)*v(t-1,:)' + 1*hit_obstacle*100000000000;
        
        
    end
    
    % Terminal cost
    S(k) = S(k) + terminal_cost(x_0,x_desired) ;
    if isinf(S(k))
        S(k) = 100000000000000000000000000000000;
    end
    
    % Display relevant information
    disp(['Smp: ', num2str(k), ' itr: ', num2str(loop), ' Sample Cost: ' num2str(S(k))]);
    
    
end

%     Calculate the weights
weights = calculate_weights(S,lambda,samples);


%     Push the control distibution to the optimal one
for t=1:1:timesteps
    
    weights_sum = zeros(1,size(u,2));
    
    for k=1:1:samples-1
        weights_sum = weights_sum + (weights(k)*noise(:,t,k))';
    end
    
    weights_vector(t,:) = weights_sum;
    
end

%         Update

% Apply Savitzky-Golay convulutional filtering for smoothness in the
% weights! and not the control input(??)
% framelen = size(u,1)-3;
% order = framelen-10;
% sgf = sgolayfilt(weights_vector,order,framelen);

% Plot the old weights and smoothed ones
% figure(1)
% hold off
% y = linspace(0,size(u,1),size(u,1));
% plot(y,sgf(:,1),'b');
% hold on
% plot(y,weights_vector(:,1),'r');


for t=1:1:timesteps
        u(t,:) = u(t,:) + weights_vector(t,:);
%     u(t,:) = u(t,:) + sgf(t,:);
    
    % Checks if the new input sequence satifies the input constraints
    for i = 1:1:size(u,2)
        if(u(t,i) < u_LB(i))
            u(t,i) = u_LB(i);
        elseif (u(t,i) > u_UB(i))
            u(t,i) = u_UB(i);
        end
    end
end

end