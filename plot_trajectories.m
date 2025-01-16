% function plot_trajectories(states_history,save_trajectory,map,S,u,timesteps,dT,system_dynamics)
% 
% figure(7)
% hold off
% show(map);
% hold on
% grid on
% 
% set(gca,'XTick',0:1:100,'YTick',0:1:100)
% 
% 
% % Normalize cost for plotting the
% colormap = winter;
% colorbar
% A = [1:size(colormap,1)/100:size(colormap,1)]';
% B = [1:1:size(colormap,1)]';
% 
% newcolormap = interp1(B, colormap ,A);
% S_norm = (S - min(S))/(max(S) - min(S)+0.000001);
% [Svalue, minidx] = min(S_norm);
% temperature_range = [0:1/98:1];
% temperature = zeros(1, size(S_norm,2));
% color = zeros(size(S_norm,2),3,1);
% 
% for i=1:1:size(S_norm,2)
%     temperature(i) = find(temperature_range < round(S_norm(i),3)+0.01 & temperature_range > round(S_norm(i),3)-0.01,1);
%     color(i,:,:) = newcolormap(temperature(i),:,:);
% end
% 
% 
% % Plot each trajectory with its respective color
% for k=1:1:size(save_trajectory,3)
%     plot(save_trajectory(:,1,k),save_trajectory(:,2,k),'Color', color(k,:,:))
% end
% 
% 
% x_ = states_history(end,1); y_ = states_history(end,2);
% 
% % Plots the velocity vector
% r = states_history(end,3); % magnitude of the velocity vector
% u_ = r * cos(states_history(end,5) - states_history(end,4)); % Direction here slip angle minus yaw angle
% v_ = r * sin(states_history(end,5) - states_history(end,4));
% h = quiver(x_,y_,u_,v_,'LineWidth',2,'MaxHeadSize',50,'Color','r');
% 
% 
% % Plot the driven trajectory of the single track model until now
% plot(states_history(:,1),states_history(:,2),'-b','DisplayName','Current Trajectory')
% 
% plot(save_trajectory(:,1,minidx),save_trajectory(:,2,minidx),'.k')
% 
% % Plot the current states given by the current input sequence
% save_current_states = [];
% x_open = states_history(end,:);
% dW = zeros(1, size(states_history,2));
% for j = 1:1:timesteps
%     
%     [ x_test_] = ode1(@(t,x)system_dynamics(t, x ,u(j,:),dW), [0 dT], x_open);
%     save_current_states = [save_current_states; x_test_];
%     x_open = x_test_(end,:);
%     
% end
% plot(save_current_states(:,1),save_current_states(:,2),'.r')
% 
% 
% % Draw thesingle track model as a car
% 
% l_f=1.19016; % front lenght
% l_r=1.37484; % back lenght
% l=l_f+l_r;  %total lenght
% R=0.302;    % whell radius
% 
% 
% car = plot([x_-l_r  x_+l_f  x_+l_f  x_-l_r  x_-l_r], [y_-0.6  y_-0.6  y_+0.6  y_+0.6  y_-0.6],'LineWidth',2);
% rotate(car, [0 0 1], rad2deg(states_history(end,5)),[x_ y_ 0]); %Rotates the car yaw angle
% wheel_1 = plot([x_+l_f-R  x_+l_f+R  x_+l_f+R  x_+l_f-R  x_+l_f-R], [y_+1.10/2  y_+1.10/2  y_+1.10/2-0.3  y_+1.10/2-0.3  y_+1.10/2],'LineWidth',2,'Color','r');
% wheel_2 = plot([x_-l_r-R  x_-l_r+R  x_-l_r+R  x_-l_r-R  x_-l_r-R], [y_+1.10/2  y_+1.10/2  y_+1.10/2-0.3  y_+1.10/2-0.3  y_+1.10/2],'LineWidth',2,'Color','b');
% wheel_3 = plot([x_+l_f-R  x_+l_f+R  x_+l_f+R  x_+l_f-R  x_+l_f-R], [y_-1.10/2  y_-1.10/2  y_-1.10/2+0.3  y_-1.10/2+0.3  y_-1.10/2],'LineWidth',2,'Color','r');
% wheel_4 = plot([x_-l_r-R  x_-l_r+R  x_-l_r+R  x_-l_r-R  x_-l_r-R], [y_-1.10/2  y_-1.10/2  y_-1.10/2+0.3  y_-1.10/2+0.3  y_-1.10/2],'LineWidth',2,'Color','b');
% 
% % Rotates the front wheels with the yaw angle and the current input
% % steering angle
% rotate(wheel_1, [0 0 1], rad2deg(states_history(end,5)),[x_ y_ 0]);
% rotate(wheel_2, [0 0 1], rad2deg(states_history(end,5)),[x_ y_ 0]);
% rotate(wheel_1, [0 0 1], rad2deg(u(1,1)),[x_+l_f*cos(states_history(end,5)) y_+l_f*sin(states_history(end,5)) 0]);
% rotate(wheel_3, [0 0 1], rad2deg(states_history(end,5)),[x_ y_ 0]);
% rotate(wheel_4, [0 0 1], rad2deg(states_history(end,5)),[x_ y_ 0]);
% rotate(wheel_3, [0 0 1], rad2deg(u(1,1)),[x_+l_f*cos(states_history(end,5)) y_+l_f*sin(states_history(end,5)) 0]);
% 
% % axis([min(min(save_trajectory(:,1,:)))-10 max(max(save_trajectory(:,1,:)))+10 min(min(save_trajectory(:,2,:)))-10 max(max(save_trajectory(:,2,:)))+10])
% 
% text(x_ +2 ,y_ + 10,num2str(states_history(end,3)), 'Color','r');
% drawnow
% 
% end
% 
% 
% % Sample the trajectories and return the optimal input sequence as well as
% % all trajectories and its costs
% % Inputs: samples:  number of samples
% %         xmeasure:  initial states measurement
% %         tmeasure:  initial time measurement
% %         sigma:    Covariance of the control input
% %         timesteps: horizon of each trajectory
% %         alpha: Base distribution parameters
% %         u_UB: control lower bound
% %         u_LB: control upper bound
% %         u: control input
% %         system_dynamics: single track model
% %         gamma: Control cost parameter
% %         dT: delta T
% %         x_desired: desired states
% %         running_cost: running cost function
% %         terminal_cost: terminal cost function
% %         loop: number of the current loop
% %         lambda: Inverse temperature
% %         map: grid map
% %         cost_map: cost matrix with the cost for every square of the map
% % Output: u: new control input sequence
% %         save_trajectory: save the sampled trajectories
% %         S: Return the cost for the sampled trajectories
% function [u, save_trajectory, S] = gettrajectories(samples,xmeasure,tmeasure,sigma,timesteps,alpha,u_UB,u_LB,u...
%     ,system_dynamics,gamma,dT,x_desired,running_cost,terminal_cost,loop,lambda,map,cost_map)