%% Model Predictive Path Integral Control
% Initialization of the Model Predictive Path Integral Control problem for
% a single track model

clear all;
close all;
clc;
warning off

%% MPPI Configuration Parameters
% Optimization parameters based on Information Theoretic MPC framework
param.timesteps = 50;     % Prediction horizon T
param.samples = 100;      % Number of trajectory samples K
param.iterations = 200;   % Maximum optimization iterations N

% Initial state configuration [x, y, v, β, ψ, ω, ẋ, ẏ, ψ̇, φ̇]
% where: x,y = position, v = velocity, β = slip angle, 
% ψ = yaw angle, ω = yaw rate, φ̇ = wheel rotation rate
param.xmeasure = [12 30 0 0 pi/2 0 0 0 0 0];    

% Target state configuration
param.x_desired = [0 0 30 0 0 0 0 0 0 0]; 

% Time parameters
param.tmeasure = 0;    % Initial time t₀
param.dT = 0.1;        % Time step Δt

% Control input configuration
num_controls = 4;      % Number of control inputs
param.u = zeros(param.timesteps, num_controls);  % Initial control sequence

% Control bounds (based on vehicle dynamics constraints)
param.u_UB = [0.5263 1 1 1];         % Upper bounds [δmax, Fb_max, ζmax, φmax]
param.u_LB = [-0.5263 0 0 0.01];     % Lower bounds [δmin, Fb_min, ζmin, φmin]

% Noise parameters (based on stochastic optimal control theory)
param.system_noise = 0.01 * ones(1,10);  % System noise covariance Σw

% MPPI specific parameters (from Information Theoretic MPC)
param.lambda = 5;      % Temperature parameter λ (controls exploration)
param.sigma = [0.1 0.15 0.01 0.1];   % Control noise covariance Σu
param.alpha = 0.01;    % Mixing coefficient α (exploration vs exploitation)
param.gamma = param.lambda*(1-param.alpha);  % Control cost weight γ

%% Execute MPPI Controller
tic
cpu_start = cputime;

[t_all, x_all, u_all] = MPPI_nav_controller(@system_dynamics, param, @running_cost,@terminal_cost);

param.computation_time = toc;
param.cpu_usage = cputime - cpu_start;

%% Cost Functions Implementation

% running cost function (running cost)
% Implements quadratic cost: l(x,u) = (x-x*)ᵀQ(x-x*)
function [cost] = running_cost(state, target)
    % State cost weights matrix Q
    Q = diag([0,0,1,1,0,0,0,0,0,0]);
    cost = (state-target)*Q*(state-target)';
end

% Terminal Cost Function
% Implements terminal cost: lf(x) = (x-x*)ᵀQf(x-x*)
function [cost] = terminal_cost(state, target)
    % Terminal state cost weights matrix Qf
    % Adjust Qf based on the specific task or optimization goals
    Qf = diag([0, 0, 0, 0, 0, 0, 0, 0, 0, 0]); % Placeholder weights
    cost = (state - target) * Qf * (state - target)';
end



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% System Dynamics Function
% Implements single-track vehicle dynamics with gear selection
function single_track = system_dynamics(t, X, U, dW)
    % Gear selection logic based on velocity thresholds
    % Adjust thresholds and logic if the vehicle model or parameters change
    G = 1; % Default gear
    if X(3) < 9
        G = 1; % Gear 1
    elseif X(3) > 8.9 && X(3) < 16 % Velocity between 9 and 16 m/s
        G = 2; % Gear 2
    elseif X(3) > 15.9 && X(3) < 23 % Velocity between 16 and 23 m/s
        G = 3; % Gear 3
    elseif X(3) > 22.9 && X(3) < 30 % Velocity between 23 and 30 m/s
        G = 4; % Gear 4
    elseif X(3) > 30
        G = 5; % Gear 5
    end

    % - The gear selection thresholds are based on standard vehicle dynamics practices.
    % - [7] and [8].
    % - Additional dynamics or external disturbances (dW) can be incorporated as needed.




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Vehicle Parameters

% Mass and Gravity Parameters
m = 1000; % Vehicle mass [kg]
g = 9.81; % Gravitational acceleration [m/s^2]

% Geometric Parameters
l_f = 1.19016; % Distance from the front axle to the center of mass [m]
l_r = 1.37484; % Distance from the rear axle to the center of mass [m]
% l = l_f + l_r; % Total wheelbase (not used, kept for reference)
R = 0.302; % Wheel radius [m]

% Inertia Parameters
I_z = 1752; % Moment of inertia about the yaw axis [kg*m^2]
I_R = 1.5; % Moment of inertia of the wheels [kg*m^2]

% Transmission Ratios
i_g = [3.91, 2.002, 1.33, 1, 0.805]; % Gear ratios for 1st to 5th gear
i_0 = 3.91; % Final drive ratio

% Tire Parameters (Pacejka Model)
B_f = 10.96; % Stiffness factor for the front wheel
C_f = 1.3; % Shape factor for the front wheel
D_f = 4560.4; % Peak factor for the front wheel
E_f = -0.5; % Curvature factor for the front wheel
B_r = 12.67; % Stiffness factor for the rear wheel
C_r = 1.3; % Shape factor for the rear wheel
D_r = 3947.81; % Peak factor for the rear wheel
E_r = -0.5; % Curvature factor for the rear wheel

% Rolling Resistance Coefficients
f_r_0 = 0.009; % Rolling resistance constant coefficient
f_r_1 = 0.002; % Rolling resistance linear coefficient
f_r_4 = 0.0003; % Rolling resistance quadratic coefficient

% Notes:
% - Parameters such as the Pacejka coefficients are consistent with values
% given for vehicle dynamics simulations in referenced literature.
% - These values may need further tuning based on the specific vehicle
% model or experimental data, as indicated in [7] and [8].


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% control inputs
delta=U(1); % steering angle 
% G=U(2); % gear 1 ... 5
F_b=U(2); %braking force
zeta=U(3); % braking force distribution
phi=U(4); % gas pedal position
% input constraints
if delta>0.5 % upper bound for steering angle exceeded?
    delta=0.5; % upper bound for steering angle
end
if delta<-0.5 % lower bound for steering angle exceeded?
    delta=-0.5; % lower bound for steering angle
end
if F_b<0 % lower bound for braking force exceeded?
    F_b=0; % lower bound for braking force
end
if F_b>10000 % upper bound for braking force exceeded?
    F_b=10000; % upper bound for braking force 
end
if zeta<0 % lower bound for braking force distribution exceeded?
    zeta=0; % lower bound for braking force distribution 
end
if zeta>1 % upper bound for braking force distribution exceeded?
    zeta=1; % upper bound for braking force distribution
end
if phi<0 % lower bound for gas pedal position exceeded?
    phi=0; % lower bound for gas pedal position
end
if phi>1 % upper bound for gas pedal position exceeded?
    phi=1; % upper bound for gas pedal position
end
F_b = 15000*F_b;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% state vector
%x=X(1); % x position (obsolete)
%y=X(2); % y position (obsolete)
v=(X(3)); % velocity
if v < 0
   v =0; 
end
beta=X(4); % side slip angle
psi=X(5); % yaw angle
omega=X(6); % yaw rate
%x_dot=X(7); % longitudinal velocity (obsolete)
%y_dot=X(8); % lateral velocity (obsolete)
psi_dot=X(9); % yaw rate (redundant)
varphi_dot=(X(10)); % wheel rotary frequency

%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% DYNAMICS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% slip
%slip angles and steering
a_f=delta-atan((l_f*psi_dot-v*sin(beta))/(v*cos(beta))); % front slip angle
a_r=atan((l_r*psi_dot+v*sin(beta))/(v*cos(beta))); %rear slip angle

if isnan(a_f) % front slip angle well-defined?
    a_f=0; % recover front slip angle
end
if isnan(a_r) % rear slip angle well-defined
    a_r=0; % recover rear slip angle
end
%wheel slip
if v<=R*varphi_dot % traction slip? (else: braking slip)
    S=1-(v/(R*varphi_dot)); %traction wheel slip
else
    S=1-((R*varphi_dot)/v); % braking slip
end
if isnan(S) % wheel slip well-defined?
    S=0; % recover wheel slip
end
S=0; % neglect wheel slip

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Traction, friction, braking

% Motor rotary frequency
n = v * i_g(G) * i_0 * (1 / (1 - S)) / R;

% Ensure rotary frequency is well-defined
if isnan(n)
    n = 0; % Recover rotary frequency
end

% Limit rotary frequency to the maximal value
if n > (4800 * pi) / 30
    n = (4800 * pi) / 30; % Recover maximal rotary frequency
end

% Motor torque
T_M = 200 * phi * (15 - 14 * phi) - 200 * phi * (15 - 14 * phi) * ...
      ((n * (30 / pi))^(5 * phi) / (4800^(5 * phi)));

% Wheel torque
M_wheel = i_g(G) * i_0 * T_M;

% Weight distribution
F_w_r = (m * l_f * g) / (l_f + l_r); % Rear weight
F_w_f = (m * l_r * g) / (l_f + l_r); % Front weight

% Approximate friction
f_r = f_r_0 + f_r_1 * ((abs(v) * 3.6) / 100) + f_r_4 * ((abs(v) * 3.6) / 100)^4;

% Braking forces
F_b_r = zeta * F_b;          % Rear braking force
F_b_f = F_b * (1 - zeta);    % Front braking force

% Friction forces
F_f_r = f_r * F_w_r; % Rear friction force
F_f_f = f_r * F_w_f; % Front friction force

% Longitudinal forces
F_x_r = (M_wheel / R) - sign(v * cos(beta)) * F_b_r - sign(v * cos(beta)) * F_f_r; % Rear wheel
F_x_f = -sign(v * cos(beta)) * F_b_f - sign(v * cos(beta)) * F_f_f;               % Front wheel

% Lateral forces
F_y_r = D_r * sin(C_r * atan(B_r * a_r - E_r * (B_r * a_r ...
                   - atan(B_r * a_r)))); % Rear lateral force

F_y_f = D_f * sin(C_f * atan(B_f * a_f - E_f * (B_f * a_f ...
                   - atan(B_f * a_f)))); % Front lateral force

%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% OUTPUT %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Vector field (right-hand side of differential equation)

% Longitudinal velocity
x_dot = v * cos(psi - beta);

% Lateral velocity
y_dot = v * sin(psi - beta);

% Acceleration
v_dot = (F_x_r * cos(beta) + F_x_f * cos(delta + beta) - F_y_r * sin(beta) ...
         - F_y_f * sin(delta + beta)) / m;

% Side slip rate
beta_dot = omega - (F_x_r * sin(beta) + F_x_f * sin(delta + beta) + F_y_r * cos(beta) ...
                    + F_y_f * cos(delta + beta)) / (m * v);

% Yaw rate
psi_dot = omega;

% Yaw angular acceleration
omega_dot = (F_y_f * l_f * cos(delta) - F_y_r * l_r ...
             + F_x_f * l_f * sin(delta)) / I_z;

% Longitudinal acceleration
x_dot_dot = (F_x_r * cos(psi) + F_x_f * cos(delta + psi) - F_y_f * sin(delta + psi) ...
             - F_y_r * sin(psi)) / m;

% Lateral acceleration
y_dot_dot = (F_x_r * sin(psi) + F_x_f * sin(delta + psi) + F_y_f * cos(delta + psi) ...
             + F_y_r * cos(psi)) / m;

% Yaw angular acceleration (repeated for clarity)
psi_dot_dot = (F_y_f * l_f * cos(delta) - F_y_r * l_r ...
               + F_x_f * l_f * sin(delta)) / I_z;

% Wheel rotary acceleration
varphi_dot_dot = (F_x_r * R) / I_R;

% Handle undefined side slip angle
if isnan(beta_dot)
    beta_dot = 0; % Recover side slip angle
end

if isinf(beta_dot)
    beta_dot = 0; % Recover side slip angle
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% write outputs
single_track=[x_dot;y_dot;v_dot;beta_dot;psi_dot;omega_dot;x_dot_dot ...
            ;y_dot_dot;psi_dot_dot;varphi_dot_dot]; % left-hand side

end
