function Y = ode1(odefun, tspan, y0, varargin)
% ODE1 Solve differential equations using the forward Euler method.

% Initialize
h = diff(tspan);
y0 = y0(:); % Ensure column vector
N = length(tspan);
neq = length(y0);
Y = zeros(N, neq); % Preallocate
Y(1, :) = y0'; % Set initial condition

% Main loop (Euler integration)
for i = 1:N-1
    Y(i+1, :) = Y(i, :) + h(i) * odefun(tspan(i), Y(i, :)', varargin{:})';
end

