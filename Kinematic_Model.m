%% Kinematic Simulation of a land-based mobile robot
clear all, clc; close all;

%% Simulation Parameter
dt = 0.1; % Step Size
ts = 10; % Simulation Time
t = 0:dt:ts; % Time Span

%% Initial Conditions
x0 = 0;
y0 = 0; 
psi0 = 0;

eta0 = [x0;y0;psi0];

eta(:,1)=eta0;

%% Loop starts here
for i = 1:length(t)
    psi = eta (3,i); % current orientation in radian OR 3rd row of eta

    % Jacobian Matrix
    J_psi = [cos(psi),-sin(psi),0;
             sin(psi),cos(psi),0;
             0,0,1];

    u = 0.1; % x_axis velocity w.r.t B frame.
    v = 0; % y_axis velocity w.r.t B frame.
    r = 0; % angular velocity w.r.t B frame.

    zeta(:,i) = [u;v;r];

    eta_dot(:,i) = J_psi * zeta(:,i);

    eta(:,i+1) = eta(:,i) + dt * eta_dot(:,i); % Euler Method

end

%% Plotting Functions
plot(t, eta(1, 1:end-1), 'r-');
set(gca, 'fontsize', 24);
xlabel('t,[s]');
ylabel('x,[m]');
