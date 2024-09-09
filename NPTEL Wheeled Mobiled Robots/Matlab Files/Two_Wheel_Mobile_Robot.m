%% Kinematic Simulation of a land-based mobile robot
clear all, clc; close all;

%% Simulation Parameter
dt = 0.1; % Step Size
ts = 100; % Simulation Time
t = 0:dt:ts; % Time Span

%% Vehicle (Mobile Robot) parameters (physical)
a = 0.2; % radius of the wheel in meters (fixed)
d = 0.5; % distance between wheel frame to vehicle frame along y axis



%% Initial Conditions
x0 = 0.5;
y0 = 0.5; 
psi0 = pi/4;

eta0 = [x0;y0;psi0];

eta(:,1)=eta0;

%% Loop starts here
for i = 1:length(t)
    psi = eta (3,i); % current orientation in radian OR 3rd row of eta

    % Jacobian Matrix
    J_psi = [cos(psi),-sin(psi),0;
             sin(psi),cos(psi),0;
             0,0,1;];

    %% inputs ( wheel angular velocity)
    omega_1 = -0.5; % left wheel angular velocity
    omega_2 = 0.5; % right wheel angular velocity
    omega = [omega_1;omega_2];

    %% Wheel Configuration Matrix
    W = [a/2,a/2;
         0,0;
         -a/(2*d), a/(2*d)];

    zeta(:,i) = W*omega;

    eta_dot(:,i) = J_psi * zeta(:,i);

    eta(:,i+1) = eta(:,i) + dt * eta_dot(:,i); % Euler Method

end

%% Plotting Functions
figure
plot(t, eta(1, 1:end-1), 'r-');
set(gca, 'fontsize', 24);
xlabel('t[s]');
ylabel('x[m]');

figure
plot(t, eta(2, 1:end-1), 'b-');
set(gca, 'fontsize', 24);
xlabel('t[s]');
ylabel('y[m]');

figure
plot(t, eta(3, 1:end-1), 'g-');
set(gca, 'fontsize', 24);
xlabel('t[s]');
ylabel('\psi[rad]');

figure; %% in a single figure
hold on;
plot(t, eta(1, 1:end-1), 'r--', 'DisplayName', 'x-position');
plot(t, eta(2, 1:end-1), 'g--', 'DisplayName', 'y-position');
plot(t, eta(3, 1:end-1), 'b--', 'DisplayName', 'orientation (\psi)');

set(gca, 'fontsize', 24);
xlabel('t[s]');
ylabel('Value');
legend('show'); % Show the legend box with the names specified
hold off;

%% Animation (mobile robot motion animation)
l = 0.5; %length of mobile robot
w = 2*d; %width of mobile robot
% Mobile robot co-ordinates
mr_co = [-l/2,l/2,l/2,-l/2,-l/2;
         -w/2,-w/2,w/2,w/2,-w/2;];

figure 
for i = 1:5:length(t) % animation starts here
    psi = eta(3,i);
    R_psi = [cos(psi),-sin(psi);
             sin(psi), cos(psi);]; % 2D - rotation matrix
    v_pos = R_psi*mr_co;
    fill(v_pos(1,:)+eta(1,i),v_pos(2,:)+eta(2,i),'g')
    hold on, grid on; axis([-1 3 -1 3]), axis square
    plot(eta(1,1:i),eta(2,1:i),'g-');
    legend('Mobile Robot','Path'), set(gca,'fontsize',18)
    xlabel('x[m]');
    ylabel('y[m]');
    pause(0.01);
    hold off

end% animation ends here




