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
             0,0,1;];

    u = 0.3; % x_axis velocity w.r.t B frame.
    v = 0; % y_axis velocity w.r.t B frame.
    r = 0.2; % angular velocity w.r.t B frame.

    zeta(:,i) = [u;v;r];

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
l = 0.6; %length of mobile robot
w = 0.4; %width of mobile robot
% Mobile robot co-ordinates
mr_co = [-l/2,l/2,l/2,-l/2,-l/2;
         -w/2,-w/2,w/2,w/2,-w/2;];

figure 
for i = 1:length(t) % animation starts here
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
    pause(0.1);
    hold off

end% animation ends here




