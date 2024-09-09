%% Kinematic Simulation of a land-based mobile robot
clear all, clc; close all;

%% Simulation Parameter
dt = 0.1; % Step Size
ts = 100; % Simulation Time
t = 0:dt:ts; % Time Span

%% Vehicle (Mobile Robot) parameters (physical)
a = 0.2; % radius of the wheel in meters (fixed)
d_w = 0.5; % distance between wheel frame to vehicle frame along y axis
l_w = 0.3; % distance between wheel frame to vehicle frame along x axis

%% Initial Conditions
x0 = 0.5;
y0 = 0.5; 
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

    %% inputs ( wheel angular velocity)
    omega_1 = 0.5; % first wheel angular velocity
    omega_2 = 0.5; % second wheel angular velocity
    omega_3 = 0.5; % third wheel angular velocity
    omega_4 = 0.5; % fourth wheel angular velocity
    omega = [omega_1;omega_2;omega_3;omega_4];

    %% Wheel Configuration Matrix
    W = a/4*[1,1,1,1;
             1,-1,1,-1;
             -1/(d_w-l_w),-1/(d_w-l_w),1/(d_w-l_w),1/(d_w-l_w)];

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
l = 2*l_w; % length of mobile robot
w = 2*d_w; % width of mobile robot

% Mobile robot coordinates for a top-view rectangle
mr_co = [-l/2, l/2, l/2, -l/2, -l/2;
         -w/2, -w/2, w/2, w/2, -w/2];

% Wheel coordinates for better visualization
wheel_length = 0.1;
wheel_width = 0.05;
wheel1 = [-wheel_length/2, wheel_length/2, wheel_length/2, -wheel_length/2;
          -wheel_width/2, -wheel_width/2, wheel_width/2, wheel_width/2];
wheel2 = [wheel_length/2, wheel_length/2, wheel_length/2, -wheel_length/2;
          -wheel_width/2, -wheel_width/2, wheel_width/2, wheel_width/2];
wheel3 = [-wheel_length/2, wheel_length/2, wheel_length/2, -wheel_length/2;
          -wheel_width/2, -wheel_width/2, wheel_width/2, wheel_width/2];
wheel4 = [-wheel_length/2, wheel_length/2, wheel_length/2, -wheel_length/2;
          -wheel_width/2, -wheel_width/2, wheel_width/2, wheel_width/2];

figure 
for i = 1:5:length(t) % animation starts here
    psi = eta(3,i);
    R_psi = [cos(psi),-sin(psi);
             sin(psi), cos(psi)]; % 2D - rotation matrix
    v_pos = R_psi*mr_co;
    fill(v_pos(1,:)+eta(1,i),v_pos(2,:)+eta(2,i),'g')
    
    % Wheels position and rotation
    wheel1_pos = R_psi*wheel1 + [eta(1,i) - l/2; eta(2,i) - w/2];
    wheel2_pos = R_psi*wheel2 + [eta(1,i) + l/2; eta(2,i) - w/2];
    wheel3_pos = R_psi*wheel3 + [eta(1,i) + l/2; eta(2,i) + w/2];
    wheel4_pos = R_psi*wheel4 + [eta(1,i) - l/2; eta(2,i) + w/2];
    
    fill(wheel1_pos(1,:), wheel1_pos(2,:), 'r')
    fill(wheel2_pos(1,:), wheel2_pos(2,:), 'r')
    fill(wheel3_pos(1,:), wheel3_pos(2,:), 'r')
    fill(wheel4_pos(1,:), wheel4_pos(2,:), 'r')
    
    hold on, grid on; axis([-1 3 -1 3]), axis square
    plot(eta(1,1:i),eta(2,1:i),'g-');
    legend('Mobile Robot','Path'), set(gca,'fontsize',18)
    xlabel('x[m]');
    ylabel('y[m]');
    pause(0.01);
    hold off

end % animation ends here
