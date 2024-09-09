%% Dynamic model of a land-based mobile robot
clear all; clc ; close all;

%% Simulation Parameters
dt = 0.1; % Step Size
ts = 50; % Simulation Time
t = 0:dt:ts; % Time Span

%% Initial Conditions
eta0 = [0;0;pi/4]; % Initial position and orientation of the vehicle
zeta0 = [0;0;0]; % Initial vector of input commands

eta(:,1) = eta0;
zeta(:,1) = zeta0;

%% Robot Parameters
m = 10; % mass of the vehicle
Iz = 0.1; % Inertia of the vehicle

xbc = 0; ybc = 0; % co-ordinates of mass center

%% State Propagation

for i = 1:length(t)
    u = zeta(1,i);
    v = zeta(2,i);
    r = zeta(3,i);

    %% Inertia matrix
    D = [m,0,-ybc*m;
         0,m,xbc*m;
         -ybc*m,xbc*m,Iz+m*(xbc^2+ybc^2);];

    %% Other Vector
    n_v = [-m*r*(v+xbc*r);
           m*r*(u-ybc*r);
           m*r*(xbc*u+ybc*v);];

    %% Input Vector
    tau(:,i) = [1;0.5;0];

    %% Jacobian Matrix
    psi = eta(3,i);
    J_eta =[cos(psi),-sin(psi),0;
            sin(psi),cos(psi),0;
            0,0,1];

    zeta_dot(:,i) = inv(D)*(tau(:,i) - n_v - 0.5*zeta(:,i));
    zeta(:,i+1) = zeta(:,i) + dt*zeta_dot(:,i); % velocity update

    eta(:,i+1) = eta(:,i) + dt * (J_eta*zeta(:,i) + dt*zeta_dot(:,i)); % State update

     


end

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
