% inital settings
close all; clear; clc;
%% main code
T = 2;
fs = 100;
t = 0:1/fs:T;
dt = 1/fs;  % Time step

% ----------- elbow_chosen = [1,1] -----------

len_t = length(t);

r_A = [0.25;0;0.6]; r_B = [0.05;-0.4;0.6];

%% Draw FK:
q = [0;0;0];
[R,d] = forward_kin(q);
figure('Units','normalized','OuterPosition',[0 0 1 1]); % Fullscreen figure
print_robot(q);
title('FK zero-state');
view([-0.3, -1, 0.3]);
axis equal;
grid on;
% Save high-res image
% filename = 'print_FK.png';
% exportgraphics(gcf, filename, 'Resolution', 300); % Higher resolution

%% Trajectory - video making

q_t = zeros(3,len_t);

for i = 1:len_t
    t_i = t(i);
    q_t(:,i) = q_plan(3,t_i);
end

% Video 1 - Izometric view
numFrames = length(t);
videoFile = 'robot_motion_ISO.avi';
v = VideoWriter(videoFile);  % Default is 'Motion JPEG AVI', allows high frame rate
v.FrameRate = floor(numFrames / T);  % Set frame rate to play in real time
open(v);

figure('Units', 'normalized', 'OuterPosition', [0, 0, 1, 1]);
set(gcf, 'Renderer', 'painters');

for i = 1:numFrames
    clf;
    print_robot(q_t(:, i));
    title(sprintf('Time: %.3f sec', t(i)), 'FontSize', 14);
    drawnow;

    frame = getframe(gcf);
    writeVideo(v, frame);
end

close(v);

%% Video 2 - Second view
numFrames = length(t);
videoFile = 'robot_motion_View2.avi';
v = VideoWriter(videoFile);  % Default is 'Motion JPEG AVI', allows high frame rate
v.FrameRate = floor(numFrames / T);  % Set frame rate to play in real time
open(v);

figure('Units', 'normalized', 'OuterPosition', [0, 0, 1, 1]);
set(gcf, 'Renderer', 'painters');

for i = 1:numFrames
    clf;
    print_robot(q_t(:, i));
    view([-100, 40]);
    title(sprintf('Time: %.3f sec', t(i)), 'FontSize', 14);
    drawnow;

    frame = getframe(gcf);
    writeVideo(v, frame);
end

close(v);

%% Print q_t for [1,+-1] and x(t),v(t) and a(t) and detJ:
print_trajectory_plots

%% Controler 1 - ID + PD control

% q to get to r_A, but with 1cm error in +z0:
err = [0;0;1e-2];
r_A_w_err = r_A + err;
q_A_w_err = inverse_kin(r_A_w_err);

% Main script to solve the system using ode45
% Define initial conditions
q0 = q_A_w_err;        % Initial position
qd0 = [0;0;0];       % Initial velocity
X0 = [q0; qd0];    % Initial state

% Time span for simulation
T_sim1 = T+1;
tspan = [0, T_sim1];

% Options: specify Max step:
options = odeset('MaxStep', 1/10 * dt);  % MaxStep

% Chose the first control law:
law = 1;

% Solve using ode45 - Xdot = state_eq(t, X, law)
[t_sim1, X] = ode45(@(t, X) state_eq(t, X, law), tspan, X0, options);

% Extract positions and velocities
q_t_sim1 = X(:, 1:3)';
qd_t_sim1 = X(:, 3+1:end)';

% Extract used tau:
len_t_sim1 = length(t_sim1);

tau_cont_t = zeros(3,len_t_sim1);

for i = 1:len_t_sim1
    t_i = t_sim1(i);
    q_ti = q_t_sim1(:,i);
    q_dot_ti = qd_t_sim1(:,i);
    tau_cont_t(:,i) = tau_cont(3,q_ti,q_dot_ti,t_i, law);
end

%% Plots sim 1, controller 1 M known
plots(t_sim1,q_t_sim1,len_t_sim1,tau_cont_t,1);

%% Controler 1 - video making

videoFile = 'robot_controler_1.avi';
v = VideoWriter(videoFile);  % Default is 'Motion JPEG AVI', allows high frame rate
fps = 100;
v.FrameRate = fps;  % Set frame rate to play in real time
open(v);

figure('Units', 'normalized', 'OuterPosition', [0, 0, 1, 1]);
set(gcf, 'Renderer', 'painters');

for i = 1:50:length(t_sim1)
    clf;
    q_ti = q_t_sim1(:, i);
    print_robot(q_ti);
    title(sprintf('Time: %.3f sec', t_sim1(i)), 'FontSize', 14);
    drawnow;

    frame = getframe(gcf);
    writeVideo(v, frame);
end

close(v);

%% Controller 1 - NO LOAD MASS

% q to get to r_A, but with 1cm error in +z0:
err = [0;0;1e-2];
r_A_w_err = r_A + err;
q_A_w_err = inverse_kin(r_A_w_err);

% Main script to solve the system using ode45
% Define initial conditions
q0 = q_A_w_err;        % Initial position
qd0 = [0;0;0];       % Initial velocity
X0 = [q0; qd0];    % Initial state

% Time span for simulation
T_sim2 = T+1;
tspan = [0, T_sim2];

% Options: specify Max step:
options = odeset('MaxStep', 1/10 * dt);  % MaxStep

% Chose the first control law:
law = 1;

% Solve using ode45 - Xdot = state_eq(t, X, law)
[t_sim2, X] = ode45(@(t, X) state_eq_noM(t, X, law), tspan, X0, options);

% Extract positions and velocities
q_t_sim2 = X(:, 1:3)';
qd_t_sim2 = X(:, 3+1:end)';

% Extract used tau:
len_t_sim2 = length(t_sim2);

tau_cont_t = zeros(3,len_t_sim2);

for i = 1:len_t_sim2
    t_i = t_sim2(i);
    q_ti = q_t_sim2(:,i);
    q_dot_ti = qd_t_sim2(:,i);
    tau_cont_t(:,i) = tau_cont(3,q_ti,q_dot_ti,t_i, law);
end

%% Plots sim 2, controller 1 M Unknown
plots(t_sim2,q_t_sim2,len_t_sim2,tau_cont_t,2);

%% Controler 2 - PD + Gravity comp

% q to get to r_A, but with 1cm error in +z0:
err = [0;0;1e-2];
r_A_w_err = r_A + err;
q_A_w_err = inverse_kin(r_A_w_err);

% Main script to solve the system using ode45
% Define initial conditions
q0 = q_A_w_err;        % Initial position
qd0 = [0;0;0];       % Initial velocity
X0 = [q0; qd0];    % Initial state

% Time span for simulation
T_sim3 = T+1;
tspan = [0, T_sim3];

% Options: specify Max step:
options = odeset('MaxStep', 1/10 * dt);  % MaxStep

% Chose the first control law:
law = 2;

% Solve using ode45 - Xdot = state_eq(t, X, law)
[t_sim3, X] = ode45(@(t, X) state_eq(t, X, law), tspan, X0, options);

% Extract positions and velocities
q_t_sim3 = X(:, 1:3)';
qd_t_sim3 = X(:, 3+1:end)';

% Extract used tau:
len_t_sim3 = length(t_sim3);

tau_cont_t = zeros(3,len_t_sim3);

for i = 1:len_t_sim3
    t_i = t_sim3(i);
    q_ti = q_t_sim3(:,i);
    q_dot_ti = qd_t_sim3(:,i);
    tau_cont_t(:,i) = tau_cont(3,q_ti,q_dot_ti,t_i, law);
end

%% Plots sim 3, controller 2 M known
plots(t_sim3,q_t_sim3,len_t_sim3,tau_cont_t,3);

%% Controler 2 - video making

videoFile = 'robot_controler_1.avi';
v = VideoWriter(videoFile);  % Default is 'Motion JPEG AVI', allows high frame rate
fps = 100;
v.FrameRate = fps;  % Set frame rate to play in real time
open(v);

figure('Units', 'normalized', 'OuterPosition', [0, 0, 1, 1]);
set(gcf, 'Renderer', 'painters');

for i = 1:50:length(t_sim3)
    clf;
    q_ti = q_t_sim3(:, i);
    print_robot(q_ti);
    title(sprintf('Time: %.3f sec', t_sim3(i)), 'FontSize', 14);
    drawnow;

    frame = getframe(gcf);
    writeVideo(v, frame);
end

close(v);

%% Controller 2 - NO LOAD MASS

% q to get to r_A, but with 1cm error in +z0:
err = [0;0;1e-2];
r_A_w_err = r_A + err;
q_A_w_err = inverse_kin(r_A_w_err);

% Main script to solve the system using ode45
% Define initial conditions
q0 = q_A_w_err;        % Initial position
qd0 = [0;0;0];       % Initial velocity
X0 = [q0; qd0];    % Initial state

% Time span for simulation
T_sim4 = T+1;
tspan = [0, T_sim4];

% Options: specify Max step:
options = odeset('MaxStep', 1/10 * dt);  % MaxStep

% Chose the first control law:
law = 2;

% Solve using ode45 - Xdot = state_eq(t, X, law)
[t_sim4, X] = ode45(@(t, X) state_eq_noM(t, X, law), tspan, X0, options);

% Extract positions and velocities
q_t_sim4 = X(:, 1:3)';
qd_t_sim4 = X(:, 3+1:end)';

% Extract used tau:
len_t_sim4 = length(t_sim4);

tau_cont_t = zeros(3,len_t_sim4);

for i = 1:len_t_sim4
    t_i = t_sim4(i);
    q_ti = q_t_sim4(:,i);
    q_dot_ti = qd_t_sim4(:,i);
    tau_cont_t(:,i) = tau_cont(3,q_ti,q_dot_ti,t_i, law);
end

%% Plots sim 4, controller 2 M Unknown
plots(t_sim4,q_t_sim4,len_t_sim4,tau_cont_t,4);

%% Controler 3 - PID

% q to get to r_A, but with 1cm error in +z0:
err = [0;0;1e-2];
r_A_w_err = r_A + err;
q_A_w_err = inverse_kin(r_A_w_err);
q_des_0 = q_plan(3,0);

% Main script to solve the system using ode45
% Define initial conditions
int_err_0 = [0;0;0];
err_0 = q_A_w_err-q_des_0;        % Initial position
err_der_0 = [0;0;0];       % Initial velocity
X0 = [int_err_0; err_0; err_der_0];    % Initial state

% Time span for simulation
T_sim5 = T+1;
tspan = [0, T_sim5];

% Options: specify Max step:
options = odeset('MaxStep', 1/10 * dt);  % MaxStep

% Chose the first control law:
law = 3;

% Solve using ode45 - Xdot = state_eq(t, X, law)
[t_sim5, X] = ode45(@(t, X) state_eq(t, X, law), tspan, X0, options);

% Extract positions and velocities
int_err_t_sim5 = X(:, 1:3)';
err_t_sim5 = X(:, 4:6)';
err_der_t_sim5 = X(:, 6+1:end)';

% Extract used tau:
len_t_sim5 = length(t_sim5);

% Get desired joint values:
q_des_t = zeros(3,len_t_sim5);
q_des_dot_t = zeros(3,len_t_sim5);

for i = 1:len_t_sim5
    t_i = t_sim5(i);
    q_des_t(:,i) = q_plan(3,t_i);
    q_des_dot_t(:,i) = q_dot_plan(3,t_i);
end

% Get actual joint values:
q_t_sim5 = err_t_sim5 + q_des_t;
qd_t_sim5 = err_der_t_sim5 + q_des_dot_t;

tau_cont_t = zeros(3,len_t_sim5);

% Chosen control gains:
Kp = 15500.*eye(3); Kd = 900.*eye(3); Ki = 1300.*eye(3);

for i = 1:len_t_sim5
    t_i = t_sim5(i);
    e_ti = err_t_sim5(:,i);
    e_dot_ti = err_der_t_sim5(:,i);
    int_e_ti = int_err_t_sim5(:,i);
    tau_cont_t(:,i) = -Kp*e_ti-Kd*e_dot_ti-Ki*int_e_ti;
end

%% Plots sim 5, controller 3 M Unknown, M=0.5[kg]
plots(t_sim5,q_t_sim5,len_t_sim5,tau_cont_t,5);

%% Controler 3 - PID with M=0[kg]

% q to get to r_A, but with 1cm error in +z0:
err = [0;0;1e-2];
r_A_w_err = r_A + err;
q_A_w_err = inverse_kin(r_A_w_err);
q_des_0 = q_plan(3,0);

% Main script to solve the system using ode45
% Define initial conditions
int_err_0 = [0;0;0];
err_0 = q_A_w_err-q_des_0;        % Initial position
err_der_0 = [0;0;0];       % Initial velocity
X0 = [int_err_0; err_0; err_der_0];    % Initial state

% Time span for simulation
T_sim6 = T+1;
tspan = [0, T_sim6];

% Options: specify Max step:
options = odeset('MaxStep', 1/10 * dt);  % MaxStep

% Chose the first control law:
law = 3;

% Solve using ode45 - Xdot = state_eq(t, X, law)
[t_sim6, X] = ode45(@(t, X) state_eq_noM(t, X, law), tspan, X0, options);

% Extract positions and velocities
int_err_t_sim6 = X(:, 1:3)';
err_t_sim6 = X(:, 4:6)';
err_der_t_sim6 = X(:, 6+1:end)';

% Extract used tau:
len_t_sim6 = length(t_sim6);

% Get desired joint values:
q_des_t = zeros(3,len_t_sim6);
q_des_dot_t = zeros(3,len_t_sim6);

for i = 1:len_t_sim6
    t_i = t_sim6(i);
    q_des_t(:,i) = q_plan(3,t_i);
    q_des_dot_t(:,i) = q_dot_plan(3,t_i);
end

% Get actual joint values:
q_t_sim6 = err_t_sim6 + q_des_t;
qd_t_sim6 = err_der_t_sim6 + q_des_dot_t;

tau_cont_t = zeros(3,len_t_sim6);

% Chosen control gains:
Kp = 15500.*eye(3); Kd = 900.*eye(3); Ki = 1300.*eye(3);

for i = 1:len_t_sim6
    t_i = t_sim6(i);
    e_ti = err_t_sim6(:,i);
    e_dot_ti = err_der_t_sim6(:,i);
    int_e_ti = int_err_t_sim6(:,i);
    tau_cont_t(:,i) = -Kp*e_ti-Kd*e_dot_ti-Ki*int_e_ti;
end

%% Plots sim 6, controller 3 M Unknown, M=0[kg]
plots(t_sim6,q_t_sim6,len_t_sim6,tau_cont_t,6);

%% Controler 4 - Min-Max

% q to get to r_A, but with 1cm error in +z0:
err = [0;0;1e-2];
r_A_w_err = r_A + err;
q_A_w_err = inverse_kin(r_A_w_err);

% Main script to solve the system using ode45
% Define initial conditions
q0 = q_A_w_err;        % Initial position
qd0 = [0;0;0];       % Initial velocity
X0 = [q0; qd0];    % Initial state

% Time span for simulation
T_sim7 = T+1;
tspan = [0, T_sim7];

% Options: specify Max step:
options = odeset('MaxStep', 1/10 * dt);  % MaxStep

% Chose the first control law:
law = 4;

% Solve using ode45 - Xdot = state_eq(t, X, law)
[t_sim7, X] = ode45(@(t, X) state_eq(t, X, law), tspan, X0, options);

% Extract positions and velocities
q_t_sim7 = X(:, 1:3)';
qd_t_sim7 = X(:, 3+1:end)';

% Extract used tau:
len_t_sim7 = length(t_sim7);

tau_cont_t = zeros(3,len_t_sim7);

for i = 1:len_t_sim7
    t_i = t_sim7(i);
    q_ti = q_t_sim7(:,i);
    q_dot_ti = qd_t_sim7(:,i);
    tau_cont_t(:,i) = tau_cont(3,q_ti,q_dot_ti,t_i, law);
end

%% Plots sim 7, controller 4 M unknown
plots(t_sim7,q_t_sim7,len_t_sim7,tau_cont_t,7);

%% Controler 4 - Min-Max BUT NO MASS M

% q to get to r_A, but with 1cm error in +z0:
err = [0;0;1e-2];
r_A_w_err = r_A + err;
q_A_w_err = inverse_kin(r_A_w_err);

% Main script to solve the system using ode45
% Define initial conditions
q0 = q_A_w_err;        % Initial position
qd0 = [0;0;0];       % Initial velocity
X0 = [q0; qd0];    % Initial state

% Time span for simulation
T_sim8 = T+1;
tspan = [0, T_sim8];

% Options: specify Max step:
options = odeset('MaxStep', 1/10 * dt);  % MaxStep

% Chose the first control law:
law = 4;

% Solve using ode45 - Xdot = state_eq_noM(t, X, law)
[t_sim8, X] = ode45(@(t, X) state_eq_noM(t, X, law), tspan, X0, options);

% Extract positions and velocities
q_t_sim8 = X(:, 1:3)';
qd_t_sim8 = X(:, 3+1:end)';

% Extract used tau:
len_t_sim8 = length(t_sim8);

tau_cont_t = zeros(3,len_t_sim8);

for i = 1:len_t_sim8
    t_i = t_sim8(i);
    q_ti = q_t_sim8(:,i);
    q_dot_ti = qd_t_sim8(:,i);
    tau_cont_t(:,i) = tau_cont(3,q_ti,q_dot_ti,t_i, law);
end

%% Plots sim 8, controller 4 M unknown
plots(t_sim8,q_t_sim8,len_t_sim8,tau_cont_t,8);

