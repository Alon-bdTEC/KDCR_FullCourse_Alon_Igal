% inital settings
close all; clear; clc;
%% main code
T = 2;
fs = 100; % Used 100Hz for videos only, req 1kHz (delta_t=0.001[s])
t = 0:1/fs:T;
dt = 1/fs;  % Time step

elbow_chosen = [1,1]; % To choose ELBOW,
% CHANGE HERE AND IN BOTH STATE EQ (state_eq.m and diffM_state_eq.m)

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
filename = 'print_FK.png';
exportgraphics(gcf, filename, 'Resolution', 300); % Higher resolution

%% Save all cases of IK:
d = r_A;

% All elbow combinations: [-1, 1] for each of 2 joints => 4 total
elbows_options = [[1;1], [-1;1], [1;-1], [-1;-1]];

figure('Units','normalized','OuterPosition',[0 0 1 1]); % Fullscreen figure
    
for i = 1:size(elbows_options,2)
    elbows = elbows_options(:,i)';

    % Compute and plot
    q = inverse_kin(d, elbows);

    subplot(2, 2, i);
    print_robot(q);
    title(['elbows = [', num2str(elbows), ']']);

    view([-0.3, -1, 0.3]);
    axis equal;
    grid on;
end
% Save high-res image
filename = 'print_elbows.png';
exportgraphics(gcf, filename, 'Resolution', 300); % Higher resolution


%% Trajectory - video making for [1,1]

elbows = [1,1];

q_t_1_1 = zeros(3,len_t);

for i = 1:len_t
    t_i = t(i);
    q_t_1_1(:,i) = q_plan(3,t_i,elbows);
end

% Video 1 - Izometric view
numFrames = length(t);
videoFile = 'robot_motion_ISO_1_1.avi';
v = VideoWriter(videoFile);  % Default is 'Motion JPEG AVI', allows high frame rate
v.FrameRate = floor(numFrames / T);  % Set frame rate to play in real time
open(v);

figure('Units', 'normalized', 'OuterPosition', [0, 0, 1, 1]);
set(gcf, 'Renderer', 'painters');

for i = 1:numFrames
    clf;
    print_robot(q_t_1_1(:, i));
    title(sprintf('Time: %.3f sec', t(i)), 'FontSize', 14);
    drawnow;

    frame = getframe(gcf);
    writeVideo(v, frame);
end

close(v);

%% Video 2 - Second view
numFrames = length(t);
videoFile = 'robot_motion_View2_1_1.avi';
v = VideoWriter(videoFile);  % Default is 'Motion JPEG AVI', allows high frame rate
v.FrameRate = floor(numFrames / T);  % Set frame rate to play in real time
open(v);

figure('Units', 'normalized', 'OuterPosition', [0, 0, 1, 1]);
set(gcf, 'Renderer', 'painters');

for i = 1:numFrames
    clf;
    print_robot(q_t_1_1(:, i));
    view([-100, 40]);
    title(sprintf('Time: %.3f sec', t(i)), 'FontSize', 14);
    drawnow;

    frame = getframe(gcf);
    writeVideo(v, frame);
end

close(v);

%% Trajectory - video making for elbow_chosen

elbows = elbow_chosen;

q_t_elbow_chosen = zeros(3,len_t);

for i = 1:len_t
    t_i = t(i);
    q_t_elbow_chosen(:,i) = q_plan(3,t_i,elbows);
end

% Video 1 - Izometric view
numFrames = length(t);
videoFile = 'robot_motion_ISO_elbow_chosen.avi';
v = VideoWriter(videoFile);  % Default is 'Motion JPEG AVI', allows high frame rate
v.FrameRate = floor(numFrames / T);  % Set frame rate to play in real time
open(v);

figure('Units', 'normalized', 'OuterPosition', [0, 0, 1, 1]);
set(gcf, 'Renderer', 'painters');

for i = 1:numFrames
    clf;
    print_robot(q_t_elbow_chosen(:, i));
    title(sprintf('Time: %.3f sec', t(i)), 'FontSize', 14);
    drawnow;

    frame = getframe(gcf);
    writeVideo(v, frame);
end

close(v);

%% Video 2 - Second view
numFrames = length(t);
videoFile = 'robot_motion_View2_elbow_chosen.avi';
v = VideoWriter(videoFile);  % Default is 'Motion JPEG AVI', allows high frame rate
v.FrameRate = floor(numFrames / T);  % Set frame rate to play in real time
open(v);

figure('Units', 'normalized', 'OuterPosition', [0, 0, 1, 1]);
set(gcf, 'Renderer', 'painters');

for i = 1:numFrames
    clf;
    print_robot(q_t_elbow_chosen(:, i));
    view([-100, 40]);
    title(sprintf('Time: %.3f sec', t(i)), 'FontSize', 14);
    drawnow;

    frame = getframe(gcf);
    writeVideo(v, frame);
end

close(v);

%% Print q_t for [1,+-1] and x(t),v(t) and a(t) and detJ:
print_trajectory_plots

%% Part 2 - print required tau

tou_vec_t = zeros(3,len_t);
tou_statics_vec_t = zeros(3,len_t);

for i = 1:len_t
    t_i = t(i);
    [tou_vec_t(:,i) , tou_statics_vec_t(:,i) ] = tau_plan(3,t_i,elbow_chosen);
end

% Plot tou:

tou_1 = zeros(1,len_t);
tou_2 = zeros(1,len_t);
tou_3 = zeros(1,len_t);

tou_stat_1 = zeros(1,len_t);
tou_stat_2 = zeros(1,len_t);
tou_stat_3 = zeros(1,len_t);
for i = 1:len_t
    tou_1(i) = tou_vec_t(1,i);
    tou_2(i) = tou_vec_t(2,i);
    tou_3(i) = tou_vec_t(3,i);
    tou_stat_1(i) = tou_statics_vec_t(1,i);
    tou_stat_2(i) = tou_statics_vec_t(2,i);
    tou_stat_3(i) = tou_statics_vec_t(3,i);
end

% Set default appearance
set(groot, 'DefaultTextInterpreter', 'latex', ...
           'DefaultLegendInterpreter', 'latex', ...
           'DefaultAxesFontName', 'Times', ...
           'DefaultAxesFontSize', 12);

% Create fullscreen figure
fig = figure('Units', 'pixels', 'Position', [100, 100, 1000, 600]);

% Global title
sgtitle('Joint Torques for Polynomial Trajectory', 'Interpreter', 'latex');

% Subplot 1
subplot(3, 1, 1);
plot(t, tou_1, 'r');
hold on;
plot(t, tou_stat_1, '--b');
xlabel('Time [s]');
ylabel('$\tau_1$ [N*m]');
legend({'$\tau_1$', '$\tau_1$ static'}, ...
       'Location', 'eastoutside', 'FontSize', 10, 'NumColumns', 1);

% Subplot 2
subplot(3, 1, 2);
plot(t, tou_2, 'g');
hold on;
plot(t, tou_stat_2, '--b');
xlabel('Time [s]');
ylabel('$\tau_2$ [N*m]');
legend({'$\tau_2$', '$\tau_2$ static'}, ...
       'Location', 'eastoutside', 'FontSize', 10, 'NumColumns', 1);

% Subplot 3
subplot(3, 1, 3);
plot(t, tou_3, 'b');
hold on;
plot(t, tou_stat_3, '--b');
xlabel('Time [s]');
ylabel('$\tau_3$ [N]');
legend({'$\tau_3$', '$\tau_3$ static'}, ...
       'Location', 'eastoutside', 'FontSize', 10, 'NumColumns', 1);

% Save high-res figure
exportgraphics(fig, 'ID_Moments_and_Force_Trajectory_Polynomial.png', 'Resolution', 600);

%% Part 4, Simulation of Robot response to tau

% q to get to r_A:
q_A = inverse_kin(r_A, elbow_chosen);

% Main script to solve the system using ode45
% Define initial conditions
q0 = q_A;        % Initial position
qd0 = [0;0;0];       % Initial velocity
X0 = [q0; qd0];    % Initial state

% Time span for simulation
tspan = [0, T];

% Options: specify Max step:
options = odeset('MaxStep', 1/10 * dt);  % MaxStep


% Solve using ode45 - Xdot = state_eq(t, X)
% [t_sim1, X] = ode45(@(t, X) state_eq(t, X), tspan, X0, options);
[t_sim1, X] = ode45(@state_eq, tspan, X0, options);

% Extract positions and velocities
q_t_sim1 = X(:, 1:3);
qd_t_sim1 = X(:, 3+1:end);

%% Plot Comparing planned q_t(t) and q_t_sim(t)

% Plot q in both cases:
q_t_elbow_chosen = zeros(3,len_t);
    
for i = 1:len_t
    t_i = t(i);
    q_t_elbow_chosen(:,i) = q_plan(3,t_i,elbow_chosen);
end

% Create high-quality figure with tighter layout
% Set default appearance
set(groot, 'DefaultTextInterpreter', 'latex', ...
           'DefaultLegendInterpreter', 'latex', ...
           'DefaultAxesFontName', 'Times', ...
           'DefaultAxesFontSize', 12);
figure('Units', 'normalized', 'Position', [0.2 0.2 0.6 0.7]);
% tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
sgtitle('Joint Values $q(t)$, Polynomial Plan Vs Sim1', 'Interpreter', 'latex', 'FontSize', 16);

% Boundary and plot styling constants
theta_bound = pi;
d3_bound = 0.7;
margin = 0.7;
margind3 = 0.2;
line_width = 2;
font_size = 13;

% Common time for text annotation
t_mid = t(round(end/2));

% --- q1 ---
subplot(3, 1, 1);
plot(t, q_t_elbow_chosen(1,:), 'b--', 'LineWidth', line_width);
hold on;
plot(t_sim1, q_t_sim1(:,1)', 'r', 'LineWidth', line_width);
xlabel('Time [s]', 'Interpreter', 'latex', 'FontSize', font_size);
ylabel('$q_1 = \theta_1(t)$ [rad]', 'Interpreter', 'latex', 'FontSize', font_size);
yline(theta_bound, 'r--', 'LineWidth', 1.5);
yline(-theta_bound, 'r--', 'LineWidth', 1.5);
ylim([-theta_bound - margin, theta_bound + margin]);
text(t_mid, theta_bound, '$\pi$', 'Interpreter', 'latex', 'Color', 'r', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
text(t_mid, -theta_bound, '$-\pi$', 'Interpreter', 'latex', 'Color', 'r', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'top');
grid on; box on;
legend({'$q_1$ Planned', '$q_1$ simulation'}, ...
       'Location', 'eastoutside', 'FontSize', 10, 'NumColumns', 1);

% --- q2 ---
subplot(3, 1, 2);
plot(t, q_t_elbow_chosen(2,:), 'b--', 'LineWidth', line_width);
hold on;
plot(t_sim1, q_t_sim1(:,2)', 'r', 'LineWidth', line_width);
xlabel('Time [s]', 'Interpreter', 'latex', 'FontSize', font_size);
ylabel('$q_2 = \theta_2(t)$ [rad]', 'Interpreter', 'latex', 'FontSize', font_size);
yline(theta_bound, 'r--', 'LineWidth', 1.5);
yline(-theta_bound, 'r--', 'LineWidth', 1.5);
ylim([-theta_bound - margin, theta_bound + margin]);
text(t_mid, theta_bound, '$\pi$', 'Interpreter', 'latex', 'Color', 'r', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
text(t_mid, -theta_bound, '$-\pi$', 'Interpreter', 'latex', 'Color', 'r', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'top');
grid on; box on;
legend({'$q_2$ Planned', '$q_2$ simulation'}, ...
       'Location', 'eastoutside', 'FontSize', 10, 'NumColumns', 1);

% --- q3 ---
subplot(3, 1, 3);
plot(t, q_t_elbow_chosen(3,:), 'b--', 'LineWidth', line_width);
hold on;
plot(t_sim1, q_t_sim1(:,3)', 'r', 'LineWidth', line_width);
xlabel('Time [s]', 'Interpreter', 'latex', 'FontSize', font_size);
ylabel('$q_3 = d_3(t)$ [m]', 'Interpreter', 'latex', 'FontSize', font_size);
yline(d3_bound, 'r--', 'LineWidth', 1.5);
yline(-d3_bound, 'r--', 'LineWidth', 1.5);
ylim([-d3_bound - margind3, d3_bound + margind3]);
text(t_mid, d3_bound, '$0.7$', 'Interpreter', 'latex', 'Color', 'r', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
text(t_mid, -d3_bound, '$-0.7$', 'Interpreter', 'latex', 'Color', 'r', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'top');
grid on; box on;
legend({'$q_3$ Planned', '$q_3$ simulation'}, ...
       'Location', 'eastoutside', 'FontSize', 10, 'NumColumns', 1);

% Save high-resolution image
exportgraphics(gcf, 'Joint_values_PlanVsSim1_elbow_chosen.png', 'Resolution', 600);

% Plot error of position:
PathLen = norm(r_A-r_B);

% Desired Positon:
% Compare using t_sim time vector:
lent_sim1 = length(t_sim1);

r_d_t = zeros(3,lent_sim1);
for i = 1:lent_sim1
    t_i = t_sim1(i);
    r_d_t(:,i) = x_plan(3,t_i);
end
% Simulated Position:
r_sim1_t = zeros(3,lent_sim1);
for i = 1:lent_sim1
    [~,r_sim1_t(:,i)] = forward_kin(q_t_sim1(i,:)');
end

err_t = zeros(1,lent_sim1);
for i = 1:lent_sim1
    err_t(i) = norm(r_d_t(:,i)-r_sim1_t(:,i)) * (100/PathLen);
end

% Set default appearance
set(groot, 'DefaultTextInterpreter', 'latex', ...
           'DefaultLegendInterpreter', 'latex', ...
           'DefaultAxesFontName', 'Times', ...
           'DefaultAxesFontSize', 12);

% Create fullscreen figure
fig = figure('Units', 'pixels', 'Position', [100, 100, 600, 600]);
set(gcf, 'Renderer', 'painters');

% Plot
plot(t_sim1, err_t, 'b');
xlabel('Time [s]');
ylabel('err$(t)$ as per. path length');
xlim([0,T])

title('err$(t)$ as percentage of full path length', 'Interpreter', 'latex');
axis equal;
grid on;

% Save high-res image
filename = 'print_err_sim1_elbow_chosen.png';
exportgraphics(gcf, filename, 'Resolution', 300); % Higher resolution

%% Part 4 - video making

videoFile = 'robot_sim1.avi';
v = VideoWriter(videoFile);  % Default is 'Motion JPEG AVI', allows high frame rate
fps = 100;
v.FrameRate = fps;  % Set frame rate to play in real time
open(v);

figure('Units', 'normalized', 'OuterPosition', [0, 0, 1, 1]);
set(gcf, 'Renderer', 'painters');

for i = 1:50:length(t_sim1)
    clf;
    q_ti = q_t_sim1(i, :)';
    print_robot(q_ti);
    title(sprintf('Time: %.3f sec', t_sim1(i)), 'FontSize', 14);
    drawnow;

    frame = getframe(gcf);
    writeVideo(v, frame);
end

close(v);

%% Part 5, Simulation w. different IC

% q to get to r_A:
err = [0;0;1e-2];
r_A_w_err = r_A + err;
q_A_w_err = inverse_kin(r_A_w_err, elbow_chosen);

% Main script to solve the system using ode45
% Define initial conditions
q0 = q_A_w_err;        % Initial position
qd0 = [0;0;0];       % Initial velocity
X0 = [q0; qd0];    % Initial state

% Time span for simulation
tspan = [0, T];

% Options: specify Max step:
options = odeset('MaxStep', 1/10 * dt);  % MaxStep


% Solve using ode45 - Xdot = state_eq(t, X)
[t_sim2, X] = ode45(@state_eq, tspan, X0, options);

% Extract positions and velocities
q_t_sim2 = X(:, 1:3);
qd_t_sim2 = X(:, 3+1:end);


%% Plot Comparing planned q_t(t) and q_t_sim(t)

% Plot q in both cases:
q_t_elbow_chosen = zeros(3,len_t);
    
for i = 1:len_t
    t_i = t(i);
    q_t_elbow_chosen(:,i) = q_plan(3,t_i,elbow_chosen);
end

% Create high-quality figure with tighter layout
% Set default appearance
set(groot, 'DefaultTextInterpreter', 'latex', ...
           'DefaultLegendInterpreter', 'latex', ...
           'DefaultAxesFontName', 'Times', ...
           'DefaultAxesFontSize', 12);
figure('Units', 'normalized', 'Position', [0.2 0.2 0.6 0.7]);
% tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
sgtitle('Joint Values $q(t)$, Polynomial Plan Vs Sim2', 'Interpreter', 'latex', 'FontSize', 16);

% Boundary and plot styling constants
theta_bound = pi;
d3_bound = 0.7;
margin = 0.7;
margind3 = 0.2;
line_width = 2;
font_size = 13;

% Common time for text annotation
t_mid = t(round(end/2));

% --- q1 ---
subplot(3, 1, 1);
plot(t, q_t_elbow_chosen(1,:), 'b--', 'LineWidth', line_width);
hold on;
plot(t_sim2, q_t_sim2(:,1)', 'r', 'LineWidth', line_width);
xlabel('Time [s]', 'Interpreter', 'latex', 'FontSize', font_size);
ylabel('$q_1 = \theta_1(t)$ [rad]', 'Interpreter', 'latex', 'FontSize', font_size);
yline(theta_bound, 'r--', 'LineWidth', 1.5);
yline(-theta_bound, 'r--', 'LineWidth', 1.5);
ylim([-theta_bound - margin, theta_bound + margin]);
text(t_mid, theta_bound, '$\pi$', 'Interpreter', 'latex', 'Color', 'r', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
text(t_mid, -theta_bound, '$-\pi$', 'Interpreter', 'latex', 'Color', 'r', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'top');
grid on; box on;
legend({'$q_1$ Planned', '$q_1$ simulation'}, ...
       'Location', 'eastoutside', 'FontSize', 10, 'NumColumns', 1);

% --- q2 ---
subplot(3, 1, 2);
plot(t, q_t_elbow_chosen(2,:), 'b--', 'LineWidth', line_width);
hold on;
plot(t_sim2, q_t_sim2(:,2)', 'r', 'LineWidth', line_width);
xlabel('Time [s]', 'Interpreter', 'latex', 'FontSize', font_size);
ylabel('$q_2 = \theta_2(t)$ [rad]', 'Interpreter', 'latex', 'FontSize', font_size);
yline(theta_bound, 'r--', 'LineWidth', 1.5);
yline(-theta_bound, 'r--', 'LineWidth', 1.5);
ylim([-theta_bound - margin, theta_bound + margin]);
text(t_mid, theta_bound, '$\pi$', 'Interpreter', 'latex', 'Color', 'r', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
text(t_mid, -theta_bound, '$-\pi$', 'Interpreter', 'latex', 'Color', 'r', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'top');
grid on; box on;
legend({'$q_2$ Planned', '$q_2$ simulation'}, ...
       'Location', 'eastoutside', 'FontSize', 10, 'NumColumns', 1);

% --- q3 ---
subplot(3, 1, 3);
plot(t, q_t_elbow_chosen(3,:), 'b--', 'LineWidth', line_width);
hold on;
plot(t_sim2, q_t_sim2(:,3)', 'r', 'LineWidth', line_width);
xlabel('Time [s]', 'Interpreter', 'latex', 'FontSize', font_size);
ylabel('$q_3 = d_3(t)$ [m]', 'Interpreter', 'latex', 'FontSize', font_size);
yline(d3_bound, 'r--', 'LineWidth', 1.5);
yline(-d3_bound, 'r--', 'LineWidth', 1.5);
ylim([-d3_bound - margind3, d3_bound + margind3]);
text(t_mid, d3_bound, '$0.7$', 'Interpreter', 'latex', 'Color', 'r', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
text(t_mid, -d3_bound, '$-0.7$', 'Interpreter', 'latex', 'Color', 'r', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'top');
grid on; box on;
legend({'$q_3$ Planned', '$q_3$ simulation'}, ...
       'Location', 'eastoutside', 'FontSize', 10, 'NumColumns', 1);

% Save high-resolution image
exportgraphics(gcf, 'Joint_values_PlanVsSim2_elbow_chosen.png', 'Resolution', 600);

% Plot error of position:
PathLen = norm(r_A-r_B);

% Desired Positon:
% Compare using t_sim time vector:
lent_sim2 = length(t_sim2);

r_d_t = zeros(3,lent_sim2);
for i = 1:lent_sim2
    t_i = t_sim2(i);
    r_d_t(:,i) = x_plan(3,t_i);
end
% Simulated Position:
r_sim2_t = zeros(3,lent_sim2);
for i = 1:lent_sim2
    [~,r_sim2_t(:,i)] = forward_kin(q_t_sim2(i,:)');
end

err_t = zeros(1,lent_sim2);
for i = 1:lent_sim2
    err_t(i) = norm(r_d_t(:,i)-r_sim2_t(:,i)) * (100/PathLen);
end

% Set default appearance
set(groot, 'DefaultTextInterpreter', 'latex', ...
           'DefaultLegendInterpreter', 'latex', ...
           'DefaultAxesFontName', 'Times', ...
           'DefaultAxesFontSize', 12);

% Create figure with specific size (not fullscreen!)
fig = figure('Units', 'inches', 'Position', [1, 1, 6, 4]);  % 6x4 inches
set(gcf, 'Renderer', 'painters');

% Plot
plot(t_sim2, err_t, 'b', 'LineWidth', 1.5);
xlabel('Time [s]');
ylabel('err$(t)$ as per. path length');
title('err$(t)$ as percentage of full path length', 'Interpreter', 'latex');

% Crop to visible data
xlim([0, T]);
ylim([min(err_t), max(err_t)]);

% Grid and formatting
grid on;
box on;

% Make axes tight to data and remove white borders
axis tight manual;
set(gca, 'LooseInset', [0,0,0,0]);

% Save high-res tight image
filename = 'print_err_sim2_elbow_chosen.png';
exportgraphics(gca, filename, 'Resolution', 300, 'ContentType', 'image');


%% Part 5 - video making

videoFile = 'robot_sim2.avi';
v = VideoWriter(videoFile);  % Default is 'Motion JPEG AVI', allows high frame rate
fps = 100;
v.FrameRate = fps;  % Set frame rate to play in real time
open(v);

figure('Units', 'normalized', 'OuterPosition', [0, 0, 1, 1]);
set(gcf, 'Renderer', 'painters');

for i = 1:50:length(t_sim2)
    clf;
    q_ti = q_t_sim2(i, :)';
    print_robot(q_ti);
    title(sprintf('Time: %.3f sec', t_sim2(i)), 'FontSize', 14);
    drawnow;

    frame = getframe(gcf);
    writeVideo(v, frame);
end

close(v);

%% Part 6, Simulation w. different mass M

% q to get to r_A:
q_A = inverse_kin(r_A, elbow_chosen);

% Main script to solve the system using ode45
% Define initial conditions
q0 = q_A;        % Initial position
qd0 = [0;0;0];       % Initial velocity
X0 = [q0; qd0];    % Initial state

% Time span for simulation
tspan = [0, T];

% Options: specify Max step:
options = odeset('MaxStep', 1/10 * dt);  % MaxStep


% Solve using ode45 - Xdot = state_eq(t, X)
[t_sim3, X] = ode45(@diffM_state_eq, tspan, X0, options);

% Extract positions and velocities
q_t_sim3 = X(:, 1:3);
qd_t_sim3 = X(:, 3+1:end);

%% Plot Comparing planned q_t(t) and q_t_sim(t)

% Plot q in both cases:
q_t_1_elbow_chosen = zeros(3,len_t);
    
for i = 1:len_t
    t_i = t(i);
    q_t_1_elbow_chosen(:,i) = q_plan(3,t_i,elbow_chosen);
end

% Create high-quality figure with tighter layout
% Set default appearance
set(groot, 'DefaultTextInterpreter', 'latex', ...
           'DefaultLegendInterpreter', 'latex', ...
           'DefaultAxesFontName', 'Times', ...
           'DefaultAxesFontSize', 12);
figure('Units', 'normalized', 'Position', [0.2 0.2 0.6 0.7]);
% tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
sgtitle('Joint Values $q(t)$, Polynomial Plan Vs Sim3', 'Interpreter', 'latex', 'FontSize', 16);

% Boundary and plot styling constants
theta_bound = pi;
d3_bound = 0.7;
margin = 0.7;
margind3 = 0.2;
line_width = 2;
font_size = 13;

% Common time for text annotation
t_mid = t(round(end/2));

% --- q1 ---
subplot(3, 1, 1);
plot(t, q_t_1_elbow_chosen(1,:), 'b--', 'LineWidth', line_width);
hold on;
plot(t_sim3, q_t_sim3(:,1)', 'r', 'LineWidth', line_width);
xlabel('Time [s]', 'Interpreter', 'latex', 'FontSize', font_size);
ylabel('$q_1 = \theta_1(t)$ [rad]', 'Interpreter', 'latex', 'FontSize', font_size);
yline(theta_bound, 'r--', 'LineWidth', 1.5);
yline(-theta_bound, 'r--', 'LineWidth', 1.5);
ylim([-theta_bound - margin, theta_bound + margin]);
text(t_mid, theta_bound, '$\pi$', 'Interpreter', 'latex', 'Color', 'r', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
text(t_mid, -theta_bound, '$-\pi$', 'Interpreter', 'latex', 'Color', 'r', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'top');
grid on; box on;
legend({'$q_1$ Planned', '$q_1$ simulation'}, ...
       'Location', 'eastoutside', 'FontSize', 10, 'NumColumns', 1);

% --- q2 ---
subplot(3, 1, 2);
plot(t, q_t_1_elbow_chosen(2,:), 'b--', 'LineWidth', line_width);
hold on;
plot(t_sim3, q_t_sim3(:,2)', 'r', 'LineWidth', line_width);
xlabel('Time [s]', 'Interpreter', 'latex', 'FontSize', font_size);
ylabel('$q_2 = \theta_2(t)$ [rad]', 'Interpreter', 'latex', 'FontSize', font_size);
yline(theta_bound, 'r--', 'LineWidth', 1.5);
yline(-theta_bound, 'r--', 'LineWidth', 1.5);
ylim([-theta_bound - margin, theta_bound + margin]);
text(t_mid, theta_bound, '$\pi$', 'Interpreter', 'latex', 'Color', 'r', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
text(t_mid, -theta_bound, '$-\pi$', 'Interpreter', 'latex', 'Color', 'r', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'top');
grid on; box on;
legend({'$q_2$ Planned', '$q_2$ simulation'}, ...
       'Location', 'eastoutside', 'FontSize', 10, 'NumColumns', 1);

% --- q3 ---
subplot(3, 1, 3);
plot(t, q_t_1_elbow_chosen(3,:), 'b--', 'LineWidth', line_width);
hold on;
plot(t_sim3, q_t_sim3(:,3)', 'r', 'LineWidth', line_width);
xlabel('Time [s]', 'Interpreter', 'latex', 'FontSize', font_size);
ylabel('$q_3 = d_3(t)$ [m]', 'Interpreter', 'latex', 'FontSize', font_size);
yline(d3_bound, 'r--', 'LineWidth', 1.5);
yline(-d3_bound, 'r--', 'LineWidth', 1.5);
ylim([-d3_bound - margind3, d3_bound + margind3]);
text(t_mid, d3_bound, '$0.7$', 'Interpreter', 'latex', 'Color', 'r', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
text(t_mid, -d3_bound, '$-0.7$', 'Interpreter', 'latex', 'Color', 'r', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'top');
grid on; box on;
legend({'$q_3$ Planned', '$q_3$ simulation'}, ...
       'Location', 'eastoutside', 'FontSize', 10, 'NumColumns', 1);

% Save high-resolution image
exportgraphics(gcf, 'Joint_values_PlanVsSim3_elbow_chosen.png', 'Resolution', 600);

% Plot error of position:
PathLen = norm(r_A-r_B);

% Desired Positon:
% Compare using t_sim time vector:
lent_sim3 = length(t_sim3);

r_d_t = zeros(3,lent_sim3);
for i = 1:lent_sim3
    t_i = t_sim3(i);
    r_d_t(:,i) = x_plan(3,t_i);
end
% Simulated Position:
r_sim3_t = zeros(3,lent_sim3);
for i = 1:lent_sim3
    [~,r_sim3_t(:,i)] = forward_kin(q_t_sim3(i,:)');
end

err_t = zeros(1,lent_sim3);
for i = 1:lent_sim3
    err_t(i) = norm(r_d_t(:,i)-r_sim3_t(:,i)) * (100/PathLen);
end

% Set default appearance
set(groot, 'DefaultTextInterpreter', 'latex', ...
           'DefaultLegendInterpreter', 'latex', ...
           'DefaultAxesFontName', 'Times', ...
           'DefaultAxesFontSize', 12);

% Create figure with specific size (not fullscreen!)
fig = figure('Units', 'inches', 'Position', [1, 1, 6, 4]);  % 6x4 inches
set(gcf, 'Renderer', 'painters');

% Plot
plot(t_sim3, err_t, 'b', 'LineWidth', 1.5);
xlabel('Time [s]');
ylabel('err$(t)$ as per. path length');
title('err$(t)$ as percentage of full path length', 'Interpreter', 'latex');

% Crop to visible data
xlim([0, T]);
ylim([min(err_t), max(err_t)]);

% Grid and formatting
grid on;
box on;

% Make axes tight to data and remove white borders
axis tight manual;
set(gca, 'LooseInset', [0,0,0,0]);

% Save high-res tight image
filename = 'print_err_sim3_elbow_chosen.png';
exportgraphics(gca, filename, 'Resolution', 300, 'ContentType', 'image');

%% Part 6 - video making

videoFile = 'robot_sim3.avi';
v = VideoWriter(videoFile);  % Default is 'Motion JPEG AVI', allows high frame rate
fps = 100;
v.FrameRate = fps;  % Set frame rate to play in real time
open(v);

figure('Units', 'normalized', 'OuterPosition', [0, 0, 1, 1]);
set(gcf, 'Renderer', 'painters');

for i = 1:50:length(t_sim3)
    clf;
    q_ti = q_t_sim3(i, :)';
    print_robot(q_ti);
    title(sprintf('Time: %.3f sec', t_sim3(i)), 'FontSize', 14);
    drawnow;

    frame = getframe(gcf);
    writeVideo(v, frame);
end

close(v);


%% Part 3 - get reaction force on joint 2:

f2_sys2_t = zeros(3,len_t);

for i = 1:len_t
    t_i = t(i);
    f2_sys2_t(:,i) = f2calc(3,t_i,elbow_chosen);
end

% Set default appearance
set(groot, 'DefaultTextInterpreter', 'latex', ...
           'DefaultLegendInterpreter', 'latex', ...
           'DefaultAxesFontName', 'Times', ...
           'DefaultAxesFontSize', 12);

% Create fullscreen figure
% 'Units', 'pixels', 'Position', [100, 100, 600, 600]
fig = figure();
set(gcf, 'Renderer', 'painters');

% Plot
plot(t, f2_sys2_t, 'b');
xlabel('Time [s]');
ylabel('$f_{on joint 2}^{(2)}$ [N]');
xlim([0,T])

title('$f_{on joint 2}^{(2)}$ for required trajectory', 'Interpreter', 'latex');
axis equal;
grid on;
% ylim([-0.8,0.4])

% Save high-res image
filename = 'print_f2_elbow_chosen.png';
exportgraphics(gcf, filename, 'Resolution', 300); % Higher resolution

