% inital settings
close all; clear; clc;
%% main code
T = 2;
fs = 100;
t = 0:1/fs:T;

%% IK for point B, all different solutions:
r_A = [0.25;0;1.1];
d = r_A;

% First junction: +-1 in theta_1:
% gives the joints parameters for the vector t 
figure();
set(gcf, 'Renderer', 'painters');
elbows = [1 1 1];
R = eye(3);
q1 = inverse_kin(d, R, elbows);
print_robot(q1);
figure();
set(gcf, 'Renderer', 'painters');
elbows = [-1 1 1];
R = eye(3);
q2 = inverse_kin(d, R, elbows);
print_robot(q2);

% Second junction: +-1 in d3:
% gives the joints parameters for the vector t 
figure();
set(gcf, 'Renderer', 'painters');
elbows = [1 1 1];
R = eye(3);
q3 = inverse_kin(d, R, elbows);
print_robot(q3);
figure();
set(gcf, 'Renderer', 'painters');
elbows = [1 -1 1];
R = eye(3);
q4 = inverse_kin(d, R, elbows);
print_robot(q4);

% Third junction: +-1 in theta_5:
% gives the joints parameters for the vector t 
figure();
set(gcf, 'Renderer', 'painters');
elbows = [1 1 1];
R = eye(3);
q5 = inverse_kin(d, R, elbows);
print_robot(q5);
figure();
set(gcf, 'Renderer', 'painters');
elbows = [1 1 -1];
R = eye(3);
q6 = inverse_kin(d, R, elbows);
print_robot(q6);

%% Save all cases of IK:
r_A = [0.25;0;1.1];
d = r_A;

% All elbow combinations: [-1,1] for each of 3 joints => 8 total
elbows_options = [-1 1];
combinations = dec2bin(0:7) - '0';  % 8 combinations
combinations(combinations==0) = -1;

% Loop in chunks of 4 to make 2 figures (each with 4 subplots)
for fig_idx = 1:2
    figure('Units','normalized','OuterPosition',[0 0 1 1]); % Fullscreen figure

    for subplot_idx = 1:4
        combo_idx = (fig_idx-1)*4 + subplot_idx;
        elbows = combinations(combo_idx, :);
        
        % Compute and plot
        q = inverse_kin(d, eye(3), elbows);

        subplot(2, 2, subplot_idx);
        print_robot(q);
        title(['elbows = [', num2str(elbows), ']']);

        view([-0.3, -1, 0.3]);
        axis equal;
        grid on;
    end

    % Save high-res image
    filename = ['print_elbows_group_', num2str(fig_idx), '.png'];
    exportgraphics(gcf, filename, 'Resolution', 300); % Higher resolution
    close(gcf);
end




%% Trajectory - video making
type_trajectory = 3;
q_t = q_plan(type_trajectory,t);

%% Recored Video
% High-quality video settings
video = VideoWriter('robot_motion_type1', 'MPEG-4');
video.FrameRate = fs;
video.Quality = 100;
open(video);

figure('Position', [100, 100, 800, 800]);
set(gcf, 'Renderer', 'painters');

for i = 1:length(t)
    clf;
    print_robot(q_t(:, i));
    title(sprintf('Time: %.2f sec', t(i)), 'FontSize', 14);
    drawnow;

    frame = getframe(gcf);
    writeVideo(video, frame);
end

close(video);

%% Plot q_t:

% Loop through each trajectory type
for i = 1:3
    % Calculate joint values for each type
    q_t = q_plan(i, t);
    
    % Create the figure with 3x2 subplots
    figure;
    sgtitle(['Joint values q(t) for Trajectory Type ', num2str(i)], 'Interpreter', 'latex');
    
    % Define the boundary values
    theta_bound = pi;
    d3_bound = 0.7;
    margin = 0.7; % Margin for ylim in theta_i
    margind3 = 0.2; % Margin for ylim in d3
    
    % Plot q1 = theta_1
    subplot(3, 2, 1);
    plot(t, q_t(1, :), 'b');
    xlabel('Time [s]', 'Interpreter', 'latex');
    ylabel('$q_1 = \theta_1(t)$ [rad]', 'Interpreter', 'latex');
    yline(theta_bound, 'r--');
    yline(-theta_bound, 'r--');
    ylim([-theta_bound - margin, theta_bound + margin]);
    text(t(round(end/2)), theta_bound, '\pi', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', 'Color', 'red');
    text(t(round(end/2)), -theta_bound, '-\pi', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'top', 'Color', 'red');
    
    % Plot q2 = theta_2
    subplot(3, 2, 2);
    plot(t, q_t(2, :), 'b');
    xlabel('Time [s]', 'Interpreter', 'latex');
    ylabel('$q_2 = \theta_2(t)$ [rad]', 'Interpreter', 'latex');
    yline(theta_bound, 'r--');
    yline(-theta_bound, 'r--');
    ylim([-theta_bound - margin, theta_bound + margin]);
    text(t(round(end/2)), theta_bound, '\pi', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', 'Color', 'red');
    text(t(round(end/2)), -theta_bound, '-\pi', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'top', 'Color', 'red');
    
    % Plot q3 = d_3
    subplot(3, 2, 3);
    plot(t, q_t(3, :), 'b');
    xlabel('Time [s]', 'Interpreter', 'latex');
    ylabel('$q_3 = d_3(t)$ [m]', 'Interpreter', 'latex');
    yline(d3_bound, 'r--');
    yline(-d3_bound, 'r--');
    ylim([-d3_bound - margind3, d3_bound + margind3]);
    text(t(round(end/2)), d3_bound, '0.7', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', 'Color', 'red');
    text(t(round(end/2)), -d3_bound, '-0.7', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'top', 'Color', 'red');
    
    % Plot q4 = theta_4
    subplot(3, 2, 4);
    plot(t, q_t(4, :), 'b');
    xlabel('Time [s]', 'Interpreter', 'latex');
    ylabel('$q_4 = \theta_4(t)$ [rad]', 'Interpreter', 'latex');
    yline(theta_bound, 'r--');
    yline(-theta_bound, 'r--');
    ylim([-theta_bound - margin, theta_bound + margin]);
    text(t(round(end/2)), theta_bound, '\pi', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', 'Color', 'red');
    text(t(round(end/2)), -theta_bound, '-\pi', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'top', 'Color', 'red');
    
    % Plot q5 = theta_5
    subplot(3, 2, 5);
    plot(t, q_t(5, :), 'b');
    xlabel('Time [s]', 'Interpreter', 'latex');
    ylabel('$q_5 = \theta_5(t)$ [rad]', 'Interpreter', 'latex');
    yline(theta_bound, 'r--');
    yline(-theta_bound, 'r--');
    ylim([-theta_bound - margin, theta_bound + margin]);
    text(t(round(end/2)), theta_bound, '\pi', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', 'Color', 'red');
    text(t(round(end/2)), -theta_bound, '-\pi', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'top', 'Color', 'red');
    
    % Plot q6 = theta_6
    subplot(3, 2, 6);
    plot(t, q_t(6, :), 'b');
    xlabel('Time [s]', 'Interpreter', 'latex');
    ylabel('$q_6 = \theta_6(t)$ [rad]', 'Interpreter', 'latex');
    yline(theta_bound, 'r--');
    yline(-theta_bound, 'r--');
    ylim([-theta_bound - margin, theta_bound + margin]);
    text(t(round(end/2)), theta_bound, '\pi', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', 'Color', 'red');
    text(t(round(end/2)), -theta_bound, '-\pi', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'top', 'Color', 'red');
    
    % Save the figure as a high-quality PNG file
    exportgraphics(gcf, ['Joint_values_q_t_Trajectory_Type_', num2str(i), '.png']);
end

%% Compute and Plot q_dot:

% Compute and Plot q_dot (dq/dt), from direct derivative
% and from J_L inversion

dt = 1/fs;  % Time step
for i = 1:3
    % Calculate joint values for each type
    q_t = q_plan(i, t);

    q_dot_FromJ_L = q_dot_plan(i, t);
    
    q_dot = diff(q_t, 1, 2) / dt;  % First derivative
    t_dot = t(1:end-1);  % Time vector for derivative
    
    figure;
    sgtitle(['Joint Velocities $\dot{q}(t)$ for Trajectory Type ', num2str(i)], 'Interpreter', 'latex');
    
    for j = 1:6
        subplot(3, 2, j);
        hold on;
        plot(t_dot, q_dot(j, :), 'm', 'LineWidth', 2, 'DisplayName', 'Direct Derivative');
        plot(t, q_dot_FromJ_L(j, :), 'b', 'DisplayName', 'From J_L');
        xlabel('Time [s]', 'Interpreter', 'latex');
        if j == 3
            ylabel(['$\dot{q}_', num2str(j), '(t)$ [m/s]'], 'Interpreter', 'latex');
        else
            ylabel(['$\dot{q}_', num2str(j), '(t)$ [rad/s]'], 'Interpreter', 'latex');
        end
        hold off;
    end
    
    % Add a single legend for the whole figure outside the subplots
    legend('Direct', 'J Inv', 'Interpreter', 'latex', 'Location', 'best', 'Orientation', 'horizontal');
    
    exportgraphics(gcf, ['Joint_velocities_q_dot_Trajectory_Type_', num2str(i), '.png']);
end


%% Compute and Plot q_dot_dot:

% Compute and Plot q_dot_dot (d^2q/dt^2), from direct second derivative
% and from planned q_dot2

dt = 1/fs;  % Time step
for i = 1:3
    % Calculate joint values and derivatives for each type
    q_t = q_plan(i, t);
    q_dot2_FromPlan = q_dot2_plan(i, t);

    q_dot = diff(q_t, 1, 2) / dt;        % First derivative
    q_dot_dot = diff(q_dot, 1, 2) / dt;   % Second derivative
    t_dot_dot = t(1:end-2);               % Time vector for second derivative

    figure;
    sgtitle(['Joint Accelerations $\ddot{q}(t)$ for Trajectory Type ', num2str(i)], 'Interpreter', 'latex');

    for j = 1:6
        subplot(3, 2, j);
        hold on;
        plot(t_dot_dot, q_dot_dot(j, :), 'm', 'LineWidth', 2, 'DisplayName', 'Direct Second Derivative');
        plot(t, q_dot2_FromPlan(j, :), 'b', 'DisplayName', 'From Plan');
        xlabel('Time [s]', 'Interpreter', 'latex');
        if j == 3
            ylabel(['$\ddot{q}_', num2str(j), '(t) [m/s^2]$'], 'Interpreter', 'latex');
        else
            ylabel(['$\ddot{q}_', num2str(j), '(t) [rad/s^2]$'], 'Interpreter', 'latex');
        end
        hold off;
    end

    % Add a single legend for the whole figure outside the subplots
    legend('Direct', 'From Plan', 'Interpreter', 'latex', 'Location', 'best', 'Orientation', 'horizontal');

    exportgraphics(gcf, ['Joint_accelerations_q_dot_dot_Trajectory_Type_', num2str(i), '.png']);
end



%% Plot r,r_dot and r_dotdot for path:

% Loop through each trajectory type
for i = 1:3
    % Calculate position, velocity, and acceleration for the current trajectory type
    r_t = x_plan(i, t);
    v_t = v_plan(i, t);
    a_t = a_plan(i, t);
    
    % Plot position
    figure;
    sgtitle(['Position for Trajectory Type ', num2str(i)]);
    subplot(3, 1, 1);
    plot(t, r_t(1, :), 'r'); % Red for X
    xlabel('Time [s]');
    ylabel('X [m]');
    
    subplot(3, 1, 2);
    plot(t, r_t(2, :), 'g'); % Green for Y
    xlabel('Time [s]');
    ylabel('Y [m]');
    
    subplot(3, 1, 3);
    plot(t, r_t(3, :), 'b'); % Blue for Z
    xlabel('Time [s]');
    ylabel('Z [m]');
    
    % Save position figure
    exportgraphics(gcf, ['Position_Trajectory_Type_', num2str(i), '.png']);
    
    % Plot velocity
    figure;
    sgtitle(['Velocity for Trajectory Type ', num2str(i)]);
    subplot(3, 1, 1);
    plot(t, v_t(1, :), 'r'); % Red for X
    xlabel('Time [s]');
    ylabel('X [m/s]');
    
    subplot(3, 1, 2);
    plot(t, v_t(2, :), 'g'); % Green for Y
    xlabel('Time [s]');
    ylabel('Y [m/s]');
    
    subplot(3, 1, 3);
    plot(t, v_t(3, :), 'b'); % Blue for Z
    xlabel('Time [s]');
    ylabel('Z [m/s]');
    
    % Save velocity figure
    exportgraphics(gcf, ['Velocity_Trajectory_Type_', num2str(i), '.png']);
    
    % Plot acceleration
    figure;
    sgtitle(['Acceleration for Trajectory Type ', num2str(i)]);
    subplot(3, 1, 1);
    plot(t, a_t(1, :), 'r'); % Red for X
    xlabel('Time [s]');
    ylabel('X [m/s^2]');
    
    subplot(3, 1, 2);
    plot(t, a_t(2, :), 'g'); % Green for Y
    xlabel('Time [s]');
    ylabel('Y [m/s^2]');
    
    subplot(3, 1, 3);
    plot(t, a_t(3, :), 'b'); % Blue for Z
    xlabel('Time [s]');
    ylabel('Z [m/s^2]');
    
    % Save acceleration figure
    exportgraphics(gcf, ['Acceleration_Trajectory_Type_', num2str(i), '.png']);
end

