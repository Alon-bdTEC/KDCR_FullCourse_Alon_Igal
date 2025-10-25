function print_trajectory_plots
    % print_trajectory_plots - plots q(t) for elbows=[1,+-1] and x(t),v(t)
    % and a(t) of path
    
    T = 2;
    fs = 100; % Used 100Hz for videos only, req 1kHz (delta_t=0.001[s])
    t = 0:1/fs:T;

    len_t = length(t);

    % Plot q_t_1_1:
    
    
    elbows = [1,1];
    
    q_t_1_1 = zeros(3,len_t);
    
    for i = 1:len_t
        t_i = t(i);
        q_t_1_1(:,i) = q_plan(3,t_i,elbows);
    end
    
    % Create high-quality figure with tighter layout
    figure('Units', 'normalized', 'Position', [0.2 0.2 0.6 0.7]);
    tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    sgtitle('Joint Values $q(t)$, [1,1] for Type: Polynomial', 'Interpreter', 'latex', 'FontSize', 16);
    
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
    nexttile;
    plot(t, q_t_1_1(1,:), 'b', 'LineWidth', line_width);
    xlabel('Time [s]', 'Interpreter', 'latex', 'FontSize', font_size);
    ylabel('$q_1 = \theta_1(t)$ [rad]', 'Interpreter', 'latex', 'FontSize', font_size);
    yline(theta_bound, 'r--', 'LineWidth', 1.5);
    yline(-theta_bound, 'r--', 'LineWidth', 1.5);
    ylim([-theta_bound - margin, theta_bound + margin]);
    text(t_mid, theta_bound, '$\pi$', 'Interpreter', 'latex', 'Color', 'r', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
    text(t_mid, -theta_bound, '$-\pi$', 'Interpreter', 'latex', 'Color', 'r', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'top');
    grid on; box on;
    
    % --- q2 ---
    nexttile;
    plot(t, q_t_1_1(2,:), 'b', 'LineWidth', line_width);
    xlabel('Time [s]', 'Interpreter', 'latex', 'FontSize', font_size);
    ylabel('$q_2 = \theta_2(t)$ [rad]', 'Interpreter', 'latex', 'FontSize', font_size);
    yline(theta_bound, 'r--', 'LineWidth', 1.5);
    yline(-theta_bound, 'r--', 'LineWidth', 1.5);
    ylim([-theta_bound - margin, theta_bound + margin]);
    text(t_mid, theta_bound, '$\pi$', 'Interpreter', 'latex', 'Color', 'r', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
    text(t_mid, -theta_bound, '$-\pi$', 'Interpreter', 'latex', 'Color', 'r', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'top');
    grid on; box on;
    
    % --- q3 ---
    nexttile;
    plot(t, q_t_1_1(3,:), 'b', 'LineWidth', line_width);
    xlabel('Time [s]', 'Interpreter', 'latex', 'FontSize', font_size);
    ylabel('$q_3 = d_3(t)$ [m]', 'Interpreter', 'latex', 'FontSize', font_size);
    yline(d3_bound, 'r--', 'LineWidth', 1.5);
    yline(-d3_bound, 'r--', 'LineWidth', 1.5);
    ylim([-d3_bound - margind3, d3_bound + margind3]);
    text(t_mid, d3_bound, '$0.7$', 'Interpreter', 'latex', 'Color', 'r', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
    text(t_mid, -d3_bound, '$-0.7$', 'Interpreter', 'latex', 'Color', 'r', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'top');
    grid on; box on;
    
    % Save high-resolution image
    exportgraphics(gcf, 'Joint_values_q_t_1_1_Polynomial.png', 'Resolution', 600);
    
    % Plot q_t_1_neg1:
    
    elbows = [1,-1];
    
    q_t_1_neg1 = zeros(3,len_t);
    
    for i = 1:len_t
        t_i = t(i);
        q_t_1_neg1(:,i) = q_plan(3,t_i,elbows);
    end
    
    % Create high-quality figure with tighter layout
    figure('Units', 'normalized', 'Position', [0.2 0.2 0.6 0.7]);
    tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    sgtitle('Joint Values $q(t)$, [1,-1] for Type: Polynomial', 'Interpreter', 'latex', 'FontSize', 16);
    
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
    nexttile;
    plot(t, q_t_1_neg1(1,:), 'b', 'LineWidth', line_width);
    xlabel('Time [s]', 'Interpreter', 'latex', 'FontSize', font_size);
    ylabel('$q_1 = \theta_1(t)$ [rad]', 'Interpreter', 'latex', 'FontSize', font_size);
    yline(theta_bound, 'r--', 'LineWidth', 1.5);
    yline(-theta_bound, 'r--', 'LineWidth', 1.5);
    ylim([-theta_bound - margin, theta_bound + margin]);
    text(t_mid, theta_bound, '$\pi$', 'Interpreter', 'latex', 'Color', 'r', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
    text(t_mid, -theta_bound, '$-\pi$', 'Interpreter', 'latex', 'Color', 'r', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'top');
    grid on; box on;
    
    % --- q2 ---
    nexttile;
    plot(t, q_t_1_neg1(2,:), 'b', 'LineWidth', line_width);
    xlabel('Time [s]', 'Interpreter', 'latex', 'FontSize', font_size);
    ylabel('$q_2 = \theta_2(t)$ [rad]', 'Interpreter', 'latex', 'FontSize', font_size);
    yline(theta_bound, 'r--', 'LineWidth', 1.5);
    yline(-theta_bound, 'r--', 'LineWidth', 1.5);
    ylim([-theta_bound - margin, theta_bound + margin]);
    text(t_mid, theta_bound, '$\pi$', 'Interpreter', 'latex', 'Color', 'r', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
    text(t_mid, -theta_bound, '$-\pi$', 'Interpreter', 'latex', 'Color', 'r', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'top');
    grid on; box on;
    
    % --- q3 ---
    nexttile;
    plot(t, q_t_1_neg1(3,:), 'b', 'LineWidth', line_width);
    xlabel('Time [s]', 'Interpreter', 'latex', 'FontSize', font_size);
    ylabel('$q_3 = d_3(t)$ [m]', 'Interpreter', 'latex', 'FontSize', font_size);
    yline(d3_bound, 'r--', 'LineWidth', 1.5);
    yline(-d3_bound, 'r--', 'LineWidth', 1.5);
    ylim([-d3_bound - margind3, d3_bound + margind3]);
    text(t_mid, d3_bound, '$0.7$', 'Interpreter', 'latex', 'Color', 'r', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
    text(t_mid, -d3_bound, '$-0.7$', 'Interpreter', 'latex', 'Color', 'r', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'top');
    grid on; box on;
    
    % Save high-resolution image
    exportgraphics(gcf, 'Joint_values_q_t_1_neg1_Polynomial.png', 'Resolution', 600);
    
    
    % Plot r,r_dot and r_dotdot for path:
    
    % Calculate position, velocity, and acceleration for the current trajectory type
    
    r_t = zeros(3,len_t);
    v_t = zeros(3,len_t);
    a_t = zeros(3,len_t);
    
    for i = 1:len_t
        t_i = t(i);
        r_t(:,i) = x_plan(3,t_i);
        v_t(:,i) = v_plan(3,t_i);
        a_t(:,i) = a_plan(3,t_i);
    end
    
    % Plot position
    figure;
    sgtitle('Position for Trajectory Type Polynomial');
    subplot(3, 1, 1);
    plot(t, r_t(1, :), 'r'); % Red for X
    xlabel('Time [s]');
    ylabel('$X$ [m]', 'Interpreter', 'latex');
    
    subplot(3, 1, 2);
    plot(t, r_t(2, :), 'g'); % Green for Y
    xlabel('Time [s]');
    ylabel('$Y$ [m]', 'Interpreter', 'latex');
    
    subplot(3, 1, 3);
    plot(t, r_t(3, :), 'b'); % Blue for Z
    xlabel('Time [s]');
    ylabel('$Z$ [m]', 'Interpreter', 'latex');
    
    % Save position figure
    exportgraphics(gcf, 'Position_Trajectory_Type_Polynomial.png');
    
    % Plot velocity
    figure;
    sgtitle('Velocity for Trajectory Type Polynomial');
    subplot(3, 1, 1);
    plot(t, v_t(1, :), 'r'); % Red for X
    xlabel('Time [s]');
    ylabel('$X$ [m]', 'Interpreter', 'latex');
    
    subplot(3, 1, 2);
    plot(t, v_t(2, :), 'g'); % Green for Y
    xlabel('Time [s]');
    ylabel('$Y$ [m]', 'Interpreter', 'latex');
    
    subplot(3, 1, 3);
    plot(t, v_t(3, :), 'b'); % Blue for Z
    xlabel('Time [s]');
    ylabel('$Z$ [m]', 'Interpreter', 'latex');
    
    % Save velocity figure
    exportgraphics(gcf, 'Velocity_Trajectory_Type_Polynomial.png');
    
    % Plot acceleration
    figure;
    sgtitle('Acceleration for Trajectory Type Polynomial');
    subplot(3, 1, 1);
    plot(t, a_t(1, :), 'r'); % Red for X
    xlabel('Time [s]');
    ylabel('$X$ [m]', 'Interpreter', 'latex');
    
    subplot(3, 1, 2);
    plot(t, a_t(2, :), 'g'); % Green for Y
    xlabel('Time [s]');
    ylabel('$Y$ [m]', 'Interpreter', 'latex');
    
    subplot(3, 1, 3);
    plot(t, a_t(3, :), 'b'); % Blue for Z
    xlabel('Time [s]');
    ylabel('$Z$ [m]', 'Interpreter', 'latex');
    
    % Save acceleration figure
    exportgraphics(gcf, 'Acceleration_Trajectory_Type_Polynomial.png');

    
end