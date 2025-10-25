function plots(t_sim,q_t_sim,len_t_sim,tau_cont_t_sim,i_sim)
    %   Inputs:
    %       t_sim - simulation time vecotr
    %       q_t_sim - simulation joint values vecotr
    %       len_t_sim - simulation time vecotr length
    %       tau_cont_t_sim - simulation tau vecotr
    %       i - simulation number
    %   Plots:
    %       q^d(t) vs. q_sim_i(t)
    %       err q(t)-q^d(t)
    %       err in position: (r-r^d)/path_length*100
    %       tau: tau_sim_i(t) and tau^d(t)
    %
    %   Where desired path is for M=0.5
    
    % POINTS A,B:
    r_A = [0.25;0;0.6]; r_B = [0.05;-0.4;0.6];

    % Plot q in both cases:
    q_des_t = zeros(3,len_t_sim);
    
    for i = 1:len_t_sim
        t_i = t_sim(i);
        q_des_t(:,i) = q_plan(3,t_i);
    end

    % Plot styling constants
    line_width = 2;
    font_size = 13;
    
    
    % Create high-quality figure with tighter layout
    % Set default appearance
    set(groot, 'DefaultTextInterpreter', 'latex', ...
               'DefaultLegendInterpreter', 'latex', ...
               'DefaultAxesFontName', 'Times', ...
               'DefaultAxesFontSize', 12);
    figure('Units', 'pixels', 'Position', [100, 100, 1000, 450]);
    sgtitle(['Joint Values $q(t)$, Polynomial Plan Vs Sim',num2str(i_sim)], 'Interpreter', 'latex', 'FontSize', 16);
    
    % --- q1 ---
    subplot(3, 1, 1);
    plot(t_sim, q_des_t(1,:), 'b--', 'LineWidth', line_width);
    hold on;
    plot(t_sim, q_t_sim(1,:), 'r', 'LineWidth', line_width);
    xlabel('Time [s]', 'Interpreter', 'latex', 'FontSize', font_size);
    ylabel('$q_1 = \theta_1(t)$ [rad]', 'Interpreter', 'latex', 'FontSize', font_size);
    grid on; box on;
    legend({'$q_1$ Planned', '$q_1$ simulation'}, ...
           'Location', 'eastoutside', 'FontSize', 10, 'NumColumns', 1);
    
    % --- q2 ---
    subplot(3, 1, 2);
    plot(t_sim, q_des_t(2,:), 'b--', 'LineWidth', line_width);
    hold on;
    plot(t_sim, q_t_sim(2,:), 'r', 'LineWidth', line_width);
    xlabel('Time [s]', 'Interpreter', 'latex', 'FontSize', font_size);
    ylabel('$q_2 = \theta_2(t)$ [rad]', 'Interpreter', 'latex', 'FontSize', font_size);
    grid on; box on;
    legend({'$q_2$ Planned', '$q_2$ simulation'}, ...
           'Location', 'eastoutside', 'FontSize', 10, 'NumColumns', 1);
    
    % --- q3 ---
    subplot(3, 1, 3);
    plot(t_sim, q_des_t(3,:), 'b--', 'LineWidth', line_width);
    hold on;
    plot(t_sim, q_t_sim(3,:), 'r', 'LineWidth', line_width);
    xlabel('Time [s]', 'Interpreter', 'latex', 'FontSize', font_size);
    ylabel('$q_3 = d_3(t)$ [m]', 'Interpreter', 'latex', 'FontSize', font_size);
    grid on; box on;
    legend({'$q_3$ Planned', '$q_3$ simulation'}, ...
           'Location', 'eastoutside', 'FontSize', 10, 'NumColumns', 1);
    
    % Save high-resolution image
    exportgraphics(gcf, ['Joint_values_PlanVsSim',num2str(i_sim),'.png'], 'Resolution', 600);

    % Err q_t-q^d_t
    figure('Units', 'pixels', 'Position', [100, 100, 1000, 450]);
    
    sgtitle(['Joint Values err $e(t)=q(t)-q^d(t)$, Sim',num2str(i_sim)], 'Interpreter', 'latex', 'FontSize', 16);
    
    err_q = q_t_sim - q_des_t;
    
    % --- q1 ---
    subplot(3, 1, 1);
    plot(t_sim, err_q(1,:), 'r', 'LineWidth', line_width);
    xlabel('Time [s]', 'Interpreter', 'latex', 'FontSize', font_size);
    ylabel('err $\theta_1(t)$ [rad]', 'Interpreter', 'latex', 'FontSize', font_size);
    grid on;
    
    % --- q2 ---
    subplot(3, 1, 2);
    plot(t_sim, err_q(2,:), 'r', 'LineWidth', line_width);
    xlabel('Time [s]', 'Interpreter', 'latex', 'FontSize', font_size);
    ylabel('err $\theta_2(t)$ [rad]', 'Interpreter', 'latex', 'FontSize', font_size);
    grid on;
    
    % --- q3 ---
    subplot(3, 1, 3);
    plot(t_sim, err_q(3,:), 'r', 'LineWidth', line_width);
    xlabel('Time [s]', 'Interpreter', 'latex', 'FontSize', font_size);
    ylabel('err $d_3(t)$ [m]', 'Interpreter', 'latex', 'FontSize', font_size);
    grid on;
    
    % Save high-resolution image
    exportgraphics(gcf, ['Joint_values_err_Sim',num2str(i_sim),'.png'], 'Resolution', 600);


    % Plot error of position:
    PathLen = norm(r_A-r_B);
    
    % Desired Positon:
    
    r_d_t = zeros(3,len_t_sim);
    for i = 1:len_t_sim
        t_i = t_sim(i);
        r_d_t(:,i) = x_plan(3,t_i);
    end
    % Simulated Position:
    r_sim_t = zeros(3,len_t_sim);
    for i = 1:len_t_sim
        [~,r_sim_t(:,i)] = forward_kin(q_t_sim(:,i));
    end
    
    err_t = zeros(1,len_t_sim);
    for i = 1:len_t_sim
        err_t(i) = norm(r_d_t(:,i)-r_sim_t(:,i)) * (100/PathLen);
    end
    
    % Set default appearance
    set(groot, 'DefaultTextInterpreter', 'latex', ...
               'DefaultLegendInterpreter', 'latex', ...
               'DefaultAxesFontName', 'Times', ...
               'DefaultAxesFontSize', 12);
    
    % Create fullscreen figure
    figure('Units', 'pixels', 'Position', [100, 100, 1000, 450]);
    % [100, 100, 800, 300]
    set(gcf, 'Renderer', 'painters');
    
    % Plot
    plot(t_sim, err_t, 'b');
    xlabel('Time [s]');
    ylabel(['err$(t)$ as per. path length Sim',num2str(i_sim)]);
    xlim([0,t_sim(end)])
    
    title('err$(t)$ as percentage of full path length', 'Interpreter', 'latex');
    grid on;
    
    % Save high-res image
    filename = ['print_err_sim',num2str(i_sim),'.png'];
    exportgraphics(gcf, filename, 'Resolution', 300); % Higher resolution
    
    % Plot tou:

    tou_cont_1 = zeros(1,len_t_sim);
    tou_cont_2 = zeros(1,len_t_sim);
    tou_cont_3 = zeros(1,len_t_sim);
    
    tou_req_1 = zeros(1,len_t_sim);
    tou_req_2 = zeros(1,len_t_sim);
    tou_req_3 = zeros(1,len_t_sim);
    for i = 1:len_t_sim
        t_i = t_sim(i);
    
        tou_cont_1(i) = tau_cont_t_sim(1,i);
        tou_cont_2(i) = tau_cont_t_sim(2,i);
        tou_cont_3(i) = tau_cont_t_sim(3,i);
        
        tau_plan_tot = tau_plan(3,t_i);
        tou_req_1(i) = tau_plan_tot(1);
        tou_req_2(i) = tau_plan_tot(2);
        tou_req_3(i) = tau_plan_tot(3);
    end
    
    % Set default appearance
    set(groot, 'DefaultTextInterpreter', 'latex', ...
               'DefaultLegendInterpreter', 'latex', ...
               'DefaultAxesFontName', 'Times', ...
               'DefaultAxesFontSize', 12);
    
    % Create fullscreen figure
    fig = figure('Units', 'pixels', 'Position', [100, 100, 1000, 450]);
    
    % Global title
    sgtitle(['Joint Torques Sim',num2str(i_sim)], 'Interpreter', 'latex');
    
    % Subplot 1
    subplot(3, 1, 1);
    plot(t_sim, tou_cont_1, 'r');
    hold on;
    plot(t_sim, tou_req_1, '--b');
    xlabel('Time [s]');
    ylabel('$\tau_1$ [N*m]');
    legend({'$\tau_1$ control', '$\tau_1$ req'}, ...
           'Location', 'eastoutside', 'FontSize', 10, 'NumColumns', 1);
    
    % Subplot 2
    subplot(3, 1, 2);
    plot(t_sim, tou_cont_2, 'g');
    hold on;
    plot(t_sim, tou_req_2, '--b');
    xlabel('Time [s]');
    ylabel('$\tau_2$ [N*m]');
    legend({'$\tau_2$ control', '$\tau_2$ req'}, ...
           'Location', 'eastoutside', 'FontSize', 10, 'NumColumns', 1);
    
    % Subplot 3
    subplot(3, 1, 3);
    plot(t_sim, tou_cont_3, 'b');
    hold on;
    plot(t_sim, tou_req_3, '--b');
    xlabel('Time [s]');
    ylabel('$\tau_3$ [N]');
    legend({'$\tau_3$ control', '$\tau_3$ req'}, ...
           'Location', 'eastoutside', 'FontSize', 10, 'NumColumns', 1);
    
    % Save high-res figure
    exportgraphics(fig, ['TauControlAndReq_Sim',num2str(i_sim),'.png'], 'Resolution', 600);

    
end