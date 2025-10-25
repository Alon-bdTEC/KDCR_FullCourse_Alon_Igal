% inital settings
close all; clear; clc;
%% main code
T = 2;
fs = 100;
t = 0:1/fs:T;

%% IK for point on part 1, all different solutions:
X_p1 = [1.5;0;deg2rad(10)];

% First junction: +-1 in theta_1:
% gives the joints parameters for the vector t 
figure();
set(gcf, 'Renderer', 'painters');
elbows = [1 1 1];
q1 = inverse_kin(X_p1, elbows);
print_robot(X_p1,q1);
figure();
set(gcf, 'Renderer', 'painters');
elbows = [-1 1 1];
q2 = inverse_kin(X_p1, elbows);
print_robot(X_p1,q2);

% Second junction: +-1 in d3:
% gives the joints parameters for the vector t 
figure();
set(gcf, 'Renderer', 'painters');
elbows = [1 1 1];
q3 = inverse_kin(X_p1, elbows);
print_robot(X_p1,q3);
figure();
set(gcf, 'Renderer', 'painters');
elbows = [1 -1 1];
q4 = inverse_kin(X_p1, elbows);
print_robot(X_p1,q4);

% Third junction: +-1 in theta_5:
% gives the joints parameters for the vector t 
figure();
set(gcf, 'Renderer', 'painters');
elbows = [1 1 1];
q5 = inverse_kin(X_p1, elbows);
print_robot(X_p1,q5);
figure();
set(gcf, 'Renderer', 'painters');
elbows = [1 1 -1];
q6 = inverse_kin(X_p1, elbows);
print_robot(X_p1,q6);

%% Save all cases of IK:
X_p1 = [1.5;0;deg2rad(10)];

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
        q = inverse_kin(X_p1, elbows);

        subplot(2, 2, subplot_idx);
        print_robot(X_p1,q);
        title(['elbows = [', num2str(elbows), ']']);

        axis equal;
        grid on;
    end

    % Save high-res image
    filename = ['print_elbows_group_', num2str(fig_idx), '.png'];
    exportgraphics(gcf, filename, 'Resolution', 300); % Higher resolution
    close(gcf);
end

%% Make high-quality video of robot doing the motion, for part 3:

% Get joint values for part 3:
[q_t, X_t] = q_part3_plan(t);

% High-quality video settings
video = VideoWriter('robot_motion_constSpeed', 'MPEG-4');
video.FrameRate = fs;
video.Quality = 100;
open(video);

figure();
set(gcf, 'Renderer', 'painters');
set(gcf, 'Position', [100, 100, 1200, 900]); % Bigger figure for better resolution

N_t = zeros(3,length(t));

for i = 1:length(t)
    clf;
    print_robot(X_t(:, i), q_t(:, i));
    title(sprintf('Time: %.2f sec', t(i)), 'FontSize', 14);
    drawnow;

    % Capture frame at higher resolution
    frame = getframe(gcf);
    writeVideo(video, frame);

end

close(video);

%% FK:
q_in = deg2rad([90;120;40]);
t1 = q_in(1);t2 = q_in(2);t3 = q_in(3);
r = 1.5;L = 3; R = 3.5;

% From a0 to a6 (p(t)=a0+a1t+...+a6t^6)
numeric_coeffs = evaluate_t_poly(q_in);

% Solve polynomial for roots of t: needs last coeff to be a0
t_roots = roots(numeric_coeffs);

% Initialize array for phi
phi_vals = [];


for k = 1:length(t_roots)
    t_val = t_roots(k);
    
    % Compute C and S
    C = (1 - t_val^2) / (1 + t_val^2);
    S = (2 * t_val) / (1 + t_val^2);
    
    % Check if both are real and in [-1, 1]
    if isreal(C) && isreal(S) && abs(C) <= 1 && abs(S) <= 1
        % Compute phi
        phi = atan2(S, C);
        phi_vals(end+1) = phi;
    end
end

disp('Valid phi values:');
disp(phi_vals);

x_vals = zeros(size(phi_vals));
y_vals = zeros(size(phi_vals));

% Loop through relevant phi's
for i = 1:length(phi_vals)
    phi_i = phi_vals(i);
    % Now, solve for x,y using the FK formulas:
    alpha = deg2rad(60);
    C = cos(phi_i);S=sin(phi_i);
    A1 = 2*R*sin(t2) - 2*R*sin(t1) + 2*C*r*cos(alpha) - 2*S*r*sin(alpha);
    A2 = 2*R*cos(t1) - 2*R*cos(t2) + 2*C*r*sin(alpha) + 2*S*r*cos(alpha);
    A3 = - (r^2 + 2*R*sin(alpha - t1)*C*r + 2*R*cos(alpha - t1)*S*r);
    B1 = 2*R*sin(t3) - 2*R*sin(t2) - 2*C*r;
    B2 = 2*R*cos(t2) - 2*S*r - 2*R*cos(t3);
    B3 = - (- r^2 + 2*R*sin(t3)*C*r - 2*R*cos(t3)*S*r);
    
    A_mat = [A1 A2;B1 B2]; % Coefficient matrix
    b_vec = [A3; B3];                     % RHS vector
    
    sol = A_mat\b_vec; % Adjugate of A
    x_vals(i) = sol(1);
    y_vals(i) = sol(2);
end

% Draw FK solution:
% Create a 2x2 subplot figure for FK solution
figure();
set(gcf, 'Renderer', 'painters');
set(gcf, 'Position', [100, 100, 1200, 900]); % Bigger figure for better resolution

for i = 1:4
    subplot(2, 2, i);
    print_robot_w_circ([x_vals(i); y_vals(i); phi_vals(i)], q_in);
    title(['FK Solution ', num2str(i)]);
end

% Save high-res image
exportgraphics(gcf, 'RealValue_FK_Sol.png', 'Resolution', 300);

%% Now solve IK for that position of plate
% Loop over 4 FK solutions
for i = 1:4
    % Extract FK task vector
    X_FK = [x_vals(i); y_vals(i); phi_vals(i)];

    % All elbow combinations: [-1,1] for each of 3 joints => 8 total
    combinations = dec2bin(0:7) - '0';  % Binary combinations
    combinations(combinations == 0) = -1;

    % Loop in chunks of 4 to make 2 figures (each with 4 subplots)
    for fig_idx = 1:2
        figure('Units', 'normalized', 'OuterPosition', [0 0 1 1]); % Fullscreen figure

        for subplot_idx = 1:4
            combo_idx = (fig_idx - 1) * 4 + subplot_idx;
            elbows = combinations(combo_idx, :);

            % Compute and plot
            q = inverse_kin(X_FK, elbows);
            
            disp('thetas for each IK:');
            disp(rad2deg(q));

            % Skip if inverse kinematics failed
            if isempty(q)
                continue;
            end

            subplot(2, 2, subplot_idx);
            print_robot(X_FK, q);
            title(['elbows = [', num2str(elbows), ']']);
            axis equal;
            grid on;
        end

        % Save high-res image
        filename = ['FK_IK_Compare_Case', num2str(i), '_', num2str(fig_idx), '.png'];
        exportgraphics(gcf, filename, 'Resolution', 300); % Higher resolution
        close(gcf);
    end
end


%% Plot q(t) for Part 3:

% Calculate joint values for each type
[q_t,~] = q_part3_plan(t);

% Create the figure with 3x2 subplots
figure;
sgtitle('Joint values q(t) for Trajectory ConstSpeed ', 'Interpreter', 'latex');


% Plot q1 = theta_1
subplot(3, 1, 1);
plot(t, rad2deg(q_t(1, :)), 'b');
xlabel('Time [s]', 'Interpreter', 'latex');
ylabel('$q_1 = \theta_1(t)$ (deg)', 'Interpreter', 'latex');

% Plot q2 = theta_2
subplot(3, 1, 2);
plot(t, rad2deg(q_t(2, :)), 'b');
xlabel('Time [s]', 'Interpreter', 'latex');
ylabel('$q_2 = \theta_2(t)$ (deg)', 'Interpreter', 'latex');

% Plot q3 = theta_3
subplot(3, 1, 3);
plot(t, rad2deg(q_t(3, :)), 'b');
xlabel('Time [s]', 'Interpreter', 'latex');
ylabel('$q_3 = \theta_3(t)$ (deg)', 'Interpreter', 'latex');

% Save the figure as a high-quality PNG file
exportgraphics(gcf, ['Joint_values_q_t_Trajectory_ConstSpeed', '.png']);

%% Print robot at specific times t = [0, 0.5, 1, 1.5, 2], for part 3:

t_sample = [0, 0.5, 1, 1.5, 2];
[q_sample, X_sample] = q_part3_plan(t_sample);

figure();
set(gcf, 'Renderer', 'painters');
set(gcf, 'Position', [100, 100, 1200, 900]);

for i = 1:length(t_sample)
    subplot(2, 3, i); % Arrange in 2 rows, 3 columns
    print_robot(X_sample(:, i), q_sample(:, i));
    title(sprintf('t = %.1f sec', t_sample(i)), 'FontSize', 12);
end

% Save the whole figure as a high-quality PNG
exportgraphics(gcf, ['Print_Robot_P3_SpecificTimes', '.png']);

%% Part 5 - Check where IK has sol:

% Parameters
phi = pi/18;
y = 0;
% Robot given link lengths
r=1.5;
L=3;
R=3.5;

x_vals = linspace(0, 2, 100);
delta1_vals = zeros(size(x_vals));
delta2_vals = zeros(size(x_vals));
delta3_vals = zeros(size(x_vals));

for i = 1:length(x_vals)
    x = x_vals(i);

    % Calculate A', B', C'
    r_B_tag = [x; y];
    r_A_tag = r_B_tag + r * [cos(phi + pi/3); sin(phi + pi/3)];
    r_C_tag = r_B_tag + r * [cos(phi); sin(phi)];

    % Coefficients for IK
    a1 = 2 * R * r_A_tag(2);
    b1 = -2 * R * r_A_tag(1);
    f1 = L^2 - r_A_tag(2)^2 - r_A_tag(1)^2 - R^2;

    a2 = 2 * R * r_B_tag(2);
    b2 = -2 * R * r_B_tag(1);
    f2 = L^2 - r_B_tag(2)^2 - r_B_tag(1)^2 - R^2;

    a3 = 2 * R * r_C_tag(2);
    b3 = -2 * R * r_C_tag(1);
    f3 = L^2 - r_C_tag(2)^2 - r_C_tag(1)^2 - R^2;

    % Discriminants
    delta1_vals(i) = a1^2 + b1^2 - f1^2;
    delta2_vals(i) = a2^2 + b2^2 - f2^2;
    delta3_vals(i) = a3^2 + b3^2 - f3^2;
end

% Normalize
delta1_vals = delta1_vals / max(abs(delta1_vals));
delta2_vals = delta2_vals / max(abs(delta2_vals));
delta3_vals = delta3_vals / max(abs(delta3_vals));

% Plot and save delta1
% Create a single figure with 3 subplots
figure;
set(gcf, 'Renderer', 'painters');
set(gcf, 'Position', [100, 100, 1200, 900]);

% Plot delta1
subplot(3,1,1);
plot(x_vals, delta1_vals, 'b', 'LineWidth', 2);
xlabel('x'); ylabel('Normalized \delta_1');
title('Normalized \delta_1 vs x');
grid on;

% Plot delta2
subplot(3,1,2);
plot(x_vals, delta2_vals, 'r', 'LineWidth', 2);
xlabel('x'); ylabel('Normalized \delta_2');
title('Normalized \delta_2 vs x');
grid on;

% Plot delta3
subplot(3,1,3);
plot(x_vals, delta3_vals, 'g', 'LineWidth', 2);
xlabel('x'); ylabel('Normalized \delta_3');
title('Normalized \delta_3 vs x');
grid on;

% Save the combined figure
exportgraphics(gcf, 'Normalized_deltas_vs_x.png');


x_cr = abs(L-R);

%% Make vids of all 8 solutions for part 5, for x:0.5->2
% Define all 8 combinations of [±1, ±1, ±1]
elbow_combinations = dec2bin(0:7) - '0';  % binary matrix
elbow_combinations(elbow_combinations == 0) = -1;  % convert 0s to -1

for idx = 1:size(elbow_combinations, 1)
    elbows = elbow_combinations(idx, :);

    % Get joint values for this elbow configuration
    [q_t, X_t] = q_part5_plan(t, elbows);

    % Create a unique filename based on the elbow configuration
    elbow_str = sprintf('%+d_%+d_%+d', elbows);
    elbow_str = strrep(elbow_str, '+', 'p');  % replace '+' with 'p'
    elbow_str = strrep(elbow_str, '-', 'm');  % replace '-' with 'm'
    filename = ['robot_motion_' elbow_str '.mp4'];

    % High-quality video settings
    video = VideoWriter(filename, 'MPEG-4');
    video.FrameRate = fs;
    video.Quality = 100;
    open(video);

    figure();
    set(gcf, 'Renderer', 'painters');
    set(gcf, 'Position', [100, 100, 1200, 900]);

    for i = 1:length(t)
        clf;
        print_robot(X_t(:, i), q_t(:, i));
        title(sprintf('Time: %.2f sec', t(i)), 'FontSize', 14);
        drawnow;

        frame = getframe(gcf);
        writeVideo(video, frame);
    end

    close(video);
    close(gcf);  % Close the figure to avoid clutter
end


%% Calculate singualr states of Jx and Jq for part 5:
% AND Go over cases of Jx singularity and prinitng robot:


% For x where IK has solutions, graph det's
phi = pi/18;
y = 0;
x_vals = linspace(0.5, 2, 1000);

% Generate all 8 elbow configurations
elbow_configs = dec2bin(0:7) - '0';
elbow_configs(elbow_configs == 0) = -1;

dir_all = [];

for idx = 1:size(elbow_configs, 1)
    elbows = elbow_configs(idx, :);

    det_Jx_vals = zeros(size(x_vals));
    det_Jq_vals = zeros(size(x_vals));

    for i = 1:length(x_vals)
        x = x_vals(i);
        X = [x; y; phi];
        q = inverse_kin(X, elbows);

        if isempty(q)
            Jx = zeros(3,3); Jq = zeros(3,3);
        else
            Jx = Calc_Jx(q, X);
            Jq = Calc_Jq(q, X);
        end
        det_Jx_vals(i) = det(Jx);
        det_Jq_vals(i) = det(Jq);
    end

    % Normalize the determinants
    det_Jx_vals = det_Jx_vals / max(abs(det_Jx_vals));
    det_Jq_vals = det_Jq_vals / max(abs(det_Jq_vals));

    % Create a unique string for the elbow configuration
    elbow_str = sprintf('%+d_%+d_%+d', elbows);
    elbow_str = strrep(elbow_str, '+', 'p');
    elbow_str = strrep(elbow_str, '-', 'm');

    % Find singular indices (sign change in det(Jx))
    singular_indices = find(diff(sign(det_Jx_vals)) ~= 0);

    % Total number of subplots: 2 (det plots) + number of singularities
    num_subplots = 2 + length(singular_indices);
    nrows = ceil(sqrt(num_subplots));
    ncols = ceil(num_subplots / nrows);

    figure('Name', ['Elbows ' elbow_str], 'Renderer', 'painters');
    set(gcf, 'Renderer', 'painters');
    set(gcf, 'Position', [100, 100, 1200, 900]);

    % Subplot 1: det(Jx)
    subplot(nrows, ncols, 1);
    plot(x_vals, det_Jx_vals, 'b', 'LineWidth', 2);
    xlabel('x'); ylabel('Normalized det(Jx)');
    title('Normalized det(Jx)');
    grid on;

    % Subplot 2: det(Jq)
    subplot(nrows, ncols, 2);
    plot(x_vals, det_Jq_vals, 'r', 'LineWidth', 2);
    xlabel('x'); ylabel('Normalized det(Jq)');
    title('Normalized det(Jq)');
    grid on;

    % Subplots 3+ for singular robot configurations
    for j = 1:length(singular_indices)
        i = singular_indices(j);
        x_cr = x_vals(i);
        X_s = [x_cr; y; phi];
        q_s = inverse_kin(X_s, elbows);

        if ~isempty(q_s)
            subplot(nrows, ncols, 2 + j);
            print_robot(X_s, q_s);

            Jx_s = Calc_Jx(q_s, X_s);
            [V, D] = eig(Jx_s);
            eigenvalues = diag(D);
            [~, min_idx] = min(abs(eigenvalues));
            min_eigenvector = V(:, min_idx);

            % Normalize and draw the direction of free motion
            dir_full = min_eigenvector;
            dir_all(:,end+1) = dir_full;
            dir = dir_full(1:2);
            dir = dir / norm(dir);
            arrow_length = 0.5;
            scaled_dir = arrow_length * dir;

            hold on;
            quiver(X_s(1), X_s(2), scaled_dir(1), scaled_dir(2), 0, ...
                   'r', 'LineWidth', 2, 'MaxHeadSize', 1.5);
            title(['Singularity at x = ' num2str(x_cr, '%.3f')]);
            hold off;
        end
    end

    % Save figure
    exportgraphics(gcf, ['Part5_' elbow_str '.png']);
end

%% ONly print singular states


% For x where IK has solutions, graph det's
phi = pi/18;
y = 0;
x_vals = linspace(0.5, 2, 1000);

% Generate all 8 elbow configurations
elbow_configs = dec2bin(0:7) - '0';
elbow_configs(elbow_configs == 0) = -1;

dir_all = [];

for idx = 1:size(elbow_configs, 1)
    elbows = elbow_configs(idx, :);

    det_Jx_vals = zeros(size(x_vals));
    det_Jq_vals = zeros(size(x_vals));

    for i = 1:length(x_vals)
        x = x_vals(i);
        X = [x; y; phi];
        q = inverse_kin(X, elbows);

        if isempty(q)
            Jx = zeros(3,3); Jq = zeros(3,3);
        else
            Jx = Calc_Jx(q, X);
            Jq = Calc_Jq(q, X);
        end
        det_Jx_vals(i) = det(Jx);
        det_Jq_vals(i) = det(Jq);
    end

    % Normalize the determinants
    det_Jx_vals = det_Jx_vals / max(abs(det_Jx_vals));
    det_Jq_vals = det_Jq_vals / max(abs(det_Jq_vals));

    % Create a unique string for the elbow configuration
    elbow_str = sprintf('%+d_%+d_%+d', elbows);
    elbow_str = strrep(elbow_str, '+', 'p');
    elbow_str = strrep(elbow_str, '-', 'm');

    % Find singular indices (sign change in det(Jx))
    singular_indices = find(diff(sign(det_Jx_vals)) ~= 0);

    % Total number of subplots: 2 (det plots) + number of singularities
    num_subplots = length(singular_indices);
    nrows = ceil(sqrt(num_subplots));
    ncols = ceil(num_subplots / nrows);

    figure('Name', ['Elbows ' elbow_str], 'Renderer', 'painters');
    set(gcf, 'Renderer', 'painters');
    set(gcf, 'Position', [100, 100, 1200, 900]);


    % Subplots 3+ for singular robot configurations
    for j = 1:length(singular_indices)
        i = singular_indices(j);
        x_cr = x_vals(i);
        X_s = [x_cr; y; phi];
        q_s = inverse_kin(X_s, elbows);

        if ~isempty(q_s)
            subplot(nrows, ncols, j);
            print_robot(X_s, q_s);

            Jx_s = Calc_Jx(q_s, X_s);
            [V, D] = eig(Jx_s);
            eigenvalues = diag(D);
            [~, min_idx] = min(abs(eigenvalues));
            min_eigenvector = V(:, min_idx);

            % Normalize and draw the direction of free motion
            dir_full = min_eigenvector;
            dir_all(:,end+1) = dir_full;
            dir = dir_full(1:2);
            dir = dir / norm(dir);
            arrow_length = 0.5;
            scaled_dir = arrow_length * dir;

            hold on;
            quiver(X_s(1), X_s(2), scaled_dir(1), scaled_dir(2), 0, ...
                   'r', 'LineWidth', 2, 'MaxHeadSize', 1.5);
            title(['Singularity at x = ' num2str(x_cr, '%.3f')]);
            hold off;
        end
    end

    % Save figure
    exportgraphics(gcf, ['OnlyPrint_Part5_' elbow_str '.png']);
end

