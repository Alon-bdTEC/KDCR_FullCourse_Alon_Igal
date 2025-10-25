function print_robot_w_circ(X,q)
    % print_robot draws the robot configuration for given joint values
    % and task vector
    % Inputs:
    %   X = [x;y;phi]; - task vector.
    %   q = [theta1; theta2; theta3] - joint values

    % Robot given link lengths
    r = 1.5;
    L = 3;
    R = 3.5;

    % Extract task variables
    x = X(1);
    y = X(2);
    phi = X(3);

    % Calculate rotated frame origin (x, y)
    r_B_tag = [x; y];
    r_A_tag = r_B_tag + r * [cos(phi + pi/3); sin(phi + pi/3)];
    r_C_tag = r_B_tag + r * [cos(phi); sin(phi)];

    % Find A, B, C points
    r_A = R * [sin(q(1)); -cos(q(1))];
    r_B = R * [sin(q(2)); -cos(q(2))];
    r_C = R * [sin(q(3)); -cos(q(3))];

    hold on; axis equal; grid on;
    xlabel('X-axis'); ylabel('Y-axis');

    % Draw ground frame x0, y0
    l_axis = 0.6; % axis stick length
    quiver(0, 0, l_axis, 0, 'r', 'LineWidth', 1.5, 'MaxHeadSize', 0.5);
    quiver(0, 0, 0, l_axis, 'g', 'LineWidth', 1.5, 'MaxHeadSize', 0.5);
    text(0.2, -0.2, 'O', 'FontSize', 12, 'FontWeight', 'bold');
    text(l_axis + 0.1, 0, 'x0', 'FontSize', 10, 'Color', 'r');
    text(0, l_axis + 0.1, 'y0', 'FontSize', 10, 'Color', 'g');

    % Draw rotated frame x, y
    R_phi = [cos(phi), -sin(phi); sin(phi), cos(phi)];
    x_axis = R_phi * [1; 0];
    y_axis = R_phi * [0; 1];
    quiver(x, y, l_axis * x_axis(1), l_axis * x_axis(2), 'r', 'LineWidth', 1.5, 'MaxHeadSize', 0.5);
    quiver(x, y, l_axis * y_axis(1), l_axis * y_axis(2), 'g', 'LineWidth', 1.5, 'MaxHeadSize', 0.5);
    text(x + 0.2, y - 0.2, '(x,y)', 'FontSize', 12, 'FontWeight', 'bold');
    text(x + l_axis * x_axis(1) + 0.1, y + l_axis * x_axis(2), 'x', 'FontSize', 10, 'Color', 'r');
    text(x + l_axis * y_axis(1) + 0.1, y + l_axis * y_axis(2), 'y', 'FontSize', 10, 'Color', 'g');

    % Draw circle around origin
    theta = linspace(0, 2*pi, 100);
    circle_thin = 1.5;
    plot(R * cos(theta), R * sin(theta), 'k', 'LineWidth', circle_thin);

    % Mark points A, B, C
    plot(r_A(1), r_A(2), 'ko', 'MarkerFaceColor', 'k');
    text(r_A(1) + 0.1, r_A(2), 'A', 'FontSize', 10);

    plot(r_B(1), r_B(2), 'ko', 'MarkerFaceColor', 'k');
    text(r_B(1) + 0.1, r_B(2), 'B', 'FontSize', 10);

    plot(r_C(1), r_C(2), 'ko', 'MarkerFaceColor', 'k');
    text(r_C(1) + 0.1, r_C(2), 'C', 'FontSize', 10);

    % Mark points A', B', C'
    plot(r_A_tag(1), r_A_tag(2), 'ko', 'MarkerFaceColor', 'k');
    text(r_A_tag(1) + 0.1, r_A_tag(2), "A'", 'FontSize', 10);

    plot(r_B_tag(1), r_B_tag(2), 'ko', 'MarkerFaceColor', 'k');
    text(r_B_tag(1) + 0.1, r_B_tag(2), "B'", 'FontSize', 10);

    plot(r_C_tag(1), r_C_tag(2), 'ko', 'MarkerFaceColor', 'k');
    text(r_C_tag(1) + 0.1, r_C_tag(2), "C'", 'FontSize', 10);

    % Draw links (AA', BB', CC')
    plot([r_A(1), r_A_tag(1)], [r_A(2), r_A_tag(2)], 'b', 'LineWidth', 2);
    plot([r_B(1), r_B_tag(1)], [r_B(2), r_B_tag(2)], 'b', 'LineWidth', 2);
    plot([r_C(1), r_C_tag(1)], [r_C(2), r_C_tag(2)], 'b', 'LineWidth', 2);

    % Draw triangle A'B'C' (metallic gray color)
    fill([r_A_tag(1), r_B_tag(1), r_C_tag(1)], [r_A_tag(2), r_B_tag(2), r_C_tag(2)], [0.7 0.7 0.7], 'FaceAlpha', 0.5);

    % Draw circles of radius L around A, B, C
    theta_circ = linspace(0, 2*pi, 100);
    circle_color = [0.3 0.3 1]; % light blue for visibility
    line_style = '--';

    % Circle around A
    plot(r_A(1) + L*cos(theta_circ), r_A(2) + L*sin(theta_circ), line_style, 'Color', circle_color, 'LineWidth', 1);

    % Circle around B
    plot(r_B(1) + L*cos(theta_circ), r_B(2) + L*sin(theta_circ), line_style, 'Color', circle_color, 'LineWidth', 1);

    % Circle around C
    plot(r_C(1) + L*cos(theta_circ), r_C(2) + L*sin(theta_circ), line_style, 'Color', circle_color, 'LineWidth', 1);


    title('Parallel Robot Printed');
end