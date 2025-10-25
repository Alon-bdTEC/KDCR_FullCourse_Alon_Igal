function print_robot(q)
    % print_robot draws the robot configuration for given joint values
    %
    % Inputs:
    %   q - Joint values for rotbot to get this position+rotation with
    %   chosen elbow values.
    %   q = [theta1; theta2; d3; theta4; theta5; theta6]
    %       theta1, theta2, theta4, theta5, theta6: Rotational joints [rad]
    %       d3: Linear joint [m]
    %
    % This function visualizes the robot configuration based on the given joint variables

    % Defining link lengths and endpoints:
    r_A = [0.25;0;0.6]; r_B = [0.05;-0.4;0.6];
    % Robot parameters
    l1 = 0.4;
    l2 = 0.15;
    l4 = 0.1;  % offset at gripper from prismatic joint
    l = 0.6;   % Value for prismatic purple segment

    theta1 = q(1); theta2 = q(2); d3 = q(3);
    C1 = cos(theta1); S1 = sin(theta1);
    C2 = cos(theta2); S2 = sin(theta2);

    A01 = [ C1 -S1  0   0;
            S1  C1  0   0;
             0   0  1   0;
             0   0  0   1];

    A12 = [ C2  0  -S2  0;
             0  1   0   0;
            S2  0   C2  l1;
             0  0   0   1];

    A23 = [1 0 0 l2;
           0 1 0 0;
           0 0 1 d3+l4;
           0 0 0 1];

    A02 = A01 * A12;
    A03 = A02 * A23;

    % Points
    p0 = [0;0;0];
    p2 = A02(1:3,4);
    p_joint3 = p2 + A02(1:3,1:3)*[l2;0;0];
    z2_axis = A02(1:3,3);
    p_bot_l = p_joint3 + z2_axis*(d3+l4-l);
    p_mid_l = p_joint3 + z2_axis*(d3);
    p_gripper = A03(1:3,4);

    % Plot from p0 to p2 to p_joint3
    plot3([p0(1), p2(1), p_joint3(1)], ...
          [p0(2), p2(2), p_joint3(2)], ...
          [p0(3), p2(3), p_joint3(3)], ...
          'b-', 'LineWidth', 2);
    hold on;
    
    % Plot from p_bot_l to p_top_l (purple)
    plot3([p_bot_l(1), p_mid_l(1)], ...
          [p_bot_l(2), p_mid_l(2)], ...
          [p_bot_l(3), p_mid_l(3)], ...
          'Color', [0.5 0 0.5], 'LineWidth', 2);
    
    % Plot from p_top_l to p_gripper
    plot3([p_mid_l(1), p_gripper(1)], ...
          [p_mid_l(2), p_gripper(2)], ...
          [p_mid_l(3), p_gripper(3)], ...
          'b-', 'LineWidth', 2);

    axis equal
    grid on
    xlabel('X'); ylabel('Y'); zlabel('Z');
    xlim([-1 1]); ylim([-1 1]); zlim([0 1.5]);

    % Cylinders on revolute joints (base and 1,2,4,5,6)
    cyl_radius = 0.05/2;
    cyl_height = 0.1/2;
    cyl_color = [1 0.6 0.2]; % orange-ish
    
    % Cylinders:
    draw_cylinder(p0 + [0;0;cyl_height/2], A01(1:3,3), cyl_radius, cyl_height, cyl_color);
    draw_cylinder(p2, -A02(1:3,2), cyl_radius, cyl_height, cyl_color);
    
    % Box for prismatic joint
    p_start = p_joint3 - A02(1:3,1:3)*[0;0;l4*1/2];
    p_end = p_joint3 + A02(1:3,1:3)*[0;0;l4*4/3];
    dir = A02(1:3,2);
    draw_square_prism(p_start, p_end, 0.03, [1 1 0], dir); % yellow box

    % Draw line and label between points A and B
    plot3([r_A(1), r_B(1)], [r_A(2), r_B(2)], [r_A(3), r_B(3)], 'r--', 'LineWidth', 2);
    scatter3(r_A(1), r_A(2), r_A(3), 60, 'k', 'filled');
    scatter3(r_B(1), r_B(2), r_B(3), 60, 'k', 'filled');
    text(r_A(1)+0.02, r_A(2)+0.02, r_A(3)+0.02, 'A', 'FontSize', 12, 'FontWeight', 'bold', 'Color', 'k');
    text(r_B(1)+0.02, r_B(2)+0.02, r_B(3)+0.02, 'B', 'FontSize', 12, 'FontWeight', 'bold', 'Color', 'k');

    % Draw coordinate systems
    l_sys = 0.22; % length of axis sticks

    % Base coordinate system (green)
    draw_coordinate_system([0;0;0], eye(3), '0', 'g', l_sys);

    % Tool coordinate system (red)
    draw_coordinate_system(A03(1:3,4), A03(1:3,1:3), 't', 'r', l_sys);
    
    % Marker for d3 length (offset along x2 axis)
    p_d3_start = p_joint3 - A02(1:3,1)*(l2/2);
    p_d3_end   = p_mid_l - A02(1:3,1)*(l2/2);
    plot3([p_d3_start(1), p_d3_end(1)], ...
          [p_d3_start(2), p_d3_end(2)], ...
          [p_d3_start(3), p_d3_end(3)], ...
          'm--', 'LineWidth', 2);  % magenta dashed line for d3

    % Add text label "d3" in the middle of the marker line
    mid_d3 = (p_d3_start + p_d3_end)/2;
    text(mid_d3(1), mid_d3(2), mid_d3(3)+0.03, '$d_3$', ...
         'FontSize', 11, 'FontWeight', 'bold', 'Color', 'k', 'Interpreter', 'latex');
    
    L = 0.6;H = 1;
    xlim([-L, L])
    ylim([-L, L])
    zlim([0, H])

    hold off
    
end

function draw_cylinder(center, direction, radius, height, color)
    % draw_cylinder Draws a cylinder in 3D space
    %
    % Inputs:
    %   center - Center of the cylinder base [x; y; z]
    %   direction - Direction vector of the cylinder axis [dx; dy; dz]
    %   radius - Radius of the cylinder
    %   height - Height of the cylinder
    %   color - Color of the cylinder
    %
    % This function draws a cylinder in 3D space with the specified parameters.

    [X, Y, Z] = cylinder(radius, 20);
    Z = Z * height - height/2;

    % Normalize direction
    dir = direction / norm(direction);
    % Compute rotation matrix to align Z axis with direction
    
    R = vrrotvec2mat(vrrotvec([0 0 1], dir));
    % Rotate and translate points
    pts = R * [X(:)'; Y(:)'; Z(:)'];
    X = reshape(pts(1,:), size(X)) + center(1);
    Y = reshape(pts(2,:), size(Y)) + center(2);
    Z = reshape(pts(3,:), size(Z)) + center(3);

    % Draw the curved surface
    surf(X, Y, Z, 'FaceColor', color, 'EdgeColor', 'k', 'LineWidth', 0.5);

    % Draw bottom and top lids
    fill3(X(1,:), Y(1,:), Z(1,:), color, 'EdgeColor', 'k');
    fill3(X(end,:), Y(end,:), Z(end,:), color, 'EdgeColor', 'k');
end

function draw_square_prism(p1, p2, thickness, color, dir_hint)
    % draw_square_prism Draws a square prismatic box between two points
    %
    % Inputs:
    %   p1        - Starting point of the box [x; y; z]
    %   p2        - Ending point of the box [x; y; z]
    %   thickness - Side length of the square cross-section
    %   color     - RGB color vector for the box
    %   dir_hint  - Direction vector to define orientation of the first edge

    % Ensure column vectors
    p1 = p1(:);
    p2 = p2(:);
    dir = p2 - p1;
    dir = dir / norm(dir);

    % Use dir_hint to create perpendicular square basis
    dir_hint = dir_hint(:);
    if abs(dot(dir, dir_hint)) > 0.99
        % dir_hint too parallel, pick a safer one
        dir_hint = [1;0;0];
        if abs(dot(dir, dir_hint)) > 0.99
            dir_hint = [0;1;0];
        end
    end

    % Create orthonormal basis (v, u) perpendicular to dir
    v = dir_hint - dot(dir_hint, dir) * dir;
    v = v / norm(v);
    u = cross(dir, v);

    % Half thickness
    t = thickness / 2;

    % Corner offsets in cross-section plane
    offsets = [ u+v, -u+v, -u-v, u-v ] * t;
    
    % 8 corners of the prism
    corners_start = p1 + offsets;
    corners_end   = p2 + offsets;
    
    % Define faces using the full 8 corners
    all_corners = [corners_start, corners_end];
    
    faces = {
        [1 2 3 4];  % bottom
        [5 6 7 8];  % top
        [1 2 6 5];  % side
        [2 3 7 6];  % side
        [3 4 8 7];  % side
        [4 1 5 8];  % side
    };
    
    hold on
    for i = 1:length(faces)
        idx = faces{i};
        verts = all_corners(:, idx);
        patch('Vertices', verts', 'Faces', 1:4, ...
          'FaceColor', color, 'EdgeColor', 'k', ...
          'FaceAlpha', 0.8, 'LineWidth', 0.5);
    end
end


function draw_coordinate_system(origin, R, label, color, l_sys)
    % Draws a coordinate system with origin, orientation R, and axis labels
    % origin: 3x1 position vector
    % R: 3x3 rotation matrix
    % label: '0' or 't'
    % color: base color for the axis
    % l_sys: length of axis sticks

    axes_labels = {'x', 'y', 'z'};
    colors = {color, color, color};

    for i = 1:3
        dir = R(:,i) * l_sys;
        endpoint = origin + dir;

        % Draw axis line
        quiver3(origin(1), origin(2), origin(3), ...
                dir(1), dir(2), dir(3), ...
                0, 'Color', colors{i}, 'LineWidth', 2, 'MaxHeadSize', 0.5);

        % Construct LaTeX axis label, e.g., '$x_0$' or '$z_t$'
        label_text = ['$' axes_labels{i} '_' label '$'];

        % Add label near end
        text(endpoint(1)+0.01, endpoint(2)+0.01, endpoint(3)+0.01, ...
             label_text, ...
             'FontSize', 10, 'FontWeight', 'bold', 'Color', 'k', 'Interpreter', 'latex');
    end
end

