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
    r_A = [0.25;0;1.1]; r_B = [0.55;-0.4;0.6];
    l1 = 0.4;
    l2 = 0.15;
    l4 = 0.1;
    l5 = 0.3;
    l6 = 0.2;

    d3 = q(3);
    C1 = cos(q(1)); S1 = sin(q(1));
    C2 = cos(q(2)); S2 = sin(q(2));
    C4 = cos(q(4)); S4 = sin(q(4));
    C5 = cos(q(5)); S5 = sin(q(5));
    C6 = cos(q(6)); S6 = sin(q(6));

    A01 = [C1 -S1 0 0; S1 C1 0 0; 0 0 1 0; 0 0 0 1];
    A12 = [C2 0 -S2 0; 0 1 0 0; S2 0 C2 l1; 0 0 0 1];
    A23 = [1 0 0 l2; 0 1 0 0; 0 0 1 d3; 0 0 0 1];
    A34 = [C4 -S4 0 0; S4 C4 0 0; 0 0 1 0; 0 0 0 1];
    A45 = [C5 0 -S5 0; 0 1 0 0; S5 0 C5 l4; 0 0 0 1];
    A56 = [C6 -S6 0 0; S6 C6 0 0; 0 0 1 l5; 0 0 0 1];
    A6t = eye(4); A6t(3,4) = l6;

    A02 = A01 * A12;
    A03 = A02 * A23;
    A04 = A03 * A34;
    A05 = A04 * A45;
    A06 = A05 * A56;
    A0t = A06 * A6t;

    origins = [[0;0;0],A02(1:3,4),A02(1:3,4)+A02(1:3,1:3)*[l2;0;0],...
        A03(1:3,4),A05(1:3,4),A06(1:3,4),A0t(1:3,4)];

    plot3(origins(1,:), origins(2,:), origins(3,:), 'b-', 'LineWidth', 2);
    hold on;
    axis equal
    grid on
    xlabel('X'); ylabel('Y'); zlabel('Z');
    xlim([-1 1]); ylim([-1 1]); zlim([0 1.5]);
    
    % Cylinders on revolute joints (base and 1,2,4,5,6)
    cyl_radius = 0.05;
    cyl_height = 0.1;
    cyl_color = [1 0.6 0.2]; % orange-ish
    
    % Cylinders:
    draw_cylinder(origins(:,1) + [0;0;cyl_height/2], A01(1:3,3), cyl_radius, cyl_height, cyl_color);
    draw_cylinder(origins(:,2), -A02(1:3,2), cyl_radius, cyl_height, cyl_color);
    draw_cylinder(origins(:,4), A03(1:3,3), 0.02, 0.05, cyl_color);
    draw_cylinder(origins(:,5), -A05(1:3,2), 0.02, 0.05, cyl_color);
    draw_cylinder(origins(:,6), A06(1:3,3), cyl_radius, cyl_height, cyl_color);

    % Box for prismatic joint between 2 and 4 (i.e., along z3)
    mid_linearJoint = (origins(:,3) + origins(:,4)) /2;
    p_start = mid_linearJoint - A03(1:3,1:3)*[0;0;d3/3];
    p_end = mid_linearJoint + A03(1:3,1:3)*[0;0;d3/3];
    draw_prismatic_box(p_start, p_end, 0.03, [1 1 0]); % yellow box

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
    draw_coordinate_system(A0t(1:3,4), A0t(1:3,1:3), 't', 'r', l_sys);

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

function draw_prismatic_box(p1, p2, thickness, color)
    % draw_prismatic_box Draws a prismatic box between two points
    %
    % Inputs:
    %   p1 - Starting point of the box [x; y; z]
    %   p2 - Ending point of the box [x; y; z]
    %   thickness - Thickness of the box
    %   color - Color of the box
    %
    % This function draws a prismatic box between two points with the specified thickness and color.

    dir = p2 - p1;
    L = norm(dir);
    dir = dir / L;

    % Get orthogonal vectors for box cross-section
    if abs(dot(dir, [0;0;1])) < 0.9
        v = cross(dir, [0;0;1]);
    else
        v = cross(dir, [0;1;0]);
    end
    v = v / norm(v);
    u = cross(v, dir);

    % 8 corners of the box
    corners = zeros(3,8);
    idx = 1;
    for sx = [-1 1]
        for sy = [-1 1]
            offset = thickness * (sx * u + sy * v);
            corners(:, idx)   = p1 + offset;
            corners(:, idx+4) = p2 + offset;
            idx = idx + 1;
        end
    end

    faces = [1 2 4 3; 5 6 8 7; 1 2 6 5; 3 4 8 7; 1 3 7 5; 2 4 8 6];
    patch('Vertices', corners', 'Faces', faces, ...
          'FaceColor', color, 'EdgeColor', 'k', ...
          'FaceAlpha', 0.8, 'LineWidth', 0.5);
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

        % Add label near end
        text(endpoint(1)+0.01, endpoint(2)+0.01, endpoint(3)+0.01, ...
             [axes_labels{i}, '_', label], ...
             'FontSize', 10, 'FontWeight', 'bold', 'Color', 'k');
    end
end
