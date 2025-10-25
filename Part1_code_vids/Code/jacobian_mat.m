function J = jacobian_mat(q)
    % jacobian_mat Calculates the Jacobian matrix for the given joint variables
    % Inputs:
    %   q - Joint values for rotbot to get this position+rotation with
    %   chosen elbow values.
    %   q = [theta1; theta2; d3; theta4; theta5; theta6]
    %       theta1, theta2, theta4, theta5, theta6: Rotational joints [rad]
    %       d3: Linear joint [m]
    % Outputs:
    %   J - full jacobian matrix (relative to system 0)

    % Robot link lengths and definning d3 as q3
    l1 = 0.4;
    l2 = 0.15;
    l4 = 0.1;
    l5 = 0.3;
    l6 = 0.2;
    d3=q(3);

    % Angle Cosine and Sines
    C1 = cos(q(1)); S1 = sin(q(1));
    C2 = cos(q(2)); S2 = sin(q(2));
    C4 = cos(q(4)); S4 = sin(q(4));
    C5 = cos(q(5)); S5 = sin(q(5));
    C6 = cos(q(6)); S6 = sin(q(6));

    % Transformation Matrices
    A01 = [ C1 -S1  0   0;
            S1  C1  0   0;
             0   0  1   0;
             0   0  0   1];

    A12 = [ C2  0  -S2  0;
             0   1   0  0;
             S2  0   C2  l1;
             0   0   0  1];

    A23 = [1  0  0  l2;
            0  1  0  0;
            0  0  1 d3;
            0  0  0  1];

    A34 = [ C4 -S4  0   0;
             S4  C4  0   0;
             0   0   1   0;
             0   0   0   1];

    A45 = [ C5   0  -S5  0;
             0   1   0  0;
             S5   0   C5  l4;
             0   0   0   1];

    A56 = [ C6  -S6  0   0;
             S6   C6  0   0;
             0     0  1  l5;
             0     0  0   1];

    A6t = [1 0 0 0;
        0 1 0 0;
        0 0 1 l6;
        0 0 0 1];
    
    % Forward Kinematics
    T01 = A01;
    T02 = T01 * A12;
    T03 = T02 * A23;
    T04 = T03 * A34;
    T05 = T04 * A45;
    T06 = T05 * A56;
    T0t = T06 * A6t;
    
    % Extract rotation matrices
    R0i = {T01(1:3,1:3), T02(1:3,1:3), T03(1:3,1:3), T04(1:3,1:3), T05(1:3,1:3), T06(1:3,1:3), T0t(1:3,1:3)};
    
    % Extract position vectors
    P0i = {T01(1:3,4), T02(1:3,4), T03(1:3,4), T04(1:3,4), T05(1:3,4), T06(1:3,4), T0t(1:3,4)};

    % Compute Linear and Angular Jacobians separately
    J_l = zeros(3,6); % Linear velocity Jacobian
    J_a = zeros(3,6); % Angular velocity Jacobian

    for i = 1:6

        % Get z-axis
        if i == 1 || i == 3 || i == 4 || i == 6
            z_axis = R0i{i}(1:3,3);
        else
            z_axis = -R0i{i}(1:3,2);
        end
        
        if i == 3  % Special handling for prismatic joint (d₃)
            J_l(:,i) = z_axis; % Linear velocity only along Z
            J_a(:,i) = [0; 0; 0]; % No angular velocity
        else
            J_l(:,i) = cross(z_axis, (P0i{7} - P0i{i})); % Linear velocity component
            J_a(:,i) = z_axis; % Angular velocity component
        end
    end

    % Combine J_l and J_a to get full Jacobian
    J = [J_l; J_a]; 

end