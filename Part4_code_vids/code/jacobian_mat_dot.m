function JL_dot = jacobian_mat_dot(q,q_dot)
    % Calculates the Jacobian matrix for the given joint variables
    % Inputs:
    %   q - Joint values for rotbot to get this position+rotation with
    %   chosen elbow values.
    %   q = [theta1; theta2; d3]
    %       theta1, theta2: Rotational joints [rad]
    %       d3: Linear joint [m]
    %   q_dot - derivatives of joint values
    %
    % Outputs:
    %   J_dot - Linear jacobian matrix (relative to system 0) derivative

    % Robot link lengths and definning d3 as q3
    l2 = 0.15;
    l4 = 0.1;
    d3=q(3);

    % Shorthand:
    q1 = q(1); q2 = q(2);

    % Preallocate J_dot
    JL_dot = zeros(3,3);

    % Compute J_dot
    
    % ----------------------------
    % Row 1
    % ----------------------------
    % J_11
    
    dJ_11 = [cos(q1)*sin(q2)*(d3 + l4) - l2*cos(q1)*cos(q2), cos(q2)*sin(q1)*(d3 + l4) + l2*sin(q1)*sin(q2), sin(q1)*sin(q2)];

    JL_dot(1,1) = dJ_11*q_dot;

    % J_12

    dJ_12 = [cos(q2)*sin(q1)*(d3 + l4) + l2*sin(q1)*sin(q2), cos(q1)*sin(q2)*(d3 + l4) - l2*cos(q1)*cos(q2), -cos(q1)*cos(q2)];

    JL_dot(1,2) = dJ_12*q_dot;
    
    % J_13 = -C1*S2
    
    dJ_13 = [sin(q1)*sin(q2), -cos(q1)*cos(q2), 0];

    JL_dot(1,3) = dJ_13*q_dot;


    % ----------------------------
    % Row 2
    % ----------------------------
    % J_21
    
    dJ_21 = [sin(q1)*sin(q2)*(d3 + l4) - l2*cos(q2)*sin(q1), - cos(q1)*cos(q2)*(d3 + l4) - l2*cos(q1)*sin(q2), -cos(q1)*sin(q2)];

    JL_dot(2,1) = dJ_21*q_dot;

    % J_22

    dJ_22 = [- cos(q1)*cos(q2)*(d3 + l4) - l2*cos(q1)*sin(q2), sin(q1)*sin(q2)*(d3 + l4) - l2*cos(q2)*sin(q1), -cos(q2)*sin(q1)];

    JL_dot(2,2) = dJ_22*q_dot;
    
    % J_23 = -S1*S2
    
    dJ_23 = [-cos(q1)*sin(q2), -cos(q2)*sin(q1), 0];
    JL_dot(2,3) = dJ_23*q_dot;

    % ----------------------------
    % Row 3
    % ----------------------------
    % J_31 = 0

    dJ_31 = [0, 0, 0];
    JL_dot(3,1) = dJ_31*q_dot;
    
    % J_32

    dJ_32 = [0, - cos(q2)*(d3 + l4) - l2*sin(q2), -sin(q2)];

    JL_dot(3,2) = dJ_32*q_dot;
    
    % J_33 = C2
    
    dJ_33 = [0, -sin(q2), 0];
    JL_dot(3,3) = dJ_33*q_dot;


end