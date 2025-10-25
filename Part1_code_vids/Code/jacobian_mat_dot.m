function J_dot = jacobian_mat_dot(q,q_dot)
    % Calculates the Jacobian matrix for the given joint variables
    % Inputs:
    %   q - Joint values for rotbot to get this position+rotation with
    %   chosen elbow values.
    %   q = [theta1; theta2; d3; theta4; theta5; theta6]
    %       theta1, theta2, theta4, theta5, theta6: Rotational joints [rad]
    %       d3: Linear joint [m]
    %   q_dot - derivatives of joint values
    %
    % Outputs:
    %   J_dot - full jacobian matrix (relative to system 0) derivative

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

    % Shorthand:
    q1 = q(1); q2 = q(2); q3 = q(3);
    q4 = q(4); q5 = q(5); q6 = q(6);

    % Preallocate J_dot
    J_dot = zeros(6,6);

    % Compute J_dot
    
    % ----------------------------
    % Row 1
    % ----------------------------
    % J_11
    
    dJ_11 = [cos(q1)*sin(q2)*(l4 + q3) - (l5 + l6)*(sin(q5)*(sin(q1)*sin(q4) - cos(q1)*cos(q2)*cos(q4)) - cos(q1)*cos(q5)*sin(q2)) - l2*cos(q1)*cos(q2), (cos(q2)*cos(q5)*sin(q1) - cos(q4)*sin(q1)*sin(q2)*sin(q5))*(l5 + l6) + cos(q2)*sin(q1)*(l4 + q3) + l2*sin(q1)*sin(q2), sin(q1)*sin(q2), sin(q5)*(l5 + l6)*(cos(q1)*cos(q4) - cos(q2)*sin(q1)*sin(q4)), (cos(q5)*(cos(q1)*sin(q4) + cos(q2)*cos(q4)*sin(q1)) - sin(q1)*sin(q2)*sin(q5))*(l5 + l6), 0];

    J_dot(1,1) = dJ_11*q_dot;

    % J_12

    dJ_12 = [(cos(q2)*cos(q5)*sin(q1) - cos(q4)*sin(q1)*sin(q2)*sin(q5))*(l5 + l6) + cos(q2)*sin(q1)*(l4 + q3) + l2*sin(q1)*sin(q2), (cos(q1)*cos(q5)*sin(q2) + cos(q1)*cos(q2)*cos(q4)*sin(q5))*(l5 + l6) + cos(q1)*sin(q2)*(l4 + q3) - l2*cos(q1)*cos(q2), -cos(q1)*cos(q2), -cos(q1)*sin(q2)*sin(q4)*sin(q5)*(l5 + l6), (cos(q1)*cos(q2)*sin(q5) + cos(q1)*cos(q4)*cos(q5)*sin(q2))*(l5 + l6), 0];
    
    J_dot(1,2) = dJ_12*q_dot;
    
    % J_13 = -C1*S2
    
    J_dot(1,3) = S1*q_dot(1)*S2 - C1*C2*q_dot(2);

    % J_14
    
    dJ_14 = [sin(q5)*(l5 + l6)*(cos(q1)*cos(q4) - cos(q2)*sin(q1)*sin(q4)), -cos(q1)*sin(q2)*sin(q4)*sin(q5)*(l5 + l6), 0, -sin(q5)*(l5 + l6)*(sin(q1)*sin(q4) - cos(q1)*cos(q2)*cos(q4)), cos(q5)*(l5 + l6)*(cos(q4)*sin(q1) + cos(q1)*cos(q2)*sin(q4)), 0];
    
    J_dot(1,4) = dJ_14*q_dot;

    % J_15

    dJ_15 = [(cos(q5)*(cos(q1)*sin(q4) + cos(q2)*cos(q4)*sin(q1)) - sin(q1)*sin(q2)*sin(q5))*(l5 + l6), (cos(q1)*cos(q2)*sin(q5) + cos(q1)*cos(q4)*cos(q5)*sin(q2))*(l5 + l6), 0, cos(q5)*(l5 + l6)*(cos(q4)*sin(q1) + cos(q1)*cos(q2)*sin(q4)), -(l5 + l6)*(sin(q5)*(sin(q1)*sin(q4) - cos(q1)*cos(q2)*cos(q4)) - cos(q1)*cos(q5)*sin(q2)), 0];
    
    J_dot(1,5) = dJ_15*q_dot;
    
    % J_16 = 0
    J_dot(1,6) = 0;

    % ----------------------------
    % Row 2
    % ----------------------------
    % J_21
    
    dJ_21 = [(sin(q5)*(cos(q1)*sin(q4) + cos(q2)*cos(q4)*sin(q1)) + cos(q5)*sin(q1)*sin(q2))*(l5 + l6) + sin(q1)*sin(q2)*(l4 + q3) - l2*cos(q2)*sin(q1), - (cos(q1)*cos(q2)*cos(q5) - cos(q1)*cos(q4)*sin(q2)*sin(q5))*(l5 + l6) - cos(q1)*cos(q2)*(l4 + q3) - l2*cos(q1)*sin(q2), -cos(q1)*sin(q2), sin(q5)*(l5 + l6)*(cos(q4)*sin(q1) + cos(q1)*cos(q2)*sin(q4)), (l5 + l6)*(cos(q5)*(sin(q1)*sin(q4) - cos(q1)*cos(q2)*cos(q4)) + cos(q1)*sin(q2)*sin(q5)), 0];
    
    J_dot(2,1) = dJ_21*q_dot;

    % J_22

    dJ_22 = [- (cos(q1)*cos(q2)*cos(q5) - cos(q1)*cos(q4)*sin(q2)*sin(q5))*(l5 + l6) - cos(q1)*cos(q2)*(l4 + q3) - l2*cos(q1)*sin(q2), (cos(q5)*sin(q1)*sin(q2) + cos(q2)*cos(q4)*sin(q1)*sin(q5))*(l5 + l6) + sin(q1)*sin(q2)*(l4 + q3) - l2*cos(q2)*sin(q1), -cos(q2)*sin(q1), -sin(q1)*sin(q2)*sin(q4)*sin(q5)*(l5 + l6), (cos(q2)*sin(q1)*sin(q5) + cos(q4)*cos(q5)*sin(q1)*sin(q2))*(l5 + l6), 0];
    
    J_dot(2,2) = dJ_22*q_dot;
    
    % J_23 = -S1*S2
    
    J_dot(2,3) = -C1*q_dot(1)*S2 - S1*C2*q_dot(2);

    % J_24 

    dJ_24 = [sin(q5)*(l5 + l6)*(cos(q4)*sin(q1) + cos(q1)*cos(q2)*sin(q4)), -sin(q1)*sin(q2)*sin(q4)*sin(q5)*(l5 + l6), 0, sin(q5)*(l5 + l6)*(cos(q1)*sin(q4) + cos(q2)*cos(q4)*sin(q1)), -cos(q5)*(l5 + l6)*(cos(q1)*cos(q4) - cos(q2)*sin(q1)*sin(q4)), 0];
    
    J_dot(2,4) = dJ_24*q_dot;

    % J_25 
    
    dJ_25 = [(l5 + l6)*(cos(q5)*(sin(q1)*sin(q4) - cos(q1)*cos(q2)*cos(q4)) + cos(q1)*sin(q2)*sin(q5)), (cos(q2)*sin(q1)*sin(q5) + cos(q4)*cos(q5)*sin(q1)*sin(q2))*(l5 + l6), 0, -cos(q5)*(l5 + l6)*(cos(q1)*cos(q4) - cos(q2)*sin(q1)*sin(q4)), (sin(q5)*(cos(q1)*sin(q4) + cos(q2)*cos(q4)*sin(q1)) + cos(q5)*sin(q1)*sin(q2))*(l5 + l6), 0];
    
    J_dot(2,5) = dJ_25*q_dot;
    
    % J_26 = 0
    J_dot(2,6) = 0;

    % ----------------------------
    % Row 3
    % ----------------------------
    % J_31 = 0

    J_dot(3,1) = 0;
    
    % J_32

    dJ_32 = [0, cos(q4)*cos(q5)*sin(q2)*(l5 + l6) - l2*sin(q2) - cos(q2)*(l4 + q3), -sin(q2), cos(q2)*cos(q5)*sin(q4)*(l5 + l6), cos(q2)*cos(q4)*sin(q5)*(l5 + l6), 0];
    
    J_dot(3,2) = dJ_32*q_dot;
    
    % J_33 = C2
    
    J_dot(3,3) = -S2*q_dot(2);

    % J_34

    dJ_34 = [0, cos(q2)*sin(q4)*sin(q5)*(l5 + l6), 0, cos(q4)*sin(q2)*sin(q5)*(l5 + l6), cos(q5)*sin(q2)*sin(q4)*(l5 + l6), 0];
    
    J_dot(3,4) = dJ_34*q_dot;

    % J_35

    dJ_35 = [0, (l5 + l6)*(sin(q2)*sin(q5) - cos(q2)*cos(q4)*cos(q5)), 0, cos(q5)*sin(q2)*sin(q4)*(l5 + l6), -(l5 + l6)*(cos(q2)*cos(q5) - cos(q4)*sin(q2)*sin(q5)), 0];
    
    J_dot(3,5) = dJ_35*q_dot;
    
    % J_36 = 0
    J_dot(3,6) = 0;

    % ----------------------------
    % Row 4
    % ----------------------------
    % J_41 = 0
    
    J_dot(4,1) = 0;

    % J_42 = S1

    J_dot(4,2) = C1*q_dot(1);
    
    % J_43 = 0
    
    J_dot(4,3) = 0;

    % J_44 = -C1S2

    J_dot(4,4) = S1*q_dot(1)*S2 - C1*C2*q_dot(2);

    % J_45

    dJ_45 = [cos(q1)*cos(q4) - cos(q2)*sin(q1)*sin(q4), -cos(q1)*sin(q2)*sin(q4), 0, cos(q1)*cos(q2)*cos(q4) - sin(q1)*sin(q4), 0, 0];
    
    J_dot(4,5) = dJ_45*q_dot;
    
    % J_46
    
    dJ_46 = [sin(q5)*(cos(q1)*sin(q4) + cos(q2)*cos(q4)*sin(q1)) + cos(q5)*sin(q1)*sin(q2), cos(q1)*cos(q4)*sin(q2)*sin(q5) - cos(q1)*cos(q2)*cos(q5), 0, sin(q5)*(cos(q4)*sin(q1) + cos(q1)*cos(q2)*sin(q4)), cos(q5)*(sin(q1)*sin(q4) - cos(q1)*cos(q2)*cos(q4)) + cos(q1)*sin(q2)*sin(q5), 0];
    
    J_dot(4,6) = dJ_46*q_dot;

    % ----------------------------
    % Row 5
    % ----------------------------
    % J_51 = 0
    
    J_dot(5,1) = 0;

    % J_52 = -C1

    J_dot(5,2) = S1*q_dot(1);
    
    % J_53 = 0
    
    J_dot(5,3) = 0;

    % J_54 = -S1S2

    J_dot(5,4) = -C1*q_dot(1)*S2 - S1*C2*q_dot(2);

    % J_55

    dJ_55 = [cos(q4)*sin(q1) + cos(q1)*cos(q2)*sin(q4), -sin(q1)*sin(q2)*sin(q4), 0, cos(q1)*sin(q4) + cos(q2)*cos(q4)*sin(q1), 0, 0];
    
    J_dot(5,5) = dJ_55*q_dot;
    
    % J_56
    
    dJ_56 = [sin(q5)*(sin(q1)*sin(q4) - cos(q1)*cos(q2)*cos(q4)) - cos(q1)*cos(q5)*sin(q2), cos(q4)*sin(q1)*sin(q2)*sin(q5) - cos(q2)*cos(q5)*sin(q1), 0, -sin(q5)*(cos(q1)*cos(q4) - cos(q2)*sin(q1)*sin(q4)), sin(q1)*sin(q2)*sin(q5) - cos(q5)*(cos(q1)*sin(q4) + cos(q2)*cos(q4)*sin(q1)), 0];
    
    J_dot(5,6) = dJ_56*q_dot;

    % ----------------------------
    % Row 6
    % ----------------------------
    % J_61 = 0
    
    J_dot(6,1) = 0;

    % J_62 = 0

    J_dot(6,2) = 0;
    
    % J_63 = 0
    
    J_dot(6,3) = 0;

    % J_64 = C2

    J_dot(6,4) = -S2*q_dot(2);

    % J_65 = S2*S4

    J_dot(6,5) = C2*q_dot(2)*S4 + S2*C4*q_dot(4);
    
    % J_66
    dJ_66 = [0, - cos(q5)*sin(q2) - cos(q2)*cos(q4)*sin(q5), 0, sin(q2)*sin(q4)*sin(q5), - cos(q2)*sin(q5) - cos(q4)*cos(q5)*sin(q2), 0];
    
    J_dot(6,6) = dJ_66*q_dot;

end