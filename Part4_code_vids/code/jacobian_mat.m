function J = jacobian_mat(q)
    % jacobian_mat Calculates the Jacobian matrix for the given joint variables
    % Inputs:
    %   q - Joint values for rotbot to get this position+rotation with
    %   chosen elbow values.
    %   q = [theta1; theta2; d3]
    %       theta1, theta2: Rotational joints [rad]
    %       d3: Linear joint [m]
    % Outputs:
    %   J - full jacobian matrix (relative to system 0)

    % Robot link lengths and definning d3 as q3
    l2 = 0.15;
    l4 = 0.1;
    d3=q(3);

    % Angle Cosine and Sines
    C1 = cos(q(1)); S1 = sin(q(1));
    C2 = cos(q(2)); S2 = sin(q(2));

    % Calculate linear Jocaobian:
    J_L = [-l2*S1*C2 + (d3+l4)*S1*S2, -l2*C1*S2 - (d3+l4)*C1*C2, -C1*S2;
           l2*C1*C2 - (d3+l4)*C1*S2, -l2*S1*S2 - (d3+l4)*S1*C2, -S1*S2;
           0, l2*C2 - (d3+l4)*S2, C2];
    
    % Calculate Angular Jocaobian:
    J_A = [0, S1, 0;
           0, -C1, 0;
           1, 0, 0];
    
    % Combine full Jacobian
    J = [J_L; J_A]; 

end