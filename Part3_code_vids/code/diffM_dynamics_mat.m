function [H,C,G] = diffM_dynamics_mat(q,q_dot)
    % dynamics_mat Calculates the H,C,G matrices as functions
    % of time t inside q and q_dot
    %
    % Inputs:
    %   q 3x1 - Joint Values in time
    %   q_dot 3x1 - Joint Velocities in time
    %
    % Outputs:
    %   H 3x3 - Inerita matrix in time
    %   C 3x3 - Speed matrix in time
    %   G 3x3 - Gravity matrix in time
    %
    
    % Robot parameters
    l2 = 0.15;
    l4 = 0.1;  % offset at gripper from prismatic joint
    l = 0.6;   % Value for prismatic purple segment
    g = 9.81;
    
    % masses
    rou = 7800; % Density of links (metal), in [kg/m^3]
    Area = pi*(0.015)^2/4; % Area of rod seciton, in [m^2]
    m2 = rou*Area*l2; % Mass of link 2, in [kg]
    m3 = rou*Area*l; % Mass of link 3, in [kg]
    M = 0.6; % Mass of point mass in gripper, in [kg]

    I2 = m2*(l2)^2/12;
    I3 = m3*(l)^2/12;
    
    H = zeros(3,3);
    C = zeros(3,3);
    G = zeros(3,1);
    

    q2 = q(2,1);d3 = q(3,1);
    q1d = q_dot(1,1);q2d = q_dot(2,1);d3d = q_dot(3,1);

    S2 = sin(q2);C2 = cos(q2);

    H(1,1) = m3*(l2*C2 - S2*(d3 - l/2 + l4))^2 +...
        M*(- l2*C2 + S2*(d3 + l4))^2 +...
        C2^2*((m2*l2^2)/4 + I2) + I3*S2^2;

    H(2,2) = I2 + I3 + m3*(d3 - l/2 + l4)^2 + M*(d3 + l4)^2 + M*l2^2 + ...
        (l2^2*m2)/4 + l2^2*m3;

    H(3,2) = l2*(m3+M);
    H(2,3) = H(3,2);
    H(3,3) = m3+M;
    
    A_term =  (sin(2*q2)/2)*(I2-I3+(m2*l2^2)/4) +...
        m3*(S2*l2 + C2*(d3 + l4 - l/2))*(C2*l2 - S2*(d3 + l4 - l/2)) +....
        M*(S2*l2 + C2*(d3 + l4))*(C2*l2 - S2*(d3 + l4));
    B_term = S2*( M*(- C2*l2 + S2*(d3 + l4)) + m3*(-C2*l2 + S2*(d3 + l4 - l/2)));
    C_term = m3*(d3 + l4 - l/2) + M*(d3 + l4);

    C(1,1) = d3d*B_term - q2d*A_term;
    C(1,2) = -q1d*A_term;
    C(1,3) = q1d*B_term;
    C(2,1) = -C(1,2);
    C(2,2) = d3d*C_term;
    C(2,3) = q2d*C_term;
    C(3,1) = -C(1,3);
    C(3,2) = -C(2,3);

    G(2,1) = (g*l2*m2*C2)/2 - M*g*(-l2*C2 + S2*(d3 + l4)) -...
        g*m3*(-l2*C2 + S2*(d3 - l/2 + l4));

    G(3,1) = g*cos(q2)*(M + m3);

end