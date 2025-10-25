function [H_0,C_0,G_0,H_kn,C_kn,G_kn] = dynamics_mat_MinMax(q,q_dot)
    % dynamics_mat Calculates the matrices for Min-Max as functions
    % of time t inside q and q_dot
    %
    % Inputs:
    %   q 3x1 - Joint Values in time
    %   q_dot 3x1 - Joint Velocities in time
    %
    % Outputs:
    %   H0,H_kn 3x3 - Inerita matrix: nominal and known part, in time
    %   C 3x3 - Speed matrix: nominal and known part, in time
    %   G 3x3 - Gravity matrix: nominal and known part, in time
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

    I2 = m2*(l2)^2/12;
    I3 = m3*(l)^2/12;
    
    H_0 = zeros(3,3);
    C_0 = zeros(3,3);
    G_0 = zeros(3,1); 
    H_kn = zeros(3,3);
    C_kn = zeros(3,3);
    G_kn = zeros(3,1); 

    q2 = q(2,1);d3 = q(3,1);
    q1d = q_dot(1,1);q2d = q_dot(2,1);d3d = q_dot(3,1);

    S2 = sin(q2);C2 = cos(q2);

    % G_0:
    G_0(2,1) = (g*l2*m2*C2)/2 - g*m3*(-l2*C2 + S2*(d3 - l/2 + l4));

    G_0(3,1) = g*cos(q2)*m3;

    % G_kn:
    G_kn(2,1) = - g*(-l2*C2 + S2*(d3 + l4));

    G_kn(3,1) = g*cos(q2);

    % H_0:
    H_0(1,1) = m3*(l2*C2 - S2*(d3 - l/2 + l4))^2 +...
    + C2^2*((m2*l2^2)/4 + I2) + I3*S2^2;

    H_0(2,2) = I2 + I3 + m3*(d3 - l/2 + l4)^2  + ...
        (l2^2*m2)/4 + l2^2*m3;

    H_0(3,2) = l2*m3;
    H_0(2,3) = H_0(3,2);
    H_0(3,3) = m3;

    % H_kn:
    H_0(1,1) = (- l2*C2 + S2*(d3 + l4))^2;

    H_0(2,2) = (d3 + l4)^2 + l2^2;

    H_0(3,2) = l2;
    H_0(2,3) = H_0(3,2);
    H_0(3,3) = 1;

    % C_0:
    A_term_0 =  (sin(2*q2)/2)*(I2-I3+(m2*l2^2)/4) +...
        m3*(S2*l2 + C2*(d3 + l4 - l/2))*(C2*l2 - S2*(d3 + l4 - l/2));
    B_term_0 = m3*S2*(-C2*l2 + S2*(d3 + l4 - l/2));
    C_term_0 = m3*(d3 + l4 - l/2);

    C_0(1,1) = d3d*B_term_0 - q2d*A_term_0;
    C_0(1,2) = -q1d*A_term_0;
    C_0(1,3) = q1d*B_term_0;
    C_0(2,1) = -C_0(1,2);
    C_0(2,2) = d3d*C_term_0;
    C_0(2,3) = q2d*C_term_0;
    C_0(3,1) = -C_0(1,3);
    C_0(3,2) = -C_0(2,3);

    % C_kn:
    A_term_kn =  (S2*l2 + C2*(d3 + l4))*(C2*l2 - S2*(d3 + l4));
    B_term_kn = S2*(- C2*l2 + S2*(d3 + l4));
    C_term_kn = d3 + l4;

    C_kn(1,1) = d3d*B_term_kn - q2d*A_term_kn;
    C_kn(1,2) = -q1d*A_term_kn;
    C_kn(1,3) = q1d*B_term_kn;
    C_kn(2,1) = -C_kn(1,2);
    C_kn(2,2) = d3d*C_term_kn;
    C_kn(2,3) = q2d*C_term_kn;
    C_kn(3,1) = -C_kn(1,3);
    C_kn(3,2) = -C_kn(2,3);

end