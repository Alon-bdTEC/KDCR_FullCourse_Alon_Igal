function [R,d] = forward_kin(q)
    % forward_kin Calculates the forward kinematics of the serial robot
    % in HW 3
    % 
    % Inputs:
    %   q - Joint values vector [theta1; theta2; d3]
    %       theta1, theta2: Rotational joints [rad]
    %       d3: Linear joint [m]
    %
    % Outputs:
    %   R - Rotation matrix of the tool frame w.r.t. the base frame 0
    %   d - Position vector of the tool frame w.r.t. the base frame 0 [m]
    
    % Robot given link lengths
    l1 = 0.4;
    l2 = 0.15;
    l4 = 0.1;
    
    % Generate variables d3 and Cosine and Sine of angles for A's matrices
    for i = 1:length(q)
        if i== 3
            d3=q(i);
        else
            eval(sprintf('C%d = cos(q(%d));', i, i)); % Creates c1, c2, ..., cn
            eval(sprintf('S%d = sin(q(%d));', i, i)); % Creates s1, s2, ..., sn
        end
    end
    
    % Find the homogenous transformation matrices between systems

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
        0  0  1 d3+l4;
        0  0  0  1];

    % Get the homogenous transformation matrix from t to 0
    T = A01 * A12 * A23;

    % Get R and d from the homogenous transformation matrix
    R = T(1:3, 1:3);
    d = T(1:3,4);
end