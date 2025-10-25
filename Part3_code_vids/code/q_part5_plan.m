function [q,X] = q_part5_plan(t,elbows)
    % q_part5_plan Calculates the joint values and X as fuction of time t.
    %
    % Inputs:
    %   t 1xn - Time vector for each we output the calculated position [s]
    %   elbows - Vector specifying elbow configurations [+-1,+-1,+-1]:
    %            elbows(1) for theta1, elbows(2) for theta2, elbows(3) for theta3
    %
    % Outputs:
    %   q 3xlength(t) [m] - Joint values at time t
    %   X 3xlength(t) [m] - Target Vectore at time t
    %
    % Description:
    %   The function calculates the joint value and X as function of time t.
    %   The Trajectory type is constant velocity in linear and angular
    %   displecement.

    % gives the joints parameters for the vector t 
    
    % Get required x(t)

    T = 2;
    x_a = [0.5;0;deg2rad(10)]; x_b = [2;0;deg2rad(10)];
    r_t = (x_b(1:2)-x_a(1:2))./T .*t + x_a(1:2);
    theta_t = (x_b(3)-x_a(3))./T .*t + x_a(3);
    x_t = [r_t;theta_t];

    q = zeros(3, length(t));
    X = x_t;
    
    for i = 1:length(t)
        q(:, i) = inverse_kin(x_t(:,i), elbows);
    end

end