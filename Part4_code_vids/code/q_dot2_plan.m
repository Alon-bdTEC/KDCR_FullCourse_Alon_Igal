function q_dot2 = q_dot2_plan(prof,t_i)
    % q_dot2 Calculates the q_dot2 with time vector t
    %
    % Inputs:
    %   prof - Profile selection (1, 2, or 3) to choose the desired motion profile
    %   Where:
    %       1 - constant velocity
    %       2 - trapezoidal velocity
    %       3 - polynomial velocity
    %   t_i 1x1 - Time for which we output the calculated accelerations [s]
    %
    % Outputs:
    %   q_dot2 3x1-  Joint Aceeleration
    %
    % Description:
    %   Compute and Plot q_dot2, from J inversion, based on
    %   polynomial velocity profile
    
    q_t = q_plan(prof,t_i);
    q_dot_t = q_dot_plan(prof,t_i);
    a_t = a_plan(prof,t_i);
    q_dot2 = zeros(3, 1);
    
    J = jacobian_mat(q_t);
    J_L = J(1:3,1:3);
    JL_dot = jacobian_mat_dot(q_t,q_dot_t);
    q_dot2(:, 1) = J_L\(a_t(:,1) - JL_dot*q_dot_t);

end