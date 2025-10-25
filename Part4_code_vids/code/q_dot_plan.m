function q_dot = q_dot_plan(prof,t_i)
    % q_dot_plan Calculates the q_dot with time vector t
    %
    % Inputs:
    %   prof - Profile selection (1, 2, or 3) to choose the desired motion profile
    %   Where:
    %       1 - constant velocity
    %       2 - trapezoidal velocity
    %       3 - polynomial velocity
    %   t_i 1x1 - Time for which we output the calculated joint derivatives [s]
    %
    % Outputs:
    %   q_dot 3x1- Joint derivatives
    %
    % Description:
    %   Compute and Plot q_dot (dq/dt), from J inversion, based on
    %   polynomial velocity profile
    

    q_t = q_plan(prof,t_i);
    v_t = v_plan(prof,t_i);
    q_dot = zeros(3, 1);
    
    J = jacobian_mat(q_t);
    J_L = J(1:3,1:3);
    q_dot(:, 1) = J_L\v_t(:,1);

end
