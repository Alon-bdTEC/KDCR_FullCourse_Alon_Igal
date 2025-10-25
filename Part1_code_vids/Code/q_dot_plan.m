function q_dot = q_dot_plan(prof, t)
    % q_dot_plan Calculates the q_dot with time vector t
    %
    % Inputs:
    %   prof - Profile selection (1, 2, or 3) to choose the desired motion profile
    %   Where:
    %       1 - constant velocity
    %       2 - trapezoidal velocity
    %       3 - polynomial velocity
    %   t 1xn - Time vector for each we output the calculated position [s]
    %
    % Outputs:
    %   q_dot 3xlength(t) - Joint derivatives
    %
    % Description:
    %   Compute and Plot q_dot (dq/dt), from J inversion, based on the selected
    %   motion profile and the given time. There are three profiles to choose from:
    %   prof = 1, 2, or 3. Each profile defines a different motion trajectory for the tool.
    
    q_t = q_plan(prof, t);
    v_t = v_plan(prof, t);
    omega_t = omega_plan(prof,t);
    X_dot = [v_t;omega_t];
    q_dot = zeros(6, length(t));
    
    for w = 1:length(t)
        J = jacobian_mat(q_t(:, w));
        q_dot(:, w) = J\X_dot(:,w);
    end

end
