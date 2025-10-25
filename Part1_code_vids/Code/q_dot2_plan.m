function q_dot2 = q_dot2_plan(prof,t)
    % q_dot2 Calculates the q_dot2 with time vector t
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
    %   q_dot2 3xlength(t) - Joint Aceeleration
    %
    % Description:
    %   Compute and Plot q_dot2, from J inversion, based on the selected
    %   motion profile and the given time. There are three profiles to choose from:
    %   prof = 1, 2, or 3. Each profile defines a different motion trajectory for the tool.
    
    q_t = q_plan(prof, t);
    q_dot_t = q_dot_plan(prof, t);
    a_t = a_plan(prof, t);
    omega_dot_t = omega_dot_plan(prof,t);
    X_dot_dot = [a_t;omega_dot_t];
    q_dot2 = zeros(6, length(t));
    
    for w = 1:length(t)
        J = jacobian_mat(q_t(:, w));
        q = q_t(:,w);
        q_dot = q_dot_t(:,w);
        q_dot2(:, w) = J\(X_dot_dot(:,w) - jacobian_mat_dot(q,q_dot)*q_dot);
    end
end