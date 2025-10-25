function tau = tau_plan(prof,t_i)
    % tau_plan Calculates the tau with time vector t
    %
    % Inputs:
    %   prof - Profile selection (1, 2, or 3) to choose the desired motion profile
    %   Where:
    %       1 - constant velocity
    %       2 - trapezoidal velocity
    %       3 - polynomial velocity
    %   t_i 1x1 - Time for which we output the calculated tau forces/torques [s]
    %
    % Outputs:
    %   tau_plan 3x1 - tau values required as function of time,
    %                          according to the known model
    %   tau_static 3x1 - tau for static case
    
    q_t = q_plan(prof,t_i);
    q_dot = q_dot_plan(prof,t_i);
    q_dot2 = q_dot2_plan(prof,t_i);
    
    [H_i,C_i,G_i] = dynamics_mat(q_t,q_dot);
    
    tau = H_i*q_dot2 + C_i*q_dot + G_i;

end