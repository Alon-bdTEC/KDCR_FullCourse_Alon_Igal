function q = q_plan(prof,t_i)
    % Inputs:
    %   prof - Profile selection (1, 2, or 3) to choose the desired motion profile
    %   Where:
    %       1 - constant velocity
    %       2 - trapezoidal velocity
    %       3 - polynomial velocity
    %   t_i 1x1 - Time for which we output the calculated joint values [s]
    %
    % Outputs:
    %   q 3x1 - Joint values as function of time

    r_t = x_plan(prof,t_i);
    q = zeros(3, 1);

    d = r_t(:, 1);
    q(:, 1) = inverse_kin(d);

end