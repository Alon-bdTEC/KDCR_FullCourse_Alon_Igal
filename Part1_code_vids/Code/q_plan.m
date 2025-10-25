function q = q_plan(prof,t)
    % gives the joints parameters for the vector t 
    type_trajectory = prof;
    r_t = x_plan(type_trajectory, t);
    R_t = R_plan(type_trajectory, t);
    elbows = [1 1 1];
    q = zeros(6, length(t));
    
    for i = 1:length(t)
        d = r_t(:, i);
        R = R_t(:, :, i);
        q(:, i) = inverse_kin(d, R, elbows);
    end
end