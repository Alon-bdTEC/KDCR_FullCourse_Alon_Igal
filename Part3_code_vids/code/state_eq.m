function Xdot = state_eq(t, X)
    % Xdot returns the X_dot as function of time t 1x1, and state X 6x1
    % where X=[q;q_dot];

    % Dimension
    n = 3;
    
    % Extract states
    q_ti = X(1:n);
    q_dot_ti = X(n+1:end);
    
    % get tau and time t
    elbow_chosen = [1,1];
    [tau_ti, ~] = tau_plan(3,t,elbow_chosen);

    % Compute dynamics matrices
    [H,C,G] = dynamics_mat(q_ti,q_dot_ti);
    
    % Add mechanical Saturation (for part 5)
    for i = 1:3
        % Revolute joint, lower & upper limit hit
        if (i ~= 3) && (q_ti(i) < -pi)
            q_ti(i) = -pi;
            if q_dot_ti(i) < 0  % moving deeper into limit
                q_dot_ti(i) = 0;  % block motion into the wall
            end
        elseif (i ~= 3) && (q_ti(i) > pi)
            q_ti(i) = pi;
            if q_dot_ti(i) > 0  % moving deeper into limit
                q_dot_ti(i) = 0;  % block motion into the wall
            end
        end

         % Linear joint, lower & upper limit hit
        if (i == 3) && (q_ti(i) < -0.7)
            q_ti(i) = -0.7;
            if q_dot_ti(i) < 0  % moving deeper into limit
                q_dot_ti(i) = 0;  % block motion into the wall
            end
        elseif (i == 3) && (q_ti(i) > 0.7)
            q_ti(i) = 0.7;
            if q_dot_ti(i) > 0  % moving deeper into limit
                q_dot_ti(i) = 0;  % block motion into the wall
            end
        end
    end

    % State derivatives
    X1_dot = q_dot_ti;
    X2_dot = H \ (tau_ti - C*q_dot_ti - G);
    
    % Return stacked state derivative
    Xdot = [X1_dot; X2_dot];
end
