function Xdot = state_eq_noM(t, X, law)
    % Xdot returns the X_dot, with the real simulation having M=0[kg]
    % as function of time t 1x1, and state X
    %   For control law 1,2 and 4 its 2nx1 = 6x1
    %   X = [q;q_dot]
    %
    %   For control law 3 its 3nx1=9x1
    %   X = [int(e),e,e_dot], where e(t)=q(t)-q^d(t)
    %
    
    if law ~= 3

        % Dimension
        n = 3;
        
        % Extract states
        q_ti = X(1:n);
        q_dot_ti = X(n+1:end);
        
        % get tau and time t
        % tau= tau_cont(prof,q,q_dot,t_i,law)
        tau_ti = tau_cont(3,q_ti,q_dot_ti,t, law);
    
        % Compute dynamics matrices
        [H,C,G] = dynamics_mat_noM(q_ti,q_dot_ti);
        
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

    else % This is the PID control law case
        % Dimension
        n = 3;
        
        % Extract states
        int_e_ti = X(1:n);
        e_ti = X(n+1:2*n);
        e_dot_ti = X(2*n+1:end);

        % Applied tau:
        % control Gains choice:
        % Kp = 1.*eye(3); Kd = 1.*eye(3); Ki = 1.*eye(3);
        % Kp = 600.*eye(3); Kd = 120.*eye(3); Ki = 140.*eye(3);
        % Kp = 4000.*eye(3); Kd = 120.*eye(3); Ki = 140.*eye(3); 
        % 0.56% err in 1.5 [sec]

        % Works
        Kp = 15500.*eye(3); Kd = 900.*eye(3); Ki = 1300.*eye(3);

        tau_ti = -Kp*e_ti-Kd*e_dot_ti-Ki*int_e_ti;

        % Get desired q^d(t):
        q_des_ti = q_plan(3,t);
        q_dot_des_ti = q_dot_plan(3,t);
        q_dot2_des_ti = q_dot2_plan(3,t);

        % Get joint values (before saturation)
        q_ti = e_ti + q_des_ti;
        q_dot_ti = e_dot_ti + q_dot_des_ti;
        
        % Find REAL matrices and adding saturation:
        [H,C,G] = dynamics_mat_noM(q_ti,q_dot_ti);
        
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
        X1_dot = e_ti;
        X2_dot = e_dot_ti;
        X3_dot = H \ (tau_ti - C*q_dot_ti - G) + q_dot2_des_ti;
        
        % Return stacked state derivative
        Xdot = [X1_dot; X2_dot;X3_dot];
    end

end
