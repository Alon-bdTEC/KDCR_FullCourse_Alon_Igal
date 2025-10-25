function tau= tau_cont(prof,q,q_dot,t_i,law)
    % tau_plan Calculates the tau with time vector t
    %
    % Inputs:
    %   prof - Profile selection (1, 2, or 3) to choose the desired motion profile
    %   Where:
    %       1 - constant velocity
    %       2 - trapezoidal velocity
    %       3 - polynomial velocity
    %   q - *simulated* joints values at time t_i
    %   q_dot - *simulated* joints derivatives values at time t_i
    %   t_i 1x1 - Time for which we output the calculated tau forces/torques [s]
    %   law - the control law chosen.
    %   Where:
    %       1 - Inverse dynamics + PD
    %       2 - PD + Gravity compensation
    %       3 - PID
    %       4 - Min-Max
    %
    % Outputs:
    %   tau_cont 3x1 - tau values in time t_i, calculates
    %                  from the control law chosen
    %
    
    if law == 1
        q_des_ti = q_plan(prof,t_i);
        q_dot_des_ti = q_dot_plan(prof,t_i);
        q_dot2_des_ti = q_dot2_plan(prof,t_i);

        % Find the H,C,G matrices from messured state
        [H,C,G] = dynamics_mat(q,q_dot);

        % Use chosen Kd and Kp:
        % Kp = eye(3); Kd = 1.732.*eye(3);
        % Kp = 10*eye(3); Kd = 1.732.*eye(3);
        % Kp = 20*eye(3); Kd = 2*1.732.*eye(3);
        % Kp = 35*eye(3); Kd = 2*1.732.*eye(3);

        % IA:
        % Kp = eye(3); Kd = 1.732.*eye(3);
        % Kp = 2.*eye(3); Kd = 1.732.*eye(3);
        % Kp = 5.*eye(3); Kd = 1.732.*eye(3);
        % Kp = 5.*eye(3); Kd = 3.*eye(3);

        % Works:
        Kp = 4.5.*eye(3); Kd = 3.*eye(3);

        % Error and its der:
        e = q-q_des_ti;
        e_dot = q_dot-q_dot_des_ti;

        % So, this control law's tau:
        tau = H*(q_dot2_des_ti-Kp*e-Kd*e_dot) + C*q_dot + G;
    elseif law == 2
        q_des_ti = q_plan(prof,t_i);
        q_dot_des_ti = q_dot_plan(prof,t_i);

        % Find the H,C,G matrices from messured state
        [~,~,G] = dynamics_mat(q,q_dot);

        % IA:
        % Kp = 1.*eye(3); Kd = 1.*eye(3);
        % Kp = 80.*eye(3); Kd = 15.*eye(3);
        % Kp = 150.*eye(3); Kd = 15.*eye(3);
        % Kp = 500.*eye(3); Kd = 15.*eye(3);
        % Kp = 300.*eye(3); Kd = 80.*eye(3);
        % Kp = 400.*eye(3); Kd = 100.*eye(3);
        % Kp = 600.*eye(3); Kd = 100.*eye(3);

        % Works:
        Kp = 600.*eye(3); Kd = 100.*eye(3);

        % Error and its der:
        e = q-q_des_ti;
        e_dot = q_dot-q_dot_des_ti;

        % So, this control law's tau:
        tau = G -Kp*e-Kd*e_dot;
    else % Law 4 - Min-Max
        M_max = 1; % Max mass, where 0<M<M_max = 1[kg]
        q_des_ti = q_plan(prof,t_i);
        q_dot_des_ti = q_dot_plan(prof,t_i);
        q_ddot_des_ti = q_dot2_plan(prof,t_i);

        % Controller Gains (need KP to be P.D as well):
        K = 30.*eye(3); P = 30.*eye(3); % 30-30
        beta = 1.2;
        delta = 0.05;

        % It1 : K,P,beta,delta = 30-30-1.2-0.001, works but discontinous tau
        % It2 : K,P,beta,delta = 270-120-1.4-0.1
        % Final : 270-90-1.2-0.1

        % Error and its der:
        e = q-q_des_ti;
        e_dot = q_dot-q_dot_des_ti;

        % Calculate s,q_dot_r and q_ddot_r:
        s = e_dot + K*e;
        q_dot_r = q_dot_des_ti - K*e;
        q_ddot_r = q_ddot_des_ti - K*e_dot;

        % Find etha_0 and rou_tilde:
        [H_0,C_0,G_0,H_kn,C_kn,G_kn] = dynamics_mat_MinMax(q,q_dot);
        
        etha_0 = -C_0*q_dot_r - G_0 - H_0*q_ddot_r + P*e;
        rou_tilde = M_max * (norm(G_kn) +...
            norm(C_kn*q_dot_r) + norm(H_kn*q_ddot_r));

        % So, this control law's tau:
        rou_u_notzero = (1/norm(s)) *...
                (s'*etha_0 - e'*(K'*P)*e) + beta*rou_tilde ;
        rou_u = max(0, rou_u_notzero);
        tau = -rou_u * s / (norm(s)+delta);

    end

end