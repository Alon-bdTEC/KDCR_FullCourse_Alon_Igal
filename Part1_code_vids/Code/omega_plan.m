function omega = omega_plan(prof,t)
    % OMEGA_PLAN Calculates the angular velocity of the tool's frame at time t
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
    %   omega 3xlength(t) [rad/s] - Angular velocity matrix of the tool's frame at time t
    %
    % Description:
    %   The function calculates the velocity of the tool's frame based on the selected
    %   motion profile and the given time. There are three profiles to choose from:
    %   prof = 1, 2, or 3. Each profile defines a different motion trajectory for the tool.

    
    % Defining time of trajectory and endpoints:
    T = 2;
    R_Ato0 = eye(3);
    R_Bto0 = [0 0 1;-1 0 0;0 -1 0];
    R_BtoA = R_Ato0'*R_Bto0;
    theta_f = acos((trace(R_BtoA)-1)/2);
    
    omega = zeros(1,length(t));

    if prof == 1
        omega = theta_f/T * ones(1,length(t));
    elseif prof == 2
        omega_m = (6/(5*T)) * theta_f;
        alpha = (36/(5*T^2)) * theta_f;
        for i = 1:length(t)
            t_i = t(i);
            if 0 <= t_i && t_i < 1/6*T            
                omega(:,i) = alpha.*(t_i);
            elseif 1/6*T <= t_i && t_i < 5/6*T
                omega(:,i) = omega_m;
            else
                omega(:,i) = omega_m - alpha.*(t_i-5/6*T); 
            end
        end
    else
        lambda_t_dot = (1/T).*(30.*(t./T).^2 - 60.*(t./T).^3 + 30.*(t./T).^4 );
        omega = (theta_f - 0) * lambda_t_dot;
    end
    
    % Get omega as vector:
    n_rot = 1/(2*sin(theta_f)).*[R_BtoA(3,2)-R_BtoA(2,3);R_BtoA(1,3)-R_BtoA(3,1);R_BtoA(2,1)-R_BtoA(1,2)];
    omega = omega.*n_rot;

end