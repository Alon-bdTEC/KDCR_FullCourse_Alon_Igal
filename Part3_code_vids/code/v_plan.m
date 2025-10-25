function v = v_plan(prof,t_i)
    % V_PLAN Calculates the velocity of the tool's origin at time t
    %
    % Inputs:
    %   prof - Profile selection (1, 2, or 3) to choose the desired motion profile
    %   Where:
    %       1 - constant velocity
    %       2 - trapezoidal velocity
    %       3 - polynomial velocity
    %   t_i 1x1 - Time for which we output the calculated velocity [s]
    %
    % Outputs:
    %   v 3x1 [m/s] - Velocity of the tool's origin at time t
    %
    % Description:
    %   The function calculates the velocity of the tool's origin based on the selected
    %   motion profile and the given time. There are three profiles to choose from:
    %   prof = 1, 2, or 3. Each profile defines a different motion trajectory for the tool.

    
    % Defining time of trajectory and endpoints:
    T = 2;
    r_A = [0.25;0;0.6]; r_B = [0.05;-0.4;0.6];

    if prof == 1
        v_in_t = (r_B-r_A)./T;
    elseif prof == 2
        v_m = (6/(5*T)) .*(r_B-r_A);
        a = (36/(5*T^2)) .*(r_B-r_A);
        v_in_t = zeros(3,1);
        if 0 <= t_i && t_i < 1/6*T            
            % fprintf('case 1 : t = %.2f\n', t_i);
            v_in_t(:,1) = a.*t_i;
        elseif 1/6*T <= t_i && t_i < 5/6*T
            % fprintf('case 2 : t = %.2f\n', t_i);
            v_in_t(:,1) = v_m;
        else
            % fprintf('case 3 : t = %.2f\n', t_i);
            v_in_t(:,1) = v_m - a.*(t_i - 5/6*T); 
        end
    else
        lambda_t_dot = (1/T)*(30.*(t_i./T).^2 - 60*(t_i./T).^3 + 30*(t_i./T).^4 );
        v_in_t = lambda_t_dot.*(r_B-r_A);
    end
    v = v_in_t;
end