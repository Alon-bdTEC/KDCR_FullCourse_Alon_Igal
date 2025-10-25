function x = x_plan(prof,t)
    % X_PLAN Calculates the position of the tool's origin at time t
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
    %   x 3xlength(t) [m] - Position matrix of the tool's origin at time t
    %
    % Description:
    %   The function calculates the position of the tool's origin based on the selected
    %   motion profile and the given time. There are three profiles to choose from:
    %   prof = 1, 2, or 3. Each profile defines a different motion trajectory for the tool.

    % Defining time of trajectory and endpoints:
    T = 2;
    r_A = [0.25;0;1.1]; r_B = [0.55;-0.4;0.6];

    if prof == 1
        r_in_t = (r_B-r_A)./T .*t + r_A;
    elseif prof == 2
        v_m = (6/(5*T)) .*(r_B-r_A);
        a = (36/(5*T^2)) .*(r_B-r_A);
        add = zeros(3,length(t));
        for i = 1:length(t)
            t_i = t(i);
            if 0 <= t_i && t_i < 1/6*T            
                % fprintf('case 1 : t = %.2f\n', t_i);
                add(:,i) = a.*(t_i^2/2);
            elseif 1/6*T <= t_i && t_i < 5/6*T
                % fprintf('case 2 : t = %.2f\n', t_i);
                add(:,i) = a.*((1/6*T)^2/2) + v_m.*(t_i-1/6*T);
            else
                % fprintf('case 3 : t = %.2f\n', t_i);
                add(:,i) = a.*((1/6*T)^2/2) + v_m.*(2/3*T) + (t_i - 5/6*T)*(v_m - a.*(t_i-5/6*T)) + a./2.*((t_i - 5/6*T)^2); 
            end
        end
        r_in_t = r_A + add;
    else
        lambda_t = 10.*(t./T).^3 - 15.*(t./T).^4 + 6.*(t./T).^5;
        r_in_t = r_A + lambda_t.*(r_B-r_A);
    end
    x = r_in_t;
    
    
end