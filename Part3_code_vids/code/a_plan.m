function a = a_plan(prof,t_i)
    % A_PLAN Calculates the acceleration of the tool's origin at time t
    %
    % Inputs:
    %   prof - Acceleration selection (1, 2, or 3) to choose the desired motion profile
    %   Where:
    %       1 - constant velocity
    %       2 - trapezoidal velocity
    %       3 - polynomial velocity
    %   t_i 1x1 - Time for which we output the calculated acceleration [s]
    %
    % Outputs:
    %   a 3x1 [m/s^2] - Acceleration of the tool's origin at time t, for each time
    %
    % Description:
    %   The function calculates the acceleration of the tool's origin based on the selected
    %   motion profile and the given time. There are three profiles to choose from:
    %   prof = 1, 2, or 3. Each profile defines a different motion trajectory for the tool.


    % Defining time of trajectory and endpoints:
    T = 2;
    r_A = [0.25;0;0.6]; r_B = [0.05;-0.4;0.6];

    if prof == 1
        a_in_t = zeros(3,1);
    elseif prof == 2
        a = (36/(5*T^2)) .*(r_B-r_A);
        a_in_t = zeros(3,1);
        if 0 <= t_i && t_i < 1/6*T            
            % fprintf('case 1 : t = %.2f\n', t_i);
            a_in_t(:,1) = a;
        elseif 1/6*T <= t_i && t_i < 5/6*T
            % fprintf('case 2 : t = %.2f\n', t_i);
            a_in_t(:,1) = zeros(3,1);
        else
            % fprintf('case 3 : t = %.2f\n', t_i);
            a_in_t(:,1) = -a; 
        end
    else
        lambda_t_dot_dot = (1/T^2)*(60.*(t_i./T) - 180*(t_i./T).^2 + 120*(t_i./T).^3 );
        a_in_t = lambda_t_dot_dot.*(r_B-r_A);
    end
    a = a_in_t;
end