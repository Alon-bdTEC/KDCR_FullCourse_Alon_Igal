function R_t = R_plan(prof,t)
    % R_plan Calculates the rotation of the tool's origin at time t
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
    %   R_t 3x3xlength(t) - Rotation matrix of the tool's origin at time t
    %
    % Description:
    %   The function calculates the rotation of the tool's origin based on the selected
    %   motion profile and the given time. There are three profiles to choose from:
    %   prof = 1, 2, or 3. Each profile defines a different motion trajectory for the tool.
    
    % Its written that we should modify the function to accept t in vetor
    % form adequatly

    % Defining time of trajectory and endpoints:
    T = 2;
    R_Ato0 = eye(3);
    R_Bto0 = [0 0 1;-1 0 0;0 -1 0];
    R_BtoA = R_Ato0'*R_Bto0;
    theta_f = acos((trace(R_BtoA)-1)/2);
    n_rot = 1/(2*sin(theta_f)).*[R_BtoA(3,2)-R_BtoA(2,3);R_BtoA(1,3)-R_BtoA(3,1);R_BtoA(2,1)-R_BtoA(1,2)];
    
    R_t = zeros(3,3,length(t));

    if prof == 1
        for i = 1:length(t)
            t_i = t(i);
            theta_t_i = theta_f/T*t_i;
            R_t(:,:,i) = axis_angle_to_rotmat(n_rot,theta_t_i);
        end
    elseif prof == 2
        omega_m = (6/(5*T)) * theta_f;
        alpha = (36/(5*T^2)) * theta_f;
        theta_t = zeros(1,length(t));
        for i = 1:length(t)
            t_i = t(i);
            if 0 <= t_i && t_i < 1/6*T            
                % fprintf('case 1 : t = %.2f\n', t_i);
                theta_t(i) = alpha.*(t_i^2/2);
            elseif 1/6*T <= t_i && t_i < 5/6*T
                % fprintf('case 2 : t = %.2f\n', t_i);
                theta_t(i) = alpha.*((1/6*T)^2/2) + omega_m.*(t_i-1/6*T);
            else
                % fprintf('case 3 : t = %.2f\n', t_i);
                theta_t(i) = alpha.*((1/6*T)^2/2) + omega_m.*(2/3*T) + (t_i - 5/6*T)*(omega_m - alpha.*(t_i-5/6*T)) + alpha./2.*((t_i - 5/6*T)^2); 
            end
        end
        for i = 1:length(t)
            theta_t_i = theta_t(i);
            R_t(:,:,i) = axis_angle_to_rotmat(n_rot,theta_t_i);
        end
    else
        lambda_t = 10.*(t./T).^3 - 15.*(t./T).^4 + 6.*(t./T).^5;
        theta_t = lambda_t.*theta_f;
        for i = 1:length(t)
            theta_t_i = theta_t(i);
            R_t(:,:,i) = axis_angle_to_rotmat(n_rot,theta_t_i);
        end
    end
    
    
end

function R = axis_angle_to_rotmat(n, theta)
% axis_angle_to_rotmat Computes a 3x3 rotation matrix from axis-angle representation.
%
%   R = axis_angle_to_rotmat(n, theta)
%
%   Inputs:
%       n     - 3x1 unit vector representing the axis of rotation.
%               If not normalized, it will be normalized internally.
%       theta - Scalar rotation angle in radians.
%
%   Output:
%       R     - 3x3 rotation matrix corresponding to a rotation of angle theta
%               around axis n, computed using Rodrigues' rotation formula.


    if norm(n) == 0
        error('Rotation axis n must be non-zero.');
    end
    n = n / norm(n);  % Ensure it's a unit vector

    nx = n(1);
    ny = n(2);
    nz = n(3);

    % Skew-symmetric cross-product matrix of n
    N_skew = [  0   -nz   ny;
               nz    0  -nx;
              -ny   nx    0 ];

    % Rodrigues' rotation formula
    R = eye(3) + sin(theta) * N_skew + (1 - cos(theta)) * (N_skew^2);
end
