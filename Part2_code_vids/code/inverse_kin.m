function q = inverse_kin(X,elbows)
    
    % inverse_kin Calculates the inverse kinematics of the paraller robot
    % in HW 2
    % 
    % Inputs:
    %   X = [x;y;phi]; - task vector, where:
    %       x,y - Desired position vector of the platform frame w.r.t. the base frame 0
    %       phi - Desired rotation angle of the platform frame w.r.t. the base frame 0
    %
    %   elbows - Vector specifying elbow configurations [+-1,+-1,+-1]:
    %            elbows(1) for theta1, elbows(2) for theta2, elbows(3) for theta3
    % Outputs:
    %   q - Joint values for robot to get this position+rotation with
    %   chosen elbow values.
    %   q = [theta1; theta2; theta3]
    %       theta1, theta2, theta3: Linear joints in [rad] (position in
    %       circualr rail)


    % Robot given link lengths
    r=1.5;
    L=3;
    R=3.5;

    % Extract task variables
    x = X(1);
    y = X(2);
    phi = X(3);

    % --- Get the required joint values q from IK solution,
    % while chosing solutions with elbows

    q = zeros(3,1);

    % Calculate A',B',C'
    r_B_tag = [x;y];
    r_A_tag = r_B_tag + r.* [cos(phi+pi/3);sin(phi+pi/3)];
    r_C_tag = r_B_tag + r.* [cos(phi);sin(phi)];

    % Coeff for caluclate IK for each angle
    a1 = 2*R*r_A_tag(2);
    b1 = -2*R*r_A_tag(1); 
    f1 = L^2-r_A_tag(2)^2-r_A_tag(1)^2-R^2;

    a2 = 2*R*r_B_tag(2);
    b2 = -2*R*r_B_tag(1);
    f2 = L^2-r_B_tag(2)^2-r_B_tag(1)^2-R^2;

    a3 = 2*R*r_C_tag(2);
    b3 = -2*R*r_C_tag(1);
    f3 = L^2-r_C_tag(2)^2-r_C_tag(1)^2-R^2;

    % Check discriminants for validity
    delta1 = a1^2 + b1^2 - f1^2;
    delta2 = a2^2 + b2^2 - f2^2;
    delta3 = a3^2 + b3^2 - f3^2;
    
    if delta1 < 0 || delta2 < 0 || delta3 < 0
        warning('Invalid configuration: discriminant is negative. No real solution exists.');
        q = [];
        return;
    end

    % Joint 1 - theta1
    % Solve eq a1*C1 + b1*S1 = f1
    theta1 = atan2(b1,a1) + atan2(elbows(1)*sqrt(delta1),f1);
    % Normalize theta in (-pi,pi]
    theta1 = mod(theta1 + pi, 2*pi) - pi;

    % Same procedure for joints 2 and 3 (same eq.)
    % Joint 2 - theta2
    theta2 = atan2(b2,a2) + atan2(elbows(2)*sqrt(delta2),f2);

    theta2 = mod(theta2 + pi, 2*pi) - pi;

    % Joint 3 - theta3
    theta3 = atan2(b3,a3) + atan2(elbows(3)*sqrt(delta3),f3);

    theta3 = mod(theta3 + pi, 2*pi) - pi;

    q = [theta1;theta2;theta3];

end