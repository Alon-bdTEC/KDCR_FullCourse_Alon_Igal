function q = inverse_kin(d,elbows)
    
    % inverse_kin Calculates the inverse kinematics of the serial robot
    % in HW 3
    % 
    % Inputs:
    %   d - Desired position vector of the tool frame w.r.t. the base frame 0 [m]
    %
    %   elbows = [+-1,+-1] - Vector specifying elbow configurations:
    %            elbows(1) for q1, elbows(2) for q3
    % Outputs:
    %   q - Joint values for rotbot to get this position+rotation with
    %   chosen elbow values.
    %   q = [theta1; theta2; d3]
    %       theta1, theta2: Rotational joints [rad]
    %       d3: Linear joint [m]

     % Robot given link lengths
    l1 = 0.4;
    l2 = 0.15;
    l4 = 0.1;

    % --- Get the required joint values q from IK solution,
    % while chosing solutions with elbows

    q = zeros(3,1);

    % --- Position problem in IK

    % getting q3 = d3
    
    if (norm(d - [0; 0; l1]))^2 - l2^2 < 0 % If desired position is unreachable
        disp('IK issue - Desired position is impossible w.r.t d3');
        q = NaN(3,1);
        return;
    end

    d3 = -l4 + elbows(2)*sqrt((norm(d-[0;0;l1]))^2-l2^2);
    q(3)=d3;

    % getting q1 = theta1. Assume path doesnt cross d.x=d.y=0.
    q(1) = atan2(elbows(1)*d(2), elbows(1)*d(1));
    if d(2) == 0 && d(1)<0
        q(1) = pi;
    end

    % getting q2 = theta2
    if cos(q(1))==0
        mat2 = [-sin(q(1))*(d3+l4),sin(q(1))*l2; l2,(d3+l4)];
    else 
        mat2 = [-cos(q(1))*(d3+l4),cos(q(1))*l2; l2,(d3+l4)];
    end
    stheta2 = (mat2)\[d(1);d(3)-l1];
    q(2) = atan2(stheta2(1), stheta2(2));

end