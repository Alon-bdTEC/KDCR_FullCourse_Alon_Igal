function q = inverse_kin(d,R,elbows)
    
    % inverse_kin Calculates the inverse kinematics of the serial robot
    % in HW 1
    % 
    % Inputs:
    %   d - Desired position vector of the tool frame w.r.t. the base frame 0 [m]
    %   R - Desired rotation matrix of the tool frame w.r.t. the base frame 0
    %
    %   elbows - Vector specifying elbow configurations:
    %            elbows(1) for q1, elbows(2) for q3, elbows(3) for q5
    % Outputs:
    %   q - Joint values for rotbot to get this position+rotation with
    %   chosen elbow values.
    %   q = [theta1; theta2; d3; theta4; theta5; theta6]
    %       theta1, theta2, theta4, theta5, theta6: Rotational joints [rad]
    %       d3: Linear joint [m]


    % gets inverse kinematics for d, R and elbows - 1 for q1, 2 for q3, 3 for
    % q5;

    % Robot given link lengths
    l1=0.4;
    l2=0.15;
    l4=0.1;
    l5=0.3;
    l6=0.2;

    % --- Get the required joint values q from IK solution,
    % while chosing solutions with elbows

    q=zeros(1,6);

    % Get joint 5 position and rotation for position problem in IK
    A0t = [R,d ; [0 0 0 1]];
    d05 = d-R*[0;0;l6+l5];

    % --- Position problem in IK

    % getting q3 = d3
    
    if (norm(d05 - [0; 0; l1]))^2 - l2^2 < 0 % If desired position is unreachable
        disp('IK issue - Desired position is impossible w.r.t d3');
        q = NaN(6,1);
        return;
    end

    d3 = -l4 + elbows(2)*sqrt((norm(d05-[0;0;l1]))^2-l2^2);
    q(3)=d3;

    % getting q1 = theta1. Assume path doesnt cross d.x=d.y=0.
    q(1) = atan2(elbows(1)*d05(2), elbows(1)*d05(1));

    % getting q2 = theta2
    if cos(q(1))==0
        mat2 = [-sin(q(1))*(d3+l4),sin(q(1))*l2; l2,(d3+l4)];
    else 
        mat2 = [-cos(q(1))*(d3+l4),cos(q(1))*l2; l2,(d3+l4)];
    end
    stheta2 = (mat2)\[d05(1);d05(3)-l1];
    q(2) = atan2(stheta2(1), stheta2(2));

    % --- Rotation problem in IK

    % A03 calculation gor get R03
    A01 = [cos(q(1)) -sin(q(1)) 0 0; sin(q(1)) cos(q(1)) 0 0; 0 0 1 0; 0 0 0 1];
    A12 = [cos(q(2))  0 -sin(q(2)) 0; 0 1 0 0; sin(q(2)) 0 cos(q(2)) l1;0 0 0 1];
    A23 = [1 0 0 l2; 0 1 0 0; 0 0 1 d3; 0 0 0 1];
    A03=A01*A12*A23;
    R03=A03(1:3,1:3);
    % Now, we have the rotiation matrix needed for finding q4,q5,q6
    R3t=R03'*R;

    % getting q5 = theta5
    c5=R3t(3,3);
    
    if abs(c5) >1 % If desired rotaion is unreachable
        disp('IK issue - Desired rotation is impossible w.r.t theta5');
        q = NaN(6,1);
        return;
    end

    s5=elbows(3)*sqrt(1-c5^2);
    q(5)=atan2(s5,c5);

    % getting q4,q6 = theta4,theta6
    if s5== 0 % Singular case - undesired in path planning
        %infinite solutions, we chose q4 to be zero.
        q(4)=0;
        c6=R3t(2,2);
        s6=R3t(2,1);
        q(6)=atan2(s6,c6);
    else % Non-singular case, one solution for each angles
        c4=-(R3t(1,3)/s5);
        s4=-(R3t(2,3)/s5);
        c6=(R3t(3,1)/s5);
        s6=-(R3t(3,2)/s5);
        q(4)=atan2(s4,c4);
        q(6)=atan2(s6,c6);
    end
end