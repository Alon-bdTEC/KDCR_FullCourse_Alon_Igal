function Jq = Calc_Jq(q,X)
    % Calc_Jq Calculated the Jq jacobian of the robot in HW2
    % Inputs:
    %   X = [x;y;phi]; - task vector.
    %   q = [theta1; theta2; theta3] - joint values
    %
    % Outputs:
    %   Jq 3x3 - Jacobian Matrix Jq
    
    % Robot given link lengths
    r = 1.5;
    R = 3.5;

    % Extract task variables
    x = X(1);
    y = X(2);
    phi = X(3);
    
    % Angles
    t1 = q(1);
    t2 = q(2);
    t3 = q(3);

    Jq =[R*(2*x*cos(t1) + 2*y*sin(t1) + r*cos(phi - t1) - 3^(1/2)*r*sin(phi - t1)), 0, 0;
         0, 2*R*(x*cos(t2) + y*sin(t2)), 0;
         0, 0, 2*R*(x*cos(t3) + y*sin(t3) + r*cos(phi - t3))];
 

end