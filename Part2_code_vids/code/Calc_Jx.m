function Jx = Calc_Jx(q,X)
    % Calc_Jx Calculated the Jx jacobian of the robot in HW2
    % Inputs:
    %   X = [x;y;phi]; - task vector.
    %   q = [theta1; theta2; theta3] - joint values
    %
    % Outputs:
    %   Jx 3x3 - Jacobian Matrix Jx
    
    % Robot given link lengths
    r = 1.5;
    R = 3.5;

    % Extract task variables
    x = X(1);
    y = X(2);
    phi = X(3);
    alpha = pi/3;
    
    % Angles
    t1 = q(1);
    t2 = q(2);
    t3 = q(3);

    Jx = [2*x - 2*R*sin(t1) + 2*r*cos(phi + pi/3), 2*y + 2*R*cos(t1) + 2*r*sin(phi + pi/3), 2*r*cos(phi + pi/3)*(y + R*cos(t1) + r*sin(phi + pi/3)) - 2*r*sin(phi + pi/3)*(x - R*sin(t1) + r*cos(phi + pi/3));
          2*x - 2*R*sin(t2),                       2*y + 2*R*cos(t2),                                                                                                                 0;
          2*x - 2*R*sin(t3) + 2*r*cos(phi),        2*y + 2*R*cos(t3) + 2*r*sin(phi),                                                                   2*r*(y*cos(phi) + R*cos(phi - t3) - x*sin(phi))];

    % 
    % Jx = [2*x + 2*r*cos(alpha + phi) - 2*R*sin(t1), 2*y + 2*r*sin(alpha + phi) + 2*R*cos(t1), 2*R*r*cos(alpha + phi - t1) + 2*r*y*cos(alpha + phi) - 2*r*x*sin(alpha + phi);
    %       2*x - 2*R*sin(t2),                        2*y + 2*R*cos(t2),                                                                             0;
    %       2*x - 2*R*sin(t3) + 2*r*cos(phi),         2*y + 2*R*cos(t3) + 2*r*sin(phi),                               2*r*(y*cos(phi) + R*cos(phi - t3) - x*sin(phi))];

end