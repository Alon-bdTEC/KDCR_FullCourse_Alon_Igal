function f2_sys2 = f2calc(prof,t_i,elbows)
    % tau_plan Calculates the tau with time vector t
    %
    % Inputs:
    %   prof - Profile selection (1, 2, or 3) to choose the desired motion profile
    %   Where:
    %       1 - constant velocity
    %       2 - trapezoidal velocity
    %       3 - polynomial velocity
    %   t_i 1x1 - Time for which we output the calculated tau forces/torques [s]
    %   elbows - choice of one of 4 solutions of IK: [+-1,+-1]
    %
    % Outputs:
    %   f2_sys2 1x1 - calculate f2 in y2 direction
    
    q_t = q_plan(prof,t_i,elbows);
    q_dot = q_dot_plan(prof,t_i,elbows);
    q_dot2 = q_dot2_plan(prof,t_i,elbows);

    q2 = q_t(2,1);d3 = q_t(3,1);
    q1d = q_dot(1,1);q2d = q_dot(2,1);d3d = q_dot(3,1);
    q1dd = q_dot2(1,1);

    % Robot parameters
    l2 = 0.15;
    l4 = 0.1;  % offset at gripper from prismatic joint
    l = 0.6;   % Value for prismatic purple segment
    g = 9.8;
    
    % masses
    rou = 7800; % Density of links (metal), in [kg/m^3]
    Area = pi*(0.015)^2/4; % Area of rod seciton, in [m^2]
    m2 = rou*Area*l2; % Mass of link 2, in [kg]
    m3 = rou*Area*l; % Mass of link 3, in [kg]
    M = 0.5; % Mass of point mass in gripper, in [kg]
    
    f2_sys2 = (- m3*(2*d3*cos(q2) - l*cos(q2) + 2*l4*cos(q2) + 2*l2*sin(q2)) - M*(2*d3*cos(q2) + 2*l4*cos(q2) + 2*l2*sin(q2)) - l2*m2*sin(q2))*q1d*q2d +...
        (- 2*M*sin(q2) - 2*m3*sin(q2))*q1d*d3d +...
        (m3*(l2*cos(q2) - d3*sin(q2) + (l*sin(q2))/2 - l4*sin(q2)) - M*(d3*sin(q2) - l2*cos(q2) + l4*sin(q2)) + (l2*m2*cos(q2))/2)*q1dd;
    
    f2_sys2 = -f2_sys2;

end