function q = inverse(H, myrobot)
    % This function calculates the joint angles (q) required for a robot
    % to reach a desired end-effector pose represented by the homogeneous
    % transformation matrix H, using the robot's DH parameters.

    DH = myrobot;
    Rd = H(1:3, 1:3);
    od = H(1:3, 4); 
    o = od - Rd*[0;0; DH.d(6)]; % Calculate the position of the end-effector relative to the last joint
    
    % cosine law
    D = (o(1)^2+o(2)^2-DH.d(2)^2+(o(3)-DH.d(1))^2-DH.a(2)^2-DH.d(4)^2)/(2*DH.a(2)*DH.d(4));
    
    % Calculate the first joint angle 
    q(1) = atan2(o(2),o(1)) - atan2(-DH.d(2), real(sqrt(o(1)^2+o(2)^2-DH.d(2)^2)));
    q(3) = atan2(D, real(sqrt(1-D^2)));
    q(2) = atan2(o(3)-DH.d(1), real(sqrt(o(1)^2+o(2)^2-DH.d(2)^2))) - atan2(-DH.d(4)*cos(q(3)),DH.a(2)+DH.d(4)*sin(q(3)));
    
    % Loop through the first three joints to compute their transformation matrices
    H3_0 = eye(4);
    for i = 1:3
        alpha = DH.alpha(i);
        theta = q(i);
        a = DH.a(i);
        d = DH.d(i);
        A = [ cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
              sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
              0,           sin(alpha),            cos(alpha),            d;
              0,           0,                     0,                     1 ];
        H3_0 = H3_0 * A;
    end
    R3_0 = H3_0(1:3,1:3);

    % Calculate the remaining joint angles
    R = R3_0' * Rd;
    q(4) = atan2(R(2,3), R(1,3));
    q(5) = atan2(sqrt(1 - R(3,3)^2), R(3,3));
    q(6) = atan2(R(3,2), -R(3,1));

end