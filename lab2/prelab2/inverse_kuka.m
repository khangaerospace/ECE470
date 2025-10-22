function q = inverse_kuka(H, myrobot)
    DH = myrobot; 
    Rd = H(1:3, 1:3); od = H(1:3, 4); 
    oc = od - Rd*[DH.a(6);0; DH.d(6)];

    q = zeros(1,6); % make all the angle

    q(1) = atan2(oc(2), oc(1));

    % Geometry for planar part
    r = sqrt(oc(1)^2 + oc(2)^2);
    s = oc(3) - DH.d(1);
    rho = r - DH.a(1);
    L = sqrt(DH.a(3)^2 + DH.d(4)^2);
    d = sqrt(rho^2 + s^2);
    phi = atan2(DH.a(3),DH.d(4));

    % Joint 3
    q(3) = atan2(d^2 - L^2 - DH.a(2)^2, sqrt( (2*DH.a(2)*L)^2 - (d^2 - L^2 - DH.a(2)^2)^2)) - phi;

    % Joint 2
    
    
    phi2 = atan2(sqrt(DH.a(3)^2 + DH.d(4)^2)*sin(q(3)+phi-pi/2), ...
        DH.a(2) + sqrt(DH.a(3)^2 + DH.d(4)^2)*cos(q(3)+phi-pi/2));
    q(2) = atan2(s, rho) - phi2;

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
