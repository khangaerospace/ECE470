function q = inverse_kuka(H, myrobot)
    % Inverse kinematics for KUKA-like 6DOF arm (partial solution)
    % H: desired homogeneous transform (4x4)
    % myrobot: struct containing DH parameters (a, d, etc.)

    DH = myrobot;
    Rd = H(1:3, 1:3);
    od = H(1:3, 4);

    % Wrist center position
    oc = od - Rd * [DH.a(6); 0; DH.d(6)];

    q = zeros(6,1);

    % Joint 1
    q(1) = atan2(oc(2), oc(1));

    % Geometry calculations
    r = sqrt(oc(1)^2 + oc(2)^2);
    s = oc(3) - DH.d(1);
    rho = r - DH.a(1);

    % Intermediate variables for link geometry
    D = 2 * DH.a(2) * sqrt(DH.a(3)^2 + DH.d(4)^2);
    N = (rho^2 + s^2) - (DH.a(3)^2 + DH.d(4)^2) - (DH.a(2)^2);

    % Joint 3
    q(3) = atan2(N, sqrt(D^2 - N^2)) - atan2(DH.a(3), DH.d(4));

    % Joint 4 (partial / orientation placeholder)
    q(4) = atan2(s, r) - atan2( ...
        sqrt(DH.a(3)^2 + DH.d(4)^2) * sin(q(3) + atan2(DH.a(3), DH.d(4)) - pi/2), ...
        (DH.a(2) + sqrt(DH.a(3)^2 + DH.d(4)^2) * cos(q(3) + atan2(DH.a(3), DH.d(4)) - pi/2) ...
    );
end