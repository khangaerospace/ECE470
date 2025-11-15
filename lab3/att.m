function tau = att(q,q2,myrobot)
    % robot torque
    tau = zeros(6, 1);

    zeta = 1; % zeta initial at 1

    % tau = ∑Joi (q)⊤Fi,att(o0i (q))

    for i = 1:6
        Joi = jacobian(q, myrobot, i);


        % find the start and end possition
        H_start = forward_kin(q, myrobot, i) ;
        H_end = forward_kin(q2, myrobot, i);

        % find the possition
        o_start = H_start(1:3,4);
        o_final = H_end(1:3,4);

        Fi_att = -zeta*(o_start - o_final);

        tau = tau + Joi' * Fi_att;
    end
    tau  = tau/norm(tau);
end


function J = jacobian(q, myrobot, i)
    % Initialize the Jacobian matrix. assuming every join is rotation
    J = zeros(3, 6);

    % find the join posision
    H = forward_kin(q,myrobot,i); % find the DH table of the current join to the next
    o_i = H(1:3,4); % find the location of the joint

    % join at position 0
    z_j_1 = [0, 0, 1]';
    o_j_1 = [0, 0, 0]';
    
    for j = 1:i
        J(1:3, j) =  cross(z_j_1, (o_i - o_j_1));
        
        H_i = forward_kin(q,myrobot,j);
        o_j_1 = H_i(1:3, 4); % get position
        z_j_1 = H_i(1:3, 3); % get the z-axis direction of the current joint
    end
end


function H = forward_kin(joint, myrobot, j)
    
     H = eye(4);

    
    for i = 1:j

        q = joint(i); % join variable
        
        alpha = myrobot.alpha(1,i);
        d = myrobot.d(1,i);
        a = myrobot.a(1,i);
        

        % Create H matrix 
        H_i = [cos(q) -sin(q)*cos(alpha) sin(q)*sin(alpha)    a*cos(q);
               sin(q) cos(q)*cos(alpha)  -cos(q)*sin(alpha)   a*sin(q);
               0      sin(alpha)         cos(alpha)           d;
               0      0                 0                   1];
        H = H* H_i;
    end
     
end