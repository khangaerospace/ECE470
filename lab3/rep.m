function tau = rep(q,myrobot,obs)
    % initialize torque 
    tau = zeros(6,1);
    eta = 1;

    % set all parameter
    rho = obs.rho0;
    c = obs.c;
    R = obs.R;
    type = obs.type;

    for i = 1:numel(q)
        J = jacobian(q, myrobot, i);
        H = forward_kin(q, myrobot, i);
        %o = H(1:3,4);
        
        if strcmp(type, 'sph')
            % if sphere then we will 
            o = H(1:3,4);
        end
        if strcmp(type, 'cyl')
            o = H(1:2,4);
        end

        % combute o - b
        o_b = o - ( (R* (o-c)/norm(o-c)) + c);
        rho_i = norm(o-c) - R;

        if rho_i <= rho
            % compute the repulsive force
            F = eta*((1/rho_i) - (1/rho) )*(1/norm(o_b))*(o_b/norm(o_b)^2);
            if strcmp(type, "cyl")
                F(3) = 0;
            end
        else
            F = zeros(3,1);
        end

        tau = tau + J'*F; % calculating torque

    end

    if norm(tau) ~= 0 % incase we divide by 0
        tau = tau/norm(tau);
    end
end

function J = jacobian(q, myrobot, j)
    % Initialize the Jacobian matrix. assuming every join is rotation
    J = zeros(3, 6);

    % find the join posision
    H = forward_kin(q,myrobot,j); % find the DH table of the current join to the next
    o_end = H(1:3,4); % find the location of the joint

    % join at position 0
    z_i = [0, 0, 1]';
    o_i = [0, 0, 0]';

    for i = 1:j
        J(1:3, i) =  cross(z_i, (o_end - o_i));

        % continue to find the current position

        H_i = forward_kin(q,myrobot,i);
        o_i = H_i(1:3, 4); % get position
        z_i = H_i(1:3, 3); % get the z-axis direction of the current joint


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