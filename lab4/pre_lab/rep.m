function tau = rep(q,myrobot,obs)
    % initialize torque 
    tau = zeros(6,1);
    eta = 1;

    for i = 1:numel(q)
        Joi = jacobian(q, myrobot, i);
        H = forward_kin(q, myrobot, i);
        o = H(1:3,4);
        F = [0,0,0]';
        for k = 1:numel(obs)
            
            ob = obs{k};
            type = string(ob.type);
            if strcmp(type, 'plane')
                try
                    z_ground = ob.z;
                catch
                    z_ground = 0;
                end
                rho0 = ob.rho0;
                if o(3) > z_ground
                    b = [o(1) o(2) z_ground];
                    v = o - b;
                    rho = norm(v);
                    if rho <= rho0
                        F = F + eta*( 1/rho - 1/rho0)*(v/rho^3);
                    end
                end
            end
            if strcmp(type, 'cyl')
                cx = ob.c(1);
                cy = ob.c(2);
                R = ob.R;
                h = ob.h;
                rho0 = ob.rho0;
                try
                    z_ground = ob.z;
                catch
                    z_ground = 0;
                end
                r_vec = [o(1)-cx; o(2)-cy];
                r = norm(r_vec);
                try
                    rhat = r_vec/r;
                catch
                    rhat = [0, 0];
                end

                z_top = z_ground + h;

                r_clamp = min(r, R);
                z_clamp = min(max(o(3), z_ground), z_top);

                b = [cx; cy; z_clamp] + [r_clamp*rhat; 0];
                v = o-b;
                rho = norm(v);
                if rho > 0 && rho<rho0
                    F = F + eta*( 1/rho - 1/rho0)*(v/rho^3);
                end

            end
        end

        tau = tau + Joi'*F; % calculating torque

    end

    if norm(tau) ~= 0 % incase we divide by 0
        tau = tau/norm(tau);
    end
end

function J = jacobian(q, myrobot, i)
    % Initialize the Jacobian matrix. assuming every join is rotation
    J = zeros(3, 6);

    % find the join posision 
    H = forward_kin(q,myrobot,i); % find the DH table of the current join to the next
    o_i = H(1:3,4); % find the location of the joint

    % join at frame 00
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