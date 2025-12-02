function tau = rep(q,myrobot,obs)
    eta = 1;

    tau = zeros(6,1);

    for i = 1:numel(q)
        Joi = jacobian(q, myrobot, i); % 3x6 linear Jacobian to oi
        H   = forward_kin(q, myrobot, i);
        o   = H(1:3,4);% 3x1
        F   = [0;0;0];

        for k = 1:numel(obs)
            
            type = obs.type;

            switch type
                case 'plane'
                    try
                        z_ground = obs.z;
                    catch
                        z_ground = 32;
                    end
                    rho0 = obs.rho0;

                    if o(3) > z_ground
                        b   = [o(1); o(2); z_ground]; % column vector!
                        v   = o - b;
                        rho = norm(v);
                        if rho > 0 && rho <= rho0
                            F = F + eta*(1/rho - 1/rho0) * (v / rho^3);
                        end
                    end

                case 'cyl'
                    try
                        z_base = obs.z;
                    catch
                        z_base = 0;
                    end
                    cx = obs.c(1);
                    cy = obs.c(2);
                    R  = obs.R;
                    h  = obs.h;     
                    rho0 = obs.rho0;
                    z_top = z_base + h;

                    r_vec = [o(1)-cx; o(2)-cy];   r = norm(r_vec);
                    if r > 0
                        rhat = r_vec / r; 
                    else
                        rhat = [0;0];
                    end

                    % Clamp to cylinder (picks side / cap / rim automatically)
                    r_clamp = min(r, R);
                    z_clamp = min(max(o(3), z_base), z_top);

                    b   = [cx; cy; z_clamp] + [r_clamp * rhat; 0];
                    v   = o - b;
                    rho = norm(v);

                    if rho > 0 && rho <= rho0
                        F = F + eta*(1/rho - 1/rho0) * (v / rho^3);
                    end

                case 'sph'
                    % Sphere of radius R centered at c
                    c   = obs.c(:);  R = obs.R;  rho0 = obs.rho0;
                    diff = o - c;   distc = norm(diff);
                    rho  = distc - R;  % clearance from surface
                    if distc > 0 && rho <= rho0 && rho > 0
                        % grad rho = diff / distc; use F = eta*(1/rho-1/rho0)*(grad rho)/rho^2
                        F = F + eta*(1/rho - 1/rho0) * ((diff / distc) / (rho^2));
                    end

                otherwise
                    % ignore unknown types
            end
        end

        tau = tau + Joi.' * F;                    % accumulate torque
    end

    % Normalize and return row
    nrm = norm(tau);
    if nrm > 0
        tau = (tau./nrm).';
    else
        tau = tau.';
    end
end
function J = jacobian(q, myrobot, i)
    % Initialize the Jacobian matrix. assuming every join is rotation
    J = zeros(3, 6);

    % find the join posision 
    H = forward_kin(q,myrobot,i); % find the DH table of the current join to the next
    o_i = H(1:3,4); % find the location of the joint

    % join at frame 0
    z_j_1 = [0, 0, 1]';
    o_j_1 = [0, 0, 0]';
    
    for j = 1:i
        % computing the first J
        J(1:3, j) =  cross(z_j_1, (o_i - o_j_1));
        
        % continue until the last J
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