function tau = att(q,q2,myrobot)
    % robot torque
    tau = zeros(6, 1);

    zeta = 1; % zeta initial at 1

    % tau = ∑Joi (q)⊤Fi,att(o0i (q))

    for i = 1:6
        Joi = jacobian(q, myrobot, i);

        % find the start and end possition
        H_start = forward(q, myrobot, i) ;
        H_end = forward(q2, myrobot, i);

        % find the possition
        o_start = H_start(1:3,4);
        o_final = H_end(1:3,4);

        Fi_att = zeta*(o_start - o_final);

        tau = tau + Joi' * Fi_att;

        
    end
end




