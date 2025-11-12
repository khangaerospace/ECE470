function J = jacobian(q, myrobot, j)
    % Initialize the Jacobian matrix. assuming every join is rotation
    J = zeros(3, 6);

    % find the join posision
    H = forward(q,myrobot,j); % find the DH table of the current join to the next
    o_end = H(1:3,4); % find the location of the joint

    % join at position 0
    z_i = [0, 0, 1]';
    o_i = [0, 0, 0]';

    for i = 1:j
        J(1:3, i) =  cross(z_i, (o_end - o_i));

        % continue to find the current position

        H_i = forward(q,myrobot,i);
        o_i = H_i(1:3, 4) % get position
        z_i = H_i(1:3, 3); % get the z-axis direction of the current joint


    end
end