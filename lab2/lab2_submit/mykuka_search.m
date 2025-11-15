function myrobot = mykuka_search(delta)

theta = zeros(6);
DH = [theta(1), 400, 25, pi/2;
      theta(2), 0, 315, 0;
      theta(3), 0, 35, pi/2;
      theta(4), 365, 0, -pi/2;
      theta(5), 0, 0, pi/2;
      theta(6), 161.44+delta(2), -296.23-delta(1), 0];

    links(1) = Link(DH(1,:));
    links(2) = Link(DH(2,:));
    links(3) = Link(DH(3,:));
    links(4) = Link(DH(4,:));
    links(5) = Link(DH(5,:)) ;
    links(6) = Link(DH(6,:));


    % Create a SerialLink robot using the defined links
    myrobot = SerialLink(links, 'name', 'Kuka');
end