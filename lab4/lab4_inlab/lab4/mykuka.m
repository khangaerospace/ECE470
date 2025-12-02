function myrobot = mykuka(DH)
    links(1) = Link(DH(1,:));
    links(2) = Link(DH(2,:));
    links(3) = Link(DH(3,:));
    links(4) = Link(DH(4,:));
    links(5) = Link(DH(5,:));
    links(6) = Link(DH(6,:));


% Create a SerialLink robot using the defined links
myrobot = SerialLink(links, 'name', 'Kuka', 'plotopt', {'notiles'});
end