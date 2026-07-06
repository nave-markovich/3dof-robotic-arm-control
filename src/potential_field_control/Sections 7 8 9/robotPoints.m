function points = robotPoints(q_deg, p)

    % q_deg is in degrees because animation uses the simulation state directly

    theta1 = deg2rad(q_deg(1));
    theta2 = deg2rad(q_deg(2));
    theta3 = deg2rad(q_deg(3));

    % Base point
    P0 = [0; 0; 0];

    % End of link 1
    P1 = [
        0;
        p.l1;
        0
    ];

    % End of link 2
    P2 = [
        cos(theta1)*p.l2*cos(theta2);
        p.l1 + p.l2*sin(theta2);
        sin(theta1)*p.l2*cos(theta2)
    ];

    % End of link 3
    P3 = [
        cos(theta1)*(p.l2*cos(theta2) + p.l3*cos(theta2 + theta3));
        p.l1 + p.l2*sin(theta2) + p.l3*sin(theta2 + theta3);
        sin(theta1)*(p.l2*cos(theta2) + p.l3*cos(theta2 + theta3))
    ];

    points = [P0, P1, P2, P3];

end