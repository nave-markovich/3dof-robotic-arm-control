function G = gravityVector(q, p)

    % q is in radians inside this function

    theta2 = q(2);
    theta3 = q(3);

    G1 = 0;

    G2 = p.g * ( ...
        (p.m2*p.lc2 + p.m3*p.l2)*cos(theta2) ...
        + p.m3*p.lc3*cos(theta2 + theta3) ...
    );

    G3 = p.g * p.m3*p.lc3*cos(theta2 + theta3);

    G = [
        G1;
        G2;
        G3
    ];

end