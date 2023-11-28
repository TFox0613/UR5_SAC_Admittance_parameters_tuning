function Output = ForwardKinematics(theta, Axis)
    % ============== Input ==============
    % theta : (Axis, 1) joint angle
    % Axis : Means that transition matrix is from base to Axis

    % ============= Output ==============
    % G : Transition Matrix from base to Axiss

    % Functions
    ROTZ = @(theta) [cos(theta) -sin(theta) 0; sin(theta) cos(theta) 0; 0 0 1];
    ROTX = @(alpha) [1 0 0; 0 cos(alpha) -sin(alpha); 0 sin(alpha) cos(alpha)];

    d1 = 0.089159; d4 = 0.10915;  d5 = 0.09465; d6 = 0.0823;
    a2 = -0.425;   a3 = -0.39225;

    params.DH.Theta0    = [pi/2, pi/2,  0, pi/2,     0,  0];      % Joint variable offset [rad]
    params.DH.D         = [  d1,    0,  0,   d4,    d5, d6];      % Joint extension [m]
    params.DH.A         = [   0,   a2, a3,    0,     0,  0];      % Joint offset [m]
    params.DH.Alpha     = [pi/2,    0,  0, pi/2, -pi/2,  0];      % Joint twist [rad]

    % Parameters
    theta = theta - params.DH.Theta0;
    d     = params.DH.D;
    a     = params.DH.A;
    alpha = params.DH.Alpha;

    Td = eye(4);
    Ta = eye(4);
    Rtheta = eye(4);
    Ralpha = eye(4);

    G = eye(4);

    for i = 1 : Axis
        Td(3,4) = d(i);
        Ta(1,4) = a(i);
        Rtheta(1:3, 1:3) = ROTZ(theta(i));
        Ralpha(1:3, 1:3) = ROTX(alpha(i));
        G = G * Td * Rtheta * Ta * Ralpha;
    end

    Output = G(1:3, 4);
end