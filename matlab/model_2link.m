run_model = true;

if run_model
    syms theta alpha a d theta1 theta2 q1(t) q2(t) x1 x2 x3 x4 tau
    
    % transformation matrix for each row of DH parameters
    fprintf('Initializing DH parameter matrix\n');
    A(alpha, a, d, theta) = [cos(theta) -sin(theta)*cos(alpha) sin(theta)*sin(alpha)  a*cos(theta);
                             sin(theta) cos(theta)*cos(alpha)  -cos(theta)*sin(alpha) a*sin(theta);
                                   0          sin(alpha)              cos(alpha)              d;
                                   0              0                       0                   1];

    fprintf('Initializing solid model parameters\n');
    m = [0.2526 0.1183]; % mass of each link

    % Inertia matrix as obtained from SolidWorks
    J1 = [33    0   0;
          0     2   5;
          0     5  32] .* 1e-4;
    J2 = [16.8  0   0;
          0    16.6 0.5;
          0     0.5 0.2] .* 1e-4;

    % Similarity transformation to tranform to the same reference frame
    I1 = rotx(-90)' * J1 * rotx(-90);
    I2 = rotx(-90)' * roty(-90)' * J2 * roty(-90) * rotx(-90);

    % measurements from SolidWorks
    dc(1) = 0; dc(2) = 0.1493;
    ac(1) = 0; ac(2) = 0.1823;
    d(1) = 0; d(2) = 0.3;
    a(1) = 0; a(2) = 0.3;

    % DH parameters for center of mass and end of links
    dhcom = [-pi/2, 0,      0,       theta1; 
               0,   0,      dc(2),   0; 
               0,   ac(2),  0.0116,  theta2 - (pi/2)];
    dh = [-pi/2, 0,     0,       theta1; 
            0,   0,     d(2),    0; 
            0,   a(2),  0.0238,  theta2 - (pi/2)];

    % transformation matrix for each link-end seperately
    fprintf('Computing DH parameters and performing coordinate transformations\n');
    A1 = A(dh(1,1), dh(1,2), dh(1,3), dh(1,4)) * A(dh(2,1), dh(2,2), dh(2,3), dh(2,4));
    A2 = A(dh(3,1), dh(3,2), dh(3,3), dh(3,4));

    % transformation chain
    T1 = A1;
    T2 = A1 * A2;

    % transformation matrix for each link' COM seperately
    Ac1 = A(dhcom(1,1), dhcom(1,2), dhcom(1,3), dhcom(1,4)) * A(dhcom(2,1), dhcom(2,2), dhcom(2,3), dhcom(2,4));
    Ac2 = A(dhcom(3,1), dhcom(3,2), dhcom(3,3), dhcom(3,4));

    % transformation matrix chain for link COMs
    Tc1 = Ac1;
    Tc2 = A1 * Ac2;

    % height of link COMs from reference plane
    z0 = [0; 0; 1];
    z1 = [Tc1(1,3); Tc1(2,3); Tc1(3,3)];

    % coordinates of each link-end
    o0 = [0; 0; 0];
    o1 = [T1(1,4); T1(2,4); T1(3,4)];
    o2 = [T2(1,4); T2(2,4); T2(3,4)];

    % coordinates of each link-end
    oc0 = [0; 0; 0];
    oc1 = [Tc1(1,4); Tc1(2,4); Tc1(3,4)];
    oc2 = [Tc2(1,4); Tc2(2,4); Tc2(3,4)];

    % jacobian matrix for linear velocity of each link
    Jcv1 = zeros(3,2);
    Jcv2 = [cross(z0, oc2 - o0), cross(z1, oc2 - o1)];

    % jacobian matrix for angular velocity of each link
    Jcw1 = [z0, [0;0;0]];
    Jcw2 = [z0, z1];

    % rotation matrices that transform link COMs to global coordinate frame
    R1 = Tc1(1:3, 1:3);
    R2 = Tc2(1:3, 1:3);

    % gravity vector
    g = [0; 0; 9.81];

    % generalized coordinate matrix
    q = [q1(t); q2(t)];
    qdot = diff(q, t);

    % potential energy
    fprintf('Computing potential energy matrix\n');
    P_theta = (m(1) .* (g' * oc1)) + (m(2) .* (g' * oc2));
    P = subs(P_theta, [theta1, theta2], [q(1), q(2)]);

    % gravity matrix
    fprintf('Computing gravity matrix\n');
    G = [diff(P_theta, theta1) ; diff(P_theta, theta2)];
    G = subs(G, [theta1, theta2], [q(1), q(2)]);

    % mass matrix
    fprintf('Computing mass matrix\n');
    tic
    M_theta = (m(1) .* (Jcv1' * Jcv1)) + (m(2) .* (Jcv2' * Jcv2)) + (Jcw1' * R1 * I1 * R1' * Jcw1) + (Jcw2' * R2 * I2 * R2' * Jcw2);
    M = subs(M_theta, [theta1, theta2], [q(1), q(2)]);
    time_M = toc;

    % total energy
    fprintf('Computing total energy\n');
    TE = P + ((1/2) .* qdot' * M * qdot);
    TE_ref = subs(TE, [q1(t), q2(t)], [0, 0]);

    % centrepetal/coriolis matrix
    fprintf('Computing centrepetal/coriolis matrix\n');
    C_theta = sym(zeros(2,2));
    theta_vector = [theta1; theta2];
    for i=1:2
        for j = 1:2
            for k = 1:2
                C_theta(i,j) = C_theta(i,j) + ((1/2) * (diff(M_theta(i,j), theta_vector(k)) + diff(M_theta(i,k), theta_vector(j)) - diff(M_theta(k,j), theta_vector(i))));
            end
        end
    end
    C = subs(C_theta, [theta1, theta2], [q(1), q(2)]);

    % dynamics
    fprintf('Computing acceleration equations\n');
    X = [x1; x2; x3; x4];   % state vector
    u = [tau; 0];    % control vector
    tic
    accel = inv(M) * (u - (C * qdot) - G);  % acceleration equation
    time_accel = toc;
    accel = subs(accel, [q(1), q(2), qdot(1), qdot(2)], [X(1), X(2), X(3), X(4)]);

    % check for equilibrium
    fprintf('Checking for equilibrium\n');
    equil = sym(zeros(2,1));
    for i = 1:2
        equil(i) = subs(accel(i), [X(1), X(2), X(3), X(4), tau], [0, 0, 0, 0, 0]);
    end

    % compute A and B matrices at operating point
    fprintf('Computing A matrix\n');
    op = [0;0;0;0];
    A_matrix = zeros(4,4);
    for i = 1:2
        A_matrix(i, i+2) = 1;
        for j = 1:4
            A_matrix(i+2, j) = subs(diff(accel(i), X(j)), [X(1) X(2) X(3) X(4) tau], [op(1) op(2) op(3) op(4) 0]);
        end
    end

    B_matrix = zeros(4,2);
    fprintf('Computing B matrix\n');
    for i = 1:2
        B_matrix(i+2) = subs(diff(accel(i), tau), [X(1) X(2) X(3) X(4) tau], [op(1) op(2) op(3) op(4) 0]);
    end
    
    C_matrix = eye(4);
    D_matrix = zeros(4,2);
    
    fprintf('Time to calc M: %f | Time to calc accel: %f\n', time_M, time_accel);
else
    if isfile('Matrices.mat')
        load Matrices.mat
    else
        fprintf('Matrices.mat file does not exist. Run this script again with run_model = true to generate Matrices.mat\n');
        return
    end
end

% LQR calculations
Q = diag([1 800 1 100]);
R = 1;

Q_kalman = eye(2) * 75e-6;
R_kalman = eye(4) * 1.21e-2;

[K, ~] = lqr(A_matrix, B_matrix, Q, R);
sys = ss(A_matrix, B_matrix, C_matrix, D_matrix);
[~, L_kalman, P] = kalman(sys, Q_kalman, R_kalman);

% Closed loop eigen values
Acl = A_matrix - (B_matrix * K);
eig_cl = eig(Acl);

% display and save outputs
disp(eig_cl)
disp(A_matrix)
disp(B_matrix)
disp(K)
save('Matrices.mat', 'A_matrix', 'B_matrix', 'C_matrix', 'D_matrix', 'K', 'L_kalman');

function retmat = rotx(alpha)
    retmat = [1 0 0; 0 cosd(alpha) -sind(alpha); 0 sind(alpha) cosd(alpha)];
end

function retmat = roty(beta)
    retmat = [cosd(beta) 0 sind(beta); 0 1 0; -sind(beta) 0 cosd(beta)];
end