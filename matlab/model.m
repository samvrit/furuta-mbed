syms theta alpha a d theta1 theta2 theta3

% transformation matrix for each row of DH parameters
A(alpha, a, d, theta) = [cos(theta) -sin(theta)*cos(alpha) sin(theta)*sin(alpha) a*cos(theta);
                         sin(theta) cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta);
                            0          sin(alpha)              cos(alpha)              d;
                            0              0                       0                   1];

m = [0.2619 0.123 0.0962];

% Inertia matrix as obtained from SolidWorks
J1 = [35  0  0;
      0  2.3 5.2;
      0  5.2 32.8] .* 1e-4;
J2 = [17  0  0;
      0 16.8 0.4;
      0  0.4 0.2] .* 1e-4;
J3 = [21  0  0;
      0  20.8 0.6;
      0  0.6  0.1] .* 1e-4;

% Similarity transformation to tranform to the same reference frame
I1 = rotx(-90)' * J1 * rotx(-90);
I2 = rotx(-90)' * roty(-90)' * J2 * roty(-90) * rotx(-90);
I3 = rotx(-90)' * roty(-90)' * J3 * roty(-90) * rotx(-90);

% measurements from SolidWorks
dc(1) = 0; dc(2) = 0.148; dc(3) = 0.009;
ac(1) = 0; ac(2) = 0.185; ac(3) = 0.18;
d(1) = 0; d(2) = 0.3; d(3) = 0.015;
a(1) = 0; a(2) = 0.3; a(3) = 0.45;

% DH parameters for center of mass and end of links
dhcom = [-pi/2, 0,     0,       theta1; 
           0,   0,     dc(2),   0; 
           0,   ac(2), 0.0075,  theta2 - (pi/2);
           0,   ac(3), dc(3),   theta3];
dh = [-pi/2, 0,     0,       theta1; 
        0,   0,     d(2),    0; 
        0,   a(2),  0.0269,  theta2 - (pi/2);
        0,   a(3),  d(3),    theta3];

% transformation matrix for each link-end seperately
A1 = A(dh(1,1), dh(1,2), dh(1,3), dh(1,4)) * A(dh(2,1), dh(2,2), dh(2,3), dh(2,4));
A2 = A(dh(3,1), dh(3,2), dh(3,3), dh(3,4));
A3 = A(dh(4,1), dh(4,2), dh(4,3), dh(4,4));

% transformation chain
T1 = A1;
T2 = A1 * A2;
T3 = A1 * A2 * A3;

% transformation matrix for each link' COM seperately
Ac1 = A(dhcom(1,1), dhcom(1,2), dhcom(1,3), dhcom(1,4)) * A(dhcom(2,1), dhcom(2,2), dhcom(2,3), dhcom(2,4));
Ac2 = A(dhcom(3,1), dhcom(3,2), dhcom(3,3), dhcom(3,4));
Ac3 = A(dhcom(4,1), dhcom(4,2), dhcom(4,3), dhcom(4,4));

% transformation matrix chain for link COMs
Tc1 = Ac1;
Tc2 = A1 * Ac2;
Tc3 = A1 * A2 * Ac3;

% height of link COMs from reference plane
z0 = [0; 0; 1];
z1 = [Tc1(1,3); Tc1(2,3); Tc1(3,3)];
z2 = [Tc2(1,3); Tc2(2,3); Tc2(3,3)];

% coordinates of each link-end
o0 = [0; 0; 0];
o1 = [T1(1,4); T1(2,4); T1(3,4)];
o2 = [T2(1,4); T2(2,4); T2(3,4)];
o3 = [T3(1,4); T3(2,4); T3(3,4)];

% coordinates of each link-end
oc0 = [0; 0; 0];
oc1 = [Tc1(1,4); Tc1(2,4); Tc1(3,4)];
oc2 = [Tc2(1,4); Tc2(2,4); Tc2(3,4)];
oc3 = [Tc3(1,4); Tc3(2,4); Tc3(3,4)];


