Ts = 1e-4;
F = eye(6) + Ts * A_matrix;
H = C_matrix;
Q = (B_matrix * B_matrix') * Ts^2 * 7.5e-5;
R = eye(6) * 1.21e-6;
P = zeros(6,6);
n = 20000;
gain_log = zeros(18,n);

for i = 1:n
    P = (F * P * F') + Q;
    S = (H * P * H') + R;
    K_Kalman = (P * H') / S;
    for j = 1:18
        if rem(j,3) == 0
            temp1 = fix(j/3);
            temp2 = 3;
        else
            temp1 = fix(j/3) + 1;
            temp2 = rem(j, 3);
        end
        gain_log(j, i) = K_Kalman(temp1, temp2);
    end
    P = (eye(6) - (K_Kalman * H)) * P;
end
figure
hold on
for i = 1:18
    plot(gain_log(i,:))
end
hold off
disp(K_Kalman)