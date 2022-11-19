F = dsys.A;
H = dsys.C;
Q = dsys.B * 1 * dsys.B';
R = eye(3) .* 1e-6;
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