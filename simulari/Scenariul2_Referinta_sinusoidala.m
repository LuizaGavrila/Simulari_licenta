% Scenariu complet: Răspunsuri P, PD, PI+FFW la referință sinusoidală
clear; clc; close all;

%% 1. Definire plantă și întârziere
G = tf(14, [0.03 0.4 1]);
[num_G, den_G] = tfdata(G, 'v');
[num_pade, den_pade] = pade(0.05, 1);  % Întârziere 0.05s
Gd = tf(conv(num_G, num_pade), conv(den_G, den_pade));

%% 2. Semnal de referință sinusoidal
f = 0.5;
t = (0:0.01:10)';  % vector coloană
theta_ref = sin(2*pi*f*t);

%% 3. Regulatori
% P
Kp_P = 0.5;
C_P = tf(Kp_P);

% PD cu derivată internă
Kp_PD = 0.1;
Kd_PD = 1.5;
C_PD = tf(Kp_PD) + tf(Kd_PD, [1 0]);

% PI + Feed-Forward
Kp_PI = 0.1;
Ki_PI = 0.4;
Kff = 0.07;
C_PI = tf([Kp_PI Ki_PI], [1 0]);

ref_dot = 2*pi*f*cos(2*pi*f*t);  % Derivata pentru FFW

%% 4. Simulări semnale de control
u_P = lsim(C_P, theta_ref, t);
u_PD = lsim(C_PD, theta_ref, t);  % derivata internă
u_PI = lsim(C_PI, theta_ref, t);
u_total = u_PI + Kff * ref_dot;

%% 5. Trecerea prin plantă
y_P = lsim(Gd, u_P, t);
y_PD = lsim(Gd, u_PD, t);
y_PI = lsim(Gd, u_total, t);

%% 6. Plot comparativ
figure;
plot(t, theta_ref, 'k--', 'LineWidth', 1.5); hold on;
plot(t, y_P, 'b', 'LineWidth', 1.3);
plot(t, y_PD, 'm', 'LineWidth', 1.3);
plot(t, y_PI, 'g', 'LineWidth', 1.3);
legend('Referință', 'Răspuns P', 'Răspuns PD', 'Răspuns PI+FFW');
xlabel('Timp [s]');
ylabel('Unghi [rad]');
title('Comparație răspunsuri – P, PD, PI+FFW la referință sinusoidală');
grid on;

%% 7. Calcul erori și RMS
error_P = theta_ref - y_P;
error_PD = theta_ref - y_PD;
error_PI = theta_ref - y_PI;

rms_P = rms(error_P);
rms_PD = rms(error_PD);
rms_PI = rms(error_PI);

fprintf('\nEroare RMS pentru fiecare regulator:\n');
fprintf('P:       %.4f rad\n', rms_P);
fprintf('PD:      %.4f rad\n', rms_PD);
fprintf('PI+FFW:  %.4f rad\n', rms_PI);

%% 8. Plot erori de urmărire
figure;
plot(t, error_P, 'b'); hold on;
plot(t, error_PD, 'm');
plot(t, error_PI, 'g');
legend('Eroare P', 'Eroare PD', 'Eroare PI+FFW');
xlabel('Timp [s]');
ylabel('Eroare [rad]');
title('Eroarea de urmărire – P, PD, PI+FFW');
grid on;
