clear; clc;

%% Timp de simulare
dt = 0.002;
t_final = 10;
t = 0:dt:t_final;
N = length(t);

%% Parametrii sistemului
K = 14;
b = 0.4;
c = 1;
delay = 0.05;
delay_samples = round(delay / dt);

%% Referința
t = 0:dt:t_final;
ref = zeros(size(t));
ref(t < 5) = 1;
ref(t >= 5 & t < 10) = 1 + 0.5 * sin(2 * pi * 0.5 * (t(t >= 5 & t < 10) - 5));
ref(t >= 10 & t < 15) = 1 + 0.2 * (t(t >= 10 & t < 15) - 10);
ref(t >= 15) = 0.5;


%% Sarcină perturbatoare variabilă
tau_disturbance = 5*sin(2*pi*0.5*t);


%% Inițializări
theta_P = zeros(1, N); theta_dot_P = zeros(1, N); u_P = zeros(1, N);
theta_PD = zeros(1, N); theta_dot_PD = zeros(1, N); u_PD = zeros(1, N);
theta_PI = zeros(1, N); theta_dot_PI = zeros(1, N); u_PI = zeros(1, N);
integral_error = 0;

%% Parametrii controlere
Kp_P = 0.36;

Kp_PD = 3.88;
Kd_PD = 0.07;

Kp_PI = 0.7;
Ki_PI = 7;
Kff = -0.03;  % Feedforward slab
alpha = -3.3;



%% Simulare
for i = 2:N
    %% --- P
    error_P = ref(i-1) - theta_P(i-1);
    u_P(i) = Kp_P * error_P;
    u_delayed = u_P(max(i - delay_samples, 1));
    theta_dot = Kp_P * (u_delayed - b*theta_P(i-1) - c*theta_P(i-1));
    theta_dot = max(min(theta_dot, 10), -10);
    theta_dot_P(i) = theta_dot;
    theta_P(i) = theta_P(i-1) + theta_dot*dt;
    theta_P(i) = max(min(theta_P(i), 2), -2);

    %% --- PD
    error_PD = ref(i-1) - theta_PD(i-1);
    if i >= 3
    error_dot = (error_PD - (ref(i-2) - theta_PD(i-2))) / dt;
else
    error_dot = 0;
end

    u_PD(i) = Kp_PD * error_PD + Kd_PD * error_dot;
    u_delayed = u_PD(max(i - delay_samples, 1));
    theta_dot = K * (u_delayed - b*theta_PD(i-1) - c*theta_PD(i-1));
    theta_dot = max(min(theta_dot, 10), -10);
    theta_dot_PD(i) = theta_dot;
    theta_PD(i) = theta_PD(i-1) + theta_dot*dt;
    theta_PD(i) = max(min(theta_PD(i), 2), -2);

    %% --- PI + FFW
    error_PI = ref(i-1) - theta_PI(i-1);
    integral_error = integral_error + error_PI * dt;
    integral_error = max(min(integral_error, 2), -2);  

    u_delayed = u_PI(max(i - delay_samples, 1));
    theta_dot = K * (u_delayed - b*theta_PI(i-1) - c*theta_PI(i-1));
    theta_dot = max(min(theta_dot, 10), -10);
    theta_dot_PI(i) = theta_dot;
    theta_PI(i) = theta_PI(i-1) + theta_dot*dt;
    theta_PI(i) = max(min(theta_PI(i), 2), -2);
    adaptive_Kff = min(Kff * (1 + alpha * abs(error_PI)), 2*Kff);
u_total = Kp_PI * error_PI + Ki_PI * integral_error + adaptive_Kff * tau_disturbance(i);
u_PI(i) = u_total;
  

end

%% Plot comparativ
figure;
plot(t, theta_P, 'r', 'LineWidth', 1.2); hold on;
plot(t, theta_PD, 'b', 'LineWidth', 1.2);
plot(t, theta_PI, 'g', 'LineWidth', 1.2);
plot(t, ref, 'k--', 'LineWidth', 1.2);  
xlabel('Timp [s]'); ylabel('Unghi [rad]');
title('Scenariul 3: Răspuns la sarcină perturbatoare variabilă');
legend('P', 'PD', 'PI+FFW', 'ref', 'Location', 'best');
ylim([-2 2]);

%% Tabel comparativ (eroare medie absolută)
err_P = mean(abs(ref - theta_P));
err_PD = mean(abs(ref - theta_PD));
err_PI = mean(abs(ref - theta_PI));
fprintf('\nEroare medie absolută [rad]:\n');
fprintf(' P       : %.4f\n', err_P);
fprintf(' PD      : %.4f\n', err_PD);
fprintf(' PI+FFW  : %.4f\n', err_PI);
fprintf('Kff optimizat = %.4f\n', Kff);





