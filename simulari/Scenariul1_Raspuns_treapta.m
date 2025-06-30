clear all;
close all;
clc;

% Parametri simulare
t_final = 5; % Timp final simulare [s]
dt = 0.001;  % Pas de timp [s]
t = 0:dt:t_final; % Vector timp
N = length(t);

% Referință - treaptă de la 0° la 10°
ref = 10*ones(1,N); % Referință constantă de 10 grade

% Parametri funcție de transfer
% H(s) = 14/(0.03s^2 + 0.4s + 1) * e^(-0.05s)
K = 14;          % Câștigul sistemului
a = 0.03;        % Coeficient s^2
b = 0.4;         % Coeficient s
c = 1;           % Termen liber
delay = 0.05;    % Întârziere [s]
delay_samples = round(delay/dt); % Întârziere în eșantioane
sys_delay = pade(tf(14, [0.03 0.4 1]), 2);  


% Parametri regulatoare
% Regulator P
Kp_P = 0.36;

% Regulator PD
Kp_PD = 3.87;
Kd_PD = 0.07;

% Regulator PI+FFW
Kp_PI = 0.11;
Ki_PI = 0.43;
Kff =0.07;     

% Inițializare vectori pentru stocarea rezultatelor
% Pentru regulator P
theta_P = zeros(1,N);
theta_dot_P = zeros(1,N);
theta_ddot_P = zeros(1,N);
error_P = zeros(1,N);
u_P = zeros(1,N);
u_delayed_P = zeros(1,N);

% Pentru regulator PD
theta_PD = zeros(1,N);
theta_dot_PD = zeros(1,N);
theta_ddot_PD = zeros(1,N);
error_PD = zeros(1,N);
u_PD = zeros(1,N);
u_delayed_PD = zeros(1,N);

% Pentru regulator PI+FFW
theta_PI = zeros(1,N);
theta_dot_PI = zeros(1,N);
theta_ddot_PI = zeros(1,N);
error_PI = zeros(1,N);
u_PI = zeros(1,N);
u_delayed_PI = zeros(1,N);
integral_error = 0;


u_P(1:delay_samples) = 0;
u_delayed_P(1:delay_samples) = 0;

u_PD(1:delay_samples) = 0;
u_delayed_PD(1:delay_samples) = 0;

u_PI(1:delay_samples) = 0;
u_delayed_PI(1:delay_samples) = 0;


% Simulare dinamică
for i = (1+delay_samples):N
    % Regulator P
    error_P(i) = ref(i-1) - theta_P(i-1);
    u_P(i) = Kp_P * error_P(i);
    
    % Aplicare întârziere
if (i > delay_samples)
    u_delayed_P(i) = u_P(i - delay_samples);
else
    u_delayed_P(i) = 0;
end

 % Modelul sistemului pentru P
 % Implementare discretă pentru G(s) = 14/(0.03s^2 + 0.4s + 1)
    theta_ddot_P(i) = (K*u_delayed_P(i) - b*theta_dot_P(i-1) - c*theta_P(i-1))/a;
    theta_dot_P(i) = (theta_dot_P(i-1) + theta_ddot_P(i) * dt)/(1+b*dt/a);
    theta_P(i) = theta_P(i-1) + theta_dot_P(i) * dt;
    
 % Regulator PD
    error_PD(i) = ref(i-1) - theta_PD(i-1);
    error_dot_PD = (error_PD(i) - error_PD(i-1))/dt;
    u_PD(i) = Kp_PD * error_PD(i) + Kd_PD * error_dot_PD;
    
 % Aplicare întârziere
if (i > delay_samples)
    u_delayed_PD(i) = u_PD(i - delay_samples);
else
    u_delayed_PD(i) = 0;
end

    
 % Modelul sistemului pentru PD
    theta_ddot_PD(i) = (K*u_delayed_PD(i) - b*theta_dot_PD(i-1) - c*theta_PD(i-1))/a;
    theta_dot_PD(i) = (theta_dot_PD(i-1) + theta_ddot_PD(i) * dt)/(1 + b*dt/a);
    theta_PD(i) = theta_PD(i-1) + theta_dot_PD(i-1) * dt;
    
 % Regulator PI+FFW
    error_PI(i) = ref(i-1) - theta_PI(i-1);
   integral_error = integral_error + error_PI(i)*dt;
  integral_error = min(max(integral_error, -0.2), 0.2);
  % Anti-windup: limitare integrator
if abs(integral_error) > 0.3
    integral_error = sign(integral_error) * 0.3;
end


    
 % Feed-Forward component (derivata referinței)
   err_PI = ref(i-1) - theta_PI(i-1);
ref_dot = [0 diff(ref)/dt];
u_PI(i) = Kp_PI * err_PI + Ki_PI * integral_error + Kff * ref_dot(i);

 if i < 3
    ref_dot = 0;
else
    ref_dot = (ref(i) - ref(i-1)) / dt;
end


    
 %Control P
   u_P(i)=Kp_P*error_P(i);
 %Control PD
 % Control PI+FFW
    u_PI(i) = Kp_PI * error_PI(i) + Ki_PI * integral_error + Kff * ref_dot;
    
    % Aplicare întârziere
    
if (i > delay_samples)
    u_delayed_PI(i) = u_PI(i - delay_samples);
else
    u_delayed_PI(i) = 0;
end

 % Modelul sistemului pentru PI+FFW
    theta_ddot_PI(i) = (K*u_delayed_PI(i) - b*theta_dot_PI(i-1) - c*theta_PI(i-1))/a;
    theta_dot_PI(i) = (theta_dot_PI(i-1) + theta_ddot_PI(i) * dt)/(1 + b*dt/a);
    theta_PI(i) = theta_PI(i-1) + theta_dot_PI(i-1) * dt;
end

% Calcularea parametrilor de performanță
% Eroare staționară
steady_state_error_P = ref(end) - theta_P(end);
steady_state_error_PD = ref(end) - theta_PD(end);
steady_state_error_PI = ref(end) - theta_PI(end);

% Timp de răspuns (când răspunsul ajunge la 98% din valoarea finală)
threshold = 0.98 * ref(end);
response_time_PI = t_final;
for i = 1:N
    if theta_PI(i) >= threshold
        response_time_PI = t(i);
        break;
    end
end

% Pentru P
response_time_P = t_final;
for i = 1:N
    if theta_P(i) >= threshold
        response_time_P = t(i);
        break;
    end
end

% Pentru PD
response_time_PD = t_final;
for i = 1:N
    if theta_PD(i) >= threshold
        response_time_PD = t(i);
        break;
    end
end

% Pentru PI+FFW
response_time_PI = t_final;
for i = 1:N
    if theta_PI(i) >= threshold
        response_time_PI = t(i);
        break;
    end
end

% Overshoot
max_theta_P = max(theta_P);
max_theta_PD = max(theta_PD);
max_theta_PI = max(theta_PI);

overshoot_P = max(0, (max_theta_P - ref(end)) / ref(end) * 100);
overshoot_PD = max(0, (max_theta_PD - ref(end)) / ref(end) * 100);
overshoot_PI = max(0, (max_theta_PI - ref(end)) / ref(end) * 100);

% Afișarea rezultatelor
figure;
plot(t, ref, 'k--', 'LineWidth', 1.5);
hold on;
plot(t, theta_P, 'r', 'LineWidth', 1.5);
plot(t, theta_PD, 'g', 'LineWidth', 1.5);
plot(t, theta_PI, 'b', 'LineWidth', 1.5);
grid on;
legend('Referință', 'Regulator P', 'Regulator PD', 'Regulator PI+FFW');
xlabel('Timp [s]');
ylabel('Unghi [grade]');
title('Răspuns la treaptă (0° -> 10°)');

% Afișarea parametrilor de performanță
fprintf('Regulator P:\n');
fprintf('  Eroare staționară: %.2f°\n', steady_state_error_P);
fprintf('  Timp de răspuns: %.2f s\n', response_time_P);
fprintf('  Overshoot: %.2f%%\n\n', overshoot_P);

fprintf('Regulator PD:\n');
fprintf('  Eroare staționară: %.2f°\n', steady_state_error_PD);
fprintf('  Timp de răspuns: %.2f s\n', response_time_PD);
fprintf('  Overshoot: %.2f%%\n\n', overshoot_PD);

fprintf('Regulator PI+FFW:\n');
fprintf('  Eroare staționară: %.2f°\n', steady_state_error_PI);
fprintf('  Timp de răspuns: %.2f s\n', response_time_PI);
fprintf('  Overshoot: %.2f%%\n\n', overshoot_PI);
ylim([0 12]); % în figura de răspuns


% Afișarea tabelului de comparație
figure;
data = {
    'P simplu', steady_state_error_P, response_time_P, overshoot_P;
    'PD', steady_state_error_PD, response_time_PD, overshoot_PD;
    'PI + FFW', steady_state_error_PI, response_time_PI, overshoot_PI
};

uitable('Data', data, ...
        'ColumnName', {'Controler', 'Eroare staționară (°)', 'Timp de răspuns (s)', 'Overshoot (%)'},...
        'RowName', [], ...
        'Position', [20 20 500 100]);
