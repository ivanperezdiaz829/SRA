% sol_ejercicio_2.m
clear; clc; close all;
rng(1);                         % semilla fija para que los resultados sean reproducibles

% Lectura de datos: columnas x [m], y [m], t [s]
datos = load('Path_SRA2526.txt');
x = datos(:,1);  y = datos(:,2);  t = datos(:,3);
N = length(t);

%% A) Transformación de la trayectoria en velocidades
dt = diff(t);                   % intervalos de tiempo (no son constantes)
dx = diff(x);
dy = diff(y);

% Velocidad lineal: distancia recorrida en cada tramo entre su duración
v = sqrt(dx.^2 + dy.^2) ./ dt;  % [m/s], tamaño N-1

% Orientación de cada tramo (el robot diferencial avanza en la dirección de su eje)
theta = atan2(dy, dx);          % [rad], tamaño N-1

% Velocidad angular: variación de orientación entre tramos consecutivos.
% Se normaliza el incremento a (-pi, pi] para evitar saltos de 2*pi.
dtheta = atan2(sin(diff(theta)), cos(diff(theta)));
w = [0; dtheta ./ dt(2:end)];   % [rad/s]; w(1)=0 porque theta(1) es la orientación inicial

% Instante asociado a cada velocidad (final de cada tramo)
tv = t(2:end);

% Gráfica de velocidades (entregable 1 de la memoria)
figure('Name','Velocidades','Position',[100 100 900 600]);
subplot(2,1,1);
plot(tv, v, 'b'); grid on;
xlabel('t [s]'); ylabel('v [m/s]'); title('Velocidad lineal');
subplot(2,1,2);
plot(tv, w, 'r'); grid on;
xlabel('t [s]'); ylabel('\omega [rad/s]'); title('Velocidad angular');

%% B) Información útil desde la perspectiva de un sistema robótico
dist_total = sum(sqrt(dx.^2 + dy.^2));                  % distancia recorrida [m]
dur_total  = t(end) - t(1);                             % duración [s]
dist_recta = sqrt((x(end)-x(1))^2 + (y(end)-y(1))^2);   % distancia inicio-fin [m]
v_media = mean(v);   v_max = max(v);   v_min = min(v);
w_media_abs = mean(abs(w));   w_max = max(abs(w));
dt_medio = mean(dt); dt_max = max(dt);                  % periodo de muestreo
a_lin = diff(v) ./ dt(2:end);                           % aceleración lineal [m/s^2]
a_lin_max = max(abs(a_lin));
a_lat = abs(v .* w);                                    % aceleración lateral (centrípeta) [m/s^2]
a_lat_max = max(a_lat);
umbral_parado = 0.05;                                   % [m/s] umbral para considerar robot parado
t_parado = sum(dt(v < umbral_parado));                  % tiempo casi parado [s]
pct_parado = 100 * t_parado / dur_total;                % porcentaje de tiempo parado [%]

% Radio de giro mínimo (calculado antes para incluirlo en la tabla)
idx = abs(w) > 0.1;
radio_min = min(abs(v(idx) ./ w(idx)));                 % [m]

% Definir los nombres de las filas en un arreglo de celdas
Metricas = {
    'Distancia recorrida [m]';
    'Duración total [s]';
    'Distancia inicio-fin [m]';
    'Velocidad lineal media [m/s]';
    'Velocidad lineal máxima [m/s]';
    'Velocidad lineal mínima [m/s]';
    'Velocidad angular media abs [rad/s]';
    'Velocidad angular máxima [rad/s]';
    'Periodo de muestreo medio [s]';
    'Periodo de muestreo máximo [s]';
    'Aceleración lineal máxima [m/s²]';
    'Aceleración lateral máxima [m/s²]';
    'Umbral de parado [m/s]';
    'Tiempo parado [s]';
    'Porcentaje de tiempo parado [%]';
    'Radio de giro mínimo [m]'
    };

% Vector columna con todos los resultados numéricos
Valores = [
    dist_total; 
    dur_total; 
    dist_recta; 
    v_media; 
    v_max; 
    v_min; 
    w_media_abs; 
    w_max; 
    dt_medio; 
    dt_max; 
    a_lin_max; 
    a_lat_max;
    umbral_parado; 
    t_parado;
    pct_parado;
    radio_min
    ];

% Crear y mostrar la tabla B
fprintf('\n--- b) Información de la trayectoria ---\n\n');
tabla_robot_V = table(Metricas, Valores);
disp(tabla_robot_V);

% Trayectoria y perfil de velocidad espacial
figure('Name','Trayectoria');
plot(x, y, 'g'); axis equal; grid on;
xlabel('x [m]'); ylabel('y [m]'); title('Trayectoria original');

%% C) Reconstrucción de la trayectoria con las velocidades
%  Pose inicial conocida: (x(1), y(1), theta(1))
[xr1, yr1, thr1] = reconstruir(v, w, dt, x(1), y(1), theta(1));      % c1: sin ruido

% c2: ruido gaussiano en v, sigma = 1 % de la media de v
sigma_v = 0.01 * mean(v);
v_ruido = v + sigma_v * randn(size(v));
[xr2, yr2, thr2] = reconstruir(v_ruido, w, dt, x(1), y(1), theta(1));

% c3: ruido gaussiano en w, sigma = 1 % de la media de |w|
sigma_w = 0.01 * mean(abs(w));
w_ruido = w + sigma_w * randn(size(w));
[xr3, yr3, thr3] = reconstruir(v, w_ruido, dt, x(1), y(1), theta(1));

%  Cálculo de errores (Una realización)
theta_real = [theta(1); theta];   % orientación real en cada muestra
casos = {'c1 (sin ruido)'; 'c2 (ruido en v)'; 'c3 (ruido en w)'};
XR = {xr1, xr2, xr3};  YR = {yr1, yr2, yr3};  TR = {thr1, thr2, thr3};

err_medio = zeros(3,1);
err_max = zeros(3,1);
err_final = zeros(3,1);
rmse = zeros(3,1);
err_ang_fin = zeros(3,1);

for k = 1:3
    ep = sqrt((XR{k}-x).^2 + (YR{k}-y).^2);
    eth = atan2(sin(TR{k}-theta_real), cos(TR{k}-theta_real));
    err_medio(k) = mean(ep);
    err_max(k) = max(ep);
    err_final(k) = ep(end);
    rmse(k) = sqrt(mean(ep.^2));
    err_ang_fin(k) = eth(end);
end

% Crear y mostrar la tabla C1
fprintf('\n--- C) Errores de la reconstrucción (una realización) ---\n\n');
tabla_restr_1 = table(casos, err_medio, err_max, err_final, rmse, err_ang_fin, ...
    'VariableNames', {'Caso', 'Err_Medio_m', 'Err_Max_m', 'Err_Final_m', 'RMSE_m', 'Err_Ang_Fin_rad'});
disp(tabla_restr_1);

% Como c2 y c3 son aleatorios, se repite con varias realizaciones (Monte Carlo)
M = 100;
res = zeros(M, 4, 2);   % [medio, max, final, rmse] x caso (c2, c3)
for m = 1:M
    [a, b] = reconstruir(v + sigma_v*randn(size(v)), w, dt, x(1), y(1), theta(1));
    e = sqrt((a-x).^2 + (b-y).^2);
    res(m,:,1) = [mean(e), max(e), e(end), sqrt(mean(e.^2))];
    [a, b] = reconstruir(v, w + sigma_w*randn(size(w)), dt, x(1), y(1), theta(1));
    e = sqrt((a-x).^2 + (b-y).^2);
    res(m,:,2) = [mean(e), max(e), e(end), sqrt(mean(e.^2))];
end

% Formatear los resultados de Monte Carlo (Media +- Desv. Típica) en una tabla
nombres = {'c2 (ruido en v)'; 'c3 (ruido en w)'};
Medio_str = cell(2,1);
Max_str = cell(2,1);
Final_str = cell(2,1);
RMSE_str = cell(2,1);

for k = 1:2
    mu = mean(res(:,:,k), 1);  sd = std(res(:,:,k), 0, 1);
    Medio_str{k} = sprintf('%.4g ± %.2g', mu(1), sd(1));
    Max_str{k}   = sprintf('%.4g ± %.2g', mu(2), sd(2));
    Final_str{k} = sprintf('%.4g ± %.2g', mu(3), sd(3));
    RMSE_str{k}  = sprintf('%.4g ± %.2g', mu(4), sd(4));
end

% Crear y mostrar la tabla C2 (Monte Carlo)
fprintf('\n--- C) Errores medios sobre %d realizaciones (media ± desv. típica) ---\n\n', M);
tabla_montecarlo = table(nombres, Medio_str, Max_str, Final_str, RMSE_str, ...
    'VariableNames', {'Caso', 'Medio', 'Max', 'Final', 'RMSE'});
disp(tabla_montecarlo);

% Gráfica comparativa de trayectorias
figure('Name','Reconstrucción','Position',[100 100 900 600]);
plot(x, y, 'g', 'LineWidth', 1.5); hold on;
plot(xr2, yr2, 'b'); plot(xr3, yr3, 'r');
axis equal; grid on; legend('Real / c1', 'c2 (ruido en v)', 'c3 (ruido en w)');
xlabel('x [m]'); ylabel('y [m]'); title('Trayectoria real vs reconstruida');

%% Función local: integración de la odometría (modelo de robot diferencial)
function [xr, yr, thr] = reconstruir(v, w, dt, x0, y0, th0)
    th_tramo = th0 + cumsum(w .* dt) - w(1)*dt(1);
    xr  = x0 + [0; cumsum(v .* dt .* cos(th_tramo))];
    yr  = y0 + [0; cumsum(v .* dt .* sin(th_tramo))];
    thr = [th0; th_tramo];
end