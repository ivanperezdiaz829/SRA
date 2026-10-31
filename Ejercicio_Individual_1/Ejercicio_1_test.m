%% test_sol_ejercicio_1.m
% Script de prueba de la función sol_ejercicio_1.
%   1) Genera una respuesta temporal subamortiguada (sistema de 2.º orden).
%   2) Llama a sol_ejercicio_1 para obtener los cuatro resultados.
%   3) Los compara con los valores teóricos del sistema.
%   4) Dibuja la señal marcando pico, banda del 5 %, tiempo de
%      establecimiento y error permanente, y guarda la figura.
%
% Requisito: sol_ejercicio_1.m debe estar en la misma carpeta.

clear; clc; close all;

%% 1. Parámetros del sistema de prueba
r    = 1;        % Referencia del escalón
K    = 0.9;      % Ganancia estática (valor final = K*r, distinto de r)
wn   = 20;       % Frecuencia natural no amortiguada [rad/s]
zeta = 0.3;      % Coeficiente de amortiguamiento (0 < zeta < 1: subamortiguado)

Tm    = 1e-3;    % Paso de muestreo [s]
t_fin = 2;       % Duración de la simulación [s]

%% 2. Generación de la señal
% Respuesta analítica al escalón de un sistema de 2.º orden subamortiguado:
%   y(t) = K*r*(1 - exp(-zeta*wn*t)/sqrt(1-zeta^2) * sin(wd*t + phi))
t   = (0:Tm:t_fin)';               % Vector columna de tiempos
wd  = wn*sqrt(1 - zeta^2);         % Frecuencia amortiguada [rad/s]
phi = acos(zeta);                  % Desfase
y   = K*r*(1 - exp(-zeta*wn*t)/sqrt(1 - zeta^2) .* sin(wd*t + phi));

% --- ALTERNATIVA: usar la señal real de Simulink (bloque To Workspace) ---
% Si la variable se llama y2 y el formato es "Structure With Time":
%   t = y2.time;   y = y2.signals.values;
% Si el formato es "Timeseries":
%   t = y2.Time;   y = y2.Data;
% y se ejecuta el resto del script igual (ajustando r a la referencia usada).

%% 3. Llamada a la función
[frec_amortiguada, temp_establecimiento, temp_pico, error_permanente] = ...
    sol_ejercicio_1(t, y, r);

%% 4. Valores teóricos para comparar
fd_teo = wd/(2*pi);                                         % Frecuencia amortiguada [Hz]
tp_teo = pi/wd;                                             % Tiempo de pico [s]
ts_teo = -log(0.05*sqrt(1 - zeta^2))/(zeta*wn);             % Establecimiento 5 % (envolvente) [s]
ess_teo = abs(r - K*r);                                     % Error permanente

%% 5. Resultados por pantalla
fprintf('\n=== RESULTADOS ===\n');
fprintf('%-30s %12s %12s\n', 'Magnitud', 'Calculado', 'Teorico');
fprintf('%-30s %12.4f %12.4f\n', 'Frecuencia amortiguada [Hz]',  frec_amortiguada,     fd_teo);
fprintf('%-30s %12.4f %12.4f\n', 'Tiempo de establecimiento [s]', temp_establecimiento, ts_teo);
fprintf('%-30s %12.4f %12.4f\n', 'Tiempo de pico [s]',            temp_pico,            tp_teo);
fprintf('%-30s %12.4f %12.4f\n', 'Error en regimen permanente',   error_permanente,     ess_teo);

%% 6. Gráfica
y_final = y(end);
[~, ip] = min(abs(t - temp_pico));      % Índice de la muestra del pico
y_pico  = y(ip);

fig = figure('Color', 'w', 'Position', [100 100 900 520]);
hold on; grid on;

plot(t, y, 'b', 'LineWidth', 1.8);                                  % Señal
plot([t(1) t(end)], [r r],                   'k-',  'LineWidth', 1.2);   % Referencia
plot([t(1) t(end)], [y_final y_final],       'k--', 'LineWidth', 1.0);   % Valor final
plot([t(1) t(end)], 1.05*y_final*[1 1],      'b:',  'LineWidth', 1.0);   % Banda +5 %
plot([t(1) t(end)], 0.95*y_final*[1 1],      'b:',  'LineWidth', 1.0);   % Banda -5 %

plot(temp_pico, y_pico, 'ro', 'MarkerFaceColor', 'r');              % Pico
plot([temp_pico temp_pico], [0 y_pico], 'r--', 'HandleVisibility', 'off');   % Línea vertical del pico
plot([temp_establecimiento temp_establecimiento], [0 1.05*y_final], ...
     'g--', 'LineWidth', 1.4);                                      % Tiempo de establecimiento
plot([t(end) t(end)], [y_final r], 'm-', 'LineWidth', 3);           % Error permanente

xlabel('Tiempo [s]');
ylabel('y_2(t)');
title('Respuesta temporal subamortiguada');
legend({'Senal y_2', 'Referencia r', 'Valor final', 'Banda +5 %', 'Banda -5 %', ...
        'Tiempo de pico', 'Tiempo de establecimiento', 'Error permanente'}, ...
        'Location', 'southeast');
xlim([0 t_fin]);
hold off;

%% 7. Exportar la figura (PDF vectorial + PNG)
try
    exportgraphics(fig, 'respuesta_temporal.pdf', 'ContentType', 'vector');
    exportgraphics(fig, 'respuesta_temporal.png', 'Resolution', 300);
catch
    % Alternativa para versiones antiguas de MATLAB
    saveas(fig, 'respuesta_temporal.png');
    print(fig, 'respuesta_temporal.pdf', '-dpdf', '-bestfit');
end