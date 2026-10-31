function [frec_amortiguada, temp_establecimiento, temp_pico, error_permanente] = sol_ejercicio_1(t, y, r)
    % t, y: vectores columna (señal y2 e instantes de tiempo)
    % r: referencia del escalón (opcional, por defecto 1)
    if nargin < 3, r = 1; end
        t = t(:); y = y(:);

    y_final = y(end);

    % Máximos locales
    idx_picos = find(diff(sign(diff(y))) < 0) + 1;

    % Tiempo de pico: primera sobreoscilación
    if ~isempty(idx_picos)
        temp_pico = t(idx_picos(1));
    else
        [~, idx] = max(y); temp_pico = t(idx);
    end

    % Frecuencia amortiguada [Hz] = 1 / pseudo-periodo
    if numel(idx_picos) >= 2
        T_d = t(idx_picos(2)) - t(idx_picos(1));
        frec_amortiguada = 1 / T_d;
    else
        frec_amortiguada = 0;
    end

    % Tiempo de establecimiento (banda ±5 % del valor final)
    banda = 0.05 * abs(y_final);
    fuera_banda = find(abs(y - y_final) > banda);
    if isempty(fuera_banda)
        temp_establecimiento = t(1);
    else
        temp_establecimiento = t(min(fuera_banda(end) + 1, numel(t)));
    end

    % Error en régimen permanente
    error_permanente = abs(r - y_final);
end