% =========================================================================
% Elicopter EC145 / BK117 C-2 - Anvelopa de Centraj Longitudinal (Figura 21)
% Autor: Irinei David
% =========================================================================

clear; clc; close all;

%% 1. Geometrie de Referință și Limite Anvelopă (Punctele A, B, C, D)
rotor_axis = 4.50; % [m] Axa rotorului principal

% Coordonatele anvelopei de centraj oficiale (D -> A -> B -> C -> D)
env_cg   = [4.337, 4.377, 4.544, 4.667, 4.337];
env_mass = [2000,  3585,  3585,  1750,  2000];

%% 2. Datele Punctelor de Verificare (Tabelele 12 - 17 & Figura 21)
% Nume, CG [m], Masa [kg], Marker, Culoare Marker
puncte = {
    '0P, Max F',    4.534, 2940, 'o', [0.00, 0.45, 0.74];  % Albastru
    '50%P, Max F',  4.474, 3260, 's', [0.85, 0.33, 0.10];  % Portocaliu
    'Max P, Max F', 4.434, 3585, 'd', [0.93, 0.69, 0.13];  % Galben / Aur
    '0P, 0F',       4.516, 2300, '^', [0.49, 0.18, 0.56];  % Mov
    '50%P, 0F',     4.441, 2620, 'v', [0.47, 0.67, 0.19];  % Verde
    'Max P, 0F',    4.386, 2940, 'p', [0.30, 0.75, 0.93];  % Bleu (Stea)
};

%% 3. Reprezentare Grafică (Figura 21 din Proiect)
figure('Color', 'w', 'Position', [100, 100, 950, 600]);

% A. Suprafața și conturul anvelopei
h_env = fill(env_cg, env_mass, [1.0, 0.92, 0.92], ...
             'EdgeColor', 'r', 'LineWidth', 2.5, ...
             'DisplayName', 'Anvelopa EC145');
hold on;

% B. Axa rotorului
h_rotor = xline(rotor_axis, '--k', 'LineWidth', 1.5, ...
                'DisplayName', 'Axa rotorului (4.50 m)');

% C. Plotarea fiecărui punct de verificare și a etichetelor
h_pts = gobjects(size(puncte, 1), 1);

for i = 1:size(puncte, 1)
    eticheta = puncte{i, 1};
    x_val    = puncte{i, 2};
    y_val    = puncte{i, 3};
    marker   = puncte{i, 4};
    culoare  = puncte{i, 5};
    
    % Plotare marker
    h_pts(i) = plot(x_val, y_val, marker, ...
                    'MarkerSize', 8, ...
                    'MarkerFaceColor', culoare, ...
                    'MarkerEdgeColor', 'k', ...
                    'DisplayName', eticheta);
    
    % Căsuță text informativă (Callout cu fundal alb)
    text_str = sprintf('%s\n(%.3f m, %.0f kg)', eticheta, x_val, y_val);
    
    % Poziționare inteligentă a etichetelor pentru a evita suprapunerea
    offset_x = 0.006;
    offset_y = 0;
    if strcmp(eticheta, 'Max P, Max F')
        offset_x = 0.005; offset_y = -30;
    elseif strcmp(eticheta, '0P, Max F')
        offset_x = 0.006; offset_y = -20;
    elseif strcmp(eticheta, '0P, 0F')
        offset_x = 0.006; offset_y = -20;
    end
    
    text(x_val + offset_x, y_val + offset_y, text_str, ...
         'FontSize', 8, ...
         'BackgroundColor', [1 1 1 0.85], ...
         'EdgeColor', [0.75 0.75 0.75], ...
         'Margin', 2);
end

%% 4. Formatare Axe și Legendă (Conform Figurii 21)
grid on;
box on;

xlabel('POZITIA CG [m]', 'FontWeight', 'normal');
ylabel('MASA TOTALA [kg]', 'FontWeight', 'normal');
title('ANVELOPA DE CENTRAJ LONGITUDINAL - EC145', 'FontWeight', 'bold');

xlim([4.3 4.7]);
ylim([1750 3650]);
set(gca, 'XTick', 4.3:0.05:4.7);
set(gca, 'YTick', 1800:200:3600);

% Legendă cu toate cele 8 elemente
legend([h_env, h_rotor, h_pts'], 'Location', 'northeastoutside');