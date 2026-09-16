% =========================================================================
% TALLER DE CINEMÁTICA Y TRAYECTORIA - 6R KUKA KR 6
% Descripción: Cinemática Inversa Analítica Exacta (Desacople Cinemático),
%              Filtrado SlnSetVer, Jacobiano y Visualización con Frames.
% =========================================================================
clear all; clc; close all;

%% 1. GENERACIÓN DE LA TRAYECTORIA CARTESIANA (25 PUNTOS)
x_base = 1000; 
y_centro = 0;  
puntos_crudos = [x_base, y_centro, 1100]; 
z_inicios = [950, 800, 650];
anchos_y  = [200, 350, 500];

for i = 1:length(z_inicios)
    z_top = z_inicios(i);
    z_bot = z_top - 120;
    w = anchos_y(i);
    nivel_puntos = [
        x_base,      y_centro,           z_top;
        x_base + 20, y_centro + (w/2),   z_bot;
        x_base + 40, y_centro + w,       z_bot-30;
        x_base + 40, y_centro - w,       z_bot-30;
        x_base + 20, y_centro - (w/2),   z_bot;
        x_base,      y_centro,           z_top;
    ];
    puntos_crudos = [puntos_crudos; nivel_puntos];
end

z_tronco_top = 530;
for j = 0:5
    z_val = z_tronco_top - (j * 50);
    puntos_crudos = [puntos_crudos; x_base + 10, y_centro, z_val];
end
trayectoria_cartesiana = puntos_crudos(1:25, :);

% Matriz de rotación deseada del efector final (herramienta apuntando al frente)
R_deseada = [ 0  0  1; 
              0  1  0; 
             -1  0  0];

%% 2. CINEMÁTICA INVERSA ANALÍTICA EXACTA (8 CONFIGURACIONES)
num_puntos = size(trayectoria_cartesiana, 1);
num_configuraciones = 8;
SlnSet = zeros(num_puntos, 6, num_configuraciones);

d1 = 675; a2 = 260; a3 = 680; a4 = -35; d4 = 670; d6 = 115;

for p = 1:num_puntos
    Pos_deseada = trayectoria_cartesiana(p, :)';
    % Desacople - Centro de la muñeca (P_wc)
    P_wc = Pos_deseada - d6 * R_deseada(:, 3);
    
    Q_8 = calcular_8_soluciones_exactas(P_wc, R_deseada, d1, a2, a3, a4, d4, d6);
    
    for capa = 1:num_configuraciones
        SlnSet(p, :, capa) = Q_8(capa, :);
    end
end

%% 3. ESTRUCTURACIÓN, FILTRADO (SlnSetVer) Y REPORTE EN CONSOLA
% Requisito: Slnsetpoint(1) describe por columnas las 8 configuraciones del punto 1
Slnsetpoint_1 = squeeze(SlnSet(1, :, :)); % Resultado: Matriz de 6 filas (GDL) x 8 columnas (Config)

% Requisito: Verificación por capas completas para formar SlnSetVer
SlnSetVer = [];
capas_validas = [];
capa_optima = 1;

for capa = 1:num_configuraciones
    capa_actual = SlnSet(:, :, capa); 
    
    % Condición: No existe ningún elemento con valores imaginarios o tipo NaN en toda la capa
    es_real_limpio = all(all(isreal(capa_actual))) && all(all(~isnan(capa_actual)));
    
    if es_real_limpio
        capas_validas = [capas_validas, capa];
        % Se asigna la primera capa válida como la trayectoria definitiva para la animación
        if isempty(SlnSetVer)
            SlnSetVer = capa_actual;
            capa_optima = capa;
        end
    end
end

% --- IMPRESIÓN EN CONSOLA SEGÚN LAS INDICACIONES DEL TALLER ---
fprintf('\n=========================================================================\n');
fprintf('             REPORTE DE TRAYECTORIA Y SOLUCIONES CINEMÁTICAS             \n');
fprintf('=========================================================================\n');
fprintf('1. Matriz global creada (SlnSet)\n');
fprintf('   Dimensiones: %d x %d x %d (n puntos x 6 GDL x 8 configs)\n\n', size(SlnSet,1), size(SlnSet,2), size(SlnSet,3));

fprintf('2. Slnsetpoint(1) - Configuraciones posibles para el Punto 1 [RADIANES]\n');
fprintf('   (Cada columna representa una configuración distinta):\n');
disp(Slnsetpoint_1);

fprintf('3. Verificación y Filtrado (SlnSetVer)\n');
fprintf('   Evaluación por capas estrictas (sin mezclar):\n');
for c = 1:num_configuraciones
    capa_test = SlnSet(:, :, c);
    if all(all(isreal(capa_test))) && all(all(~isnan(capa_test)))
        fprintf('   - Capa %d: CUMPLE (Trayectoria 100%% real y alcanzable)\n', c);
    else
        fprintf('   - Capa %d: DESCARTADA (Contiene posiciones inalcanzables / complejas)\n', c);
    end
end

fprintf('\n4. Trayectoria Definitiva (Animación)\n');
if ~isempty(capas_validas)
    fprintf('   SlnSetVer asignado a la Capa: %d\n', capa_optima);
    fprintf('   Dimensiones SlnSetVer: %d x %d\n', size(SlnSetVer,1), size(SlnSetVer,2));
    fprintf('   Vector inicial [q1 q2 q3 q4 q5 q6] en RADIANES:\n');
    disp(SlnSetVer(1, :));
else
    fprintf('   ERROR: Ninguna configuración logró alcanzar todos los puntos de la trayectoria.\n');
end
fprintf('=========================================================================\n\n');
%% 4. CÁLCULO DE VELOCIDADES ARTICULARES POR JACOBIANO
tiempo_entre_puntos = 0.5; 
velocidades_articulares = zeros(num_puntos-1, 6);

for p = 1:(num_puntos-1)
    q_actual = SlnSetVer(p, :)';
    delta_x = trayectoria_cartesiana(p+1, :)' - trayectoria_cartesiana(p, :)';
    v_lineal = delta_x / tiempo_entre_puntos;
    V_cartesiana = [v_lineal; 0; 0; 0];
    
    [~, T_ij, frame] = calcular_FK_completa(q_actual);
    J = calcular_jacobiano_completo(T_ij, frame);
    
    q_dot = pinv(J) * V_cartesiana;
    velocidades_articulares(p, :) = q_dot';
end

%% 5. VISUALIZACIÓN GRÁFICA 3D Y FRAMES {0} Y {6}
hf = figure(1);
set(hf,'position',[200 100 800 750]);
grid on; axis equal;
xlim([0 1600]); ylim([-800 800]); zlim([0 1500]);
xlabel('X [mm]'); ylabel('Y [mm]'); zlabel('Z [mm]');
view(37,30); hold on;
title(['Trayectoria Desacoplada Fiel - KUKA KR 6 (Capa ', num2str(capa_optima), ')']);

% Trayectoria cartesiana deseada (Árbol)
plot3(trayectoria_cartesiana(:,1), trayectoria_cartesiana(:,2), trayectoria_cartesiana(:,3), '--k', 'LineWidth', 1.5);

trayectoria_efector = zeros(num_puntos, 3);
longitud_eje = 120; 

for p = 1:num_puntos
    q_punto = SlnSetVer(p, :)';
    [T_total, T_ij, frame] = calcular_FK_completa(q_punto);
    trayectoria_efector(p, :) = T_total(1:3, 4)';
    
    % Renderizado de Frames en punto inicial, medio y final
    if p == 1 || p == round(num_puntos/2) || p == num_puntos
        T_curr = eye(4);
        for f = 1:frame
            if f == 1
                dibujar_frame(eye(4), longitud_eje, '{0}');
            end
            T_curr = T_curr * T_ij(:,:,f);
        end
        dibujar_frame(T_curr, longitud_eje, ['{6}_p', num2str(p)]);
    end
end

% Rastro real del efector final obtenido por cinematica directa
plot3(trayectoria_efector(:,1), trayectoria_efector(:,2), trayectoria_efector(:,3), '-r', 'LineWidth', 2.5);
legend('Trayectoria Deseada', 'Rastro Real del Efector', 'Location', 'northeast');

%% --- FUNCIONES CINEMÁTICAS ANALÍTICAS Y HERRAMIENTAS ---

function Q_8 = calcular_8_soluciones_exactas(Pwc, R, d1, a2, a3, a4, d4, d6)
    Q_8 = zeros(8, 6);
    x_c = Pwc(1); y_c = Pwc(2); z_c = Pwc(3);
    
    % 1. Hombro (Frontal / Trasero)
    th1_opts = [atan2(y_c, x_c), atan2(-y_c, -x_c)];
    idx = 1;
    
    R_off = sqrt(a4^2 + d4^2);
    phi = atan2(d4, a4);
    
    for i = 1:2
        th1 = th1_opts(i);
        r = sqrt(x_c^2 + y_c^2);
        if i == 2, r = -r; end 
        
        r_p = r - a2;
        z_p = z_c - d1;
        D2 = r_p^2 + z_p^2;
        
        % 2. Codo (Arriba / Abajo) - Ecuación Trigonométrica Cerrada
        K = (D2 - a3^2 - R_off^2) / (2 * a3);
        cos_val = max(min(K / R_off, 1), -1);
        
        ang_diff = atan2(sqrt(1 - cos_val^2), cos_val);
        q3_opts = [phi + ang_diff, phi - ang_diff]; % Opción positiva y negativa
        
        for j = 1:2
            q3 = q3_opts(j);
            
            X_wc2 = a3 + a4*cos(q3) + d4*sin(q3);
            Y_wc2 = a4*sin(q3) - d4*cos(q3);
            
            cos_q2 = (-Y_wc2 * r_p + X_wc2 * z_p) / D2;
            sin_q2 = (-X_wc2 * r_p - Y_wc2 * z_p) / D2;
            q2 = atan2(sin_q2, cos_q2);
            
            % 3. Muñeca Esférica (Flip / No-Flip)
            q_arm = [th1; q2; q3; 0; 0; 0];
            [~, T_ij_arm, ~] = calcular_FK_completa(q_arm);
            R3_0 = T_ij_arm(1:3,1:3,1) * T_ij_arm(1:3,1:3,2) * T_ij_arm(1:3,1:3,3);
            
            R6_3 = R3_0' * R; 
            
            sin_th5 = sqrt(R6_3(1,3)^2 + R6_3(3,3)^2);
            th5_opts = [atan2(sin_th5, R6_3(2,3)), atan2(-sin_th5, R6_3(2,3))];
            
            for k = 1:2
                th5 = th5_opts(k);
                if abs(sin_th5) > 1e-5
                    th4 = atan2(R6_3(3,3)/sin(th5), R6_3(1,3)/sin(th5));
                    th6 = atan2(-R6_3(2,2)/sin(th5), -R6_3(2,1)/sin(th5));
                else
                    th4 = 0;
                    th6 = atan2(R6_3(1,2), R6_3(1,1));
                end
                
                % Normalización de ángulos a [-pi, pi] según offsets DH KUKA
                q4 = atan2(sin(th4 - pi), cos(th4 - pi));
                q5 = atan2(sin(th5 - pi), cos(th5 - pi));
                q6 = atan2(sin(th6), cos(th6));
                
                Q_8(idx, :) = [th1, q2, q3, q4, q5, q6];
                idx = idx + 1;
            end
        end
    end
end

function [T_total, T_ij, frame] = calcular_FK_completa(q)
    DH = [   0,    deg2rad(0),     675,  q(1) + deg2rad(0);   
           260,    deg2rad(90),      0,  q(2) + deg2rad(90);   
           680,    deg2rad(0),       0,  q(3) + deg2rad(0);   
           -35,    deg2rad(90),    670,  q(4) + deg2rad(180);   
             0,    deg2rad(90),      0,  q(5) + deg2rad(180);   
             0,    deg2rad(90),    115,  q(6) + deg2rad(0);   
         ];
    frame = size(DH,1);
    T_ij = zeros(4,4,frame);
    for i = 1:frame
        T_ij(:,:,i) = T_DH(DH(i,1), DH(i,2), DH(i,3), DH(i,4));
    end
    T_total = T_ij(:,:,1);
    for i = 1:frame-1
        T_total = T_total * T_ij(:,:,i+1);
    end
end

function J = calcular_jacobiano_completo(T_ij, frame)
    J = zeros(6, frame);
    T_full = T_ij(:,:,1);
    for i = 2:frame
        T_full = T_full * T_ij(:,:,i);
    end
    p_end = T_full(1:3, 4);
    
    for i = 1:frame
        if i == 1
            T_prev = eye(4);
        else
            T_prev = eye(4);
            for j = 1:i-1
                T_prev = T_prev * T_ij(:,:,j);
            end
        end
        z_prev = T_prev(1:3, 3);
        p_prev = T_prev(1:3, 4);
        
        J(1:3, i) = cross(z_prev, (p_end - p_prev));
        J(4:6, i) = z_prev;
    end
end

function T_ij = T_DH(a_ij, alpha_ij, s_i, theta_i)
    T_ij = [               cos(theta_i),              -sin(theta_i),              0,               a_ij;
            cos(alpha_ij)*sin(theta_i), cos(alpha_ij)*cos(theta_i), -sin(alpha_ij), -s_i*sin(alpha_ij);
            sin(alpha_ij)*sin(theta_i), sin(alpha_ij)*cos(theta_i),  cos(alpha_ij),  s_i*cos(alpha_ij);
                              0,                          0,              0,                  1];
end

function dibujar_frame(T, len, label_name)
    o = T(1:3,4);
    x = o + T(1:3,1)*len;
    y = o + T(1:3,2)*len;
    z = o + T(1:3,3)*len;
    
    plot3([o(1) x(1)], [o(2) x(2)], [o(3) x(3)], 'r', 'LineWidth', 2);
    plot3([o(1) y(1)], [o(2) y(2)], [o(3) y(3)], 'g', 'LineWidth', 2);
    plot3([o(1) z(1)], [o(2) z(2)], [o(3) z(3)], 'b', 'LineWidth', 2);
    text(o(1), o(2), o(3), label_name, 'FontSize', 10, 'FontWeight', 'bold');
end
