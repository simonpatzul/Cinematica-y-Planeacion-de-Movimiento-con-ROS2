%% Genera las 8 graficas de evidencia del UR5 a partir de resultados reales.
% Lee los CSV que ya escribieron kinematics_report (C++) y trajectory_profiles.py
% Compatible con MATLAB y con Octave (sin toolboxes, sin readtable).
function generar_evidencia_grafica()
    raiz = fileparts(fileparts(mfilename('fullpath')));
    ros2dir = fullfile(raiz, 'resultados', 'ros2');
    tablasdir = fullfile(raiz, 'resultados', 'tablas');
    outdir = fullfile(raiz, 'resultados', 'matlab');
    if ~exist(outdir, 'dir'); mkdir(outdir); end

    d1 = 0.089159; a2 = -0.42500; a3 = -0.39225;
    d4 = 0.10915; d5 = 0.09465; d6 = 0.08230;
    Rz = [-1 0 0 0; 0 -1 0 0; 0 0 1 0; 0 0 0 1];

    q_home  = [0 -pi/2 pi/2 -pi/2 -pi/2 0];
    [q_pick, ok_pick]   = leer_q_csv(fullfile(ros2dir, 'kinematics_PICK.csv'));
    [q_place, ok_place] = leer_q_csv(fullfile(ros2dir, 'kinematics_PLACE.csv'));

    %% 01: esqueleto UR5 en HOME
    fig01(q_home, d1, a2, a3, d4, d5, d6, Rz, fullfile(outdir, '01_UR5_HOME_DH.png'));

    %% 02: DH vs ROS en HOME (comparacion de la posicion del efector)
    fig02(fullfile(ros2dir, 'kinematics_HOME.csv'), fullfile(outdir, '02_DH_vs_ROS_HOME.png'));

    %% 03 y 04: esqueletos PICK y PLACE
    if ok_pick
        fig01(q_pick, d1, a2, a3, d4, d5, d6, Rz, fullfile(outdir, '03_UR5_PICK_DH.png'));
    else
        fprintf('AVISO: no se encontro kinematics_PICK.csv; se omite 03_UR5_PICK_DH.png\n');
    end
    if ok_place
        fig01(q_place, d1, a2, a3, d4, d5, d6, Rz, fullfile(outdir, '04_UR5_PLACE_DH.png'));
    else
        fprintf('AVISO: no se encontro kinematics_PLACE.csv; se omite 04_UR5_PLACE_DH.png\n');
    end

    %% 05: error DH vs MoveIt por estado (posicion, orientacion, matriz)
    fig05(ros2dir, fullfile(outdir, '05_ERROR_DH_MOVEIT.png'));

    %% 06: comparacion de Jacobianos (norma de Frobenius por estado)
    fig06(ros2dir, fullfile(outdir, '06_JACOBIANO_COMPARACION.png'));

    %% 07 y 08: velocidad TCP en 4B y 4D
    fig_tcp(fullfile(ros2dir, 'kinematics_4B.txt'), '4B', 0.200, fullfile(outdir, '07_VELOCIDAD_TCP_4B.png'));
    fig_tcp(fullfile(ros2dir, 'kinematics_4D.txt'), '4D', 0.100, fullfile(outdir, '08_VELOCIDAD_TCP_4D.png'));

    fprintf('Graficas guardadas en: %s\n', outdir);
end

%% ---------- utilidades ----------

function A = matrizDH(a, alfa, d, theta)
    A = [cos(theta),            -sin(theta),           0,             a;
         cos(alfa)*sin(theta),  cos(alfa)*cos(theta), -sin(alfa), -d*sin(alfa);
         sin(alfa)*sin(theta),  sin(alfa)*cos(theta),  cos(alfa),  d*cos(alfa);
         0,                     0,                      0,             1];
end

function P = esqueletoDH(q, d1, a2, a3, d4, d5, d6, Rz)
    DH = [0   0      d1 q(1);
          0   pi/2   0  q(2);
          a2  0      0  q(3);
          a3  0      d4 q(4);
          0   pi/2   d5 q(5);
          0  -pi/2   d6 q(6)];
    T = eye(4);
    P = zeros(3, 7);
    for i = 1:6
        T = T * matrizDH(DH(i,1), DH(i,2), DH(i,3), DH(i,4));
        P(:, i+1) = T(1:3, 4);
    end
    P = Rz(1:3,1:3) * P;
end

% El CSV real que escribe kinematics_report.cpp es formato largo:
% section,row,column,value (ver src/ur5_pick_place/src/kinematics_report.cpp).
% Esta funcion lo carga completo como celdas {seccion, fila, columna, valor}.
function filas = leer_csv_largo(ruta)
    filas = {};
    if ~exist(ruta, 'file'); return; end
    fid = fopen(ruta, 'r');
    fgetl(fid); % descarta encabezado
    linea = fgetl(fid);
    while ischar(linea)
        partes = strsplit(strtrim(linea), ',');
        if numel(partes) == 4
            filas(end+1, :) = { partes{1}, str2double(partes{2}), ...
                str2double(partes{3}), str2double(partes{4}) }; %#ok<AGROW>
        end
        linea = fgetl(fid);
    end
    fclose(fid);
end

% Reconstruye una matriz (nfil x ncol) a partir de una seccion del CSV largo.
function [M, encontrado] = leer_matriz_seccion(filas, seccion, nfil, ncol)
    M = nan(nfil, ncol);
    encontrado = false;
    if isempty(filas); return; end
    idx = strcmp(filas(:,1), seccion);
    if ~any(idx); return; end
    sub = filas(idx, :);
    for i = 1:size(sub, 1)
        r = sub{i,2}; c = sub{i,3};
        if r >= 1 && r <= nfil && c >= 1 && c <= ncol
            M(r, c) = sub{i,4};
        end
    end
    encontrado = true;
end

function [v, encontrado] = leer_escalar_seccion(filas, seccion)
    v = NaN;
    encontrado = false;
    [M, ok] = leer_matriz_seccion(filas, seccion, 1, 1);
    if ok
        v = M(1,1);
        encontrado = true;
    end
end

function [q, encontrado] = leer_q_csv(ruta)
    filas = leer_csv_largo(ruta);
    [M, encontrado] = leer_matriz_seccion(filas, 'joint_position_rad', 6, 1);
    q = M(:,1)';
end

%% ---------- figuras ----------

function fig01(q, d1, a2, a3, d4, d5, d6, Rz, ruta)
    P = esqueletoDH(q, d1, a2, a3, d4, d5, d6, Rz);
    f = figure('Visible', 'off', 'Color', 'w');
    plot3(P(1,:), P(2,:), P(3,:), '-o', 'LineWidth', 2.5, 'MarkerSize', 6, ...
        'MarkerFaceColor', [0 0.45 0.74]);
    grid on; axis equal; view(135, 25);
    xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
    title('UR5 - esqueleto calculado con DH modificado');
    legend('Eslabones', 'Location', 'best');
    print(f, ruta, '-dpng', '-r140');
    close(f);
end

function fig02(rutaCsv, ruta)
    filas = leer_csv_largo(rutaCsv);
    f = figure('Visible', 'off', 'Color', 'w');
    [xyz_ros, ok1] = leer_matriz_seccion(filas, 'fk_moveit', 4, 4);
    [xyz_dh, ok2] = leer_matriz_seccion(filas, 'fk_dh', 4, 4);
    if ~ok1 || ~ok2
        text(0.5, 0.5, 'Sin datos de kinematics_HOME.csv', 'HorizontalAlignment', 'center');
        axis off;
    else
        datos = [xyz_ros(1:3,4), xyz_dh(1:3,4)];
        bar(datos);
        set(gca, 'XTickLabel', {'X','Y','Z'});
        legend('T ROS/MoveIt', 'T DH', 'Location', 'best');
        ylabel('Posición [m]');
        title('HOME: posición del efector, ROS/MoveIt vs DH');
        grid on;
    end
    print(f, ruta, '-dpng', '-r140');
    close(f);
end

function fig05(ros2dir, ruta)
    estados = {'HOME','PICK','PLACE','4B','4D'};
    err_pos = nan(1, numel(estados));
    err_ori = nan(1, numel(estados));
    err_mat = nan(1, numel(estados));
    for i = 1:numel(estados)
        filas = leer_csv_largo(fullfile(ros2dir, ['kinematics_' estados{i} '.csv']));
        [v, ok] = leer_escalar_seccion(filas, 'position_error_m'); if ok; err_pos(i) = v; end
        [v, ok] = leer_escalar_seccion(filas, 'orientation_error_rad'); if ok; err_ori(i) = v; end
        [v, ok] = leer_escalar_seccion(filas, 'fk_matrix_max_error'); if ok; err_mat(i) = v; end
    end
    f = figure('Visible', 'off', 'Color', 'w');
    bar([err_pos; err_ori; err_mat]');
    set(gca, 'XTickLabel', estados);
    legend('Error posición [m]', 'Error orientación [rad]', 'Error máx. matriz', 'Location', 'best');
    ylabel('Error (DH vs MoveIt)');
    title('Error DH vs MoveIt por estado');
    grid on;
    print(f, ruta, '-dpng', '-r140');
    close(f);
end

function fig06(ros2dir, ruta)
    estados = {'HOME','PICK','PLACE','4B','4D'};
    frob = nan(1, numel(estados));
    for i = 1:numel(estados)
        filas = leer_csv_largo(fullfile(ros2dir, ['kinematics_' estados{i} '.csv']));
        [v, ok] = leer_escalar_seccion(filas, 'jacobian_frobenius_error');
        if ok; frob(i) = v; end
    end
    f = figure('Visible', 'off', 'Color', 'w');
    bar(frob);
    set(gca, 'XTickLabel', estados, 'YScale', 'log');
    ylabel('||J_{DH} - J_{MoveIt}||_F (escala log)');
    title('Comparación de Jacobianos DH vs MoveIt/KDL');
    grid on;
    print(f, ruta, '-dpng', '-r140');
    close(f);
end

function fig_tcp(rutaTxt, etiqueta, limite, ruta)
    f = figure('Visible', 'off', 'Color', 'w');
    if ~exist(rutaTxt, 'file')
        text(0.5, 0.5, sprintf('Sin datos de %s', rutaTxt), 'HorizontalAlignment', 'center');
        axis off;
        print(f, ruta, '-dpng', '-r140');
        close(f);
        return;
    end
    texto = fileread(rutaTxt);
    obtenida = regexp(texto, 'xdot MoveIt.*?\n([^\n]+)', 'tokens', 'once');
    exigida = regexp(texto, 'xdot exigida por perfil.*?\n([^\n]+)', 'tokens', 'once');
    if isempty(obtenida) || isempty(exigida)
        text(0.5, 0.5, 'Formato inesperado en kinematics_*.txt', 'HorizontalAlignment', 'center');
        axis off;
    else
        v_obt = sscanf(obtenida{1}, '%f');
        v_exi = sscanf(exigida{1}, '%f');
        vlin_obt = norm(v_obt(1:3));
        vlin_exi = norm(v_exi(1:3));
        bar([vlin_exi, vlin_obt, limite]);
        set(gca, 'XTickLabel', {'Exigida por perfil', 'Obtenida (J*qdot)', 'Límite taller'});
        ylabel('Velocidad TCP lineal [m/s]');
        title(sprintf('Velocidad TCP en %s', etiqueta));
        grid on;
    end
    print(f, ruta, '-dpng', '-r140');
    close(f);
end
