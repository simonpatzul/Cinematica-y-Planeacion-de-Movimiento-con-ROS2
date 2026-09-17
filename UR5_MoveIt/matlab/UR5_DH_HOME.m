%% UR5 en HOME: modelo DH y modelo 3D
clear; clc; close all;

% Angulos HOME [q1 q2 q3 q4 q5 q6] en radianes
q = [0 -pi/2 pi/2 -pi/2 -pi/2 0];

% Medidas exactas del UR5 clasico en metros
d1 = 0.089159;
a2 = -0.42500;
a3 = -0.39225;
d4 = 0.10915;
d5 = 0.09465;
d6 = 0.08230;

% Tabla DH modificada: [a(i-1), alfa(i-1), d(i), theta(i)]
DH = [0   0      d1 q(1);
      0   pi/2   0  q(2);
      a2  0      0  q(3);
      a3  0      d4 q(4);
      0   pi/2   d5 q(5);
      0  -pi/2   d6 q(6)];

% Cinematica directa y posiciones de las articulaciones
T = eye(4);
P = zeros(3,7);

for i = 1:6
    T = T * matrizDH(DH(i,1),DH(i,2),DH(i,3),DH(i,4));
    P(:,i+1) = T(1:3,4);
end

% Cambio del frame DH "base" al frame de ROS "base_link"
Rz = [-1 0 0 0;
       0 -1 0 0;
       0  0 1 0;
       0  0 0 1];

T_base_link_tool0 = Rz*T;
P = Rz(1:3,1:3)*P;

disp('Configuracion HOME [rad]:');
disp(q);
disp('Transformacion base_link -> tool0:');
disp(T_base_link_tool0);
fprintf('Posicion tool0: x = %.6f, y = %.6f, z = %.6f m\n', ...
    T_base_link_tool0(1,4),T_base_link_tool0(2,4),T_base_link_tool0(3,4));

%% Comparacion visual
figure('Name','UR5: DH contra modelo 3D','Color','w');
tiledlayout(1,2,'Padding','compact','TileSpacing','compact');

% Izquierda: esqueleto calculado con DH
nexttile;
plot3(P(1,:),P(2,:),P(3,:),'-o','LineWidth',3, ...
    'MarkerSize',7,'MarkerFaceColor',[0 0.45 0.74]);
grid on; axis equal; view(135,25);
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
title('Esqueleto calculado con DH');

% Derecha: modelo tridimensional de MATLAB
nexttile;
try
    robot = loadrobot('universalUR5','DataFormat','row');
    show(robot,q,'Visuals','on','Collisions','off','Frames','off');
    axis equal; view(135,25);
    title('Modelo 3D del UR5');
catch
    axis off;
    text(0.5,0.5,{'No se pudo cargar el modelo 3D.', ...
        'Instala Robotics System Toolbox y', ...
        'Robot Library Data.'}, ...
        'HorizontalAlignment','center','FontSize',12);
end

%% Matriz DH modificada
function A = matrizDH(a,alfa,d,theta)
    A = [cos(theta),            -sin(theta),           0,             a;
         cos(alfa)*sin(theta),  cos(alfa)*cos(theta), -sin(alfa), -d*sin(alfa);
         sin(alfa)*sin(theta),  sin(alfa)*cos(theta),  cos(alfa),  d*cos(alfa);
         0,                     0,                      0,             1];
end
