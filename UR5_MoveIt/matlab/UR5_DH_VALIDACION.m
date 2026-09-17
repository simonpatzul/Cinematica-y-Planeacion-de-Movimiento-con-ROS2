%% Validacion DH del UR5 clasico sin Robotics System Toolbox
% Compara HOME y permite validar las q obtenidas por MoveIt para PICK/PLACE.
% Si se vuelve a ejecutar la IK, copie aqui los seis valores nuevos de q.
clear; clc;

% Valores HOME del SRDF.
q_home = [0 -pi/2 pi/2 -pi/2 -pi/2 0];

% Valores REALES conservados de la ejecucion ROS 2 del 15-sep-2026.
% Si kinematics_report entrega otra rama de IK, reemplace estas dos filas.
q_pick  = [-0.585472148 -0.852696179 1.408340502 -2.126440836 -1.570796144 -0.585472148];
q_place = [ 0.279342916 -0.852697095 1.408342400 -2.126441829 -1.570796536  0.279342916];

nombres = {'HOME','PICK','PLACE'};
Q = [q_home; q_pick; q_place];

for k = 1:size(Q,1)
    T = fkUR5(Q(k,:));
    fprintf('\n%s\n', nombres{k});
    fprintf('q [rad] = '); fprintf(' %.9f', Q(k,:)); fprintf('\n');
    disp('T base_link -> tool0 =');
    disp(T);
    fprintf('XYZ [m] = %.9f %.9f %.9f\n', T(1,4), T(2,4), T(3,4));
end

%% Esqueleto HOME para explicacion visual (sin toolbox)
[~, P] = fkUR5(q_home);
figure('Name','UR5 HOME - modelo DH','Color','w');
plot3(P(1,:),P(2,:),P(3,:),'-o','LineWidth',2,'MarkerSize',6);
grid on; axis equal; view(135,25);
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
title('UR5 clasico en HOME - esqueleto calculado con DH');

%% Funciones locales
function [T, P] = fkUR5(q)
    d1 = 0.089159; a2 = -0.42500; a3 = -0.39225;
    d4 = 0.10915; d5 = 0.09465; d6 = 0.08230;

    DH = [0   0      d1 q(1);
          0   pi/2   0  q(2);
          a2  0      0  q(3);
          a3  0      d4 q(4);
          0   pi/2   d5 q(5);
          0  -pi/2   d6 q(6)];

    Tdh = eye(4);
    Pdh = zeros(3,7);
    for i = 1:6
        Tdh = Tdh * matrizDH(DH(i,1),DH(i,2),DH(i,3),DH(i,4));
        Pdh(:,i+1) = Tdh(1:3,4);
    end

    % Cambio de la base DH al frame ROS base_link.
    Rz = [-1 0 0 0; 0 -1 0 0; 0 0 1 0; 0 0 0 1];
    T = Rz*Tdh;
    P = Rz(1:3,1:3)*Pdh;
end

function A = matrizDH(a,alfa,d,theta)
    A = [cos(theta),            -sin(theta),           0,             a;
         cos(alfa)*sin(theta),  cos(alfa)*cos(theta), -sin(alfa), -d*sin(alfa);
         sin(alfa)*sin(theta),  sin(alfa)*cos(theta),  cos(alfa),  d*cos(alfa);
         0,                     0,                      0,             1];
end
