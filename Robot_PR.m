clear; close all; clc;
%% Initialisation
try
    startup_rvc;
catch
warning('Robotics Toolbox non trouvée');
end
%% Paramètres robot
L1 = 0.2;
L2 = 0.25;
q1_min = 0.17;
q1_max = 0.37;
%% Modèle DH correct
L1_link = Link('prismatic', 'theta', 0, 'a', L1, 'alpha',0, 'qlim', [q1_min q1_max]);
L2_link = Link('revolute', 'd', 0, 'a', L2, 'alpha', 0,'qlim', [-pi/2 pi/2]);
robot = SerialLink([L1_link L2_link], 'name', 'Robot PR');
robot.plotopt = {'workspace', [-0.2 0.8 -0.5 0.5 -0.1 0.5]};
%% Trajectoire (cycle réel PFE)
waypoints = [
    0.0   0.370   0;      %HOME
    1.5   0.200   0;      %Descente
    2.5   0.200   0;      %Prise LCD
    3.5   0.370   0;      %Remontée
    4.5   0.370   deg2rad(45);        %Rotation
    5.5   0.180   deg2rad(45);        %Descente
    5.7   0.178   deg2rad(45);        %Collage
    6.2   0.178   deg2rad(45);        %Maintien
    6.7   0.370   deg2rad(45);        %Remontée
    7.0   0.370   0                   %Retour HOME
];
t_way = waypoints(:,1);
q1_way = waypoints(:,2);
theta_way = waypoints(:,3);
%% Temps
dt = 0.05;
t = 0:dt:t_way(end);
%% Interpolation
q1 = interp1(t_way, q1_way, t, 'pchip');
theta = interp1(t_way, theta_way, t, 'pchip');
%% Sécurité
q1 = max(q1_min, min(q1_max, q1));
theta = max(-pi/2, min(pi/2, theta));
%% Lissage (important pour dérivées)
q1_s = smoothdata(q1,'gaussian',5);
theta_s = smoothdata(theta,'gaussian',5);
%% Dérivées
q1_dot = gradient(q1_s, dt);
q1_ddot = gradient(q1_dot, dt);
theta_dot = gradient(theta_s, dt);
theta_ddot = gradient(theta_dot, dt);
%% Paramètres dynamiques
m_total = 5.936;
I_eq = 0.06864;
fv1 = 10;
fv2 = 0.5;
g_term = 4.68;
%% Efforts moteurs
Fz = m_total .* q1_ddot + fv1 .* q1_dot;
tau = I_eq .* theta_ddot + g_term .* cos(theta) + fv2 .* theta_dot;
%% Figure efforts
figure;
subplot(2,1,1)
plot(t, Fz,'LineWidth',2);
title('Effort axe Z');
ylabel('Fz (N)');
grid on;
subplot(2,1,2)
plot(t, tau,'LineWidth',2);
title('Couple rotation');
ylabel('? (N.m)');
xlabel('Temps (s)');
grid on;
%% Simulation suivi réaliste
tau_sys = 0.15;
q1_real = q1(1)*ones(size(q1));
theta_real = theta(1)*ones(size(theta));
for i=2:length(t)
q1_real(i)=q1_real(i-1)+dt*((q1(i)-q1_real(i-1))/tau_sys);theta_real(i)=theta_real(i-1)+dt*((theta(i)-theta_real(i-1))/tau_sys);
end
%% Positions
pos_des = zeros(length(t),3);
pos_real = zeros(length(t),3);
for i=1:length(t)
pos_des(i,:) = robot.fkine([q1(i) theta(i)]).t';
pos_real(i,:) = robot.fkine([q1_real(i)
theta_real(i)]).t';
end
%% Erreur
error = vecnorm(pos_des-pos_real,2,2);
error = smoothdata(error,'movmean',5);
figure;
plot(t,error*1000,'LineWidth',2);
title('Erreur de suivi');
ylabel('Erreur (mm)');
xlabel('Temps (s)');
grid on;
fprintf('Erreur max = %.4f mm\n',max(error)*1000);
fprintf('Erreur RMS = %.4f mm\n',rms(error)*1000);
%% Animation fluide avec enregistrement vidéo
fprintf('\nDémarrage de l''animation et enregistrement vidéo...\n');
% Créer la figure d'animation
fig_anim = figure('Name', 'Robot PR - Animation');
set(fig_anim, 'Position', [100, 100, 800, 600]);
% Préparer la vidéo
video_filename = 'animation_robot_PR.avi';
v = VideoWriter(video_filename, 'Motion JPEG AVI');
v.FrameRate = 1/dt; % 20 images par seconde
open(v);
% Initialiser l'affichage
robot.plot([q1(1) theta(1)]);
for i = 1:length(t)
robot.animate([q1(i) theta(i)]);
title(sprintf('Temps = %.2f s', t(i)));
drawnow;
% Capturer la frame
frame = getframe(fig_anim);
writeVideo(v, frame);
% Pause pour l'affichage en temps réel (optionnel)
pause(dt);
end
% Fermer la vidéo
close(v);
fprintf('Vidéo sauvegardée sous : %s\n', video_filename);
%% Trajectoire 3D
pos = zeros(length(t),3);
for i=1:length(t)
pos(i,:) = robot.fkine([q1(i) theta(i)]).t';
end
figure;
plot3(pos(:,1),pos(:,2),pos(:,3),'LineWidth',2);
grid on; axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Trajectoire effecteur');
%% Graphes articulaires
figure;
subplot(2,1,1)
plot(t,q1*1000,'LineWidth',2);
ylabel('q1 (mm)');
title('Translation');
grid on;
subplot(2,1,2)
plot(t,rad2deg(theta),'LineWidth',2);
ylabel('? (deg)');
xlabel('Temps (s)');
title('Rotation');
grid on;
disp('Simulation terminée avec succès.');
%%video_filename = fullfile('C:\Users\VotreNom\Desktop','animation_robot_PR.avi');
