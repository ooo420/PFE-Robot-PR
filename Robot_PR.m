%% ========================================================================
% ROBOT PR - SIMULATION MATLAB
% PFE : Pick & Place LCD
% ========================================================================

clear;
close all;
clc;

%% ========================================================================
% 1. INITIALISATION ROBOTICS TOOLBOX
% ========================================================================

try
    startup_rvc;
catch
    warning('Robotics Toolbox non trouvee.');
end

%% ========================================================================
% 2. PARAMETRES ROBOT
% ========================================================================

L1 = 0.20;              % [m]
L2 = 0.25;              % [m]

q1_min = 0.17;          % [m]
q1_max = 0.37;          % [m]

theta_min = -pi/2;      % -90 deg
theta_max =  pi/2;      % +90 deg

%% ========================================================================
% 3. MODELE DH
% ========================================================================

L1_link = Link( ...
    'prismatic', ...
    'theta', 0, ...
    'a', L1, ...
    'alpha', 0, ...
    'qlim', [q1_min q1_max]);

L2_link = Link( ...
    'revolute', ...
    'd', 0, ...
    'a', L2, ...
    'alpha', 0, ...
    'qlim', [theta_min theta_max]);

robot = SerialLink([L1_link L2_link], ...
                   'name', 'Robot PR');

%% ========================================================================
% 4. WAYPOINTS DU CYCLE REEL
% ========================================================================

% Colonnes :
% t [s] | q1 [m] | theta [rad]

waypoints = [

    0.0   0.370   deg2rad(0);      % HOME

    1.5   0.200   deg2rad(0);      % Approche LCD

    2.5   0.178   deg2rad(0);      % PRISE LCD

    3.5   0.370   deg2rad(0);      % LIFT

    4.5   0.370   deg2rad(45);     % ROTATION

    5.5   0.200   deg2rad(45);     % Approche support

    5.7   0.178   deg2rad(45);     % DEPOSE

    6.2   0.178   deg2rad(45);     % MAINTIEN

    6.7   0.370   deg2rad(45);     % LIFT

    7.0   0.370   deg2rad(0)       % RETOUR HOME

];

t_way    = waypoints(:,1);
q1_way   = waypoints(:,2);
theta_way = waypoints(:,3);

%% ========================================================================
% 5. VERIFICATION DES LIMITES
% ========================================================================

if any(q1_way < q1_min) || any(q1_way > q1_max)

    error('Erreur : q1 hors limites.');

end

if any(theta_way < theta_min) || any(theta_way > theta_max)

    error('Erreur : theta hors limites.');

end

fprintf('Verification des limites : OK\n');

%% ========================================================================
% 6. DISCRETISATION TEMPORELLE
% ========================================================================

dt = 0.01;

t = 0:dt:t_way(end);

%% ========================================================================
% 7. INTERPOLATION
% ========================================================================

q1 = interp1( ...
    t_way, ...
    q1_way, ...
    t, ...
    'pchip');

theta = interp1( ...
    t_way, ...
    theta_way, ...
    t, ...
    'pchip');

%% ========================================================================
% 8. LIMITATION DE SECURITE
% ========================================================================

q1 = min(max(q1,q1_min),q1_max);

theta = min(max(theta,theta_min),theta_max);

%% ========================================================================
% 9. VITESSES ET ACCELERATIONS
% ========================================================================

q1_dot = gradient(q1,dt);
q1_ddot = gradient(q1_dot,dt);

theta_dot = gradient(theta,dt);
theta_ddot = gradient(theta_dot,dt);

%% ========================================================================
% 10. PARAMETRES DYNAMIQUES
% ========================================================================

m_total = 5.936;        % [kg]

I_eq = 0.06864;         % [kg.m^2]

fv1 = 10;               % coefficient frottement axe Z

fv2 = 0.5;              % coefficient frottement rotation

g_term = 4.68;          % [N.m]

%% ========================================================================
% 11. EFFORT AXE PRISMATIQUE
% ========================================================================

Fz = ...
    m_total .* q1_ddot ...
    + fv1 .* q1_dot;

%% ========================================================================
% 12. COUPLE AXE ROTATIF
% ========================================================================

tau = ...
    I_eq .* theta_ddot ...
    + g_term .* cos(theta) ...
    + fv2 .* theta_dot;

%% ========================================================================
% 13. VALEURS MAXIMALES
% ========================================================================

Fz_max = max(abs(Fz));

tau_max = max(abs(tau));

fprintf('\n');
fprintf('============================================\n');
fprintf('RESULTATS DYNAMIQUES\n');
fprintf('============================================\n');

fprintf('Effort maximal axe Z      = %.2f N\n',Fz_max);

fprintf('Couple maximal rotation   = %.2f N.m\n',tau_max);

fprintf('Duree cycle               = %.2f s\n',t(end));

%% ========================================================================
% 14. MGD
% ========================================================================

N = length(t);

pos = zeros(N,3);

for i = 1:N

    T = robot.fkine([q1(i) theta(i)]);

    pos(i,:) = T.t';

end

%% ========================================================================
% 15. POSITIONS PRINCIPALES
% ========================================================================

fprintf('\n');
fprintf('============================================\n');
fprintf('POSITIONS CARTESIENNES\n');
fprintf('============================================\n');

% Prise

T_pick = robot.fkine([0.178 0]);

fprintf('\nPRISE LCD\n');

fprintf('X = %.2f mm\n',T_pick.t(1)*1000);
fprintf('Y = %.2f mm\n',T_pick.t(2)*1000);
fprintf('Z = %.2f mm\n',T_pick.t(3)*1000);

% Dépose

T_place = robot.fkine([0.178 deg2rad(45)]);

fprintf('\nDEPOSE SUPPORT\n');

fprintf('X = %.2f mm\n',T_place.t(1)*1000);
fprintf('Y = %.2f mm\n',T_place.t(2)*1000);
fprintf('Z = %.2f mm\n',T_place.t(3)*1000);

%% ========================================================================
% 16. GRAPHE EFFORT AXE Z
% ========================================================================

figure('Name','Effort axe Z');

plot(t,Fz,'LineWidth',2);

grid on;

xlabel('Temps [s]');
ylabel('F_z [N]');

title('Effort dynamique de l''axe prismatique');

%% ========================================================================
% 17. GRAPHE COUPLE
% ========================================================================

figure('Name','Couple rotation');

plot(t,tau,'LineWidth',2);

grid on;

xlabel('Temps [s]');
ylabel('\tau [N.m]');

title('Couple dynamique de l''axe rotatif');

%% ========================================================================
% TRAJECTOIRES ARTICULAIRES
% ========================================================================

figure('Name','Trajectoires articulaires');

subplot(2,1,1);

plot(t,q1*1000,'LineWidth',2);

grid on;

xlabel('Temps [s]');
ylabel('q_1 [mm]');

title('Position axe prismatique');

hold on;

% Limite basse
plot([t(1) t(end)], ...
     [q1_min*1000 q1_min*1000], ...
     '--');

% Limite haute
plot([t(1) t(end)], ...
     [q1_max*1000 q1_max*1000], ...
     '--');

hold off;


subplot(2,1,2);

plot(t,rad2deg(theta),'LineWidth',2);

grid on;

xlabel('Temps [s]');
ylabel('\theta [deg]');

title('Position axe rotatif');

hold on;

% Limite -90°
plot([t(1) t(end)],[-90 -90],'--');

% Limite +90°
plot([t(1) t(end)],[90 90],'--');

hold off;

%% ========================================================================
% 19. VITESSES
% ========================================================================

figure('Name','Vitesses');

subplot(2,1,1);

plot(t,q1_dot*1000,'LineWidth',2);

grid on;

xlabel('Temps [s]');
ylabel('dq_1/dt [mm/s]');

title('Vitesse axe Z');

subplot(2,1,2);

plot(t,rad2deg(theta_dot),'LineWidth',2);

grid on;

xlabel('Temps [s]');
ylabel('d\theta/dt [deg/s]');

title('Vitesse axe rotation');

%% ========================================================================
% 20. ACCELERATIONS
% ========================================================================

figure('Name','Accelerations');

subplot(2,1,1);

plot(t,q1_ddot*1000,'LineWidth',2);

grid on;

xlabel('Temps [s]');
ylabel('d^2q_1/dt^2 [mm/s^2]');

title('Acceleration axe Z');

subplot(2,1,2);

plot(t,rad2deg(theta_ddot),'LineWidth',2);

grid on;

xlabel('Temps [s]');
ylabel('d^2\theta/dt^2 [deg/s^2]');

title('Acceleration axe rotation');

%% ========================================================================
% 21. TRAJECTOIRE CARTESIENNE 3D
% ========================================================================

figure('Name','Trajectoire cartésienne');

plot3( ...
    pos(:,1)*1000, ...
    pos(:,2)*1000, ...
    pos(:,3)*1000, ...
    'LineWidth',2);

grid on;
axis equal;

xlabel('X [mm]');
ylabel('Y [mm]');
zlabel('Z [mm]');

title('Trajectoire de l''effecteur');

hold on;

% Prise

plot3( ...
    T_pick.t(1)*1000, ...
    T_pick.t(2)*1000, ...
    T_pick.t(3)*1000, ...
    'o','MarkerSize',8,'LineWidth',2);

% Dépose

plot3( ...
    T_place.t(1)*1000, ...
    T_place.t(2)*1000, ...
    T_place.t(3)*1000, ...
    'o','MarkerSize',8,'LineWidth',2);

legend('Trajectoire','Prise LCD','Dépose');

%% ========================================================================
% 22. ANIMATION
% ========================================================================

fprintf('\n');
fprintf('Démarrage de l''animation...\n');

fig_anim = figure( ...
    'Name','Robot PR - Animation', ...
    'Position',[100 100 900 700]);

for i = 1:5:length(t)

    figure(fig_anim);

    robot.plot( ...
        [q1(i) theta(i)], ...
        'workspace',[-0.1 0.6 -0.4 0.4 0 0.5], ...
        'scale',0.7);

    title(sprintf( ...
        'Robot PR | t = %.2f s | q1 = %.1f mm | theta = %.1f deg', ...
        t(i), ...
        q1(i)*1000, ...
        rad2deg(theta(i))));

    drawnow;

end

fprintf('Animation terminée.\n');

%% ========================================================================
% 23. ENREGISTREMENT VIDEO
% ========================================================================

fprintf('\n');
fprintf('Enregistrement vidéo...\n');

video_filename = 'Robot_PR_Pick_Place_LCD.avi';

video = VideoWriter( ...
    video_filename, ...
    'Motion JPEG AVI');

video.FrameRate = 20;

open(video);

fig_video = figure( ...
    'Name','Robot PR - Video');

for i = 1:5:length(t)

    figure(fig_video);

    robot.plot( ...
        [q1(i) theta(i)], ...
        'workspace',[-0.1 0.6 -0.4 0.4 0 0.5], ...
        'scale',0.7);

    title(sprintf( ...
        'Robot PR - Pick & Place LCD | t = %.2f s', ...
        t(i)));

    drawnow;

    frame = getframe(fig_video);

    writeVideo(video,frame);

end

close(video);

fprintf('Video sauvegardée : %s\n',video_filename);

%% ========================================================================
% 24. FIN
% ========================================================================

fprintf('\n');
fprintf('============================================\n');
fprintf('SIMULATION TERMINEE AVEC SUCCES\n');
fprintf('============================================\n');
