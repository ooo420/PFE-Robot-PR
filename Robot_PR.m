clear;
close all;
clc;

fprintf('\n');
fprintf('============================================================\n');
fprintf('       ROBOT PR - SIMULATION PICK & PLACE LCD\n');
fprintf('============================================================\n');

%% ========================================================================
% 1. VERIFICATION ROBOTICS TOOLBOX
% ========================================================================

fprintf('\nVerification de la Robotics Toolbox...\n');

if isempty(which('SerialLink'))
    error(['SerialLink est introuvable.' newline ...
           'Installe/active la Robotics Toolbox for MATLAB.']);
end

if isempty(which('Link'))
    error(['Link est introuvable.' newline ...
           'Installe/active la Robotics Toolbox for MATLAB.']);
end

fprintf('SerialLink trouve : OK\n');
fprintf('Link trouve       : OK\n');

%% ========================================================================
% 2. PARAMETRES DU ROBOT PR
% ========================================================================

L1 = 0.20;       % longueur/offset horizontal de base (m)
L2 = 0.25;       % longueur arbre 2 (m)

q1_min = 0.17;   % limite minimale axe prismatique
q1_max = 0.37;   % limite maximale axe prismatique

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
    'qlim', [-pi/2 pi/2]);

robot = SerialLink( ...
    [L1_link L2_link], ...
    'name', 'Robot PR');

%% ========================================================================
% 4. CONFIGURATIONS IMPORTANTES
% ========================================================================

% HOME
q_home = [0.370 0];

% POSITION DE PRISE DU LCD
q_pick = [0.178 deg2rad(45)];

% POSITION DE DEPOT
q_place = [0.180 0];

% Cinématique directe
T_pick = robot.fkine(q_pick);
P_pick = T_pick.t;

T_place = robot.fkine(q_place);
P_place = T_place.t;

fprintf('\n============================================================\n');
fprintf('POSITIONS DU ROBOT\n');
fprintf('============================================================\n');

fprintf('\nHOME\n');
fprintf('q1     = %.3f m\n', q_home(1));
fprintf('theta  = %.2f deg\n', rad2deg(q_home(2)));

fprintf('\nPRISE LCD\n');
fprintf('q1     = %.3f m\n', q_pick(1));
fprintf('theta  = %.2f deg\n', rad2deg(q_pick(2)));
fprintf('X      = %.3f m\n', P_pick(1));
fprintf('Y      = %.3f m\n', P_pick(2));
fprintf('Z      = %.3f m\n', P_pick(3));

fprintf('\nDEPOT SUPPORT\n');
fprintf('q1     = %.3f m\n', q_place(1));
fprintf('theta  = %.2f deg\n', rad2deg(q_place(2)));
fprintf('X      = %.3f m\n', P_place(1));
fprintf('Y      = %.3f m\n', P_place(2));
fprintf('Z      = %.3f m\n', P_place(3));

%% ========================================================================
% 5. TRAJECTOIRE DU CYCLE PFE
% ========================================================================

% Colonnes :
% temps | q1 | theta

waypoints = [
    0.0   0.370   0;
    1.5   0.200   0;
    2.5   0.200   0;
    3.5   0.370   0;
    4.5   0.370   deg2rad(45);
    5.5   0.180   deg2rad(45);
    5.7   0.178   deg2rad(45);
    6.2   0.178   deg2rad(45);
    6.7   0.370   deg2rad(45);
    7.0   0.370   0
];

t_way = waypoints(:,1);
q1_way = waypoints(:,2);
theta_way = waypoints(:,3);

%% ========================================================================
% 6. INTERPOLATION
% ========================================================================

dt = 0.05;

t = 0:dt:t_way(end);

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

% Limites de sécurité
q1 = max(q1_min, min(q1_max, q1));

theta = max( ...
    -pi/2, ...
    min(pi/2, theta));

%% ========================================================================
% 7. TABLE
% ========================================================================

table_x = 0.05;
table_y = -0.25;

table_L = 0.65;
table_W = 0.50;

table_H = 0.04;

%% ========================================================================
% 8. SUPPORT PLASTIQUE
% ========================================================================

support_x = P_place(1);
support_y = P_place(2);

support_L = 0.18;
support_W = 0.12;
support_H = 0.025;

support_z = table_H;

%% ========================================================================
% 9. LCD
% ========================================================================

lcd_L = 0.16;
lcd_W = 0.10;
lcd_H = 0.006;

% Position initiale du LCD
lcd_initial_x = P_pick(1);
lcd_initial_y = P_pick(2);

% LCD posé légèrement au-dessus de la table
lcd_initial_z = table_H + 0.008;

%% ========================================================================
% 10. VENTOUSE
% ========================================================================

vacuum_radius = 0.012;
vacuum_length = 0.025;

%% ========================================================================
% 11. FIGURE
% ========================================================================

fig = figure( ...
    'Name','Robot PR - Pick & Place LCD', ...
    'Color','white', ...
    'Position',[50 50 1200 750]);

%% ========================================================================
% 12. INITIALISATION SERIALINK/PLOT
% ========================================================================

% IMPORTANT :
% robot.plot est utilisé UNE SEULE FOIS.
%
% Si ta version de animate.m est corrompue, cette ligne peut encore
% produire l'erreur "Attempt to execute SCRIPT animate as a function".
%
% Dans ce cas, il faut réparer/remplacer animate.m dans la Toolbox.

robot.plot( ...
    q_home, ...
    'workspace', ...
    [-0.05 0.75 -0.35 0.35 0 0.55], ...
    'floorlevel',0);

hold on;

grid on;
axis equal;

xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');

view(135,25);

%% ========================================================================
% 13. TABLE
% ========================================================================

table_X = [ ...
    table_x ...
    table_x+table_L ...
    table_x+table_L ...
    table_x];

table_Y = [ ...
    table_y ...
    table_y ...
    table_y+table_W ...
    table_y+table_W];

table_Z = table_H * ones(1,4);

patch( ...
    table_X, ...
    table_Y, ...
    table_Z, ...
    [0.75 0.75 0.75], ...
    'FaceAlpha',0.5, ...
    'EdgeColor','k');

%% ========================================================================
% 14. SUPPORT
% ========================================================================

support_X = [ ...
    support_x-support_L/2 ...
    support_x+support_L/2 ...
    support_x+support_L/2 ...
    support_x-support_L/2];

support_Y = [ ...
    support_y-support_W/2 ...
    support_y-support_W/2 ...
    support_y+support_W/2 ...
    support_y+support_W/2];

support_Z = ...
    (support_z+support_H)*ones(1,4);

support_patch = patch( ...
    support_X, ...
    support_Y, ...
    support_Z, ...
    [0.2 0.6 0.8], ...
    'FaceAlpha',0.8, ...
    'EdgeColor','k');

%% ========================================================================
% 15. LCD INITIAL
% ========================================================================

lcd_X = [ ...
    -lcd_L/2 ...
     lcd_L/2 ...
     lcd_L/2 ...
    -lcd_L/2];

lcd_Y = [ ...
    -lcd_W/2 ...
    -lcd_W/2 ...
     lcd_W/2 ...
     lcd_W/2];

lcd_Z = lcd_H * ones(1,4);

lcd_patch = patch( ...
    lcd_X + lcd_initial_x, ...
    lcd_Y + lcd_initial_y, ...
    lcd_Z + lcd_initial_z, ...
    [0.1 0.1 0.1], ...
    'FaceAlpha',0.95, ...
    'EdgeColor','k');

%% ========================================================================
% 16. TEXTE
% ========================================================================

text( ...
    support_x, ...
    support_y, ...
    support_z+support_H+0.04, ...
    'SUPPORT PLASTIQUE', ...
    'HorizontalAlignment','center', ...
    'FontSize',10, ...
    'FontWeight','bold');

text( ...
    lcd_initial_x, ...
    lcd_initial_y, ...
    lcd_initial_z+0.025, ...
    'LCD', ...
    'HorizontalAlignment','center', ...
    'FontSize',10, ...
    'FontWeight','bold');

%% ========================================================================
% 17. CREATION DE LA VENTOUSE
% ========================================================================

P0 = robot.fkine(q_home).t;

vac_x = P0(1);
vac_y = P0(2);
vac_z = P0(3) - vacuum_length;

[VX,VY,VZ] = cylinder( ...
    vacuum_radius, ...
    30);

VZ = VZ * vacuum_length + vac_z;

ventouse = surf( ...
    VX + vac_x, ...
    VY + vac_y, ...
    VZ, ...
    'FaceAlpha',0.7, ...
    'EdgeColor','none');

%% ========================================================================
% 18. TEXTE ETAT
% ========================================================================

status_text = text( ...
    0.05, ...
    -0.32, ...
    0.48, ...
    'VENTOUSE : LIBRE', ...
    'FontSize',11, ...
    'FontWeight','bold');

cycle_text = text( ...
    0.05, ...
    -0.32, ...
    0.43, ...
    'HOME', ...
    'FontSize',12, ...
    'FontWeight','bold');

%% ========================================================================
% 19. VIDEO
% ========================================================================

video_filename = 'Robot_PR_Pick_Place_LCD.avi';

v = VideoWriter( ...
    video_filename, ...
    'Motion JPEG AVI');

v.FrameRate = 1/dt;

open(v);

fprintf('\nDemarrage de la simulation...\n');

%% ========================================================================
% 20. ANIMATION
% ========================================================================

for i = 1:length(t)

    %% ------------------------------------------------------------
    % CONFIGURATION ROBOT
    %% ------------------------------------------------------------

    q_current = [q1(i) theta(i)];

    % On calcule la position avec la Toolbox
    T = robot.fkine(q_current);
    P = T.t;

    %% ------------------------------------------------------------
    % ETAT DU CYCLE
    %% ------------------------------------------------------------

    if t(i) < 1.5

        etat = 'HOME';

        lcd_x = lcd_initial_x;
        lcd_y = lcd_initial_y;
        lcd_z = lcd_initial_z;

        lcd_pris = false;

    elseif t(i) < 3.5

        etat = 'DESCENTE / POSITIONNEMENT LCD';

        lcd_x = lcd_initial_x;
        lcd_y = lcd_initial_y;
        lcd_z = lcd_initial_z;

        lcd_pris = false;

    elseif t(i) < 4.5

        etat = 'POSITIONNEMENT AU-DESSUS DU LCD';

        lcd_x = lcd_initial_x;
        lcd_y = lcd_initial_y;
        lcd_z = lcd_initial_z;

        lcd_pris = false;

    elseif t(i) < 5.7

        etat = 'DESCENTE VERS LCD';

        lcd_x = lcd_initial_x;
        lcd_y = lcd_initial_y;
        lcd_z = lcd_initial_z;

        lcd_pris = false;

    elseif t(i) < 6.7

        etat = 'PRISE + TRANSPORT LCD';

        % Le LCD devient solidaire de la ventouse
        lcd_x = P(1);
        lcd_y = P(2);
        lcd_z = P(3) - vacuum_length - lcd_H;

        lcd_pris = true;

    else

        etat = 'RETOUR HOME';

        % LCD reste sur le support
        lcd_x = support_x;
        lcd_y = support_y;
        lcd_z = support_z + support_H + 0.008;

        lcd_pris = false;

    end

    %% ------------------------------------------------------------
    % DEPOSE EXACTE DU LCD
    %% ------------------------------------------------------------

    % Lorsque le robot atteint le support,
    % le LCD se détache et reste dessus.

    if t(i) >= 6.0

        if t(i) < 6.7

            % Positionnement au-dessus du support
            if t(i) >= 6.2

                etat = 'DEPOSE LCD';

                lcd_x = support_x;
                lcd_y = support_y;
                lcd_z = support_z + support_H + 0.008;

                lcd_pris = false;

            end

        end

    end

    %% ------------------------------------------------------------
    % MISE A JOUR LCD
    %% ------------------------------------------------------------

    set( ...
        lcd_patch, ...
        'XData',lcd_X + lcd_x, ...
        'YData',lcd_Y + lcd_y, ...
        'ZData',lcd_Z + lcd_z);

    %% ------------------------------------------------------------
    % VENTOUSE
    %% ------------------------------------------------------------

    % La ventouse est toujours sous l'arbre 2.

    vac_x = P(1);
    vac_y = P(2);
    vac_z = P(3) - vacuum_length;

    set( ...
        ventouse, ...
        'XData',VX + vac_x, ...
        'YData',VY + vac_y, ...
        'ZData',VZ - P0(3) + P(3));

    %% ------------------------------------------------------------
    % ETAT VENTOUSE
    %% ------------------------------------------------------------

    if lcd_pris

        set( ...
            status_text, ...
            'String','VENTOUSE : LCD SAISI');

    else

        set( ...
            status_text, ...
            'String','VENTOUSE : LIBRE');

    end

    set( ...
        cycle_text, ...
        'String',sprintf( ...
        '%s | t = %.2f s', ...
        etat,t(i)));

    %% ------------------------------------------------------------
    % TITRE
    %% ------------------------------------------------------------

    title( ...
        sprintf( ...
        'Robot PR - %s', ...
        etat), ...
        'FontSize',14, ...
        'FontWeight','bold');

    %% ------------------------------------------------------------
    % AFFICHAGE
    %% ------------------------------------------------------------

    drawnow;

    %% ------------------------------------------------------------
    % VIDEO
    %% ------------------------------------------------------------

    frame = getframe(fig);

    writeVideo(v,frame);

    pause(dt);

end

%% ========================================================================
% 21. FERMETURE VIDEO
% ========================================================================

close(v);

fprintf('\n============================================================\n');
fprintf('SIMULATION TERMINEE\n');
fprintf('============================================================\n');

fprintf('\nVideo : %s\n',video_filename);

%% ========================================================================
% 22. TRAJECTOIRE 3D
% ========================================================================

pos = zeros(length(t),3);

for i = 1:length(t)

    T = robot.fkine([q1(i) theta(i)]);

    pos(i,:) = T.t';

end

figure( ...
    'Name','Trajectoire Robot PR', ...
    'Color','white');

plot3( ...
    pos(:,1), ...
    pos(:,2), ...
    pos(:,3), ...
    'LineWidth',2);

grid on;
axis equal;

xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');

title('Trajectoire de l''effecteur Robot PR');

%% ========================================================================
% 23. GRAPHE q1
% ========================================================================

figure( ...
    'Name','Axe prismatique', ...
    'Color','white');

plot( ...
    t, ...
    q1*1000, ...
    'LineWidth',2);

grid on;

xlabel('Temps (s)');
ylabel('q1 (mm)');

title('Déplacement de l''axe prismatique');

%% ========================================================================
% 24. GRAPHE THETA
% ========================================================================

figure( ...
    'Name','Axe rotatif', ...
    'Color','white');

plot( ...
    t, ...
    rad2deg(theta), ...
    'LineWidth',2);

grid on;

xlabel('Temps (s)');
ylabel('\theta (deg)');

title('Rotation de l''axe 2');

%% ========================================================================
% 25. RESUME
% ========================================================================

fprintf('\n============================================================\n');
fprintf('RESUME DU CYCLE PICK & PLACE\n');
fprintf('============================================================\n');

fprintf('1. HOME\n');
fprintf('2. Descente vers le LCD\n');
fprintf('3. Positionnement de la ventouse\n');
fprintf('4. Préhension du LCD\n');
fprintf('5. Remontée avec LCD\n');
fprintf('6. Rotation vers le support\n');
fprintf('7. Descente vers le support\n');
fprintf('8. Dépose du LCD\n');
fprintf('9. Remontée\n');
fprintf('10. Retour HOME\n');

fprintf('\nPosition prise LCD :\n');
fprintf('q1 = %.3f m\n',q_pick(1));
fprintf('theta = %.2f deg\n',rad2deg(q_pick(2)));

fprintf('\nPosition support :\n');
fprintf('q1 = %.3f m\n',q_place(1));
fprintf('theta = %.2f deg\n',rad2deg(q_place(2)));

fprintf('\n============================================================\n');
fprintf('FIN DE LA SIMULATION\n');
fprintf('============================================================\n');
