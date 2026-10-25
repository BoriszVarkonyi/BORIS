%% ============================================================
% TDK - Balanszrobot szabályozás kiértékelése
% =============================================================

clear out

%% Inicializálás
run('init_pendulum.m');

%% Szimuláció
stopTime = 60;

out = sim('pendulum.slx', ...
    'StopTime', num2str(stopTime));


%% ============================================================
% JELEK KIOLVASÁSA
% =============================================================

% Redukált állapotvektor
t = out.q.Time;
q = out.q.Data;

s     = q(:,1);      % [m]
theta = q(:,2);      % [rad]
psi   = q(:,3);      % [rad]


% Sebesség
tV = out.vLog.Time;
v  = out.vLog.Data;


% Pozíciószabályozó által generált sebességreferencia
tVRef = out.vRefLog.Time;
vRefSignal = out.vRefLog.Data;


% Dőlésszög referencia
tThetaRef = out.thetaRefLog.Time;
thetaRef  = out.thetaRefLog.Data;


% Dőlési szögsebesség
tThetaDot = out.thetaDotLog.Time;
thetaDot  = out.thetaDotLog.Data;


% Közös motor-nyomaték
tTau = out.tauCommon.Time;
tauCommon = out.tauCommon.Data;


%% ============================================================
% ÁTSZÁMÍTÁSOK
% =============================================================

thetaDeg    = rad2deg(theta);
thetaRefDeg = rad2deg(thetaRef);
thetaDotDeg = rad2deg(thetaDot);
psiDeg      = rad2deg(psi);


%% ============================================================
% 1. ÁBRA - KASZKÁD SZABÁLYOZÁS
% =============================================================

figure('Name','Kaszkád szabályozás');

tl = tiledlayout(4,1);
tl.TileSpacing = 'compact';
tl.Padding = 'compact';


% Pozíció
nexttile

plot(t, s, 'LineWidth',1.3);
hold on;
yline(posRef, '--', 's_{ref}');

ylabel('s [m]');
title('Pozíciókövetés');
grid on;


% Sebesség
nexttile

plot(tV, v, 'LineWidth',1.3);
hold on;

plot(tVRef, vRefSignal, '--', ...
    'LineWidth',1.1);

ylabel('v [m/s]');
title('Sebességkövetés');

legend('v','v_{ref}', ...
    'Location','best');

grid on;


% Dőlésszög
nexttile

plot(t, thetaDeg, 'LineWidth',1.3);
hold on;

plot(tThetaRef, thetaRefDeg, '--', ...
    'LineWidth',1.1);

yline(0, ':');

ylabel('\theta [deg]');
title('Dőlésszög követés');

legend('\theta','\theta_{ref}', ...
    'Location','best');

grid on;


% Motor nyomaték
nexttile

plot(tTau, tauCommon, ...
    'LineWidth',1.3);

hold on;

yline(tauMax, '--');
yline(-tauMax, '--');

ylabel('\tau_c [Nm]');
xlabel('Idő [s]');
title('Közös hajtónyomaték');

grid on;


title(tl, ...
    'Pozíció--sebesség--dőlésszög kaszkád szabályozás');


%% ============================================================
% 2. ÁBRA - DINAMIKAI ÁLLAPOTOK
% =============================================================

figure('Name','Dinamikai állapotok');

tl2 = tiledlayout(3,1);
tl2.TileSpacing = 'compact';
tl2.Padding = 'compact';


% Dőlésszög
nexttile

plot(t, thetaDeg, ...
    'LineWidth',1.3);

ylabel('\theta [deg]');
title('Dőlésszög');

grid on;


% Dőlési szögsebesség
nexttile

plot(tThetaDot, thetaDotDeg, ...
    'LineWidth',1.3);

ylabel('$\dot{\theta}$ [deg/s]', ...
    'Interpreter','latex');

title('Dőlési szögsebesség');

grid on;


% Irányszög
nexttile

plot(t, psiDeg, ...
    'LineWidth',1.3);

ylabel('\psi [deg]');
xlabel('Idő [s]');
title('Irányszög');

grid on;


title(tl2, ...
    'A robot dinamikai állapotai');


%% ============================================================
% NUMERIKUS EREDMÉNYEK
% =============================================================

positionError = posRef - s(end);

fprintf('\n');
fprintf('====================================================\n');
fprintf('       SZABÁLYOZÁSI EREDMÉNYEK\n');
fprintf('====================================================\n');

fprintf('Referencia pozíció:         %.3f m\n', posRef);
fprintf('Végső pozíció:              %.3f m\n', s(end));
fprintf('Végső pozícióhiba:          %.4f m\n', positionError);

fprintf('\n');

fprintf('Végső sebesség:             %.4f m/s\n', v(end));
fprintf('Max. sebesség:              %.4f m/s\n', max(abs(v)));

fprintf('\n');

fprintf('Max. |dőlésszög|:           %.3f deg\n', ...
    max(abs(thetaDeg)));

fprintf('Végső dőlésszög:            %.3f deg\n', ...
    thetaDeg(end));

fprintf('\n');

fprintf('Max. |motor nyomaték|:      %.4f Nm\n', ...
    max(abs(tauCommon)));

fprintf('====================================================\n');


%% ============================================================
% VILÁGKOORDINÁTA RENDSZERBELI PÁLYA PLOTOLÁSA
% =============================================================

xWorld = out.xWorldLog.Data;
yWorld = out.yWorldLog.Data;

figure('Name','Világkoordinátás pálya');

plot(xWorld, yWorld, 'LineWidth', 1.8);
hold on;

plot(xWorld(1), yWorld(1), 'o', ...
    'MarkerSize', 8, 'LineWidth', 2);

plot(xWorld(end), yWorld(end), 'x', ...
    'MarkerSize', 10, 'LineWidth', 2);

axis equal;
grid on;

xlabel('x [m]');
ylabel('y [m]');
title('A robot világkoordinátás pályája');

legend('Pálya','Start','Végpont','Location','best');

psi = out.q.Data(:,3);

L = 0.15;

quiver( ...
    xWorld(end), ...
    yWorld(end), ...
    L*cos(psi(end)), ...
    L*sin(psi(end)), ...
    0, ...
    'LineWidth', 2);