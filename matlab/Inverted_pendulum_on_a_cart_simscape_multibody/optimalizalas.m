%% Fordított inga - külső pozíciószabályozó optimalizálása
% fminsearch + Fast Restart
%
% Optimalizált paraméterek:
%   Kz - pozícióhiba erősítés
%   Kv - kocsi sebesség visszacsatolás
%
% A belső dőlésszög PD egyelőre fix:
%   Kp = 30
%   Kd = 2

clear;
clc;
close all;

%% =========================================================
%  MODELL NEVE
% ==========================================================

mdl = "inverted_pendulum";

load_system(mdl);


%% =========================================================
%  FIZIKAI PARAMÉTEREK
% ==========================================================

N = 10;

M = 0.5;            % [kg] kocsi tömege

m = 1.5;            % [kg] inga/felső test tömege

l = 0.15;           % [m] forgástengely -> tömegközéppont

J = 0.01125;        % [kg*m^2] tehetetlenségi nyomaték

g = 9.81;           % [m/s^2]


%% =========================================================
%  CSILLAPÍTÁS
% ==========================================================

b_z = 0;            % [N*s/m]

b_0 = 0.1;          % [N*m*s/rad]


%% =========================================================
%  KEZDETI ÁLLAPOTOK
% ==========================================================

z_0 = 0;                % [m]

z_dot_0 = 0;            % [m/s]

phi_0 = 5*pi/180;       % [rad]

phi_dot_0 = 0;          % [rad/s]


%% =========================================================
%  SEGÉDVÁLTOZÓK
% ==========================================================

A = M + m;

C = J + m*l^2;


%% =========================================================
%  BELSŐ DŐLÉSSZÖG-SZABÁLYOZÓ
% ==========================================================

Kp = 30;

Kd = 3;

F_max = 10;             % [N]


%% =========================================================
%  KÜLSŐ POZÍCIÓSZABÁLYOZÓ
% ==========================================================

z_ref = 0;              % [m]

Pos_limit = 2*pi/180;   % [rad]


%% =========================================================
%  KEZDŐ Kz, Kv
% ==========================================================

Kz0 = 0.003;

Kv0 = 0.5;


%% =========================================================
%  LOGARITMIKUS PARAMÉTEREZÉS
%
%  Kz = exp(p1)
%  Kv = exp(p2)
% ==========================================================

p0 = log([Kz0, Kv0]);


%% =========================================================
%  OPTIMALIZÁCIÓS IDŐHORIZONT
%
%  Fontos: ne legyen túl rövid.
%  25 s alatt már látszik az oda-vissza mozgás.
% ==========================================================

optimizationStopTime = 8;

set_param( ...
    mdl, ...
    "StopTime", ...
    num2str(optimizationStopTime));


%% =========================================================
%  FMINSEARCH BEÁLLÍTÁSOK
% ==========================================================

options = optimset( ...
    "Display", "iter", ...
    "MaxIter", 100, ...
    "MaxFunEvals", 300, ...
    "TolX", 1e-4, ...
    "TolFun", 1e-3);


%% =========================================================
%  FAST RESTART
% ==========================================================

disp("Fast Restart bekapcsolasa...");

set_param(mdl, "FastRestart", "on");


%% =========================================================
%  OPTIMALIZÁLÁS
% ==========================================================

try

    [p_opt, J_opt] = fminsearch( ...
        @(p) pendulum_cost(p, mdl), ...
        p0, ...
        options);

catch ME

    set_param(mdl, "FastRestart", "off");

    rethrow(ME);

end


%% =========================================================
%  FAST RESTART KIKAPCSOLÁSA
% ==========================================================

set_param(mdl, "FastRestart", "off");

disp("Fast Restart kikapcsolva.");


%% =========================================================
%  OPTIMÁLIS PARAMÉTEREK
% ==========================================================

Kz_opt = exp(p_opt(1));

Kv_opt = exp(p_opt(2));


fprintf("\n");
fprintf("========================================\n");
fprintf("OPTIMALIZALAS EREDMENYE\n");
fprintf("========================================\n");

fprintf("Kz = %.10f\n", Kz_opt);
fprintf("Kv = %.10f\n", Kv_opt);

fprintf("J  = %.6f\n", J_opt);

fprintf("========================================\n\n");


%% =========================================================
%  OPTIMÁLIS ÉRTÉKEK BEÁLLÍTÁSA
% ==========================================================

Kz = Kz_opt;

Kv = Kv_opt;


%% =========================================================
%  VÉGSŐ VALIDÁCIÓ
% ==========================================================

validationStopTime = 30;

set_param( ...
    mdl, ...
    "StopTime", ...
    num2str(validationStopTime));

disp("Vegso validacios szimulacio...");

simOut = sim(mdl);


%% =========================================================
%  LOGOLT JELEK KIOLVASÁSA
% ==========================================================

z_ts = simOut.z_log;

phi_ts = simOut.phi_log;

F_ts = simOut.F_log;


t_z = z_ts.Time;

z = squeeze(z_ts.Data);


t_phi = phi_ts.Time;

phi = squeeze(phi_ts.Data);


t_F = F_ts.Time;

F = squeeze(F_ts.Data);


%% =========================================================
%  VALIDÁCIÓS MÉRŐSZÁMOK
% ==========================================================

max_z = max(abs(z));

max_phi_deg = max(abs(phi))*180/pi;

max_F = max(abs(F));

final_z = z(end);

final_phi_deg = phi(end)*180/pi;


%% =========================================================
%  ÁBRÁZOLÁS
% ==========================================================

figure;

subplot(3,1,1);

plot( ...
    t_z, ...
    z, ...
    "LineWidth", 1.5);

grid on;

ylabel("z (m)");

title( ...
    sprintf( ...
        "Optimalizalt szabalyazo: Kz = %.5g, Kv = %.5g", ...
        Kz, Kv));


subplot(3,1,2);

plot( ...
    t_phi, ...
    phi*180/pi, ...
    "LineWidth", 1.5);

grid on;

ylabel("\phi (fok)");


subplot(3,1,3);

plot( ...
    t_F, ...
    F, ...
    "LineWidth", 1.5);

grid on;

ylabel("F (N)");

xlabel("Ido (s)");


%% =========================================================
%  EREDMÉNYEK KIÍRÁSA
% ==========================================================

fprintf("\n");
fprintf("VALIDACIOS EREDMENYEK\n");
fprintf("----------------------------------------\n");

fprintf( ...
    "Max |z|   = %.4f m\n", ...
    max_z);

fprintf( ...
    "Max |phi| = %.4f fok\n", ...
    max_phi_deg);

fprintf( ...
    "Max |F|   = %.4f N\n", ...
    max_F);

fprintf( ...
    "Vegso z   = %.4f m\n", ...
    final_z);

fprintf( ...
    "Vegso phi = %.4f fok\n", ...
    final_phi_deg);

fprintf("----------------------------------------\n");


%% =========================================================
%  Opcionális: optimális értékek külön kiírása
% ==========================================================

fprintf("\nMásold vissza a normál futtató scriptedbe:\n\n");

fprintf("Kz = %.10f;\n", Kz_opt);
fprintf("Kv = %.10f;\n", Kv_opt);