%Script a fordított inga simulink modell futtatásához

%% Adatok deklarálása

M = 0.5;        % A kocsi tömege
m = 1.5;        % A felső test tömege
l = 0.15;       % A forgástengely és a test tömegközéppontja közti távolság
J = 0.01125;    % A felső test tömegközéppontjára vett tehetetlenségi nyomaték
g = 9.81;       % A gravitációs gyorsulás

b_z = 0;        % A kocsi súrlódása
b_0 = 0.1;        % A csapágy súrlódása

u = 0;          % A kocsira ható erő

z_0 = 0;        % A kocsi kezdeti pozíciója
z_dot_0 = 0;    % A kocsi kezdeti sebessége

phi_0 = 5*pi/180;  % A kezdeti dőlésszög
phi_dot_0 = 0;     % A kezdeti szögsebesség

A = M + m;
C = J + m * l^2;

Kp = 30;        % Arányos tag
Kd = 2;         % Deriváló tag

F_max = 10;     % Szaturáció

z_ref = 0;      % Kocsi referencia pozíció

Kz = 0.05;       % Pozíciószabályozás arányos tagja
Kv = 0.15;       % Arányos tag a sebességhez

Pos_limit = 5*pi/180;  % A pozíciószabályzás dőlésszög limitációja

%% A szimuláció futtatása

simOut = sim("pendilum.slx",'StopTime', '10');

%% Eredmények megjelenítése
t = simOut.tout;
z = simOut.yout{1}.Values.Data;
phi = simOut.yout{2}.Values.Data;
figure;
subplot(2, 1, 1);
plot(t, z, "LineWidth", 1.5);
grid on;
xlabel("Idő (s)");
ylabel("Kocsi pozíciója (m)");
subplot(2, 1, 2);
plot(t, phi * 180 / pi, "LineWidth", 1.5);
grid on;
xlabel("Idő (s)");
ylabel("Dőlésszög (fok)");