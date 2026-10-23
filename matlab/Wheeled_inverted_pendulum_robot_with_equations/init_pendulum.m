%% TDK - ideiglenes numerikus paraméterek
% CSAK az analitikus modell első ellenőrzéséhez!

mb  = 3.0;        % főtest tömege [kg]
mw  = 0.25;       % egy kerék tömege [kg]

r   = 0.08;       % keréksugár [m]
b   = 0.30;       % nyomtáv [m]
l   = 0.18;       % tengely - főtest CoM távolság [m]

Ix  = 0.0400;     % főtest tehetetlenség [kg*m^2]
Iy  = 0.0292;
Iz  = 0.0180;

Ja  = 8.0e-4;     % kerék saját tengely körüli tehetetlensége
Jzw = 4.33e-4;    % kerék yaw tehetetlensége

g   = 9.81;

%% Szabályozási paraméterek

Kp_theta = 2;     % Belső dőlésszög P
Kd_theta = 0.2;   % Belső dőlésszög D

Kp_pos = 0.05;    % Külső pozíció P
Kd_pos = 0;       % MOST kapcsold ki

Kp_vel = 0.25;    % Sebesség P
Ki_vel = 0.3;     % Sebesség I

%% Refereciaértékek deklarálása

%thetaRef = 0;   % Dőlésszög referencia
%vRef = 0.2;     % Referecia sebesség
posRef = 0;     % Pozíció referencia

vErrorDeadband = 0.01;    % [m/s]


%% Szaturációk deklarálása

tauMax = 2;             % Maximális motornyomaték
thetaMax = deg2rad(8);  % Maximális dőlésszögreferencia a sebességszabályzóhoz
vMax = 0.2;             % Maximális sebesség a pozíciószabályzáshoz

%% Yaw paraméterek

Kp_yaw = 0.02;              % yaw-rate P szabályzó
psiDotRef = deg2rad(30);    % 30 deg/s
tauDiffMax = 0.2;           % Nm

%% Súrlódási paraméterek
% KEZDETI, becsült tesztértékek

cMotor   = 0.003;    % N*m*s/rad
tauMotor = 0.01;     % N*m

cRoll    = 0.15;     % N*s/m
FRoll    = 0.08;     % N

cAir     = 0.02;     % N/(m/s)^2

vEps     = 0.02;     % m/s
omegaEps = 0.10;     % rad/s

%% Vezérlőparancsok inicializálása

manualMode = 0;        % 0 = AUTO, 1 = MANUAL
controllerMode = 0;    % 0 = PID, 1 = RL (egyelőre PID)

vManual = 0.15;        % [m/s]
yawManual = deg2rad(30);

%% Paramétervektor a nonlinearDynamics függvénynek

p = [
    mb;
    mw;
    r;
    b;
    l;
    Ix;
    Iy;
    Iz;
    Ja;
    Jzw;
    g;
    cMotor;
    tauMotor;
    cRoll;
    FRoll;
    cAir;
    vEps;
    omegaEps
];

%% Kezdeti állapot

q0 = [
    0;              % s
    0;              % theta
    0               % psi
    ];

qd0 = [
    0;              % v
    0;              % thetaDot
    0               % psiDot
    ];

%% Motorbemenet

u0 = [
    0;              % tauCommon
    0               % tauDifferential
    ];