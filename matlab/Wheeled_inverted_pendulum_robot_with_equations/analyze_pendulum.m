out = sim('pendulum.slx', 'StopTime', '2');

tTheta = out.thetaFeedback.Time;
theta  = out.thetaFeedback.Data;

tDot     = out.thetaDotLog.Time;
thetaDot = out.thetaDotLog.Data;

tTau = out.tauCommon.Time;
tau  = out.tauCommon.Data;

t = out.q.Time;
q = out.q.Data;

s     = q(:,1);
theta = q(:,2);
psi   = q(:,3);

figure;

subplot(3,1,1);
plot(t, s, 'LineWidth', 1.2);
ylabel('s [m]');
title('Hosszirányú elmozdulás');
grid on;

subplot(3,1,2);
plot(t, rad2deg(theta), 'LineWidth', 1.2);
hold on;
yline(0, '--', '\theta_{ref}=0°');
ylabel('\theta [°]');
title('Dőlésszög');
grid on;

subplot(3,1,3);
plot(t, rad2deg(psi), 'LineWidth', 1.2);
ylabel('\psi [°]');
xlabel('Idő [s]');
title('Irányszög');
grid on;

figure;

subplot(3,1,1)
plot(tTheta,rad2deg(theta),'LineWidth',1.2)
yline(0,'--')
ylabel('\theta [deg]')
grid on

subplot(3,1,2)
plot(tDot,rad2deg(thetaDot),'LineWidth',1.2)
ylabel('\dot{\theta} [deg/s]')
grid on

subplot(3,1,3)
plot(tTau,tau,'LineWidth',1.2)
ylabel('\tau_c [Nm]')
xlabel('Idő [s]')
grid on