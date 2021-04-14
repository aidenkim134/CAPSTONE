close all; clc; clear
g = tf(0.00849247* 9.5492965 / 20,[0.00030207 0.00135412 0.00395008]);

[y, t] = step(g);



ypp = diff(y, 2); 

t_inf1 = fzero(@(T) interp1(t(2:end-1), ypp, T, 'linear', 'extrap'),0);
y_inf1 = interp1(t, y, t_inf1, 'linear');
figure(); hold on; 
plot(t,y); plot(t_inf1, y_inf1, 'x');
xlabel('time (s)'); ylabel('speed (rad/s)')

h = mean(diff(t));
dy = gradient(y, h);
[~,idx] = max(dy);

b = [t([idx-1, idx+1]) ones(2,1)] \ y([idx-1, idx+1]);

% max line for the slope (2.5)
tv = [(-b(2))/b(1); (2.5-b(2))/b(1)];

f = [tv ones(2,1)] * b;
plot(tv, f, '--');


L = tv(1);
T = tv(2) - tv(1);

kp = 1.2 * T / L;
Ti = 2*L; ki = kp/Ti;
Td = 0.5*L; kd = kp * Td;


fprintf('Kp is: %d', kp);
fprintf('\n');
fprintf('Ki is: %d', ki);
fprintf('\n');
fprintf('Kd is: %d', kd);
C = tf([Td, 1, 1/Ti], [1 0]);
figure()
rlocus(C * g)
