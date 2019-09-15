clc; clear all; close all;

function y = f(x, c)
	l1 = c(1);
	l2 = c(2);
	rb = c(3);
	rp = c(4);

	theta1 = x(1);
	theta2 = x(2);
	psi = x(3);
	h = x(4);
	
	y = zeros(4, 1);
	
	y(1) = l1 * cos(theta1) + rp * cos(psi) - rb;
	y(2) = l1 * sin(theta1) + rp * sin(psi) - h;
	y(3) = l2 * cos(theta2) - rp * cos(psi) - rb;
	y(4) = l2 * sin(theta2) - rp * sin(psi) - h;
end

l1 = 0.2
l2 = 0.25
rb = 0.12
rp = 0.1

c = [l1, l2, rb, rp];

guess = [deg2rad([60 110 20]) mean([l1 l2])];
[x, fval, ino] = fsolve(@(x)(f(x, c)), guess);

theta1 = x(1);
theta2 = x(2);
psi = x(3);
h = x(4);

disp('Solution: ');
disp(['theta1 = ' num2str(rad2deg(theta1))]);
disp(['theta2 = ' num2str(rad2deg(theta2))]);
disp(['psi = ' num2str(rad2deg(psi))]);
disp(['h = ' num2str(h)]);

A = l1 * [cos(theta1), sin(theta1)];
B = A + 2 * rp * [cos(psi), sin(psi)];
C = [2 * rb, 0];
Bprime = C + l2 * [cos(theta2), sin(theta2)];

hold on;
grid on;

plot([0, C(1)], [0, C(2)], 'b');
plot([0, A(1)],[0, A(2)], 'r');
plot([C(1), B(1)], [C(2), B(2)], 'r');
plot([A(1), B(1)], [A(2), B(2)], 'g');

plot(0, 0, "markersize", 20, 'k');
plot(A(1), A(2), "markersize", 20, 'k');
plot(B(1), B(2), "markersize", 20, 'k');
plot(C(1), C(2), "markersize", 20, 'k');
plot(Bprime(1), Bprime(2), "markersize", 20, 'k');
