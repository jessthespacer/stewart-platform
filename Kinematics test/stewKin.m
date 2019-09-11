clc; close all; clear all;
% Algorithm taken from: https://www.instructables.com/id/Stewart-Platform/
% --- Constants ---
hexAngles = [0 60 120 180 240 300];
hexAngles = degtorad(hexAngles);

% --- Stewart platform design parameters ---
% Platform radius [m]
Rp = 0.1;

% Base radius [m]
Rb = 0.12;

% --- Calculate neutral position ---
% Calculate rod-platform joints (platform coords.)
P = zeros(3, 6);
P(1, :) = Rp .* cos(hexAngles);
P(2, :) = Rp .* sin(hexAngles);

% Calculate base-rod joints (base coords.)
B = zeros(3, 6);
B(1, :) = Rb .* cos(hexAngles);
B(2, :) = Rb .* sin(hexAngles);

O = [0 0 0]';

% --- Define kinematic matrices ---
ProtB = @(phi, theta, psi) ...
    [   cos(psi) * cos(theta),     -sin(psi) * cos(phi) + cos(psi) * sin(theta) * sin(phi),    sin(psi) * sin(phi) + cos(psi) * sin(theta) * cos(phi); ...
        sin(psi) * cos(theta),      cos(psi) * cos(phi) + sin(psi) * sin(theta) * sin(phi),   -cos(psi) * sin(phi) + sin(psi) * sin(theta) * cos(phi); ...
       -sin(theta)           ,      cos(theta) * sin(phi)                                 ,    cos(theta) * cos(phi)];

% --- Test ---
% Position of platform centroid relative to base centroid [m]
T = [0.05, 0, 0.25]';

% Platform angles, (x, y, z) [deg]
Pang = [45, 0, 45];

% Do not touch this
Pang = degtorad(Pang);
L = zeros(3, 6);

% Calculate leg vectors
for i = 1:6
    L(:, i) = T + ProtB(Pang(1), Pang(2), Pang(3)) * P(:, i) - B(:, i);
end

% Transform platform vectors
for i = 1:6
   Pxyz(:, i) = ProtB(Pang(1), Pang(2), Pang(3)) * P(:, i); 
end

for i = 1:6
    plotVec(T, Pxyz(:, i) + T, 'r');
    hold on;
    plotVec(O, B(:, i), 'g');
    plotVec(B(:, i), B(:, i) + L(:, i), 'b');
end

xlabel('x [m]');
ylabel('y [m]');
zlabel('z [m]');

disp('Linkage lengths:');
Llen = sqrt(sum(L.^2));
for i = 1:6
    disp(['Link ' num2str(i) ': ' num2str(Llen(i)) ' m']);
end