clc; close all; clear all;
% Algorithm taken from: https://www.instructables.com/id/Stewart-Platform/

% --- Stewart platform design parameters ---
% Platform radius [m]
Rp = 0.1;

% Base radius [m]
Rb = 0.12;

% Horn length [m]
a = 0.01;

% Angles of base plate joints
baseAngles = [0 60 120 180 240 300];
baseAngles = degtorad(baseAngles);

% Angles of platform joints
platAngles = [0 0 120 120 240 240];	% Triangle legs
% platAngles = [0 60 120 180 240 300];	% Equispaced legs
platAngles = degtorad(platAngles);

% Position of platform centroid (relative to base centroid) [m]
T = [0.05, 0, 0.25]';

% Platform angles (x, y, z) [deg]
Pang = [45, 0, 45];

% --- Calculate rod vectors ---
% Calculate rod-platform joints (platform coords.)
P = zeros(3, 6);
P(1, :) = Rp .* cos(platAngles);
P(2, :) = Rp .* sin(platAngles);

% Calculate base-rod joints (base coords.)
B = zeros(3, 6);
B(1, :) = Rb .* cos(baseAngles);
B(2, :) = Rb .* sin(baseAngles);

% --- Define kinematic matrices ---
ProtB = @(phi, theta, psi) ...
   [cos(psi) * cos(theta),     -sin(psi) * cos(phi) + cos(psi) * sin(theta) * sin(phi),    sin(psi) * sin(phi) + cos(psi) * sin(theta) * cos(phi); ...
    sin(psi) * cos(theta),      cos(psi) * cos(phi) + sin(psi) * sin(theta) * sin(phi),   -cos(psi) * sin(phi) + sin(psi) * sin(theta) * cos(phi); ...
   -sin(theta)           ,      cos(theta) * sin(phi)                                 ,    cos(theta) * cos(phi)];

% --- Compute reverse kinematics ---
Pang = degtorad(Pang);
L = zeros(3, 6);
O = [0 0 0]';

% Calculate leg vectors
for ii = 1:6
    L(:, ii) = T + ProtB(Pang(1), Pang(2), Pang(3)) * P(:, ii) - B(:, ii);
end

% Transform platform vectors
for ii = 1:6
   Pxyz(:, ii) = ProtB(Pang(1), Pang(2), Pang(3)) * P(:, ii); 
end

for ii = 1:6
    plotVec(T, Pxyz(:, ii) + T, 'r');
    hold on;
    plotVec(O, B(:, ii), 'g');
    plotVec(B(:, ii), B(:, ii) + L(:, ii), 'b');
end

xlabel('x [m]');
ylabel('y [m]');
zlabel('z [m]');

disp('Linkage lengths:');
Llen = sqrt(sum(L.^2));
for ii = 1:6
    disp(['Link ' num2str(ii) ': ' num2str(Llen(ii)) ' m']);
end