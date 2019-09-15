clc; close all; clear all;
% Algorithm taken from: https://www.instructables.com/id/Stewart-Platform/

% --- Stewart platform design parameters ---
% Platform radius [m]
Rp = 0.1;

% Base radius [m]
Rb = 0.12;

% Horn length [m]
a = 0.05;

% Rod length [m]
s = 0.25;

% Angles of base plate joints
baseAngles = [0 60 120 180 240 300];
baseAngles = deg2rad(baseAngles);

% Angles of platform joints
% platAngles = [0 0 120 120 240 240];	% Triangle legs
platAngles = [0 60 120 180 240 300];	% Equispaced legs
platAngles = deg2rad(platAngles);

% Position of platform centroid (relative to base centroid) [m]
T = [0, 0, 0.2]';

% Platform angles (x, y, z) [deg]
Pang = [0, 0, 0];

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

rotUtheta = @(u, theta) ...
   [cos(theta) + u(1)^2 * (1 - cos(theta)),   u(1) * u(2) * (1 - cos(theta)) - u(3) * sin(theta),   u(1) * u(3) * (1 - cos(theta)) + u(2) * sin(theta); ...
    u(2) * u(1) * (1 - cos(theta)) + u(3) * sin(theta),   cos(theta) + u(2)^2 * (1 - cos(theta)),   u(2) * u(3) * (1 - cos(theta)) - u(1) * sin(theta); ...
    u(3) * u(1) * (1 - cos(theta)) - u(2) * sin(theta),   u(3) * u(2) * (1 - cos(theta)) + u(1) * sin(theta),   cos(theta) + u(3)^2 * (1 - cos(theta))];

% --- Compute reverse kinematics ---
Pang = deg2rad(Pang);
L = zeros(3, 6);
O = [0 0 0]';

% Calculate leg vectors
for ii = 1:6
    L(:, ii) = T + ProtB(Pang(1), Pang(2), Pang(3)) * P(:, ii) - B(:, ii);
end
Llen = sqrt(sum(L.^2));

% Transform platform vectors
for ii = 1:6
   Pxyz(:, ii) = ProtB(Pang(1), Pang(2), Pang(3)) * P(:, ii); 
end

% Find tau vectors
tau = zeros(3, 6);
alph = zeros(1, 6);
beta = zeros(1, 6);

for n = 1:6
    taun = cross(B(:, n), [0 0 1]');
    % Convert to unit vector
    taun /= norm(taun);
    % Invert if odd
    if rem(n, 2)
        taun *= -1;
    end

    % beta(n) = acos(dot(taun, [1 0 0]'));
    beta(n) = atan2(norm(cross(taun, [1 0 0]')), dot(taun, [1 0 0]'));
    % if rem(n, 2) ~= 0
    %     % If odd, use pi + beta
    %     beta(n) += pi;
    % end
    tau(:, n) = taun;
end

A = zeros(3, 6);
for n = 1:6
    l = Llen(:, 1);
    xp = Pxyz(1, n) + T(1);
    yp = Pxyz(2, n) + T(2);
    zp = Pxyz(3, n) + T(3);
    xb = B(1, n);
    yb = B(2, n);
    zb = B(3, n);
    taun = tau(:, n);

    % Calculate servo angles
    betan = beta(n);

    LL = l^2 - (s^2 - a^2);
    M = 2 * a * (zp - zb);
    N = 2 * a * ((xp - xb) * cos(betan) + (yp - yb) * sin(betan));

    alphn = asin(LL^2 / sqrt(M^2 + N^2)) - atan2(N, M);
    alph(n) = alphn;
    % if rem(n, 2) ~= 0
    %     % If odd, use pi - alph
    %     alph(n) = pi - alph(n);
    % end

    % Calculate servo vectors
    % if rem(n, 2) == 0
    %     A(1, n) = a * cos(alphn) * cos(betan) + xb;
    %     A(2, n) = a * cos(alphn) * sin(betan) + yb;
    %     A(3, n) = a * sin(alphn) + zb;
    % else
    %     A(1, n) = a * cos(pi - alphn) * cos(pi + betan) + xb;
    %     A(2, n) = a * cos(pi - alphn) * sin(pi + betan) + yb;
    %     A(3, n) = a * sin(pi - alphn) + zb;
    % end
    A(:, n) = a * taun;
    A(:, n) = rotUtheta(B(:, n), alphn) * A(:, n);
end

% Calculate rod vectors
S = zeros(3, 6);
S = (T + Pxyz) - (B + A);

hold on;

for ii = 1:6
    % Plot platform vectors
    plotVec(T, Pxyz(:, ii) + T, 'r');

    % Plot base vectors
    plotVec(O, B(:, ii), 'g');

    % Plot l-vectors
    plotVec(B(:, ii), B(:, ii) + L(:, ii), 'b');

    % Plot tangent vectors
    % plotVec(B(:, ii), B(:, ii) + tau(:, ii), 'k');

    % Plot servo horns
    plotVec(B(:, ii), B(:, ii) + A(:, ii), 'k');

    % Plot rod vectors
    plotVec(B(:, ii) + A(:, ii), B(:, ii) + A(:, ii) + S(:, ii), 'k');
end

pbaspect([1 1 1]);
xlabel('x [m]');
ylabel('y [m]');
zlabel('z [m]');

disp('Linkage lengths:');
for ii = 1:6
    disp(['Link ' num2str(ii) ': ' num2str(Llen(ii)) ' m']);
end

disp('');
disp('Servo angles: ');
for ii = 1:6
    disp(['Servo ' num2str(ii) ': ' num2str(rad2deg(alph(ii))) ' deg']);
end