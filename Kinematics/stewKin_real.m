close all; clear all;
% Sources:
% https://www.instructables.com/id/Stewart-Platform/
% https://www.xarg.org/paper/inverse-kinematics-of-a-stewart-platform/

% --- Mode configuration ---
% Plot Stewart platform in 3D?
plotit = true;

% Use servo horns?
horn = true;

% --- Platform design ---
% Rod centre-to-centre length [m]
s = 0.177;

% Horn centre-to-centre distance [m]
a = 0.016;

% Rod-platform joints (platform coords.) [mm]
% Matrix of six column 3D vectors
P = [57.4337,	57.4337,	0,		   -57.4337,   -57.4337,	0; 	   ...
	-26.6658,	26.6658,	60.96,		26.6658,   -26.6658,   -60.96; ...
	 0,			0,			0,			0,			0,			0];
P = P ./ 1000;		% Convert to [m]

% Base-rod joints (base coords.) [mm]
B = [86.4235,	86.4235,   -13.9557,   -72.4678,   -72.4678,   -13.9557; ...
	-33.7820,	33.7820,	91.7359, 	57.9539,   -57.9539,   -91.7359; ...
	 0,			0, 			0,			0,			0,			0];
B = B ./ 1000;		% Convert to [m]

% --- Platform translation and rotation ---
% Position of platform centroid (relative to base centroid) [m]
% Neutral: T = [0, 0, 0.1734]'
T = [0.1, 0, 0.1]';

% Platform angles [phi, theta, psi] [deg]
% phi: Rotation on x-axis
% theta: Rotation on y-axis
% psi: Rotation on z-axis
Pang = [0, 0, 0];

disp('');
disp(['Platform position: T = [' num2str(T(1)) ', ' num2str(T(2)) ', ' ...
	num2str(T(3)) '] m']);
disp(['Platform angle: R = [' num2str(Pang(1)) ', ' num2str(Pang(2)) ', ' ...
	num2str(Pang(3)) '] deg']);
disp('');

Pang = deg2rad(Pang);

% --- Define 3D rotation matrix --- 
ProtB = @(phi, theta, psi) ...
   [cos(psi) * cos(theta),     -sin(psi) * cos(phi) + cos(psi) * sin(theta) * sin(phi),    sin(psi) * sin(phi) + cos(psi) * sin(theta) * cos(phi); ...
    sin(psi) * cos(theta),      cos(psi) * cos(phi) + sin(psi) * sin(theta) * sin(phi),   -cos(psi) * sin(phi) + sin(psi) * sin(theta) * cos(phi); ...
   -sin(theta)           ,      cos(theta) * sin(phi)                                 ,    cos(theta) * cos(phi)];

% --- Compute reverse kinematics ---
% Leg length matrix (six column 3D vectors)
L = zeros(3, 6);

% Origin
O = [0 0 0]';

% Locations of servo horn joints rel. to origin
% Matrix of six column 3D vectors
A = zeros(3, 6);

% Row vector of servo angles relative to horizontal
alph = zeros(1, 6);

% Row vector of servo plane angles relative to x-axis (curl around z-axis)
beta = deg2rad([90, 270, 210, 30, 330, 150])';

% Alternate beta: Use this if horn arms point away from each other
% beta = deg2rad([270, 90, 30, 210, 150, 330])';

% Leg/servo angle calculation loop
for n = 1:6
    % Rotate platform vector in global coords.
    Pxyz(:, n) = ProtB(Pang(1), Pang(2), Pang(3)) * P(:, n); 

    % Calculate leg vectors
    l = T + Pxyz(:, n) - B(:, n);

    if horn
    	betan = beta(n);

	    % Calculate servo angles
	    en = 2 * a * l(3);
	    fn = 2 * a * (cos(betan) * l(1) + sin(betan) * l(2));
	    gn = (norm(l))^2 - (s^2 - a^2);
	    
	    alphn = asin(gn / sqrt(en^2 + fn^2)) - atan2(fn, en);

	    if ~isreal(alphn)
	    	disp('ERROR: This position is not possible with the current setup. Please change lengths or target position.')
	    	disp('');
	    	return;
	    end

	    % Calculate servo vectors
	    A(:, n) = B(:, n) + a * [cos(alphn) * cos(betan); ...
	                             cos(alphn) * sin(betan); ...
	                             sin(alphn)];

	    % Write angles and leg vectors to memory
	    alph(n) = alphn;
	end

    L(:, n) = l;
end

if horn
	% Calculate rod vectors
	S = L - (A - B);
end

% Row vector of leg lengths
Llen = sqrt(sum(L.^2));

% Plotting loop
if plotit
	hold on;
	for ii = 1:6
	    % Plot platform vectors
	    plotVec(T, Pxyz(:, ii) + T, 'r');

	    % Plot base vectors
	    plotVec(O, B(:, ii), 'g');

	    % Plot leg vectors
	    plotVec(B(:, ii), B(:, ii) + L(:, ii), 'b');

	    % Plot servo horns
	    plotVec(B(:, ii), A(:, ii), 'k');

	    % Plot rod vectors
	    plotVec(A(:, ii), S(:, ii) + A(:, ii), 'k');
	end
	pbaspect([1 1 1]);
	xlabel('x [m]');
	ylabel('y [m]');
	zlabel('z [m]');
	xlim([-0.2 0.2]);
	ylim([-0.2 0.2]);
	zlim([-0.1 0.2]);
end

% Output final results
disp('Linkage lengths:');
for ii = 1:6
    disp(['Link ' num2str(ii) ': ' num2str(Llen(ii)) ' m']);
end

disp('');

if horn
	disp('Servo angles: ');
	for ii = 1:6
	    disp(['Servo ' num2str(ii) ': ' num2str(rad2deg(alph(ii))) ' deg']);
	end

	disp('');

	disp(['Avg angle: ' num2str(rad2deg(mean(alph))) ' deg']);
end