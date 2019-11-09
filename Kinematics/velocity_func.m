close all; clear all;

% Initial position [m]
r0 = [0 0 0];

% Final position [m]
rf = [1 0 0];

% Timestep [s]
dt = 0.01;

% Top speed [m/s]
vmax = 0.5;

% Max accel [m/s^2]
amax = 10;

% Construct motion
rdot = zeros(5 / dt + 1, 2);
rdot(:, 1) = 0:dt:5;

for ii = 1:size(rdot, 2)
	if rdot(ii, 2) >= vmax
		break;
	end
	rdot(ii + 1, 2) = rdot(ii, 2) + dt * amax;
end