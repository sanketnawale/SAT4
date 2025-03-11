clc; clear; close all;

%% ✅ Call the Unmodified Walker Delta Function
t = 12;            % Total satellites
p = 4;             % Number of orbital planes
f = 1;             % Phasing parameter
RAANspread = 2*pi; % Spread of RAAN (full 360 degrees)
a = 7000;          % Semi-major axis (Earth radius + altitude) in km
inc = deg2rad(55); % Inclination in radians

% Call the function (no changes to walker_delta.m)
oev = walker_delta(t, p, f, RAANspread, a, inc);

%% 🌍 Plot the Earth
Earth_Radius = 6378; % Earth's radius in km
figure; hold on;
[xEarth, yEarth, zEarth] = sphere(50);
surf(Earth_Radius*xEarth, Earth_Radius*yEarth, Earth_Radius*zEarth, ...
    'FaceColor', 'b', 'EdgeColor', 'none', 'FaceAlpha', 0.5);

%% 🛰️ Convert Orbital Elements to Cartesian Coordinates
theta = linspace(0, 2*pi, 100); % For orbit plotting

for i = 1:t
    % Extract orbital elements
    sma = oev(i, 1);  % Semi-major axis
    inc = oev(i, 3);  % Inclination
    RAAN = oev(i, 5); % Right Ascension of Ascending Node
    M = oev(i, 6);    % Mean anomaly (assumed as true anomaly)

    % Compute position in orbital plane
    x_orbit = sma * cos(M);
    y_orbit = sma * sin(M);
    z_orbit = 0;
    
    % Rotation Matrices
    Rz_RAAN = [cos(RAAN), -sin(RAAN), 0; sin(RAAN), cos(RAAN), 0; 0, 0, 1]; % Rotate by RAAN
    Rx_inc = [1, 0, 0; 0, cos(inc), -sin(inc); 0, sin(inc), cos(inc)]; % Rotate by inclination
    
    % Apply rotations
    pos = Rz_RAAN * Rx_inc * [x_orbit; y_orbit; z_orbit];

    % Plot satellites
    plot3(pos(1), pos(2), pos(3), 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 6);

    % Draw orbits
    orbit_x = sma * cos(theta);
    orbit_y = sma * sin(theta);
    orbit_z = zeros(size(theta));
    orbit_points = Rz_RAAN * Rx_inc * [orbit_x; orbit_y; orbit_z];
    plot3(orbit_points(1, :), orbit_points(2, :), orbit_points(3, :), 'k--');
end

%% 🏆 Final Plot Formatting
xlabel('X (km)'); ylabel('Y (km)'); zlabel('Z (km)');
title('Walker Delta Constellation');
axis equal; grid on; view(3);
legend('Earth', 'Satellites', 'Orbits');
hold off;
