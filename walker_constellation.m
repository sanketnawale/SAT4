function oev = walker_constellation(t, p, f, RAANspread, a, inc)


%% 🌍 Define Walker Delta Parameters
t = 12;            % Total satellites
p = 4;             % Number of orbital planes
f = 1;             % Phasing parameter
RAANspread = 2*pi; % Spread of RAAN (full 360 degrees)
a = 7000;          % Semi-major axis (Earth radius + altitude) in km
inc = deg2rad(55); % Inclination in radians

%% 🛰️ Generate Walker Delta Constellation
%oev = walker_constellation(t, p, f, RAANspread, a, inc);

%% 🛠️ Convert Orbital Elements to Cartesian Coordinates
Earth_Radius = 6378; % Earth radius in km
theta = linspace(0, 2*pi, 100); % Angle for orbit plotting

figure; hold on;
[xEarth, yEarth, zEarth] = sphere(50); % 🌍 Earth model
surf(Earth_Radius*xEarth, Earth_Radius*yEarth, Earth_Radius*zEarth, 'FaceColor', 'b', 'EdgeColor', 'none', 'FaceAlpha', 0.5);

for i = 1:t
    % Extract orbital elements
    sma = oev(i, 1);      % Semi-major axis
    ecc = oev(i, 2);      % Eccentricity (always 0 for Walker)
    inc = oev(i, 3);      % Inclination
    RAAN = oev(i, 5);     % Right Ascension of Ascending Node
    M = oev(i, 6);        % Mean anomaly
    
    % Compute true anomaly (assume circular orbit => true anomaly = mean anomaly)
    nu = M;
    
    % Compute position in orbital plane (parametric form)
    x_orbit = sma * cos(nu);
    y_orbit = sma * sin(nu);
    z_orbit = 0;
    
    % Rotation matrices (to align orbital plane with Earth's equator)
    Rz_RAAN = [cos(RAAN), -sin(RAAN), 0; sin(RAAN), cos(RAAN), 0; 0, 0, 1]; % Rotate by RAAN
    Rx_inc = [1, 0, 0; 0, cos(inc), -sin(inc); 0, sin(inc), cos(inc)]; % Rotate by inclination
    
    % Apply rotations to get final position
    pos = Rz_RAAN * Rx_inc * [x_orbit; y_orbit; z_orbit];
    
    % Plot satellite position
    plot3(pos(1), pos(2), pos(3), 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 6);
end

%% 📊 Final Plot Formatting
xlabel('X (km)'); ylabel('Y (km)'); zlabel('Z (km)');
title('Walker Delta Constellation');
axis equal; grid on; view(3);
legend('Earth', 'Satellites');

hold off;
