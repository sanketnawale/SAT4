function [X, Y, Z] = orbital_to_cartesian(a, e, i, RAAN, w, M)


    % Convert orbital elements to Cartesian coordinates (updates over time)
    % Inputs:
    %   a    - Semi-major axis (km)
    %   e    - Eccentricity (0 for circular orbits)
    %   i    - Inclination (radians)
    %   RAAN - Right Ascension of Ascending Node (radians)
    %   w    - Argument of Perigee (not used for circular orbit)
    %   M    - Mean Anomaly (radians) (updated over time)
    % Outputs:
    %   X, Y, Z - Cartesian coordinates in km

    mu = 398600; % Earth's gravitational parameter (km^3/s^2)
    
    % Compute mean motion (n) and updated mean anomaly
    n = sqrt(mu / a^3);  % Mean motion (rad/s)
    E = M; % Assume circular orbit: Eccentric anomaly ≈ Mean anomaly
    nu = E; % True anomaly = E (for circular orbits)
    
    % Compute position in orbital plane
    X_orbit = a * cos(nu);
    Y_orbit = a * sin(nu);
    Z_orbit = 0;

    % Apply rotations to get final 3D position
    Rz_RAAN = [cos(RAAN), -sin(RAAN), 0; sin(RAAN), cos(RAAN), 0; 0, 0, 1]; % Rotate by RAAN
    Rx_inc = [1, 0, 0; 0, cos(i), -sin(i); 0, sin(i), cos(i)]; % Rotate by inclination
    pos = Rz_RAAN * Rx_inc * [X_orbit; Y_orbit; Z_orbit];

    % Output final (X, Y, Z)
    X = pos(1);
    Y = pos(2);
    Z = pos(3);
end

