
function oev = walker_delta(t, p, f, RAANspread, a, inc)
% Author: Mauro De Sanctis
%RAAN, w, and i
% oe = walker(t, p, f, sma, inc)
% Build a walker constellation for the specified parameters.
% A Walker constellation consists of a group of satellites (t) that are in 
% circular orbits and have the same period (or, equivalently, radius sma) and 
% inclination (inc). The pattern of  the constellation consists of evenly 
% spaced satellites (s) in each of the orbital planes (p) specified so 
% that t = s * p.
% The ascending nodes of the orbital planes are also evenly spaced over a 
% range of right ascensions (RAAN).

%%%%%%%%% Input
%   t       number of satellites (adim) (integer)
%   p       number of planes (adim) (integer, 2<=p<=t)
%   f       phasing parameter (adim) (integer, 0<=f<=p-1)
%   RAANspread Spreading of the RAAN (rad) (0<RAANspread<=2*pi)
%   a       semi-major axis (km)
%   inc     inclination (rad)

t = 18;            % Total satellites
p = 6;             % Number of orbital planes
f = 1;             % Phasing parameter
RAANspread = pi; % Spread of RAAN (full 360 degrees)
a = 12000e3;          % Semi-major axis (Earth radius + altitude) in km
inc = deg2rad(87); % Inclination in radians



%oev = walker_delta(t, p, f, RAANspread, a, inc);
%disp(oev);
%%%%%%%%% Output
%   oe      orbital elements matrix, 6 x t. 
%           Each row is the orbital elements of i-th satellite:
%           Semi-major axis, Eccentricity, Inclination, RAAN, Arg of Perigee, Mean
%           anomaly (a e i RAAN w M)

dtr = pi / 180;
rtd = 180 / pi;
pi2 = 2 * pi;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%% Check of the input variables
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
check_input(1) = (round(t)==t); %check if t is integer
check_input(2) = (round(p)==p);
check_input(3) = (round(f)==f);
check_input(4) = (round(t/p)==t/p);
check_input(5) = p>1;
check_input(6) = p<t+1;
check_input(7) = t>2;
check_input(8) = f>-1;
check_input(9) = f<p;
check_input(10) = RAANspread<=pi2;
check_input(11) = RAANspread>0;
t;
p;
check_input;
n_check = size(check_input);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


if sum(check_input) ~= n_check(2)
    %error('////////// - Error on Input Variables - \\\\\\\\\\');
    oev = walker(8, 4, 3, pi2, 6500, 0);
else
    % Number of satellites per orbital plane
    s = t / p;

    % Pattern Unit
    PU = pi2/t;

    % Determine first series of RAAN
    iRAAN = zeros(1,t);
    RAAN = 0;

    % Delta-RAAN between adjacent orbital planes (radians)
    dRAAN = (RAANspread/t)*s;

    isat = 0;
    for i = 1:p
        for j = 1:s
            isat = isat + 1;
            iRAAN(isat) = RAAN;
        end
        RAAN = RAAN + dRAAN;
    end

    % determine mean anomalies

    % delta mean anomaly - inplane (radians)
    dM0_ip = PU*p;

    % delta mean anomaly - plane to plane (radians)
    dM0_pp = PU*f;

    % if (j == 1)
    %     xmasav = M0(isat);
    % end



    iM0 = 0;
    isat = 1;
    for i = 1:p
        for j = 1:s;
            M0(isat) = iM0;
            iM0 = iM0 + dM0_ip;
            if (j == 1)
                M0_ref = M0(isat);
            end
            isat = isat + 1;
        end
        iM0 = M0_ref + dM0_pp;
    end


    % load orbital elements matrix
    for i = 1:t
        oev(i, 1) = a;         % semi-major axis
        oev(i, 2) = 0.0;       % eccentricity
        oev(i, 3) = inc;       % inclination
        oev(i, 4) = 0.0;       % arg of perigee
        oev(i, 5) = iRAAN(i);  % RAAN
        oev(i, 6) = M0(i);     % Mean anomaly
    end

end

% ✅ Append Cartesian Coordinates (Minimal change)
XYZ = zeros(t, 3);
for i = 1:t
    [XYZ(i, 1), XYZ(i, 2), XYZ(i, 3)] = orbital_to_cartesian(a, 0, inc, iRAAN(i), 0, M0(i));
end

% ✅ Final output: Append (X, Y, Z) to orbital elements
oev = [a * ones(t, 1), zeros(t, 1), inc * ones(t, 1), zeros(t, 1), iRAAN', M0', XYZ];

end

% for i=1:11
%     if check_input(i) ~= 1
%     %error('////////// - Error on Input Variables - \\\\\\\\\\');
%     oev = walker(8, 4, 3, pi2, 6500, 0);
%     break;
%     end
% end

%save('walker_gen.txt','oe','-ascii')
