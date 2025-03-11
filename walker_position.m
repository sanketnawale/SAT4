function [Sat_X, Sat_Y, Sat_Z] = walker_position(orbital_element, time)
    if ~isstruct(orbital_element)
        error('walker_position: Invalid orbital element structure.');
    end

    a = orbital_element.Semi_Major_Axis;  
    e = orbital_element.Eccentricity;  
    incl = orbital_element.Inclination;  
    RAAN = orbital_element.RAAN;  
    w = orbital_element.Arg_Perigee;  
    M0 = orbital_element.Mean_Anomaly;  
