function state = v2_get_celestial_state(epoch, body, config, mode)
if nargin < 4 || isempty(mode)
    mode = 'auto';
end
epoch = v2_as_datetime(epoch);
body = lower(string(body));
state = struct('epoch', epoch, 'body', char(body), 'position_m', zeros(3, 1), 'velocity_mps', zeros(3, 1), 'source', config.ephemeris.source, 'isHighFidelity', false);
if config.ephemeris.allowToolbox && ~config.ephemeris.planar2D && ~strcmpi(mode, 'analytical') && exist('planetEphemeris', 'file') == 2 && body == "moon"
    try
        jd = juliandate(epoch);
        [position_m, velocity_mps] = planetEphemeris(jd, 'Earth', 'Moon', '432t');
        state.position_m = position_m(:);
        state.velocity_mps = velocity_mps(:);
        state.source = 'Aerospace Toolbox planetEphemeris 432t';
        state.isHighFidelity = true;
        return
    catch
    end
end
if body == "moon"
    state = analytical_moon_state(state, epoch, config);
elseif body == "sun"
    state = analytical_sun_state(state, epoch, config);
elseif body == "earth"
    state.source = 'Earth-centered inertial frame';
else
    error('V2:UnknownBody', 'Unsupported celestial body: %s', body)
end
end

function state = analytical_moon_state(state, epoch, config)
mu = config.physics.muEarth_m3ps2;
a_m = 384400e3;
e = 0.0549;
i_rad = deg2rad(5.145);
raan_rad = deg2rad(125.08);
argument_rad = deg2rad(318.15);
if config.ephemeris.planar2D
    i_rad = 0;
    raan_rad = 0;
    argument_rad = 0;
end
meanAnomalyJ2000_rad = deg2rad(115.3654);
period_s = config.physics.moonSiderealPeriod_s;
elapsed_s = seconds(epoch - config.ephemeris.j2000);
meanAnomaly_rad = mod(meanAnomalyJ2000_rad + 2 * pi * elapsed_s / period_s, 2 * pi);
eccentricAnomaly_rad = solve_kepler(meanAnomaly_rad, e);
trueAnomaly_rad = 2 * atan2(sqrt(1 + e) * sin(eccentricAnomaly_rad / 2), sqrt(1 - e) * cos(eccentricAnomaly_rad / 2));
radius_m = a_m * (1 - e * cos(eccentricAnomaly_rad));
positionPerifocal_m = radius_m .* [cos(trueAnomaly_rad); sin(trueAnomaly_rad); 0];
speedFactor = sqrt(mu * a_m) / radius_m;
velocityPerifocal_mps = speedFactor .* [-sin(eccentricAnomaly_rad); sqrt(1 - e ^ 2) * cos(eccentricAnomaly_rad); 0];
rotation = rotation_z(raan_rad) * rotation_x(i_rad) * rotation_z(argument_rad);
state.position_m = rotation * positionPerifocal_m;
state.velocity_mps = rotation * velocityPerifocal_mps;
state.source = 'Analytical mean lunar elements fallback; lower fidelity than SPICE';
state.isHighFidelity = false;
end

function state = analytical_sun_state(state, epoch, config)
elapsed_s = seconds(epoch - config.ephemeris.j2000);
angle_rad = 2 * pi * elapsed_s / config.physics.earthYear_s;
radius_m = config.physics.astronomicalUnit_m;
state.position_m = -radius_m .* [cos(angle_rad); sin(angle_rad); 0];
state.velocity_mps = -radius_m * 2 * pi / config.physics.earthYear_s .* [-sin(angle_rad); cos(angle_rad); 0];
state.source = 'Analytical circular Earth-Sun fallback';
state.isHighFidelity = false;
end

function eccentricAnomaly_rad = solve_kepler(meanAnomaly_rad, eccentricity)
eccentricAnomaly_rad = meanAnomaly_rad;
for iteration = 1:12
    correction = (eccentricAnomaly_rad - eccentricity * sin(eccentricAnomaly_rad) - meanAnomaly_rad) / (1 - eccentricity * cos(eccentricAnomaly_rad));
    eccentricAnomaly_rad = eccentricAnomaly_rad - correction;
    if abs(correction) < 1e-13
        break
    end
end
end

function rotation = rotation_x(angle_rad)
rotation = [1, 0, 0; 0, cos(angle_rad), -sin(angle_rad); 0, sin(angle_rad), cos(angle_rad)];
end

function rotation = rotation_z(angle_rad)
rotation = [cos(angle_rad), -sin(angle_rad), 0; sin(angle_rad), cos(angle_rad), 0; 0, 0, 1];
end
