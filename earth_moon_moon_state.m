function [position_m, velocity_mps] = earth_moon_moon_state(config, time_s, phase_deg)
phase_rad = deg2rad(phase_deg);
angle_rad = config.moon.meanMotion_radps * time_s + phase_rad;
position_m = config.moon.orbitalRadius_m .* [cos(angle_rad); sin(angle_rad)];
velocity_mps = config.moon.orbitalRadius_m * config.moon.meanMotion_radps .* [-sin(angle_rad); cos(angle_rad)];
end
