function departure = v2_select_translunar_departure(config, targetPosition_m, transferDuration_s)
parkingRadius_m = config.physics.earthParkingRadius_m;
parkingSpeed_mps = sqrt(config.physics.muEarth_m3ps2 / parkingRadius_m);
candidateAngles_deg = 0:2:358;
bestScore = inf;
departure = struct('valid', false, 'position_m', nan(3, 1), 'velocity_mps', nan(3, 1), 'lambert', struct, 'radialVelocity_mps', nan, 'tangentialVelocity_mps', nan, 'injectionDeltaV_mps', nan(3, 1));
for angle_deg = candidateAngles_deg
    radialDirection_m = [cosd(angle_deg); sind(angle_deg); 0];
    tangentialDirection_m = [-sind(angle_deg); cosd(angle_deg); 0];
    position_m = parkingRadius_m * radialDirection_m;
    parkingVelocity_mps = parkingSpeed_mps * tangentialDirection_m;
    lambert = v2_solve_lambert(position_m, targetPosition_m, transferDuration_s, config.physics.muEarth_m3ps2, true);
    if ~lambert.valid
        continue
    end
    radialVelocity_mps = dot(lambert.velocity1_mps, radialDirection_m);
    tangentialVelocity_mps = dot(lambert.velocity1_mps, tangentialDirection_m);
    injectionDeltaV_mps = lambert.velocity1_mps - parkingVelocity_mps;
    progradeDeltaV_mps = dot(injectionDeltaV_mps, tangentialDirection_m);
    if tangentialVelocity_mps <= 0 || progradeDeltaV_mps <= 0
        continue
    end
    score = norm(injectionDeltaV_mps) + 3 * abs(radialVelocity_mps) + 20 * max(0, -radialVelocity_mps);
    if score < bestScore
        bestScore = score;
        departure = struct('valid', true, 'position_m', position_m, 'velocity_mps', parkingVelocity_mps, 'lambert', lambert, 'radialVelocity_mps', radialVelocity_mps, 'tangentialVelocity_mps', tangentialVelocity_mps, 'injectionDeltaV_mps', injectionDeltaV_mps);
    end
end
end
