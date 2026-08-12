function result = v2_evaluate_candidate(candidate, config)
candidate = normalize_candidate(candidate, config);
result = blank_result(candidate, config);
launchEpoch = candidate.launchEpoch;
arrivalEpoch = launchEpoch + days(candidate.outboundFlightTime_days);
departureEpoch = arrivalEpoch + days(candidate.lunarOrbitDuration_days);
returnEpoch = departureEpoch + days(candidate.returnFlightTime_days);
if returnEpoch > config.searchEndEpoch + days(config.mission.maximumFlightTime_days)
    result.rejectReason = 'Mission exceeds configured search horizon';
    return
end
moonLaunch = v2_get_celestial_state(launchEpoch, 'moon', config);
moonArrival = v2_get_celestial_state(arrivalEpoch, 'moon', config);
moonDeparture = v2_get_celestial_state(departureEpoch, 'moon', config);
earthReturn = v2_get_celestial_state(returnEpoch, 'earth', config);
earthParkingRadialDirection = moonLaunch.position_m / norm(moonLaunch.position_m);
earthParkingPosition_m = config.physics.earthParkingRadius_m * earthParkingRadialDirection;
earthParkingSpeed_mps = sqrt(config.physics.muEarth_m3ps2 / config.physics.earthParkingRadius_m);
earthParkingTangentialDirection = cross([0; 0; 1], earthParkingRadialDirection);
earthParkingTangentialDirection = earthParkingTangentialDirection / norm(earthParkingTangentialDirection);
earthParkingVelocity_mps = earthParkingSpeed_mps * earthParkingTangentialDirection;
moonArrivalRadialDirection = -moonArrival.position_m / norm(moonArrival.position_m);
moonArrivalInterfacePosition_m = moonArrival.position_m + moonArrivalRadialDirection * config.physics.moonSOI_m;
outbound = v2_solve_lambert(earthParkingPosition_m, moonArrivalInterfacePosition_m, candidate.outboundFlightTime_days * 86400, config.physics.muEarth_m3ps2, true);
if ~outbound.valid
    result.rejectReason = ['Outbound Lambert failure: ', outbound.message];
    return
end
moonArrivalVelocity_mps = outbound.velocity2_mps - moonArrival.velocity_mps;
outboundVInf_mps = norm(moonArrivalVelocity_mps);
moonArrivalDirection_m = moonArrivalInterfacePosition_m - moonArrival.position_m;
moonApproachValid = dot(moonArrivalVelocity_mps, moonArrivalDirection_m) < 0;
departureDeltaV_mps = norm(outbound.velocity1_mps - earthParkingVelocity_mps);
moonRadialDirection = -moonDeparture.position_m / norm(moonDeparture.position_m);
moonInterfacePosition_m = moonDeparture.position_m + moonRadialDirection * config.physics.moonSOI_m;
earthArrivalRadialDirection = cross([0; 0; 1], moonDeparture.position_m / norm(moonDeparture.position_m));
earthArrivalRadialDirection = earthArrivalRadialDirection / norm(earthArrivalRadialDirection);
earthArrivalPosition_m = config.physics.earthParkingRadius_m * earthArrivalRadialDirection;
returnTransfer = v2_solve_lambert(moonInterfacePosition_m, earthArrivalPosition_m, candidate.returnFlightTime_days * 86400, config.physics.muEarth_m3ps2, true);
if ~returnTransfer.valid
    result.rejectReason = ['Return Lambert failure: ', returnTransfer.message];
    return
end
returnVInf_mps = norm(returnTransfer.velocity1_mps - moonDeparture.velocity_mps);
moonDepartureDirection_m = moonInterfacePosition_m - moonDeparture.position_m;
moonDepartureOutwardValid = dot(returnTransfer.velocity1_mps - moonDeparture.velocity_mps, moonDepartureDirection_m) > 0;
periapsisRadius_m = config.physics.moonRadius_m + candidate.lunarOrbitAltitude_m;
circularLunarSpeed_mps = sqrt(config.physics.muMoon_m3ps2 / periapsisRadius_m);
insertionPeriapsisSpeed_mps = sqrt(outboundVInf_mps ^ 2 + 2 * config.physics.muMoon_m3ps2 / periapsisRadius_m);
departurePeriapsisSpeed_mps = sqrt(returnVInf_mps ^ 2 + 2 * config.physics.muMoon_m3ps2 / periapsisRadius_m);
insertionDeltaV_mps = max(0, insertionPeriapsisSpeed_mps - circularLunarSpeed_mps);
departureDeltaVLunar_mps = max(0, departurePeriapsisSpeed_mps - circularLunarSpeed_mps);
arrivalSpeed_mps = norm(returnTransfer.velocity2_mps - earthReturn.velocity_mps);
earthArrivalCircularSpeed_mps = sqrt(config.physics.muEarth_m3ps2 / config.physics.earthParkingRadius_m);
earthCaptureDeltaV_mps = max(0, arrivalSpeed_mps - earthArrivalCircularSpeed_mps);
totalDeltaV_mps = departureDeltaV_mps + insertionDeltaV_mps + departureDeltaVLunar_mps + earthCaptureDeltaV_mps;
fuel = v2_calculate_fuel(totalDeltaV_mps, config.vehicle, config);
orbitQuality = lunar_orbit_quality(candidate, periapsisRadius_m, outboundVInf_mps, config);
returnSafety = max(0, 1 - arrivalSpeed_mps / config.mission.maximumEarthArrivalSpeed_mps);
flightTime_days = candidate.outboundFlightTime_days + candidate.lunarOrbitDuration_days + candidate.returnFlightTime_days;
flightTimeQuality = max(0, 1 - flightTime_days / config.mission.maximumFlightTime_days);
constraints = struct;
constraints.outboundLambertValid = outbound.valid;
constraints.returnLambertValid = returnTransfer.valid;
constraints.departureOutwardValid = dot(outbound.velocity1_mps, earthParkingRadialDirection) > 0;
constraints.moonApproachValid = moonApproachValid;
constraints.moonDepartureOutwardValid = moonDepartureOutwardValid;
constraints.lunarPeriapsisValid = candidate.lunarOrbitAltitude_m >= config.mission.minimumLunarPeriapsisAltitude_m && candidate.lunarOrbitAltitude_m <= config.mission.maximumLunarPeriapsisAltitude_m;
constraints.lunarApoapsisValid = candidate.lunarOrbitAltitude_m <= config.mission.maximumLunarApoapsisAltitude_m;
constraints.lunarOrbitDurationValid = candidate.lunarOrbitDuration_days >= config.mission.minimumLunarOrbitDuration_days;
constraints.returnSpeedValid = arrivalSpeed_mps <= config.mission.maximumEarthArrivalSpeed_mps;
constraints.deltaVValid = totalDeltaV_mps <= config.vehicle.maxMissionDeltaV_mps;
constraints.fuelCapacityValid = fuel.withinVehicleCapacity;
constraints.flightTimeValid = flightTime_days <= config.mission.maximumFlightTime_days;
constraints.physicallyValid = all(struct2array(constraints));
if ~constraints.physicallyValid
    result.rejectReason = first_failed_constraint(constraints);
    result.score = config.mission.penaltyInvalid;
    result.valid = false;
    result = populate_result(result);
    return
end
weights = config.mission.scoreWeights;
fuelTerm = -totalDeltaV_mps / config.vehicle.maxMissionDeltaV_mps;
feasibilityTerm = double(constraints.physicallyValid);
score = weights.fuel * fuelTerm + weights.feasibility * feasibilityTerm + weights.orbit * orbitQuality + weights.returnSafety * returnSafety + weights.flightTime * flightTimeQuality;
result.valid = true;
result.score = score;
result.rejectReason = '';
result = populate_result(result);

    function output = populate_result(output)
        output.arrivalEpoch = arrivalEpoch;
        output.departureEpoch = departureEpoch;
        output.returnEpoch = returnEpoch;
        output.moonArrivalState = moonArrival;
        output.moonLaunchState = moonLaunch;
        output.moonDepartureState = moonDeparture;
        output.outboundLambert = outbound;
        output.returnLambert = returnTransfer;
        output.outboundVInf_mps = outboundVInf_mps;
        output.returnVInf_mps = returnVInf_mps;
        output.moonApproachValid = moonApproachValid;
        output.moonDepartureOutwardValid = moonDepartureOutwardValid;
        output.departureDeltaV_mps = departureDeltaV_mps;
        output.lunarOrbitInsertionDeltaV_mps = insertionDeltaV_mps;
        output.lunarDepartureDeltaV_mps = departureDeltaVLunar_mps;
        output.earthCaptureDeltaV_mps = earthCaptureDeltaV_mps;
        output.earthArrivalSpeed_mps = arrivalSpeed_mps;
        output.totalDeltaV_mps = totalDeltaV_mps;
        output.fuel = fuel;
        output.orbitQuality = orbitQuality;
        output.returnSafety = returnSafety;
        output.flightTime_days = flightTime_days;
        output.constraints = constraints;
        output.earthParkingPosition_m = earthParkingPosition_m;
        output.earthParkingVelocity_mps = earthParkingVelocity_mps;
        output.earthArrivalPosition_m = earthArrivalPosition_m;
    end
end

function candidate = normalize_candidate(candidate, config)
if ~isfield(candidate, 'launchEpoch')
    candidate.launchEpoch = config.searchStartEpoch;
end
candidate.launchEpoch = v2_as_datetime(candidate.launchEpoch);
if ~isfield(candidate, 'outboundFlightTime_days')
    candidate.outboundFlightTime_days = 4;
end
if ~isfield(candidate, 'returnFlightTime_days')
    candidate.returnFlightTime_days = 4;
end
if ~isfield(candidate, 'lunarOrbitDuration_days')
    candidate.lunarOrbitDuration_days = config.mission.lunarOrbitDuration_days;
end
if ~isfield(candidate, 'lunarOrbitAltitude_m')
    candidate.lunarOrbitAltitude_m = config.physics.moonParkingAltitude_m;
end
end

function result = blank_result(candidate, config)
result = struct('valid', false, 'score', config.mission.penaltyInvalid, 'candidate', candidate, 'rejectReason', 'Not evaluated', 'ephemerisSource', config.ephemeris.source);
end

function quality = lunar_orbit_quality(candidate, periapsisRadius_m, inboundVInf_mps, config)
altitudeQuality = 1 - abs(candidate.lunarOrbitAltitude_m - config.physics.moonParkingAltitude_m) / max(config.mission.maximumLunarPeriapsisAltitude_m, 1);
energyQuality = 1 / (1 + inboundVInf_mps / 3000);
durationQuality = min(1, candidate.lunarOrbitDuration_days / max(config.mission.lunarOrbitDuration_days, eps));
quality = max(0, min(1, 0.45 * altitudeQuality + 0.35 * energyQuality + 0.20 * durationQuality));
end

function reason = first_failed_constraint(constraints)
names = fieldnames(constraints);
reason = 'Mission constraints rejected candidate';
for index = 1:numel(names)
    if ~constraints.(names{index})
        reason = ['Constraint failed: ', names{index}];
        return
    end
end
end
