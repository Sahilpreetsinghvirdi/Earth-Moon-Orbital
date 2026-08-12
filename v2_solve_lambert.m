function solution = v2_solve_lambert(position1_m, position2_m, timeOfFlight_s, mu_m3ps2, prograde)
if nargin < 5 || isempty(prograde)
    prograde = true;
end
position1_m = position1_m(:);
position2_m = position2_m(:);
radius1_m = norm(position1_m);
radius2_m = norm(position2_m);
solution = struct('valid', false, 'velocity1_mps', nan(3, 1), 'velocity2_mps', nan(3, 1), 'transferAngle_rad', nan, 'iterations', 0, 'residual_s', inf, 'message', '');
if radius1_m <= 0 || radius2_m <= 0 || timeOfFlight_s <= 0 || mu_m3ps2 <= 0
    solution.message = 'Invalid Lambert inputs';
    return
end
cosAngle = min(max(dot(position1_m, position2_m) / (radius1_m * radius2_m), -1), 1);
angle_rad = acos(cosAngle);
crossProduct = cross(position1_m, position2_m);
if prograde
    if crossProduct(3) < 0
        angle_rad = 2 * pi - angle_rad;
    end
else
    if crossProduct(3) >= 0
        angle_rad = 2 * pi - angle_rad;
    end
end
sinAngle = sin(angle_rad);
denominator = 1 - cos(angle_rad);
if abs(denominator) < 1e-12 || abs(sinAngle) < 1e-12
    solution.message = 'Degenerate transfer geometry';
    return
end
A_m = sinAngle * sqrt(radius1_m * radius2_m / denominator);
if abs(A_m) < 1e-6
    solution.message = 'Lambert geometry coefficient is singular';
    return
end
zGrid = linspace(-4 * pi ^ 2 + 1e-6, 4 * pi ^ 2 - 1e-6, 360);
functionValue = nan(size(zGrid));
for index = 1:numel(zGrid)
    functionValue(index) = time_residual(zGrid(index));
end
validIndices = find(isfinite(functionValue));
if isempty(validIndices)
    solution.message = 'No valid Lambert universal-variable domain';
    return
end
bracketIndex = [];
for index = validIndices(1:end-1)
    if functionValue(index) == 0 || functionValue(index) * functionValue(index + 1) < 0
        bracketIndex = index;
        break
    end
end
if isempty(bracketIndex)
    [~, closestIndex] = min(abs(functionValue(validIndices)));
    z = zGrid(validIndices(closestIndex));
else
    lower = zGrid(bracketIndex);
    upper = zGrid(bracketIndex + 1);
    lowerValue = functionValue(bracketIndex);
    for iteration = 1:80
        z = 0.5 * (lower + upper);
        midpointValue = time_residual(z);
        if ~isfinite(midpointValue)
            lower = 0.5 * (lower + z);
            continue
        end
        if abs(midpointValue) < 1e-7
            break
        end
        if lowerValue * midpointValue <= 0
            upper = z;
        else
            lower = z;
            lowerValue = midpointValue;
        end
    end
end
[y_m, C2, ~, valid] = geometry(z);
if ~valid || y_m <= 0
    solution.message = 'Lambert solution has nonpositive radial geometry';
    return
end
f = 1 - y_m / radius1_m;
g_s = A_m * sqrt(y_m / mu_m3ps2);
gDot = 1 - y_m / radius2_m;
if abs(g_s) < 1e-12 || ~isfinite(C2)
    solution.message = 'Lambert solution has singular f-g coefficients';
    return
end
velocity1_mps = (position2_m - f * position1_m) / g_s;
velocity2_mps = (gDot * position2_m - position1_m) / g_s;
solution.valid = all(isfinite([velocity1_mps; velocity2_mps]));
solution.velocity1_mps = velocity1_mps;
solution.velocity2_mps = velocity2_mps;
solution.transferAngle_rad = angle_rad;
solution.iterations = 80;
solution.residual_s = abs(time_residual(z));
if ~solution.valid
    solution.message = 'Nonfinite Lambert velocity solution';
else
    solution.message = 'Universal-variable Lambert solution';
end

    function residual_s = time_residual(zValue)
        [yValue, C2Value, C3Value, validValue] = geometry(zValue);
        if ~validValue || yValue <= 0 || C2Value <= 0
            residual_s = nan;
            return
        end
        residual_s = (yValue / C2Value) ^ 1.5 * C3Value + A_m * sqrt(yValue) - sqrt(mu_m3ps2) * timeOfFlight_s;
    end

    function [yValue, C2Value, C3Value, validValue] = geometry(zValue)
        [C2Value, C3Value] = stumpff(zValue);
        if C2Value <= 0 || ~isfinite(C2Value) || ~isfinite(C3Value)
            yValue = nan;
            validValue = false;
            return
        end
        yValue = radius1_m + radius2_m + A_m * (zValue * C3Value - 1) / sqrt(C2Value);
        validValue = isfinite(yValue);
    end
end

function [C2Value, C3Value] = stumpff(zValue)
if zValue > 1e-6
    root = sqrt(zValue);
    C2Value = (1 - cos(root)) / zValue;
    C3Value = (root - sin(root)) / root ^ 3;
elseif zValue < -1e-6
    root = sqrt(-zValue);
    C2Value = (1 - cosh(root)) / zValue;
    C3Value = (sinh(root) - root) / root ^ 3;
else
    C2Value = 1 / 2 - zValue / 24 + zValue ^ 2 / 720 - zValue ^ 3 / 40320;
    C3Value = 1 / 6 - zValue / 120 + zValue ^ 2 / 5040 - zValue ^ 3 / 362880;
end
end
