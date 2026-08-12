function input = earth_moon_trial_input(inputs, trialIndex)
names = fieldnames(inputs);
input = struct;
for nameIndex = 1:numel(names)
    name = names{nameIndex};
    input.(name) = inputs.(name)(trialIndex);
end
end
