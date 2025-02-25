% replay multiroomdata
% this is needed to design your models and check correct functioning of chosen parameters from model and observer

clc
clear variables
close all
dbstop if error

diary('debug_Floorheating.txt');
diary on

addpath('../../Design')
addpath('..')

execute_compiledinsteadofmatlab = false; %run copmiled c-code instead of octave / matlab
applysetpoints = false; %send calcaulted outputs (valve, temperatures, ...) to home assistant

if ~execute_compiledinsteadofmatlab
    % casadi should not be a dependency if c-code / compiled c-code is used
    addpath('/home/stijn/Projects/Casadi/casadi-3.6.7-linux64-octave7.3.0')
    import casadi.*
else
    %remove all casadi references from path for standalone testing purposes
    try
        rmpath(genpath('/home/stijn/Projects/Casadi'));
        clc
    end

end

Initialize
entities_controls_no_observer = generate_ordered_entity_list(controls_no_observer, room_configs);

entity_id = '';

for i = 1:size(entities_controls_no_observer, 2)

    entity_id = strcat(entity_id, entities_controls_no_observer{i}, ',');
end

entity_id = entity_id(1:end - 1);

[entity_data, hass_time, most_recent_states] = DownloadHomeAssistant(address_hass, auth_token, entity_id);
resampled_data = ResampleData(entity_data, Ts);
fields = fieldnames(resampled_data);

% initialize state
for i = 0:num_rooms - 1

    if i >= 1
        sensor_air{i + 1} = ['tairmeas_', num2str(i)];
    else
        sensor_air{i + 1} = 'tairmeas';
    end

end

entities_airtemp = generate_ordered_entity_list(sensor_air, room_configs);

[most_recent_states, airtemps] = getLatestStates(address_hass, auth_token, entities_airtemp);
state = zeros(num_rooms * 2, 1);
state(:, :) = NaN;

for i = 1:num_rooms * 2

    if isnan(state(i))
        state(i) = airtemps(ceil(i / 2));
    end

end

% without observer
state_floor_total = [];
state_air_total = [];

for i = 1:length(resampled_data.(fields{1}).times)

    controlinputs = [];

    for j = 1:length(fields)
        controlinputs = [controlinputs; resampled_data.(fields{j}).data(i)];
    end

    state = nextTimeStep(ode_no_observer, intg_no_observer, all_model_params_no_observer, controlinputs, state, timestep);

    state_floor_total = [state_floor_total; state(1)];
    state_air_total = [state_air_total; state(2)];
end

% with observer

state(:, :) = NaN;

for i = 1:num_rooms * 2

    if isnan(state(i))
        state(i) = airtemps(ceil(i / 2));
    end

end

entities_controls_observer = generate_ordered_entity_list(controls_observer, room_configs);

entity_id = '';
for i = 1:size(entities_controls_observer, 2)
    entity_id = strcat(entity_id, entities_controls_observer{i}, ',');
end
entity_id = entity_id(1:end - 1);

[entity_data, hass_time, most_recent_states] = DownloadHomeAssistant(address_hass, auth_token, entity_id);
resampled_data = ResampleData(entity_data, Ts);
fields = fieldnames(resampled_data);

state_floor_total_obs = [];
state_air_total_obs = [];

for i = 1:length(resampled_data.(fields{1}).times)

    controlinputs = [];

    for j = 1:length(fields)
        controlinputs = [controlinputs; resampled_data.(fields{j}).data(i)];
    end

    state = nextTimeStep(ode_observer, intg_observer, all_model_params_observer, controlinputs, state, timestep);

    state_floor_total_obs = [state_floor_total_obs; state(1)];
    state_air_total_obs = [state_air_total_obs; state(2)];
end


entity_id = '';
%for i = 1:size(entities_airtemp, 2)
%    entity_id = strcat(entity_id, entities_airtemp{i}, ',');
%end
%entity_id = entity_id(1:end - 1);

entity_id = strcat(entity_id,entities_airtemp{1},',','sensor.floor_temperature')
[entity_data, hass_time, most_recent_states] = DownloadHomeAssistant(address_hass, auth_token, entity_id);
resampled_data = ResampleData(entity_data, Ts);
fields = fieldnames(resampled_data);


figure
hold on

plot(state_floor_total, 'DisplayName', 'estimated floor')
pause(1)
plot(state_air_total, 'DisplayName', 'estimated air')
pause(1)
plot(state_air_total_obs, '*', 'DisplayName', 'estimated air - observer correction')
pause(1)
plot(state_floor_total_obs, '*', 'DisplayName', 'estimated floor - observer correction')
pause(1)
plot(resampled_data.(fields{1}).data, '*', 'DisplayName', fields{1})
pause(1)
plot(resampled_data.(fields{2}).data, '*', 'DisplayName', fields{2})
pause(1)

legend
grid minor
