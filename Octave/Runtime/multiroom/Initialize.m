%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% initialization settings
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
address_hass = '192.168.0.205:8123';
Secrets
heat_line = 0.5; %heat line used in gas heater / thermostat

num_rooms = 3; %how many rooms from Roomconfig.m to use
Ts = 5 * 60;

delay_valve_off2on=80; %time to wait between turning valve on and requesting setpoint to heater

timestep = Ts;
predictionhorizon = ceil(2.5 * 3600 / Ts); %this is not a parameter in the c-code functions, regenerate c-code if changed
simulationhorizon = 1; % ceil(0.1 * 3600 / Ts); %additional simulation time after prediction (e.g. inputs kept constant during this time)

% load configurations
[room_configs, all_model_params_observer, all_model_params_no_observer] = Roomconfig(num_rooms);

if ~execute_compiledinsteadofmatlab
    [ode_observer, ode_no_observer, states, controls_observer, controls_no_observer, params_observer, params_no_observer, intg_observer, intg_no_observer] = generate_room_model(num_rooms, room_configs, Ts);
else
    load('model_config'); %generated in generate_room_model
end

% generate various lists of entities to interact with home assistant
entities_controls_observer = generate_ordered_entity_list(controls_observer, room_configs);

sensor_data{1} = 'toutside';
entities_out = generate_ordered_entity_list(sensor_data, room_configs);

sensor_data = {};
for i = 0:num_rooms - 1
    base_entity = 'tairsetpoint';
    if i >= 1
        sensor_data{i + 1} = [base_entity, '_', num2str(i)];
    else
        sensor_data{i + 1} = base_entity;
    end

end

entities_setp = generate_ordered_entity_list(sensor_data, room_configs);

%unique valves (if there are two valve-relais for the same room, only one is unique)
valve_data = {};

for i = 0:num_rooms - 1
    base_entity = 'valve';

    if i >= 1
        valve_data{i + 1} = [base_entity, '_', num2str(i)];
    else
        valve_data{i + 1} = base_entity;
    end

end

entities_valves = generate_ordered_entity_list(valve_data, room_configs);

watertgt_data{1} = 'airtemp_tgt';
entities_watertgt = generate_ordered_entity_list(watertgt_data, room_configs);
