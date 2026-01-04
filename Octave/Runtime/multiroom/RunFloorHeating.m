%% requires following octave,octave-dev, pkg install -forge control
clc
clear variables
close all
dbstop if error

diary('debug_Floorheating.txt');
diary on

addpath('../')

execute_compiledinsteadofmatlab = true; %run copmiled c-code instead of octave / matlab, to generate c-code put this to false
applysetpoints = false; %send calcaulted outputs (valve, temperatures, ...) to home assistant

if ~execute_compiledinsteadofmatlab
    % casadi should not be a dependency if c-code / compiled c-code is used
%    addpath('/home/stijn/Projects/Casadi/casadi-3.6.7-linux64-octave7.3.0')
    addpath('/home/stijn/Projects/Casadi/casadi-3.7.2-linux64-octave7.3.0')
    import casadi.*
else
    %remove all casadi references from path for standalone testing purposes
    try
        rmpath(genpath('/home/stijn/Projects/Casadi'));
        clc
    end

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% initialization settings
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Initialize

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% step function
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
firstrun = true; updatehass = false;
starttime = time(); %time in seconds

airtemp_setpoints = zeros(num_rooms, 1);
state = zeros(num_rooms * 2, 1);
state(:, :) = NaN;

while true

    % a crappy time scheduler, don't know if octave has better options
    pause(10);
    currenttime = time();

    % react faster if a temperature setpoint was changed
    [~, airtemp_setpoints_new] = getLatestStates(address_hass, auth_token, entities_setp);

    % make sure all measurements made it to home assistant before starting the model predictive conroller, this can be done in advance
    % it's thermal and slow, there are no hard realtime constraints
    if ((currenttime - starttime >= Ts - 60 || firstrun) && updatehass == false)
        PostHomeAssistant(address_hass, auth_token, 'shell_command.read_ebus', 0);
        updatehass = true;
        close all
    end

    % Model predictive control loop
    if (currenttime - starttime >= Ts || firstrun || any(airtemp_setpoints != airtemp_setpoints_new))

        starttime = time();
        updatehass = false;

        clc
        diary on

        disp(['run ', datestr(now, 0)])

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % simulate model with observer
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % download data for observer
        [most_recent_states, controlinputs] = getLatestStates(address_hass, auth_token, entities_controls_observer);

        % state initialization / error handling
        if any(isnan(state))
            firstrun = false;
            for i = 0:num_rooms - 1
                if i >= 1
                    sensor_air{i + 1} = ['tairmeas_', num2str(i)];
                else
                    sensor_air{i + 1} = 'tairmeas';
                end
            end

            entities_airtemp = generate_ordered_entity_list(sensor_air, room_configs);
            [most_recent_states, airtemps] = getLatestStates(address_hass, auth_token, entities_airtemp);

            for i = 1:num_rooms
                namesrooms = fieldnames(room_configs{i});
                index_floor = find(strcmp(namesrooms, 'tfloormeas'));

                if (isnan(state(2 * i - 1)) || isnan(state(2 * i )))

                    if ~isempty(index_floor)
                        entity_floor = generate_ordered_entity_list({'tfloormeas'}, room_configs);
                        [most_recent_states, floortemp] = getLatestStates(address_hass, auth_token, entity_floor);

                        state(2 * i - 1) = floortemp;
                    else
                        state(2 * i - 1) = airtemps(i);
                    end

                    state(2 * i) = airtemps(i);
                end

            end

            %            tfl = airtemps(1);
            %            tair = airtemps(1);
            %            tfl_1 = airtemps(2);
            %            tair_1 = airtemps(2);
            %            state = [[tfl; tair]; [tfl_1; tair_1]]; %vertcat(tfloor, tair, tfloor_1, tair_1)

        end

        % signal check
        if any(isnan(state)) || any(isnan(controlinputs))
            continue;
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % update states of model
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        if ~execute_compiledinsteadofmatlab
            state = nextTimeStep(ode_observer, intg_observer, all_model_params_observer, controlinputs, state, timestep);

        else

            disp("state")
            disp(state);
            disp("controlinputs")
            disp(controlinputs);
            input_string = sprintf('%f\n', [state; all_model_params_observer; controlinputs]);
            functionname = 'intg_observer';
            %               functionname = intg.name();
            command = sprintf(['echo "%s" | ./integrator_c_code ', functionname], input_string);

            [status, output] = system(command);
            values = str2num(output);
            state = values';

        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
        % optimal controls
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5

        % get data needed for optimizer
        [most_recent_states, outdoortemp] = getLatestStates(address_hass, auth_token, entities_out);
        [most_recent_states, airtemp_setpoints] = getLatestStates(address_hass, auth_token, entities_setp);

        disp('outdoortemp');
        disp(outdoortemp);
        disp('airtemp_setpoints');
        disp(airtemp_setpoints);

        if ~execute_compiledinsteadofmatlab

            [most_recent_states, valve_states] = getLatestStates(address_hass, auth_token, entities_valves);
            setpoint = OptimalControlMultiRoom(ode_no_observer, intg_no_observer, all_model_params_no_observer, state, outdoortemp, airtemp_setpoints, predictionhorizon, simulationhorizon, num_rooms, valve_states,watertemp_max);

            optimized_watersetpoints = setpoint.watersetp;
            optimized_valvesetpoints = setpoint.valvesetp;
        else

            % optimizer run 1, calculates initial guesses for run 2
            disp('run 1')

            default_tfloor_init = 20 * ones(predictionhorizon + 1, num_rooms);
            default_tair_init = 20 * ones(predictionhorizon + 1, num_rooms);

            % do some initial guesses for optimizer
            [most_recent_states, valve_states] = getLatestStates(address_hass, auth_token, entities_valves);
            for i = 1:num_rooms
                if airtemp_setpoints(i) <= state(2 * i) - 3
                    % setpoint is significantly below air temperature
                    default_valve_init(:, i) = zeros(predictionhorizon + 1, 1);
                elseif airtemp_setpoints(i) > state(2 * i)
                    % setpoint is higher than air temperature
                    default_valve_init(:, i) = ones(predictionhorizon + 1, 1);
                else
                    default_valve_init(:, i) = valve_states(i) * ones(predictionhorizon + 1, 1);
                end
            end

            % heating request initial guess
            if any(airtemp_setpoints > state(2:2:end))
                default_water_init = 25 * ones(predictionhorizon + 1, 1);
            else
                default_water_init = 0 * ones(predictionhorizon + 1, 1);
            end

            input_string = sprintf(['%f\n%f\n%f\n' ...
                                        repmat('%f\n', 1, predictionhorizon + 1), ... % water init
                                        repmat('%f\n', 1, (predictionhorizon + 1) * num_rooms), ... % valve init
                                        repmat('%f\n', 1, (predictionhorizon + 1) * num_rooms), ... % floor init
                                        repmat('%f\n', 1, (predictionhorizon + 1) * num_rooms)], ... % air init
                state, outdoortemp, airtemp_setpoints, ...
                default_water_init(:), default_valve_init(:), ...
                default_tfloor_init(:), default_tair_init(:));

            command = sprintf('echo "%s" | ./optimal_control_pass1 optimal_control_pass1', input_string);
            [status, output] = system(command);
            optimalvalues_compiledcode = str2num(output);

            vector_length = predictionhorizon + 1;

            optimized_watersetpoints = optimalvalues_compiledcode(1:vector_length);
            optimized_valvesetpoints = []; optimized_tfloor = []; optimized_tair = [];

            for i = 1:num_rooms * 3

                if i <= num_rooms
                    optimized_valvesetpoints = [optimized_valvesetpoints, optimalvalues_compiledcode(i * vector_length + 1:(i + 1) * vector_length)'];
                elseif i > num_rooms && i <= num_rooms * 2
                    optimized_tfloor = [optimized_tfloor, optimalvalues_compiledcode(i * vector_length + 1:(i + 1) * vector_length)'];
                else
                    optimized_tair = [optimized_tair, optimalvalues_compiledcode(i * vector_length + 1:(i + 1) * vector_length)'];
                end

            end

            %            disp("pass1 water setpoints ")
            %            disp(optimized_watersetpoints);
            %
            %            disp("pass1 valve setpoints ")
            %            disp(optimized_valvesetpoints);
            %
            %            disp("pass 1 air state")
            %            disp(optimized_tair);
            %            plotoptimizer(optimized_watersetpoints, optimized_valvesetpoints, optimized_tfloor, optimized_tair, airtemp_setpoints, num_rooms);

            % run 2 - with binary valve constraints applied
            disp('run 2')
            input_string = sprintf(['%f\n%f\n%f\n' ...
                                        repmat('%f\n', 1, predictionhorizon + 1), ... % water init
                                        repmat('%f\n', 1, (predictionhorizon + 1) * num_rooms), ... % valve init
                                        repmat('%f\n', 1, (predictionhorizon + 1) * num_rooms), ... % floor init
                                        repmat('%f\n', 1, (predictionhorizon + 1) * num_rooms)], ... % air init
                state, outdoortemp, airtemp_setpoints, ...
                optimized_watersetpoints(:), optimized_valvesetpoints(:), ...
                optimized_tfloor(:), optimized_tair(:));

            %pass inputs
            command = sprintf('echo "%s" | ./optimal_control_pass2 optimal_control_pass2', input_string);
            [status, output] = system(command);

            optimalvalues_compiledcode = str2num(output);

            optimized_watersetpoints = optimalvalues_compiledcode(1:predictionhorizon + 1);
            optimized_valvesetpoints = []; optimized_tfloor = []; optimized_tair = [];

            for i = 1:num_rooms * 3

                if i <= num_rooms
                    optimized_valvesetpoints = [optimized_valvesetpoints, optimalvalues_compiledcode(i * vector_length + 1:(i + 1) * vector_length)'];
                elseif i > num_rooms && i <= num_rooms * 2
                    optimized_tfloor = [optimized_tfloor, optimalvalues_compiledcode(i * vector_length + 1:(i + 1) * vector_length)'];
                else
                    optimized_tair = [optimized_tair, optimalvalues_compiledcode(i * vector_length + 1:(i + 1) * vector_length)'];
                end

            end

            %            plotoptimizer(optimized_watersetpoints, optimized_valvesetpoints, optimized_tfloor, optimized_tair, airtemp_setpoints, num_rooms);

            %
            %            disp("pass2 water setpoints ")
            %            disp(optimized_watersetpoints);
            %
            %            disp("pass2 valve setpoints ")
            %            disp(optimized_valvesetpoints);
            %
            %            disp("pass2 air state")
            %            disp(optimized_tair);

        end

        disp("watersetpoint")
        disp(optimized_watersetpoints(1))

        % apply setpoints
        if optimized_watersetpoints(1) > 10
            ThermostatSetp = WaterSetpointToThermostatSetpoint(optimized_watersetpoints(1), outdoortemp, heat_line,watertemp_max);
        else
            ThermostatSetp = 0;
        end

        disp("thermostatsetp")
        disp(ThermostatSetp)

        valves_setp = round(optimized_valvesetpoints(1, :));

        % augment with circuits with multiple valves
        [valves_setp, entities_valves] = process_valve_configs(room_configs, valves_setp, entities_valves);

        disp("valvesetp")
        disp(valves_setp)

        % check current state of valves
        [most_recent_states, valve_states] = getLatestStates(address_hass, auth_token, entities_valves);

        if applysetpoints

            if sum(valves_setp) < 1e-4
                ThermostatSetp = 0; %if all valves are commanded off, force heating request off
            end

            if isequal(valves_setp, valve_states') || any(valves_setp & valve_states')
                % or no change or at least the same valve is kept open
                PostHomeAssistant(address_hass, auth_token, entities_watertgt{1}, ThermostatSetp);

                for i = 1:length(valves_setp)
                    PostHomeAssistant(address_hass, auth_token, entities_valves{i}, valves_setp(i));
                end

            elseif sum(valves_setp) < 1e-4 && sum(valve_states) > 1e-4
                % turn all valves off after being on before; allow time to shutdown pump first before applying valves
                PostHomeAssistant(address_hass, auth_token, entities_watertgt{1}, ThermostatSetp);

                pause(180); %a dumb way of delaying

                for i = 1:length(valves_setp)
                    PostHomeAssistant(address_hass, auth_token, entities_valves{i}, valves_setp(i));
                end

            else
                % if valve position change (e.g. from all blocking to 1 or more open, or a switch of rooms)
                % some more intelligence should be added here
                for i = 1:length(valves_setp)
                    PostHomeAssistant(address_hass, auth_token, entities_valves{i}, valves_setp(i));
                end

                pause(delay_valve_off2on);

                PostHomeAssistant(address_hass, auth_token, entities_watertgt{1}, ThermostatSetp);

            end

        else
            keyboard

        end

        PostHomeAssistant(address_hass, auth_token, 'shell_command.read_ebus', 0);

        diary off
    end

end
