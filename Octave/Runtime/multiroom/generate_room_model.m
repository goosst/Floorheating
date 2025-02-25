function [ode_observer, ode_no_observer, states, controls_observer, controls_no_observer, params_observer, params_no_observer, intg_observer, intg_no_observer] = generate_room_model(n_rooms, room_configs, timestep)
% generate the differential equations, parameters, inputs, ...of the mathematical model of the floorheating in casadi format

% Function to merge two structs
    function out = merge_structs(s1, s2)
        out = s1;
        fields = fieldnames(s2);

        for i = 1:length(fields)
            out.(fields{i}) = s2.(fields{i});
        end

    end

    % Sigmoid function definition
    function y = sigmoid(x)
        y = 1 / (1 + exp(-6 * (x - 15)));
    end

    % Initialize empty arrays
    states = [];
    controls_observer = [];
    controls_no_observer = [];
    params_observer = [];
    params_no_observer = [];
    rhs_observer = [];
    rhs_no_observer = [];

    % Common control inputs

    %make room names consistent with casadi model
    namescommon = fieldnames(room_configs{end});

    index = find(strcmp(namescommon, 'twatersetp'));
    twatersetp = MX.sym(char(namescommon{index}));

    index = find(strcmp(namescommon, 'toutside'));
    toutside = MX.sym(char(namescommon{2}));

    common_controls = [twatersetp; toutside];

    % Generate model for each room
    for i = 0:(n_rooms - 1)
        % Generate suffix for variable names
        if i == 0
            suffix = '';
        else
            suffix = ['_', num2str(i)];
        end

        namesrooms = fieldnames(room_configs{i + 1});

        % States

        tfloor_i = MX.sym(['tfloor', suffix]);
        tair_i = MX.sym(['tair', suffix]);

        states = [states; tfloor_i; tair_i];

        % Room-specific controls
        %        tairmeas_i = MX.sym(['tairmeas', suffix]);
        %        valve_i = MX.sym(['valve', suffix]);

        %        clear index;
        index = find(strcmp(namesrooms, 'tairmeas'));
        tairmeas_i = MX.sym([char(namesrooms{index}), suffix]);

        %        clear index;
        index = find(strcmp(namesrooms, 'valve'));
        valve_i = MX.sym([char(namesrooms{index}), suffix]);

        % Common parameters for both models
        mCfloor_i = MX.sym(['mCfloor', suffix]);
        mCair_i = MX.sym(['mCair', suffix]);
        hair_i = MX.sym(['hair', suffix]);
        hisol_i = MX.sym(['hisol', suffix]);
        kwfl_i = MX.sym(['kwfl', suffix]);

        % Add common parameters to both parameter vectors
        common_params = [mCfloor_i; mCair_i; hair_i; hisol_i; kwfl_i];
        params_no_observer = [params_no_observer; common_params];

        %optional floor temperature sensor
        index_floor = find(strcmp(namesrooms, 'tfloormeas'));

        if ~isempty(index_floor)
            tfloormeas_i = MX.sym([char(namesrooms{index_floor}), suffix]);
            % Add to observer controls (includes measured values)
            controls_observer = [controls_observer; tairmeas_i; valve_i; tfloormeas_i];

            % Observer gains (only for observer model)
            Lfl1_i = MX.sym(['Lfl1', suffix]);
            Lair1_i = MX.sym(['Lair1', suffix]);
            Lfl2_i = MX.sym(['Lfl2', suffix]);
            Lair2_i = MX.sym(['Lair2', suffix]);

            params_observer = [params_observer; common_params; Lfl1_i; Lair1_i; Lfl2_i; Lair2_i];

        else
            % Add to observer controls (includes measured values)
            controls_observer = [controls_observer; tairmeas_i; valve_i];

            % Observer gains (only for observer model)
            L1_i = MX.sym(['L1', suffix]);
            L2_i = MX.sym(['L2', suffix]);

            % Add all parameters including L gains to observer parameters
            params_observer = [params_observer; common_params; L1_i; L2_i];

        end

        % Add to no-observer controls (excludes tairmeas)
        controls_no_observer = [controls_no_observer; valve_i];

        % Model with observer (L terms)
        if isempty(index_floor)
            %only air temperature measurement, no floor temperature
            rhs_floor_obs = (kwfl_i * valve_i * twatersetp * sigmoid(twatersetp)) / mCfloor_i + ...
                ((-kwfl_i * valve_i * sigmoid(twatersetp) - hair_i) * tfloor_i) / mCfloor_i + ...
                L1_i * (tairmeas_i - tair_i) + ...
                (hair_i * tair_i) / mCfloor_i;

            rhs_air_obs = (hisol_i * toutside) / mCair_i + ...
                (hair_i * tfloor_i) / mCair_i + ...
                L2_i * (tairmeas_i - tair_i) + ...
                ((-hisol_i - hair_i) * tair_i) / mCair_i;

        else
            % air and floor temperature
            rhs_floor_obs = (kwfl_i * valve_i * twatersetp * sigmoid(twatersetp)) / mCfloor_i + ...
                ((-kwfl_i * valve_i * sigmoid(twatersetp) - hair_i) * tfloor_i) / mCfloor_i + ...
                Lfl1_i * (tfloormeas_i - tfloor_i) + Lair1_i * (tairmeas_i - tair_i) + ...
                (hair_i * tair_i) / mCfloor_i;

            rhs_air_obs = (hisol_i * toutside) / mCair_i + ...
                (hair_i * tfloor_i) / mCair_i + ...
                Lfl2_i * (tfloormeas_i - tfloor_i) + Lair2_i * (tairmeas_i - tair_i) + ...
                ((-hisol_i - hair_i) * tair_i) / mCair_i;

        end

        % Model without observer (reformulated version)
        rhs_floor_no_obs = 1 / mCfloor_i * (kwfl_i * sigmoid(twatersetp) * valve_i * (twatersetp - tfloor_i) + ...
            hair_i * (tair_i - tfloor_i));

        rhs_air_no_obs = 1 / mCair_i * (hair_i * (tfloor_i - tair_i) + ...
            hisol_i * (toutside - tair_i));

        % Append equations to RHS
        rhs_observer = [rhs_observer; rhs_floor_obs; rhs_air_obs];
        rhs_no_observer = [rhs_no_observer; rhs_floor_no_obs; rhs_air_no_obs];
    end

    % Add common controls to both control vectors
    controls_observer = [common_controls; controls_observer];
    controls_no_observer = [common_controls; controls_no_observer];

    % Create ODE structs for both versions
    ode_observer = struct;
    ode_observer.x = states;
    ode_observer.u = controls_observer;
    ode_observer.ode = rhs_observer;
    ode_observer.p = params_observer;

    ode_no_observer = struct;
    ode_no_observer.x = states;
    ode_no_observer.u = controls_no_observer;
    ode_no_observer.ode = rhs_no_observer;
    ode_no_observer.p = params_no_observer;

    % create integrators
    opts = struct('simplify', true); % Code generation settings
    integrator_opts = struct(); % Initialize integrator options
    merged_opts = merge_structs(integrator_opts, opts);

    % Create the integrator
    t0 = 0; % Start time
    tf = timestep; % End time (single time step)
    intg_observer = integrator('intg_observer', 'rk', ode_observer, t0, tf, merged_opts); % Runge-Kutta

    % Create the integrator
    t0 = 0; % Start time
    tf = timestep; % End time (single time step)
    intg_no_observer = integrator('intg_no_observer', 'rk', ode_no_observer, t0, tf, merged_opts); % Runge-Kutta

    controls_observer = {};

    for i = 1:length(ode_observer.u)
        temp = ode_observer.u{i};
        controls_observer{end + 1} = temp.name;
    end

    controls_no_observer = {};

    for i = 1:length(ode_no_observer.u)
        temp = ode_no_observer.u{i};
        controls_no_observer{end + 1} = temp.name;
    end

    save ('model_config', "controls_observer", "controls_no_observer")

end
