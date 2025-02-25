function setpoint = OptimalControlMultiRoom(ode, intg, model_params, state, outdoortemp, airtemp_setpoints, predictionhorizon, simulationhorizon, num_rooms,valve_states)
    % find optimal setpoints of inputs (valves, water etc.) to control floor heating
    % two iterations of solving have to happen to handle the discrete nature of valves (0 or 1)
    % (or at least I did not find a reliable way to do it in one go with ipopt)
    % function can generate code as well to be able to run this on a standalone board

    codegeneration = true;

    % Penalty factors for the cost function
    %    penalty_watertemp = 10 ^ (-3);
    %    penalty_valve = 10 ^ (-3);
    %    penalty_changewater = 10 ^ (-4);
    %    penalty_changevalve = 10 ^ (-4);

    watertemp_max=37;
    penalty_watertemp = 10 ^ (-3);
    penalty_valve = 10 ^ (-3);
    penalty_changewater = 10 ^ (-3);
    penalty_changevalve = 0;

    %    penalty_watertemp = 0;
    %    penalty_valve = 0;
    %    penalty_changewater = 0;
    %    penalty_changevalve =0;

    % Create Opti instance
    opti = Opti();

    % Parameters, defined as parameters for c-code generation purposes
    state_init = opti.parameter(2 * num_rooms, 1); % Initial states for all rooms
    outdoortemp_opti = opti.parameter(); % future improvements could be to have some weather prediction included
    setpoints_opti = opti.parameter(num_rooms, 1); % Air temperature setpoints for each room

    % Decision variables
    watersetpoints = opti.variable(predictionhorizon + 1, 1); % Shared water setpoints for all rooms
    valvesetpoints = opti.variable(predictionhorizon + 1, num_rooms); % One valve per room
    states_tfloor = opti.variable(predictionhorizon + 1, num_rooms);
    states_tair = opti.variable(predictionhorizon + 1, num_rooms);

    % Initial state constraint
    opti.subject_to(states_tfloor(1, :) == state_init(1:2:end)');
    opti.subject_to(states_tair(1, :) == state_init(2:2:end)');

    % Initialize cost
    J = 0;

    % Loop through prediction horizon, using multishooting approach
    for k = 1:predictionhorizon
        % reshape stuff to get state
        concatenated = [states_tfloor(k, :); states_tair(k, :)];
        n = prod(size(concatenated)); % For CasADi objects
        current_state = reshape(concatenated, 1, n);

        % Integrate to get the next state
        control_inputs = [watersetpoints(k); outdoortemp_opti; valvesetpoints(k, :)']; % Combine all inputs
        result = intg('x0', current_state, 'u', control_inputs, 'p', model_params);
        next_state = result.xf;

        % Continuity constraints for multiple shooting
        %        opti.subject_to(states(k + 1, :)' == next_state);

        opti.subject_to(states_tfloor(k + 1, :) == next_state(1:2:end)');
        opti.subject_to(states_tair(k + 1, :) == next_state(2:2:end)');

        % temperature cost for all rooms
        for room = 1:num_rooms
            J = J + (states_tair(k, room) - setpoints_opti(room)) ^ 2;
        end

        % Penalty for (changing) control inputs
        J = J + penalty_watertemp * watersetpoints(k) ...
            + penalty_changewater * (watersetpoints(k) - watersetpoints(k + 1)) ^ 2;

        for room = 1:num_rooms
            J = J ...
                + penalty_valve * valvesetpoints(k, room) * watersetpoints(k) ...
                + penalty_changevalve * (valvesetpoints(k + 1, room) - valvesetpoints(k, room)) ^ 2;
        end

    end

    % Handle simulation horizon with final constant control inputs
    watersetpoint_final = watersetpoints(end);
    valvesetpoint_final = valvesetpoints(end, :);

    concatenated = [states_tfloor(end, :); states_tair(end, :)];
    n = prod(size(concatenated));
    current_state = reshape(concatenated, 1, n);

    for k = 1:simulationhorizon
        control_inputs = [watersetpoint_final; outdoortemp_opti; valvesetpoint_final'];
        result = intg('x0', current_state, 'u', control_inputs, 'p', model_params);
        current_state = result.xf;

        tair_final = next_state(2:2:end);
        % temperature cost for all rooms
        for room = 1:num_rooms
            J = J + (tair_final(room) - setpoints_opti(room)) ^ 2 ...
                + penalty_watertemp * watersetpoint_final;
            + penalty_valve * valvesetpoint_final(room) * watersetpoint_final
        end

    end

    % Add bounds for control inputs
    opti.subject_to(0 <= watersetpoints <= watertemp_max);
    opti.subject_to(0 <= valvesetpoints <= 1);

    % Add state bounds
    opti.subject_to(0 <= states_tfloor <= 45);
    opti.subject_to(0 <= states_tair <= 40);

    % Binary valve constraints, valve can only be open or closed
    % for some reason this doesn't work when applied from the first time, the initial conditions need to be very good
    %            opti.subject_to(valvesetpoints .* (valvesetpoints - 1) == 0);

    % Valve control constraints with water setpoints
    % if valve is zero force watersetpoint to zero
    for k = 1:predictionhorizon + 1
        opti.subject_to(50 * num_rooms * (sum(valvesetpoints(k, :))) >= watersetpoints(k));
    end

    % Set objective
    opti.minimize(J);

    % Solver options
    solver_options = struct();
    solver_options.ipopt.print_level = 5;
    solver_options.ipopt.file_print_level = 0;
    solver_options.print_time = true;
    solver_options.ipopt.max_iter = 1000; % Maximum number of iterations
    opti.solver('ipopt', solver_options);

    % Set parameter values
    opti.set_value(state_init, state);
    opti.set_value(outdoortemp_opti, outdoortemp);
    opti.set_value(setpoints_opti, airtemp_setpoints);

    % Initial guesses
    %    default_water_init = 0 * ones(predictionhorizon + 1, 1);
    %    default_valve_init = 0.5 * ones(predictionhorizon + 1, num_rooms);
    default_tfloor_init = 20 * ones(predictionhorizon + 1, num_rooms);
    default_tair_init = 20 * ones(predictionhorizon + 1, num_rooms);

    for i = 1:num_rooms

        if airtemp_setpoints(i) <= state(2 * i) - 3
            % setpoint is significantly below air temperature
            default_valve_init(:, i) = zeros(predictionhorizon + 1, 1);
        elseif airtemp_setpoints(i) > state(2 * i)
            % setpoint is higher than air temperature
            default_valve_init(:, i) = ones(predictionhorizon + 1, 1);
        else
            %            default_valve_init(:, i) = 0.5 * ones(predictionhorizon + 1, 1);
            default_valve_init(:, i) = valve_states(i) * ones(predictionhorizon + 1, 1);

        end

    end

    % water temp initial guess
    if any(airtemp_setpoints > state(2:2:end))
        default_water_init = 25 * ones(predictionhorizon + 1, 1);
    else
        default_water_init = 0 * ones(predictionhorizon + 1, 1);

    end

    % Initial guesses
    %        default_water_init = 35 * ones(predictionhorizon + 1, 1);
    %        default_valve_init = 1 * ones(predictionhorizon + 1, num_rooms);
    %        default_tfloor_init = 30 * ones(predictionhorizon + 1, num_rooms);
    %        default_tair_init = 30 * ones(predictionhorizon + 1, num_rooms);

    opti.set_initial(watersetpoints, default_water_init);
    opti.set_initial(valvesetpoints, default_valve_init);
    opti.set_initial(states_tfloor, default_tfloor_init);
    opti.set_initial(states_tair, default_tair_init);

    opti_common = Opti();
    opti_common = opti.copy();

    % the heater cannot cool, water should be warmer than the floor
    for i = 1:num_rooms
        %    opti.subject_to(watersetpoints >= (valvesetpoints(:, i)>0.2) .* states_tfloor(:, i));
        x = valvesetpoints(:, i);
        y = 1 ./ (1 + exp(-50 * (x - 0.1)));
        opti.subject_to(watersetpoints >= y .* states_tfloor(:, i));
    end

    % Solve optimization

    solution = opti.solve();

    % Extract optimized values
    optimized_watersetpoints = solution.value(watersetpoints);
    optimized_valvesetpoints = solution.value(valvesetpoints);
    optimized_tfloor = solution.value(states_tfloor);
    optimized_tair = solution.value(states_tair);

    plotoptimizer(optimized_watersetpoints, optimized_valvesetpoints, optimized_tfloor, optimized_tair, airtemp_setpoints, num_rooms)

    %    keyboard
    %
    %        end
    %    rms(optimized_tair(:, 1))
    %    rms(optimized_tair(:, 2))

    %    keyboard

    if (codegeneration)

        solver_options = struct();
        solver_options.ipopt.print_level = 0; % Suppress IPOPT console output
        solver_options.ipopt.file_print_level = 0; % Suppress IPOPT file output
        solver_options.ipopt.sb = 'yes'; % suppress banner
        solver_options.print_time = false; % Suppress CasADi timing information
        solver_options.error_on_fail = false;
        solver_options.ipopt.max_iter = 3000;
        opti.solver('ipopt', solver_options);


        % Create CasADi function for C code generation: inputs (opti parmaeters) in first part, outputs in second part
        %  c_function = opti.to_function('optimal_control_pass1', {state_init, outdoortemp_opti, setpoints_opti}, {watersetpoints(1), valvesetpoints(1, :)});

        %c_function = opti.to_function('optimal_control_pass1', ...
        %            {state_init, outdoortemp_opti, setpoints_opti, ...
        %             watersetpoints, valvesetpoints, states_tfloor, states_tair}, ...
        %            {watersetpoints(1), valvesetpoints(1, :)});

        c_function = opti.to_function('optimal_control_pass1', ...
            {state_init, outdoortemp_opti, setpoints_opti, ... % opti parameters
             watersetpoints, valvesetpoints, states_tfloor, states_tair}, ... % initialization values opti.variables
            {watersetpoints, valvesetpoints, states_tfloor, states_tair}); % outputs of c-code function

        % Set options for C code generation, including main and verbose
        opts = struct();
        opts.main = true; % Include a main function
        opts.verbose = true;
        opts.with_header = false; % I don't know, maybe

        % Generate C code with the specified options
        c_function.generate('optimal_control_pass1.c', opts);

        % compile c-code
        DIR = GlobalOptions.getCasadiPath();
        str_include = GlobalOptions.getCasadiIncludePath();
        str_compile = strcat('gcc -L', DIR, ' -Wl,-rpath,', DIR, ' -I', str_include, ' optimal_control_pass1.c -lm -lipopt -o optimal_control_pass1')
        %str_compile=strcat('gcc -L',DIR,' -I',str_include,' optimal_control.c -lm -lipopt -o optimal_control')
        system(str_compile)

        % test the compiled code
        % Prepare the input as a string
        %  input_string = sprintf('%f\n%f\n%f\n%f\n', state,outdoortemp,airtemp_setpoints);
        %

        % Test code - create regular numeric arrays for initial guesses

        % Update input string for testing - using regular numeric arrays
        input_string = sprintf(['%f\n%f\n%f\n' ...
                                    repmat('%f\n', 1, predictionhorizon + 1), ... % water init
                                    repmat('%f\n', 1, (predictionhorizon + 1) * num_rooms), ... % valve init
                                    repmat('%f\n', 1, (predictionhorizon + 1) * num_rooms), ... % floor init
                                    repmat('%f\n', 1, (predictionhorizon + 1) * num_rooms)], ... % air init
            state, outdoortemp, airtemp_setpoints, ...
            default_water_init(:), default_valve_init(:), ...
            default_tfloor_init(:), default_tair_init(:));

        %  % Use pipes to pass input
        command = sprintf('echo "%s" | ./optimal_control_pass1 optimal_control_pass1', input_string);
        [status, output] = system(command);

        %compare outputs with octave-casadi
        optimalvalues_compiledcode = str2num(output);

        % quick checks
        disp('check difference between octave and c-code');
        optimalvalues_compiledcode(1:predictionhorizon + 1) - optimized_watersetpoints' < 0.1
        optimalvalues_compiledcode(predictionhorizon + 2:2 * predictionhorizon + 2) - optimized_valvesetpoints(:, 1)' < 0.1
        optimalvalues_compiledcode(2 * predictionhorizon + 3:3 * predictionhorizon + 3) - optimized_valvesetpoints(:, 2)' < 0.1

        figure
        hax1 = subplot(3, 1, 1)
        hold on
        plot(optimalvalues_compiledcode(1:predictionhorizon + 1), 'DisplayName', 'c-code')
        pause(0.5)
        plot(optimized_watersetpoints, 'DisplayName', 'octave')
        legend
        grid minor

        hold on
        hax2 = subplot(3, 1, 2)
        hold on
        plot(optimalvalues_compiledcode(predictionhorizon + 2:2 * predictionhorizon + 2), 'DisplayName', 'c-code')
        pause(0.5)
        plot(optimized_valvesetpoints(:, 1), 'DisplayName', 'octave')
        legend
        grid minor

        hold on
        hax3 = subplot(3, 1, 3)
        hold on
        plot(optimalvalues_compiledcode(2 * predictionhorizon + 3:3 * predictionhorizon + 3), 'DisplayName', 'c-code')
        plot(optimized_valvesetpoints(:, 2), 'DisplayName', 'octave')
        legend
        grid minor

        %  check1=abs(optimalvalues_compiledcode(1)-optimized_watersetpoints(1))<1e-4;
        %  if ~check1
        %    display(check1)
        %    error('compiled c-code doesn not produce correct water  result')
        %  end
        %  for i=1:num_rooms
        %    check2=abs(optimalvalues_compiledcode(i+1)-optimized_valvesetpoints(i))<1e-4;
        %    if ~check2
        %          display(optimalvalues_compiledcode(i+1))
        %          display(optimized_valvesetpoints(i))
        %          error('compiled c-code doesn not produce correct result')
        %    end
        %  end

        % use c-code generated values instead of octave generated ones
        optimized_watersetpoints = optimalvalues_compiledcode(1:predictionhorizon + 1);

%        optimized_valvesetpoints = [optimalvalues_compiledcode(predictionhorizon + 2:2 * predictionhorizon + 2)', optimalvalues_compiledcode(2 * predictionhorizon + 3:3 * predictionhorizon + 3)'];
%        optimized_tfloor = [optimalvalues_compiledcode(3 * predictionhorizon + 4:4 * predictionhorizon + 4)', optimalvalues_compiledcode(4 * predictionhorizon + 5:5 * predictionhorizon + 5)'];
%        optimized_tair = [optimalvalues_compiledcode(5 * predictionhorizon + 6:6 * predictionhorizon + 6)', optimalvalues_compiledcode(6 * predictionhorizon + 7:7 * predictionhorizon + 7)'];


        optimized_valvesetpoints=[];optimized_tfloor=[];optimized_tair=[];
        vector_length=predictionhorizon + 1;
        for i=1:num_rooms*3
          if i<=num_rooms
            optimized_valvesetpoints = [optimized_valvesetpoints,optimalvalues_compiledcode(i*vector_length+1:(i+1)*vector_length)'];
          elseif i>num_rooms && i<=num_rooms*2
            optimized_tfloor=[optimized_tfloor,optimalvalues_compiledcode(i*vector_length+1:(i+1)*vector_length)'];
          else
            optimized_tair=[optimized_tair,optimalvalues_compiledcode(i*vector_length+1:(i+1)*vector_length)'];
          end

      end

%      keyboard
        %

    end

%    keyboard

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % second iteration of solving but applying 0/1 constraints to opti
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    disp('start optimiziation 2')

    opti = opti_common;
    % the heater cannot cool, water should be warmer than the floor
    for i = 1:num_rooms
        opti.subject_to(watersetpoints >= valvesetpoints(:, i) .* states_tfloor(:, i));
    end

    opti.subject_to(valvesetpoints .* (valvesetpoints - 1) == 0);

    opti.set_initial(watersetpoints, optimized_watersetpoints);
    opti.set_initial(valvesetpoints, optimized_valvesetpoints);
    opti.set_initial(states_tfloor, optimized_tfloor);
    opti.set_initial(states_tair, optimized_tair);

    solution = opti.solve();
    optimized_watersetpoints = solution.value(watersetpoints);
    optimized_valvesetpoints = solution.value(valvesetpoints);
    optimized_tfloor = solution.value(states_tfloor);
    optimized_tair = solution.value(states_tair);

    % Return the first control inputs as the setpoint
    setpoint = {};
    setpoint.watersetp = optimized_watersetpoints(1);
    setpoint.valvesetp = optimized_valvesetpoints(1, :);

    plotoptimizer(optimized_watersetpoints, optimized_valvesetpoints, optimized_tfloor, optimized_tair, airtemp_setpoints, num_rooms)

%    keyboard

    if (codegeneration)

        solver_options = struct();
        solver_options.ipopt.print_level = 0; % Suppress IPOPT console output
        solver_options.ipopt.file_print_level = 0; % Suppress IPOPT file output
        solver_options.ipopt.sb = 'yes'; % suppress banner
        solver_options.print_time = false; % Suppress CasADi timing information
        solver_options.error_on_fail = false;
        opti.solver('ipopt', solver_options);

        %      c_function = opti.to_function('optimal_control_pass2', ...
        %            {state_init, outdoortemp_opti, setpoints_opti, ...
        %             watersetpoints, valvesetpoints, states_tfloor, states_tair}, ...
        %            {watersetpoints(1), valvesetpoints(1, :)});

        c_function = opti.to_function('optimal_control_pass2', ...
            {state_init, outdoortemp_opti, setpoints_opti, ...
             watersetpoints, valvesetpoints, states_tfloor, states_tair}, ...
            {watersetpoints, valvesetpoints, states_tfloor, states_tair});

        % Set options for C code generation, including main and verbose
        opts = struct();
        opts.main = true; % Include a main function
        opts.verbose = true;
        opts.with_header = false; % I don't know, maybe

        % Generate C code with the specified options
        c_function.generate('optimal_control_pass2.c', opts);

        % compile c-code
        DIR = GlobalOptions.getCasadiPath();
        str_include = GlobalOptions.getCasadiIncludePath();
        str_compile = strcat('gcc -L', DIR, ' -Wl,-rpath,', DIR, ' -I', str_include, ' optimal_control_pass2.c -lm -lipopt -o optimal_control_pass2')
        system(str_compile)

        % test the compiled code
        % Update input string for testing - using regular numeric arrays
        input_string = sprintf(['%f\n%f\n%f\n' ...
                                    repmat('%f\n', 1, predictionhorizon + 1), ... % water init
                                    repmat('%f\n', 1, (predictionhorizon + 1) * num_rooms), ... % valve init
                                    repmat('%f\n', 1, (predictionhorizon + 1) * num_rooms), ... % floor init
                                    repmat('%f\n', 1, (predictionhorizon + 1) * num_rooms)], ... % air init
            state, outdoortemp, airtemp_setpoints, ...
            optimized_watersetpoints(:), optimized_valvesetpoints(:), ...
            optimized_tfloor(:), optimized_tair(:));

        %  % Use pipes to pass input
        command = sprintf('echo "%s" | ./optimal_control_pass2 optimal_control_pass2', input_string);
        [status, output] = system(command);

        %compare outputs with octave-casadi
        optimalvalues_compiledcode = str2num(output);

        % use c-code generated values instead of octave generated ones
        optimized_watersetpoints = optimalvalues_compiledcode(1:predictionhorizon + 1);
%        optimized_valvesetpoints = [optimalvalues_compiledcode(predictionhorizon + 2:2 * predictionhorizon + 2)', optimalvalues_compiledcode(2 * predictionhorizon + 3:3 * predictionhorizon + 3)'];
%        optimized_tfloor = [optimalvalues_compiledcode(3 * predictionhorizon + 4:4 * predictionhorizon + 4)', optimalvalues_compiledcode(4 * predictionhorizon + 5:5 * predictionhorizon + 5)'];
%        optimized_tair = [optimalvalues_compiledcode(5 * predictionhorizon + 6:6 * predictionhorizon + 6)', optimalvalues_compiledcode(6 * predictionhorizon + 7:7 * predictionhorizon + 7)'];

        optimized_valvesetpoints=[];optimized_tfloor=[];optimized_tair=[];
        vector_length=predictionhorizon + 1;
        for i=1:num_rooms*3
          if i<=num_rooms
            optimized_valvesetpoints = [optimized_valvesetpoints,optimalvalues_compiledcode(i*vector_length+1:(i+1)*vector_length)'];
          elseif i>num_rooms && i<=num_rooms*2
            optimized_tfloor=[optimized_tfloor,optimalvalues_compiledcode(i*vector_length+1:(i+1)*vector_length)'];
          else
            optimized_tair=[optimized_tair,optimalvalues_compiledcode(i*vector_length+1:(i+1)*vector_length)'];
         end

    end

        %        keyboard

    end

%    keyboard
    % plot variables
    if false

        %  optimized_tfloor=solution.value(states_tfloor);
        %  optimized_tair=solution.value(states_tair);

        figure(1)
        clf

        hold on
        hax1 = subplot(4, 1, 1);
        hold on
        plot(optimized_watersetpoints, '*', 'DisplayName', 'optimal water setpoint')
        pause(1)
        legend
        grid minor

        hax3 = subplot(4, 1, 2);
        hold on

        for i = 1:num_rooms
            plot(optimized_tfloor(:, i), '*', 'DisplayName', ['floor temp prediction', num2str(i)])
            pause(1)
        end

        legend
        grid minor

        hax4 = subplot(4, 1, 3);
        hold on

        for i = 1:num_rooms
            plot(optimized_tair(:, i), '*', 'DisplayName', ['air temp prediction', num2str(i)])
            pause(1)
            plot(ones(size(optimized_tair(:, i))) * airtemp_setpoints(i), 'DisplayName', ['temp setpoint', num2str(i)])
            pause(0.5)
        end

        legend
        grid minor

        hax2 = subplot(4, 1, 4);
        hold on
        pause(1)

        for i = 1:num_rooms
            plot(optimized_valvesetpoints(:, i), '*', 'DisplayName', strcat('optimal valve setpoint', num2str(i)))
            pause(1)
        end

        legend
        grid minor

        linkaxes ([hax1, hax2, hax3, hax4], "x");

        keyboard
    end

end
