function state_new = nextTimeStep(ode, intg, parameters, controlinputs, state, timestep)

    generate_code = true;

    % Run CasADi integrator
    res = intg('x0', state, 'u', controlinputs, 'p', parameters);
    state_new = full(res.xf); % Extract the new state as a full matrix

    if generate_code
        % Code generation options
        codegen_opts = struct;
        codegen_opts.main = true; % Generate a main function for testing
        codegen_opts.mex = false; % Disable MATLAB MEX generation
        codegen_opts.verbose = true; % Verbose output during code generation
        codegen_opts.with_header = false; % Do not generate a header file

        % Generate the C code
        intg.generate('integrator_c_code', codegen_opts);

        DIR = GlobalOptions.getCasadiPath();
        str_include = GlobalOptions.getCasadiIncludePath();
        str_compile = strcat('gcc -L', DIR, ' -Wl,-rpath,', DIR, ' -I', str_include, ' integrator_c_code.c -lm -lipopt -o integrator_c_code')
        [status, output] = system(str_compile)

        input_string = sprintf('%f\n', [state; parameters; controlinputs]);
        %  command = sprintf('echo "%s" | ./integrator_c_code intg', input_string);
        functionname = intg.name();
        command = sprintf(['echo "%s" | ./integrator_c_code ', functionname], input_string)
        [status, output] = system(command)
        state_new = str2num(output);

    end

end
