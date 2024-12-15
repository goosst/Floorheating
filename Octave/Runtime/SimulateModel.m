function state_new=SimulateModel(modeldefinition,intg,parameters,controlinputs,state)

generate_code=true
%keyboard

    % run casadi integrator
      res = intg('x0',state,'u',controlinputs,'p',parameters);
      state_new = full(res.xf);



if generate_code
  % Code generation options
  codegen_opts = struct;
  codegen_opts.main = true;       % Generate a main function for testing
  codegen_opts.mex = false;       % Disable MATLAB MEX generation
  codegen_opts.verbose = true;    % Verbose output during code generation
  codegen_opts.with_header = false; % Do not generate a header file

%  keyboard

  % Generate the C code
  intg.generate('integrator_c_code', codegen_opts);

  % compile c-code
  DIR=GlobalOptions.getCasadiPath();
  str_include = GlobalOptions.getCasadiIncludePath();
  str_compile=strcat('gcc -L',DIR,' -Wl,-rpath,',DIR,' -I',str_include,' integrator_c_code.c -lm -lipopt -o integrator_c_code')
  [status,output]=system(str_compile)

      %run compiled code (first generate code for it)

    % Prepare the input as a string
    input_string = sprintf('%f\n', [state;parameters;controlinputs]);

  %  command = sprintf('echo "%s" | ./integrator_c_code intg', input_string);
    functionname=intg.name();
    command = sprintf(['echo "%s" | ./integrator_c_code ',functionname], input_string)
    [status,output]=system(command)

    values = str2num(output);

    state_new(1)=values(1);
    state_new(2)=values(2);

    keyboard

end



end
