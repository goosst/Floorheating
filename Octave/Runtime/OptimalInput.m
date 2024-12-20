function setpoint=OptimalInput(ode,intg,model_params,state,outdoortemp,airtemp_setpoint,predictionhorizon,simulationhorizon)

codegeneration=true; %only possible when using multiple_shooting_opticlass


% select optimization method
%selected_method='single_shooting';
%selected_method='multiple_shooting';
selected_method='multiple_shooting_opticlass';


% penalty factors cost function
penalty_watertemp=10^(-3);
penalty_valve=10^(-3);
penalty_changewater=10^(-4);
penalty_changevalve=10^(-4);

%penalty_watertemp=0;
%penalty_valve=0;
%penalty_changewater=0;
%penalty_changevalve=0;



initstate=state;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%watersetpoints = MX.sym('watersetpoints',predictionhorizon+1,1);
%valvesetpoints = MX.sym('valvesetpoints',predictionhorizon+1,1);
%if strcmp(selected_method,'single_shooting')
%  % Create the cost function
%  J = 0;
%  % Loop through each time step, propagate dynamics, and accumulate error
%  for k = 1:predictionhorizon
%
%      % Simulate the next state
%      result = intg('x0', state, 'u',[watersetpoints(k);outdoortemp;valvesetpoints(k)],'p', model_params); %third control input, measued air temp is irrelevant here
%      state = result.xf;
%
%      % Get predicted `tair` and accumulate squared error
%      predicted_tair = state(2); % tair state
%      predicted_tfloor=state(1); %tfloor state
%      J = J + (predicted_tair - setpoint)^2+...
%      penalty_watertemp*watersetpoints(k)+...
%      penalty_valve*valvesetpoints(k); %evaluate cost function of this state + some penalty to minimze inputs (reduce energy consumption)
%
%  end
%
%  watersetpoint_final=watersetpoints(predictionhorizon+1);
%  valvesetpoint_final=valvesetpoints(predictionhorizon+1);
%  for k=1:simulationhorizon
%      % just keep setpoint constant after prediction
%      % Simulate the next state
%      result = intg('x0', state, 'u',[watersetpoint_final;outdoortemp;valvesetpoint_final],'p', model_params); %third control input, measued air temp is irrelevant here
%      state = result.xf;
%
%      % Get predicted `tair` and accumulate squared error
%      predicted_tair = state(2); % tair state
%      predicted_tfloor=state(1); %tfloor state
%      J = J + (predicted_tair - setpoint)^2+...
%      penalty_watertemp*watersetpoint_final+...
%      penalty_valve*valvesetpoint_final; %evaluate cost function of this state + some penalty to minimze inputs (reduce energy consumption)
%
%  end
%
%
%
%  % Bounds for control inputs, inherently the model is defined if you ask for a very low watersetpoint it will consider it as zero
%  lb_watersetpoints = 0 * ones(predictionhorizon+1, 1); % Lower bound
%  ub_watersetpoints = 35 * ones(predictionhorizon+1, 1); % Upper bound
%
%  lb_valvesetpoints = zeros(size(valvesetpoints));
%  ub_valvesetpoints= ones(size(valvesetpoints));
%
%  lb_controls=[lb_watersetpoints;lb_valvesetpoints];
%  ub_controls=[ub_watersetpoints;ub_valvesetpoints];
%
%  % Constraints
%  constraint_binary=valvesetpoints.*(valvesetpoints-1);
%  ubg_binary=zeros(size(constraint_binary)); %setting upper and lower boundary to zero --> equality constraint
%  lbg_binary=zeros(size(constraint_binary));
%
%  constraint_valveopen=valvesetpoints-(watersetpoints>2);
%  ubg_valveopen=inf*ones(size(constraint_valveopen));
%  lbg_valveopen=zeros(size(constraint_valveopen));
%
%  constraints=[constraint_binary;constraint_valveopen];
%  lbg=[lbg_binary;lbg_valveopen];
%  ubg=[ubg_binary;ubg_valveopen];
%
%
% % NLP problem setup
%  nlp = struct('x', [watersetpoints;valvesetpoints], 'f', J,'g',constraints); % Decision variables and cost
%  solver = nlpsol('solver', 'ipopt', nlp);
%
%  % Solve the problem
%  watersetpoints_init = 20 * ones(predictionhorizon+1, 1); % Initial guess
%  valvesetpoints_init=ones(predictionhorizon+1,1);
%  solution = solver('x0', [watersetpoints_init;valvesetpoints_init], 'lbx', lb_controls, 'ubx', ub_controls,'lbg',lbg,'ubg',ubg);
%
%  optimized_watersetpoints=full(solution.x(1:predictionhorizon+1));
%  optimized_valvesetpoints=full(solution.x(predictionhorizon+2:end));
%  %keyboard
%
%  % Extract optimized control inputs
%  %optimized_watersetpoints = full(solution.x);
%  setpoint.watersetp=optimized_watersetpoints(1);
%  setpoint.valvesetp=optimized_valvesetpoints(1);





%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
if strcmp(selected_method,'multiple_shooting')


% Control vector to be optimized
watersetpoints = MX.sym('watersetpoints',predictionhorizon+1,1);
valvesetpoints = MX.sym('valvesetpoints',predictionhorizon+1,1);

% State variables for multiple shooting
states_tfloor = MX.sym('states_tfloor', predictionhorizon+1, 1);
states_tair = MX.sym('states_tair', predictionhorizon+1, 1);

% Initialize cost and constraints
J = 0;
constraints = {};
lbg = {};
ubg = {};

% Initial state constraint
constraints = {constraints{:}, [states_tfloor(1); states_tair(1)] - state};
lbg = {lbg{:}, [0; 0]};
ubg = {ubg{:}, [0; 0]};

% Loop through prediction horizon
for k = 1:predictionhorizon
    % Current state
    current_state = [states_tfloor(k); states_tair(k)];

    % Simulate the next state
    result = intg('x0', current_state, 'u', [watersetpoints(k); outdoortemp; valvesetpoints(k)], 'p', model_params);
    next_state = result.xf;

    % Continuity constraints between intervals
    constraints = {constraints{:}, [states_tfloor(k+1); states_tair(k+1)] - next_state};
    lbg = {lbg{:}, [0; 0]};
    ubg = {ubg{:}, [0; 0]};

    % Accumulate cost
    J = J + (states_tair(k) - airtemp_setpoint)^2 + ...
    penalty_watertemp*watersetpoints(k)+ ...
    penalty_valve*valvesetpoints(k)*watersetpoints(k)+ ...
    penalty_changewater*(watersetpoints(k)-watersetpoints(k+1))^2+ ...
    penalty_changevalve*(valvesetpoints(k)-valvesetpoints(k+1))^2;

end

% Handle simulation horizon with final control inputs
watersetpoint_final = watersetpoints(predictionhorizon+1);
valvesetpoint_final = valvesetpoints(predictionhorizon+1);
current_state = [states_tfloor(end); states_tair(end)];

for k = 1:simulationhorizon
    result = intg('x0', current_state, 'u', [watersetpoint_final; outdoortemp; valvesetpoint_final], 'p', model_params);
    current_state = result.xf;
    J = J + (current_state(2) - airtemp_setpoint)^2 + ...
    penalty_watertemp*watersetpoint_final+ ...
    penalty_valve*valvesetpoint_final;
end

% Bounds for control inputs: lower bound, upper bound
lb_watersetpoints = zeros(predictionhorizon+1, 1);
ub_watersetpoints = 35 * ones(predictionhorizon+1, 1);
lb_valvesetpoints = zeros(size(valvesetpoints));
ub_valvesetpoints = 1*ones(size(valvesetpoints));

%keyboard

% State bounds (add reasonable bounds for your system)
lb_tfloor = 0 * ones(predictionhorizon+1, 1);
ub_tfloor = 40 * ones(predictionhorizon+1, 1);
lb_tair = 0 * ones(predictionhorizon+1, 1);
ub_tair = 40 * ones(predictionhorizon+1, 1);

% Combine all decision variables
w = [watersetpoints; valvesetpoints; states_tfloor; states_tair];
lbw = [lb_watersetpoints; lb_valvesetpoints; lb_tfloor; lb_tair];
ubw = [ub_watersetpoints; ub_valvesetpoints; ub_tfloor; ub_tair];

%% Binary valve constraints
constraint_binary = valvesetpoints.*(valvesetpoints-1); % valve can only be open or closed, other option is to use some mixed integer solver
constraints = {constraints{:}, constraint_binary};
lbg = {lbg{:}, zeros(size(constraint_binary))};
ubg = {ubg{:}, zeros(size(constraint_binary))};

for k = 1:predictionhorizon+1
    % If valve is 0, watersetpoint must be zero
    M = 50;  % A sufficiently large number
    constraints = {constraints{:},M * valvesetpoints(k) - watersetpoints(k)};
    lbg = {lbg{:}, 0};
    ubg = {ubg{:}, Inf};
end

% Convert constraint cell arrays to vectors
constraints_vec = vertcat(constraints{:});
lbg_vec = vertcat(lbg{:});
ubg_vec = vertcat(ubg{:});

%keyboard

% NLP problem setup
nlp = struct('x', w, 'f', J, 'g', constraints_vec);

opts = struct;
opts.ipopt.print_level = 8;
%opts.ipopt.tol = 1e-10;  % Tighter convergence tolerance

%opts.ipopt.mu_strategy = 'adaptive';  % More robust barrier parameter strategy
%opts.ipopt.max_iter = 500;  % iteration limit
%opts.ipopt.constr_viol_tol = 1e-6;  % More strict constraint violation tolerance
%opts.ipopt.least_square_init_primal ='yes'% or no
%opts.ipopt.accept_every_trial_step='yes'
%opts.ipopt.linear_solver='ma27'

solver = nlpsol('solver', 'ipopt', nlp);
%solver = nlpsol('solver', 'fatrop', nlp);
%solver = nlpsol('solver', 'alpaqa', nlp);


%keyboard

% Initial guess
watersetpoints_init = 20 * ones(predictionhorizon+1, 1);
valvesetpoints_init = 0.5*ones(predictionhorizon+1, 1);
%valvesetpoints_init(2:2:end)=0;
states_tfloor_init = 20 * ones(predictionhorizon+1, 1);
states_tair_init = 20 * ones(predictionhorizon+1, 1);
w0 = [watersetpoints_init; valvesetpoints_init; states_tfloor_init; states_tair_init];

% Solve the problem
solution = solver('x0', w0, 'lbx', lbw, 'ubx', ubw, 'lbg', lbg_vec, 'ubg', ubg_vec);

%keyboard

% Extract solutions
sol_w = full(solution.x);
optimized_watersetpoints = sol_w(1:predictionhorizon+1);
optimized_valvesetpoints = sol_w(predictionhorizon+2:2*(predictionhorizon+1));
optimized_tfloor = sol_w(2*(predictionhorizon+1)+1:3*(predictionhorizon+1));
optimized_tair = sol_w(3*(predictionhorizon+1)+1:end);


% return values of function
setpoint={};
setpoint.watersetp=optimized_watersetpoints(1);
setpoint.valvesetp=optimized_valvesetpoints(1);


end



if strcmp(selected_method,'multiple_shooting_opticlass')

% Create Opti instance
    opti = Opti();

    % Decision variables
    watersetpoints = opti.variable(predictionhorizon + 1, 1);
    valvesetpoints = opti.variable(predictionhorizon + 1, 1);
    states_tfloor = opti.variable(predictionhorizon + 1, 1);
    states_tair = opti.variable(predictionhorizon + 1, 1);

    % Parameters to be passed, defined here to allow c-code generation later
    state_init = opti.parameter(2, 1);
    outdoortemp_opti = opti.parameter();
    setpoint_opti = opti.parameter();

    % Initialize cost
    J = 0;

    % Initial state constraint
    opti.subject_to(states_tfloor(1) == state_init(1));
    opti.subject_to(states_tair(1) == state_init(2));

    % Loop through prediction horizon
    for k = 1:predictionhorizon
        current_state = [states_tfloor(k); states_tair(k)];

        % Integrate to get the next state
        result = intg('x0', current_state,'u',[watersetpoints(k); outdoortemp_opti; valvesetpoints(k)] ,'p', model_params);
        next_state = result.xf;

        % Continuity constraints multiple shooting
        opti.subject_to(states_tfloor(k + 1) == next_state(1));
        opti.subject_to(states_tair(k + 1) == next_state(2));

        % Accumulate cost
        J = J + (states_tair(k) - setpoint_opti)^2 + ...
            penalty_watertemp * watersetpoints(k) + ...
            penalty_valve * valvesetpoints(k) * watersetpoints(k) + ...
            penalty_changewater * (watersetpoints(k) - watersetpoints(k + 1))^2 + ...
            penalty_changevalve * (valvesetpoints(k) - valvesetpoints(k + 1))^2;
    end

 %     % Handle simulation horizon with final  constant control inputs
     watersetpoint_final = watersetpoints(end);
     valvesetpoint_final = valvesetpoints(end);
     current_state = [states_tfloor(end); states_tair(end)];

     for k = 1:simulationhorizon
         result = intg('x0', current_state, 'u', [watersetpoint_final; outdoortemp_opti; valvesetpoint_final], 'p', model_params);
         current_state = result.xf;

         J = J + (current_state(2) - setpoint_opti)^2 + ...
             penalty_watertemp * watersetpoint_final + ...
             penalty_valve * valvesetpoint_final;
     end

    % Add bounds for control inputs
    opti.subject_to(0 <= watersetpoints <= 35);
    opti.subject_to(0 <= valvesetpoints <= 1);

    % watersetpoint cannot be below 15 degrees for my gas boiler
    % for some reason, adding this makes the whole problem suboptimal
%    opti.subject_to((watersetpoints<=2) .*(watersetpoints>=15) == 0);

    % Add state bounds
    opti.subject_to(0 <= states_tfloor <= 45);
    opti.subject_to(0 <= states_tair <= 40);

    % Binary valve constraints, valve can only be open or closed
    opti.subject_to(valvesetpoints .* (valvesetpoints - 1) == 0);

    % if valve is open bring watersetpoint to zero
    for k = 1:predictionhorizon + 1
        opti.subject_to(50 * valvesetpoints(k) >= watersetpoints(k));
    end

    % Set objective
    opti.minimize(J);

opti.set_initial(watersetpoints, 20 * ones(predictionhorizon + 1, 1));
opti.set_initial(valvesetpoints, 0.5 * ones(predictionhorizon + 1, 1));
opti.set_initial(states_tfloor, 20 * ones(predictionhorizon + 1, 1));
opti.set_initial(states_tair, 20 * ones(predictionhorizon + 1, 1));

solver_options = struct();
solver_options.ipopt.print_level = 5;
solver_options.ipopt.file_print_level = 0;
solver_options.print_time = true;
opti.solver('ipopt',solver_options);

%give a value to opti parameters for simulating result using casadi
opti.set_value(state_init, state);
opti.set_value(outdoortemp_opti, outdoortemp);
opti.set_value(setpoint_opti, airtemp_setpoint);

solution = opti.solve();
optimized_watersetpoints = solution.value(watersetpoints);
optimized_valvesetpoints = solution.value(valvesetpoints);
optimized_tfloor = solution.value(states_tfloor);
optimized_tair = solution.value(states_tair);

setpoint={};
setpoint.watersetp=optimized_watersetpoints(1);
setpoint.valvesetp=optimized_valvesetpoints(1);

%keyboard

if(codegeneration)

  solver_options = struct();
  solver_options.ipopt.print_level = 0;      % Suppress IPOPT console output
  solver_options.ipopt.file_print_level = 0; % Suppress IPOPT file output
  solver_options.ipopt.sb = 'yes';           % suppress banner
  solver_options.print_time = false;         % Suppress CasADi timing information
  solver_options.error_on_fail=true;
  opti.solver('ipopt',solver_options);

  % Create CasADi function for C code generation: inputs (opti parmaeters) in first part, outputs in second part
%  c_function = opti.to_function('optimal_control', {state_init, outdoortemp_opti, setpoint_opti}, {watersetpoints(1), valvesetpoints(1)});
    c_function = opti.to_function('optimal_control', {state_init, outdoortemp_opti, setpoint_opti}, {watersetpoints(1), valvesetpoints(1)});



  % Set options for C code generation, including main and verbose
  opts = struct();
  opts.main = true;       % Include a main function
  opts.verbose = true;
  opts.with_header=false; % I don't know, maybe

  % Generate C code with the specified options
  c_function.generate('optimal_control.c', opts);

  % compile c-code
  DIR=GlobalOptions.getCasadiPath();
  str_include = GlobalOptions.getCasadiIncludePath();
  str_compile=strcat('gcc -L',DIR,' -Wl,-rpath,',DIR,' -I',str_include,' optimal_control.c -lm -lipopt -o optimal_control')
  %str_compile=strcat('gcc -L',DIR,' -I',str_include,' optimal_control.c -lm -lipopt -o optimal_control')
  system(str_compile)

  % test the compiled code
  % state_value = [15; 15];  % Example values for state
  % outdoortemp_value = 6.0;   % Example value for outdoor temperature
  % setpoint_value = 21.0;      % Example value for the setpoint
  % Prepare the input as a string
  input_string = sprintf('%f\n%f\n%f\n%f\n', state(1), state(2),outdoortemp,airtemp_setpoint);

  % Use pipes to pass input
  command = sprintf('echo "%s" | ./optimal_control optimal_control', input_string);
  [status, output] = system(command);

  %compare outputs with octave-casadi
  optimalvalues_compiledcode = str2num(output);

  check1=abs(optimalvalues_compiledcode(1)-optimized_watersetpoints(1))<1e-4;
  check2=abs(optimalvalues_compiledcode(2)-optimized_valvesetpoints(1))<1e-4;

  if (check1 & check2)
    display('compiled c-code working successfully !!')
  else
    display(check1)
    display(check2)
    error('compiled c-code doesn not produce correct result')
  end

end

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
% plot variables
if true
figure(1)
clf

hold on
hax1=subplot(3,1,1);
hold on
plot(optimized_watersetpoints,'*','DisplayName','optimal water setpoint')
pause(1)
legend
grid minor

hax3=subplot(3,1,2);
hold on
plot(optimized_tfloor,'*','DisplayName','floor temp prediction')
pause(1)
plot(optimized_tair,'*','DisplayName','air temp prediction')
pause(1)
legend
grid minor

hax2=subplot(3,1,3);
pause(1)
plot(optimized_valvesetpoints,'*','DisplayName','optimal valve setpoint')
pause(1)
legend
grid minor

linkaxes ([hax1, hax2,hax3],"x");

keyboard
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% check outputs / debugging
if false

figure(1)
hold on
hax1 =subplot(4,1,1)
plot([optimized_watersetpoints;optimized_watersetpoints(end)*ones(simulationhorizon-1,1)],'*','DisplayName','heat setpoints')
legend
grid

pause(1)

hold on
hax4 =subplot(4,1,2)
plot([optimized_valvesetpoints;optimized_valvesetpoints(end)*ones(simulationhorizon-1,1)],'*','DisplayName','valve setpoints')
legend
grid

% Loop through each time step, propagate dynamics, and accumulate error
state=initstate;
integrations=[];
for k = 1:predictionhorizon

    % Simulate the next state
    result = intg('x0', state, 'u',[optimized_watersetpoints(k);outdoortemp;optimized_valvesetpoints(k)],'p', model_params); %third control input, measued air temp is irrelevant here
    state = result.xf;

    % Get predicted `tair` and accumulate squared error
    predicted_tair = state(2); % tair state
    predicted_tfloor=state(1); %tfloor state
    integrations=[integrations;[state(1),state(2)]];

end
watersetpoint_final=optimized_watersetpoints(predictionhorizon+1);
valvesetpoint_final=optimized_valvesetpoints(predictionhorizon+1);
for k=1:simulationhorizon
    % just keep setpoint constant after prediction
    % Simulate the next state
    result = intg('x0', state, 'u',[watersetpoint_final;outdoortemp;valvesetpoint_final],'p', model_params); %third control input, measued air temp is irrelevant here
    state = result.xf;

    % Get predicted `tair` and accumulate squared error
    predicted_tair = state(2); % tair state
    predicted_tfloor=state(1); %tfloor state
    integrations=[integrations;[state(1),state(2)]];
%    J = J + (predicted_tair - setpoint)^2+penalty_watertemp*watersetpoint_final; %evaluate cost function of this state + some penalty to minimze inputs (reduce energy consumption)

end

%keyboard

hold on
hax2 =subplot(4,1,3)
hold on
plot(full(integrations(:,1)),'*','DisplayName','floor temp prediction')
legend
grid

hold on
hax3 =subplot(4,1,4)
hold on
plot(full(integrations(:,2)),'*','DisplayName','living air temp prediction')

linkaxes ([hax1, hax2, hax3,hax4],"x");
pause(1)

legend
grid

keyboard
end
end
