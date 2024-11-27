function setpoint=OptimalInput(ode,intg,model_params,state,outdoortemp,setpoint,predictionhorizon,simulationhorizon)

% select optimization method
%selected_method='single_shooting';
selected_method='multiple_shooting';

% Control vector to be optimized
watersetpoints = MX.sym('watersetpoints',predictionhorizon+1,1);
valvesetpoints = MX.sym('valvesetpoints',predictionhorizon+1,1);

% direct single shooting method
initstate=state;
penalty_watertemp=10^(-4);
penalty_valve=0;
penalty_changewater=10^(-4);
penalty_changevalve=0%10^(-4);
%keyboard




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if strcmp(selected_method,'single_shooting')
  % Create the cost function
  J = 0;
  % Loop through each time step, propagate dynamics, and accumulate error
  for k = 1:predictionhorizon

      % Simulate the next state
      result = intg('x0', state, 'u',[watersetpoints(k);outdoortemp;valvesetpoints(k)],'p', model_params); %third control input, measued air temp is irrelevant here
      state = result.xf;

      % Get predicted `tair` and accumulate squared error
      predicted_tair = state(2); % tair state
      predicted_tfloor=state(1); %tfloor state
      J = J + (predicted_tair - setpoint)^2+...
      penalty_watertemp*watersetpoints(k)+...
      penalty_valve*valvesetpoints(k); %evaluate cost function of this state + some penalty to minimze inputs (reduce energy consumption)

  end

  watersetpoint_final=watersetpoints(predictionhorizon+1);
  valvesetpoint_final=valvesetpoints(predictionhorizon+1);
  for k=1:simulationhorizon
      % just keep setpoint constant after prediction
      % Simulate the next state
      result = intg('x0', state, 'u',[watersetpoint_final;outdoortemp;valvesetpoint_final],'p', model_params); %third control input, measued air temp is irrelevant here
      state = result.xf;

      % Get predicted `tair` and accumulate squared error
      predicted_tair = state(2); % tair state
      predicted_tfloor=state(1); %tfloor state
      J = J + (predicted_tair - setpoint)^2+...
      penalty_watertemp*watersetpoint_final+...
      penalty_valve*valvesetpoint_final; %evaluate cost function of this state + some penalty to minimze inputs (reduce energy consumption)

  end



  % Bounds for control inputs, inherently the model is defined if you ask for a very low watersetpoint it will consider it as zero
  lb_watersetpoints = 0 * ones(predictionhorizon+1, 1); % Lower bound
  ub_watersetpoints = 35 * ones(predictionhorizon+1, 1); % Upper bound

  lb_valvesetpoints = zeros(size(valvesetpoints));
  ub_valvesetpoints= ones(size(valvesetpoints));

  lb_controls=[lb_watersetpoints;lb_valvesetpoints];
  ub_controls=[ub_watersetpoints;ub_valvesetpoints];

  % Constraints
  constraint_binary=valvesetpoints.*(valvesetpoints-1);
  ubg_binary=zeros(size(constraint_binary)); %setting upper and lower boundary to zero --> equality constraint
  lbg_binary=zeros(size(constraint_binary));

  constraint_valveopen=valvesetpoints-(watersetpoints>2);
  ubg_valveopen=inf*ones(size(constraint_valveopen));
  lbg_valveopen=zeros(size(constraint_valveopen));

  constraints=[constraint_binary;constraint_valveopen];
  lbg=[lbg_binary;lbg_valveopen];
  ubg=[ubg_binary;ubg_valveopen];


 % NLP problem setup
  nlp = struct('x', [watersetpoints;valvesetpoints], 'f', J,'g',constraints); % Decision variables and cost
  solver = nlpsol('solver', 'ipopt', nlp);

  % Solve the problem
  watersetpoints_init = 20 * ones(predictionhorizon+1, 1); % Initial guess
  valvesetpoints_init=ones(predictionhorizon+1,1);
  solution = solver('x0', [watersetpoints_init;valvesetpoints_init], 'lbx', lb_controls, 'ubx', ub_controls,'lbg',lbg,'ubg',ubg);

  optimized_watersetpoints=full(solution.x(1:predictionhorizon+1));
  optimized_valvesetpoints=full(solution.x(predictionhorizon+2:end));
  %keyboard

  % Extract optimized control inputs
  %optimized_watersetpoints = full(solution.x);
  setpoint=optimized_watersetpoints(1);













%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
elseif strcmp(selected_method,'multiple_shooting')


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
    J = J + (states_tair(k) - setpoint)^2 + ...
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
    J = J + (current_state(2) - setpoint)^2 + ...
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
constraint_binary = valvesetpoints.*(valvesetpoints-1); % valve can only be open or closed, other option is to use some mixed integer solvers
constraints = {constraints{:}, constraint_binary};
lbg = {lbg{:}, zeros(size(constraint_binary))};
ubg = {ubg{:}, zeros(size(constraint_binary))};

% force valve closed
%M=40;
%constraints = {constraints{:}, watersetpoints};
%lbg = {lbg{:}, 0};
%ubg = {ubg{:}, M * valvesetpoints};


for k = 1:predictionhorizon+1
    % 1. If valve is zero, watersetpoint must be zero
    M = 40;  % A sufficiently large number
    constraints = {constraints{:},M * valvesetpoints(k)-watersetpoints(k)};
    lbg = {lbg{:}, 0};
    ubg = {ubg{:}, Inf};
end

%keyboard
%water_temp_threshold=3;
%for k = 1:predictionhorizon+1
%    % Create indicator functions
%    cold_water_constraint = valvesetpoints(k) * (watersetpoints(k) <= water_temp_threshold);
%    hot_water_constraint = (1 - valvesetpoints(k)) * (watersetpoints(k) > water_temp_threshold);
%    constraints = {constraints{:}, cold_water_constraint, hot_water_constraint};
%    lbg = {lbg{:}, 0, 0};
%    ubg = {ubg{:}, 0, 0};
%end


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
%opts.ipopt.max_iter = 500;  % Increased iteration limit
%opts.ipopt.constr_viol_tol = 1e-6;  % More strict constraint violation tolerance
%opts.ipopt.least_square_init_primal ='yes'% or no
%opts.ipopt.accept_every_trial_step='yes'
%opts.ipopt.linear_solver='ma27'

solver = nlpsol('solver', 'ipopt', nlp);

keyboard

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

end

% try to formulate as


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% check outputs / debugging
%keyboard

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
