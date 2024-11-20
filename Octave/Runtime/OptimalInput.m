function setpoint=OptimalInput(ode,intg,model_params,state,outdoortemp,setpoint,predictionhorizon,simulationhorizon)

% Control vector to be optimized
watersetpoints = MX.sym('u',predictionhorizon+1,1);
initstate=state;

penalty_inputs=10^(-3);
%keyboard

% Create the cost function
J = 0;
% Loop through each time step, propagate dynamics, and accumulate error
for k = 1:predictionhorizon

    % Simulate the next state
    result = intg('x0', state, 'u',[watersetpoints(k);outdoortemp],'p', model_params); %third control input, measued air temp is irrelevant here
    state = result.xf;

    % Get predicted `tair` and accumulate squared error
    predicted_tair = state(2); % tair state
    predicted_tfloor=state(1); %tfloor state
    J = J + (predicted_tair - setpoint)^2+penalty_inputs*watersetpoints(k); %evaluate cost function of this state + some penalty to minimze inputs (reduce energy consumption)

end

watersetpoint_final=watersetpoints(predictionhorizon+1);
for k=1:simulationhorizon
    % just keep setpoint constant after prediction
    % Simulate the next state
    result = intg('x0', state, 'u',[watersetpoint_final;outdoortemp],'p', model_params); %third control input, measued air temp is irrelevant here
    state = result.xf;

    % Get predicted `tair` and accumulate squared error
    predicted_tair = state(2); % tair state
    predicted_tfloor=state(1); %tfloor state
    J = J + (predicted_tair - setpoint)^2+penalty_inputs*watersetpoint_final; %evaluate cost function of this state + some penalty to minimze inputs (reduce energy consumption)

end



% NLP problem setup
nlp = struct('x', watersetpoints, 'f', J); % Decision variables and cost
solver = nlpsol('solver', 'ipopt', nlp);

% Bounds for control inputs, inherently the model is defined if you ask for a very low watersetpoint it will consider it as zero
lb_watersetpoints = 0 * ones(predictionhorizon+1, 1); % Lower bound
ub_watersetpoints = 35 * ones(predictionhorizon+1, 1); % Upper bound

% Solve the problem
watersetpoints_init = 20 * ones(predictionhorizon+1, 1); % Initial guess
solution = solver('x0', watersetpoints_init, 'lbx', lb_watersetpoints, 'ubx', ub_watersetpoints);

% Extract optimized control inputs
optimized_watersetpoints = full(solution.x);
setpoint=optimized_watersetpoints(1);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% check outputs / debugging
%keyboard
%
%figure(1)
%hold on
%hax1 =subplot(3,1,1)
%plot([optimized_watersetpoints;optimized_watersetpoints(end)*ones(simulationhorizon-1,1)],'*','DisplayName','heat setpoints')
%legend
%grid
%
%pause(1)
%
%% Loop through each time step, propagate dynamics, and accumulate error
%state=initstate;
%integrations=[];
%for k = 1:predictionhorizon
%
%    % Simulate the next state
%    result = intg('x0', state, 'u',[optimized_watersetpoints(k);outdoortemp],'p', model_params); %third control input, measued air temp is irrelevant here
%    state = result.xf;
%
%    % Get predicted `tair` and accumulate squared error
%    predicted_tair = state(2); % tair state
%    predicted_tfloor=state(1); %tfloor state
%    integrations=[integrations;[state(1),state(2)]];
%
%end
%watersetpoint_final=optimized_watersetpoints(predictionhorizon+1);
%for k=1:simulationhorizon
%    % just keep setpoint constant after prediction
%    % Simulate the next state
%    result = intg('x0', state, 'u',[watersetpoint_final;outdoortemp],'p', model_params); %third control input, measued air temp is irrelevant here
%    state = result.xf;
%
%    % Get predicted `tair` and accumulate squared error
%    predicted_tair = state(2); % tair state
%    predicted_tfloor=state(1); %tfloor state
%    integrations=[integrations;[state(1),state(2)]];
%%    J = J + (predicted_tair - setpoint)^2+penalty_inputs*watersetpoint_final; %evaluate cost function of this state + some penalty to minimze inputs (reduce energy consumption)
%
%end
%
%%keyboard
%
%hold on
%hax2 =subplot(3,1,2)
%hold on
%plot(full(integrations(:,1)),'*','DisplayName','floor temp prediction')
%legend
%grid
%
%hold on
%hax3 =subplot(3,1,3)
%hold on
%plot(full(integrations(:,2)),'*','DisplayName','living air temp prediction')
%
%linkaxes ([hax1, hax2, hax3],"x");
%pause(1)
%
%legend
%grid
%
%keyboard

end

