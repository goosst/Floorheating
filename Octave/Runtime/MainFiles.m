clc
clear variables
close all
dbstop if error

execute_compiledprogram=true;

if ~execute_compiledprogram
  % casadi should not be a dependency if c-code / compiled c-code is used
  addpath('/home/stijn/Projects/Casadi/casadi-3.6.7-linux64-octave7.3.0')
  import casadi.*
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Secrets %import passwords for home assistant etc.
% general parameters
Ts=5*60; %sample time in seconds
address_hass='192.168.0.205:8123';

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
% define model of heating system
[ode,intg,ode_noobs,intg_noobs]=plantModelDefinition(Ts);

%plant model parameters
mCfloor = 3.1542e+06;
mCair = 6.1871e+06;
hair= 3.1215e+02;
hisol= 5.1943e+01;
kwfl= 4.0000e+02;

%observer correction parameters
L1=-7.8058e-04;
L2=2.8463e-04;
model_params=[mCfloor;mCair;hair;hisol;kwfl;L1;L2];
model_params_noobs=[mCfloor;mCair;hair;hisol;kwfl];% model without observer


predictionhorizon=ceil(4*3600/Ts);
simulationhorizon=ceil(1*3600/Ts); %additional simulation time after prediction

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% update model,
% the state of this model will be used as starting state of predictor

firstrun=true;
starttime = time();  %time in seconds
k=0;

while true

  % a crappy time scheduler, don't know if octave has better options
  pause(1);
  currenttime = time();
   if (currenttime-starttime>=Ts || firstrun)

  starttime= time();

    clc
  disp(['run',num2str(k)])


  k=k+1;
  % get data from home assistant
  entity_id='sensor.temperature_flow_radiator,sensor.temperature_living,sensor.temperature_buiten_vaillant,sensor.temperature_setpoint_living,switch.collector';
  [entity_data,hass_time,most_recent_states]=DownloadHomeAssistant(address_hass,auth_token,entity_id);
  watertempsetp=most_recent_states.(fieldnames(entity_data){1});
  roomairtemp=most_recent_states.(fieldnames(entity_data){2});
  outdoortemp=most_recent_states.(fieldnames(entity_data){3});
  tempsetp_living=most_recent_states.(fieldnames(entity_data){4});
  valve_living=most_recent_states.(fieldnames(entity_data){5});


  if(firstrun)
    %% initialize states of model, something more fancy could be made for floortemperature
    state=[roomairtemp;roomairtemp]; %[tfloor,tair]
    firstrun=false;
  end

  %model update including observer correction;
  controlinputs=[watertempsetp;outdoortemp;roomairtemp;valve_living];

if (~execute_compiledprogram)
  state=SimulateModel(ode,intg,model_params,controlinputs,state);
else
  % code is generated in SimulateModel
    % Prepare the input as a string
  input_string = sprintf('%f\n', [state;model_params;controlinputs]);
  % Use pipes to pass input
  command = sprintf('echo "%s" | ./integrator_c_code intg', input_string);
  [status,output]=system(command);
  values = str2num(output);

  state(1)=values(1)
  state(2)=values(2)

end

  % predict and optimize
  tempsetp_living=16.1;

if (~execute_compiledprogram)
  %run casadi
  setpoint=OptimalInput(ode_noobs,intg_noobs,model_params_noobs,state,outdoortemp,tempsetp_living,predictionhorizon,simulationhorizon);
else
  % code is generated in optimalinput
  %execute compiled code, model parameters and optimization settings were compiled in c-code
  % Prepare the input as a string
  input_string = sprintf('%f\n%f\n%f\n%f\n', state(1), state(2),outdoortemp,tempsetp_living);
  % Use pipes to pass input
  command = sprintf('echo "%s" | ./optimal_control optimal_control', input_string);
  [status, output] = system(command);
  values = str2num(output);

  setpoint.watersetp=values(1)
  setpoint.valvesetp=values(2)

%  keyboard
end

  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  % apply setpoints
  entity_id='sensor.MPCwatersetp';
  PostHomeAssistant(address_hass,auth_token,entity_id,setpoint.watersetp);

%  entity_id='switch.collector';
  entity_id='sensor.collectorMPC';
  PostHomeAssistant(address_hass,auth_token,entity_id,setpoint.valvesetp);

%  keyboard

  if (k>4)

    break

  end

end

end



