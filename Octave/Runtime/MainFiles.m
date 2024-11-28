clc
clear variables
close all
dbstop if error


addpath('/home/stijn/Projects/Casadi/casadi-3.6.7-linux64-octave7.3.0')
import casadi.*

Secrets

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
simulationhorizon=ceil(2*3600/Ts); %additional simulation time after prediction

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% update model,
% the state of this model will be used as starting state of predictor

firstrun=true;
starttime = time();  %time in seconds
k=0;

while true

  % a crappy time scheduler, don't if octave has better options
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
  state=SimulateModel(ode,intg,model_params,controlinputs,state);


  % predict and optimize
%  tempsetp_living=21;
  setpoint=OptimalInput(ode_noobs,intg_noobs,model_params_noobs,state,outdoortemp,tempsetp_living,predictionhorizon,simulationhorizon);

  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  % apply setpoints
  entity_id='sensor.MPCwatersetp';
  PostHomeAssistant(address_hass,auth_token,entity_id,setpoint.watersetp);

  entity_id='switch.collector';
%  entity_id='switch.collector_3';
  PostHomeAssistant(address_hass,auth_token,entity_id,setpoint.valvesetp);


  if (k>4)

    break

  end

end

end
