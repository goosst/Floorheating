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

predictionhorizon=ceil(4*3600/Ts);
simulationhorizon=ceil(2*3600/Ts); %additional simulation time after prediction

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% update model,
% the state of this model will be used as starting state of predictor

firstrun=true;
starttime = time();  %time in seconds
k=0;

while true
  currenttime = time();

  % a crappy time scheduler
   if (currenttime-starttime>=Ts || firstrun)


  starttime= time();

  k=k+1;
  % get data from home assistant
  entity_id='sensor.temperature_flow_radiator,sensor.temperature_living,sensor.temperature_buiten_vaillant,sensor.temperature_setpoint_living';
  [entity_data,hass_time,most_recent_states]=DownloadHomeAssistant(address_hass,auth_token,entity_id);
  watertempsetp=most_recent_states.(fieldnames(entity_data){1});
  roomairtemp=most_recent_states.(fieldnames(entity_data){2});
  outdoortemp=most_recent_states.(fieldnames(entity_data){3});
  controlinputs=[watertempsetp;outdoortemp;roomairtemp];

  if(firstrun)
    %% initialize states of model, something more fancy could be made for floortemperature
    state=[roomairtemp;roomairtemp]; %[tfloor,tair]
    firstrun=false;
  end

  % predict and optimize
  tempsetp_living=most_recent_states.(fieldnames(entity_data){4});
  model_params=[mCfloor;mCair;hair;hisol;kwfl];% set observer gains to zero

  tempsetp_living=15;
  setpoint=OptimalInput(ode_noobs,intg_noobs,model_params,state,outdoortemp,tempsetp_living,predictionhorizon,simulationhorizon);

  entity_id='sensor.MPCwaterreq';
  PostHomeAssistant(address_hass,auth_token,entity_id,setpoint);

%  keyboard
  %model update including observer correction
%  state=SimulateModel(ode,intg,model_params,controlinputs,state);


  if (k>4)

    break

  end



   end
end
