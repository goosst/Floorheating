% replay data and design observer

clc
clear variables
close all
dbstop if error
pkg load control


currentdir=pwd;
cd ..
addpath(genpath(pwd))
cd(currentdir)

Secrets %import auth_token for home assistant etc.
% general parameters
Ts=5*60; %sample time in seconds
address_hass='192.168.0.205:8123';


%replay data

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
% download data from home assistant
  entity_id='sensor.temperature_flow_radiator,sensor.temperature_living,sensor.temperature_buiten_vaillant,sensor.temperature_setpoint_living,switch.collector';
  [entity_data,hass_time,most_recent_states]=DownloadHomeAssistant(address_hass,auth_token,entity_id);

  watertempsetp=entity_data.(fieldnames(entity_data){1});
  roomairtemp=entity_data.(fieldnames(entity_data){2});
  outdoortemp=entity_data.(fieldnames(entity_data){3});
  tempsetp_living=entity_data.(fieldnames(entity_data){4});
  valve_living=entity_data.(fieldnames(entity_data){5});

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% resampling crap

% convert timesteps to some other format than  yyyy-mm-ddTHH:MM:SS
data_converted={};
for j=1:size(fieldnames(entity_data),1)

  datatemp=entity_data.(fieldnames(entity_data){j});
  datatemp_time=datatemp(:,1);

  temp_signals=[];
  for i=1:size(datatemp_time,1)

  %  temp_ambient=[temp_ambient,str2num(data.ambienttemp(i,:))];
    time_full=datatemp_time(i,:){1};
    fixed_times = strrep(time_full, '+00:00', ''); % Remove timezone
    fixed_times = regexprep(fixed_times, '\.\d+', ''); % Remove fractional seconds
    temp_signals(i).time=datenum(fixed_times, "yyyy-mm-ddTHH:MM:SS");

    data_iter=datatemp(i,2){1};
    if strcmp(data_iter,'on')
      temp_signals(i).data = 1;
    elseif strcmp(data_iter,'off')
      temp_signals(i).data = 0;
    else
      temp_signals(i).data = str2double(data_iter);
    end

  end

  data_converted.(fieldnames(entity_data){j})=temp_signals;
end

% Define the sampling time (in seconds)

all_times = [];
% Collect all timestamps from data_converted for alignment
fields = fieldnames(data_converted);
for j = 1:numel(fields)
    data_field = data_converted.(fields{j});
    all_times = [all_times, [data_field.time]]; % Append all times
end

% Determine the common time range
start_time = max(cellfun(@(f) data_converted.(f)(1).time, fields));
end_time = max(cellfun(@(f) data_converted.(f)(end).time, fields));

% Create a common time grid
common_times = start_time:(Ts/(24*60*60)):end_time;


% Resample data for each field
resampled_data = struct(); % Store the resampled data
for j = 1:numel(fields)
    field_name = fields{j};
    data_field = data_converted.(field_name);

    % Extract times and data
    times = [data_field.time] ;
    data = [data_field.data];

    % Interpolate data to the common time grid
    resampled_values = interp1(times, data, common_times, 'previous', 'extrap');

%    figure(j)
%    clf
%    hold on
%    plot(times,data,'*')
%    plot(common_times,resampled_values,'r*')
%    keyboard

    % Store the resampled results in the structure
    resampled_data.(field_name).times = common_times;
    resampled_data.(field_name).data = resampled_values;
end

%keyboard

% Display the resampled results for each sensor
%fprintf('Resampled Data:\n');
%for j = 1:numel(fields)
%    field_name = fields{j};
%    fprintf('Sensor: %s\n', field_name);
%    for i = 1:numel(resampled_data.(field_name).times)
%        fprintf('%s\t%.2f\n', ...
%            datestr(resampled_data.(field_name).times(i), 'yyyy-mm-dd HH:MM:SS'), ...
%            resampled_data.(field_name).data(i));
%    end
%    fprintf('\n');
%end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% replay the model

addpath('/home/stijn/Projects/Casadi/casadi-3.6.7-linux64-octave7.3.0')
import casadi.*
firstrun=true;
if(firstrun)
  %% initialize states of model, something more fancy could be made for floortemperature

  state=[resampled_data.(fields{2}).data(1);resampled_data.(fields{2}).data(1)]; %[tfloor,tair]
  state_observer=state;
  firstrun=false;
end

%keyboard

  %plant model parameters
  mCfloor = 3.1542e+06;
  mCair = 6.1871e+06;
  hair= 3.1215e+02;
  hisol= 5.1943e+01;
  kwfl= 4.0000e+02;

  %observer correction parameters

  A=[(-kwfl-hair)/mCfloor, hair/mCfloor;
hair/mCair, (-hisol-hair)/mCair];

B=[kwfl/mCfloor, 0;
0, hisol/mCair];

C=[0, 1];

SYS = ss (A, B, C);
PolesDyn=pole(SYS);

OB = obsv (A, C);
rank(OB);

% relation to "natural dynamics"
pole1=PolesDyn(1)*1.5; % first state
pole2=PolesDyn(2); % second state

pole2=-1/500;
P=[pole1,pole2];

%timepole=[0:7200];
%figure
%plot(timepole,exp(pole2*timepole))

L=place (SYS.', P').';

   L1=L(1);
   L2=L(2);

  [ode_obs,intg_obs,ode_noobs,intg_noobs]=plantModelDefinition(Ts);

  state_floor_total=[];
  state_air_total=[];

  state_floor_observer=[];
  state_air_observer=[];

for i=1:length(common_times)

%  model_params=[mCfloor;mCair;hair;hisol;kwfl;L1;L2];
%  controlinputs=[watertempsetp;outdoortemp;roomairtemp;valve_living];
%  controlinputs=[resampled_data.(fields{1}).data(i);resampled_data.(fields{3}).data(i);resampled_data.(fields{2}).data(i);resampled_data.(fields{5}).data(i)];

%  params=[mCfloor;mCair;hair;hisol;kwfl];
%    controlinputs=[resampled_data.(fields{1}).data(i);resampled_data.(fields{3}).data(i);resampled_data.(fields{2}).data(i);resampled_data.(fields{5}).data(i)];

  controlinputs = [resampled_data.(fields{1}).data(i);resampled_data.(fields{3}).data(i);resampled_data.(fields{5}).data(i)];
  model_params=[mCfloor;mCair;hair;hisol;kwfl];

  state=SimulateModel(ode_noobs,intg_noobs,model_params,controlinputs,state);
  state_floor_total=[state_floor_total;state(1)];
  state_air_total=[state_air_total;state(2)];


  controlinputs=[resampled_data.(fields{1}).data(i);resampled_data.(fields{3}).data(i);resampled_data.(fields{2}).data(i);resampled_data.(fields{5}).data(i)];
%  keyboard

  model_params=[mCfloor;mCair;hair;hisol;kwfl;L1;L2];

  state_observer=SimulateModel(ode_obs,intg_obs,model_params,controlinputs,state_observer);
%[twater(k);toutside(k);tairtemp(k)]
%    res = intg_obs('x0',state_observer,'u',controlinputs,'p',[model_params;L(1);L(2)]);
%state_observer = full(res.xf);

  state_floor_observer=[state_floor_observer;state_observer(1)];
  state_air_observer=[state_air_observer;state_observer(2)];
%  keyboard
end


figure
hold on

plot(state_floor_total,'DisplayName','estimated floor')
pause(1)
plot(state_air_total,'DisplayName','estimated air')
pause(1)
plot(resampled_data.(fields{2}).data(:),'DisplayName','measured air temp')
pause(1)
plot(state_air_observer,'*','DisplayName','estimated air - observer correction')
pause(1)
plot(state_floor_observer,'*','DisplayName','estimated floor - observer correction')
pause(1)
legend
grid minor
