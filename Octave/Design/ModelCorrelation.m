% fit parameters of model based on measurements

clc
clear variables
close all
dbstop if error

pkg load control
pkg load signal

    addpath('/home/stijn/Projects/Casadi/casadi-3.6.7-linux64-octave7.3.0')
    import casadi.*

currentdir=pwd;
cd ..
addpath(genpath(pwd))
cd(currentdir)

Secrets %import auth_token for home assistant etc.
% general parameters
Ts=5*60; %sample time in seconds
address_hass='192.168.0.205:8123';

%keyboard

%replay data

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
% download data from home assistant
  entity_id='sensor.temperature_flow_radiator,sensor.temperature_living,sensor.temperature_buiten_vaillant,sensor.temperature_setpoint_living,switch.collector,sensor.floor_temperature,sensor.temperature_flow';

  [entity_data,hass_time,most_recent_states]=DownloadHomeAssistant(address_hass,auth_token,entity_id);
  resampled_data=ResampleData(entity_data,Ts);
fields=fieldnames(resampled_data);



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
%% model correlation
[ode_obs,intg_obs,ode_noobs,intg_noobs]=plantModelDefinition(Ts);
x0= [resampled_data.(fields{6}).data(1);resampled_data.(fields{2}).data(1)]; %[tfloor,tair]


standardfit=true
if standardfit
opti = Opti();
opt_params = opti.variable(length(ode_noobs.p),1);

% Initialize optimization problem
state = x0; % Initial state
total_error = 0;

%opts = struct('simplify', true); % codegeneration stuff
integrator_opts = struct('tf', Ts);  % tf option
%merged_opts = merge_structs(integrator_opts, opts);
integrator = integrator('integrator', 'rk', struct('x', ode_noobs.x, 'p', [ode_noobs.u; ode_noobs.p], 'ode', ode_noobs.ode), integrator_opts);

for i=1:size(resampled_data.(fields{2}).data,2)

  measured = resampled_data.(fields{2}).data(i); %measued air temperature
  measured_floor = resampled_data.(fields{6}).data(i);
  total_error=total_error+(measured-state(2))^2 + (measured_floor-state(1))^2;

%  controlinputs = [resampled_data.(fields{7}).data(i);resampled_data.(fields{3}).data(i);resampled_data.(fields{5}).data(i)];
  controlinputs = [resampled_data.(fields{1}).data(i);resampled_data.(fields{3}).data(i);resampled_data.(fields{5}).data(i)];
  result = integrator('x0', state, 'p', [controlinputs; opt_params]);
  state = result.xf;
end


opti.minimize(total_error);

% Set initial guess and bounds for parameters if desired
opti.subject_to(opt_params > 0); % Enforce non-negativity

opti.subject_to(opt_params(5)==4.0000e+02);

%make sure the poles are and sampling time are far enough away from each other
pole1=-(sqrt((opt_params(4)^2+2*opt_params(3)*opt_params(4)+opt_params(3)^2)*opt_params(1)^2+((-2*opt_params(4)-2*opt_params(3))*opt_params(5)-2*opt_params(3)*opt_params(4)+2*opt_params(3)^2)*opt_params(2)*opt_params(1)+(opt_params(5)^2+2*opt_params(3)*opt_params(5)+opt_params(3)^2)*opt_params(2)^2)+(opt_params(4)+opt_params(3))*opt_params(1)+(opt_params(5)+opt_params(3))*opt_params(2))/(2*opt_params(2)*opt_params(1))
pole2=(sqrt((opt_params(4)^2+2*opt_params(3)*opt_params(4)+opt_params(3)^2)*opt_params(1)^2+((-2*opt_params(4)-2*opt_params(3))*opt_params(5)-2*opt_params(3)*opt_params(4)+2*opt_params(3)^2)*opt_params(2)*opt_params(1)+(opt_params(5)^2+2*opt_params(3)*opt_params(5)+opt_params(3)^2)*opt_params(2)^2)+(-opt_params(4)-opt_params(3))*opt_params(1)+(-opt_params(5)-opt_params(3))*opt_params(2))/(2*opt_params(2)*opt_params(1))

opti.subject_to(-1/pole1 > 500);
opti.subject_to(-1/pole2 > 500);

opti.set_initial(opt_params, [12984400; 1700; 200; 100; 4.0000e+02]); % Adjust initial guesses as needed



%pole1=-(sqrt((hisol^2+2*hair*hisol+hair^2)*mCfloor^2+((-2*hisol-2*hair)*kwfl-2*hair*hisol+2*hair^2)*mCair*mCfloor+(kwfl^2+2*hair*kwfl+hair^2)*mCair^2)+(hisol+hair)*mCfloor+(kwfl+hair)*mCair)/(2*mCair*mCfloor)
%pole2=(sqrt((hisol^2+2*hair*hisol+hair^2)*mCfloor^2+((-2*hisol-2*hair)*kwfl-2*hair*hisol+2*hair^2)*mCair*mCfloor+(kwfl^2+2*hair*kwfl+hair^2)*mCair^2)+(-hisol-hair)*mCfloor+(-kwfl-hair)*mCair)/(2*mCair*mCfloor)

% Set up the solver and solve the optimization problem
opti.solver('ipopt'); % Using IPOPT as the solver

% Solve the optimization problem
sol = opti.solve();

% Retrieve the estimated parameters
estimated_params = sol.value(opt_params);

% Display estimated parameters
disp(ode_noobs.p)
disp('Estimated Parameters:');
disp(estimated_params);

sym_params=ode_noobs.p
    for i = 1:sym_params.size1()
        % Extract name from symbolic expression
        name = str(sym_params(i));
        value=estimated_params(i);

        disp([name,'=',num2str(value)])

  end


end



