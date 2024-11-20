function [ode,intg,ode1,intg1]=plantModelDefinition(Ts)

% Add states
tfloor = MX.sym('tfloor');
tair = MX.sym('tair');
states = [tfloor;tair]; n_states = length(states);

% control inputs, measurement added for observer
twatersetp = MX.sym('twatersetp');
toutside = MX.sym('toutside');
tairmeas = MX.sym('tairmeas');
controls = [twatersetp;toutside;tairmeas]; n_controls = length(controls);

% parameters
mCfloor = MX.sym('mCfloor');
mCair = MX.sym('mCair');
hair= MX.sym('hair');
hisol= MX.sym('hisol');
kwfl= MX.sym('kwfl');
params=[mCfloor;mCair;hair;hisol;kwfl];

% observer gains
L1 = MX.sym('L1');
L2 = MX.sym('L2');
params=[params;L1;L2];

%reason for sigmoid function: 1/(1+exp(-6*(twatersetp-5))) if you ask for a very low watersetpoint it will consider it as zero; this makes life easier for the optimzier

rhs=[(1/(1+exp(-6*(twatersetp-5)))*kwfl*twatersetp+L1*mCfloor*tairmeas+(-1/(1+exp(-6*(twatersetp-5)))*kwfl-hair)*tfloor+(hair-L1*mCfloor)*tairmeas)/mCfloor;...
(hisol*toutside+L2*mCair*tairmeas+hair*tfloor+(-L2*mCair-hisol-hair)*tair)/mCair];

% casadi stuff
ode = struct;
ode.x=states;
ode.u=controls;
ode.ode=rhs;
ode.p=params;

intg = integrator('intg','cvodes',ode,0,Ts); %integrator function, integrate over 1 sample


%%%%%%%%%%%%%%%%%%%% model without observer

% control inputs, measurement added for observer
twatersetp = MX.sym('twatersetp');
toutside = MX.sym('toutside');
controls = [twatersetp;toutside]; n_controls = length(controls);

% parameters
mCfloor = MX.sym('mCfloor');
mCair = MX.sym('mCair');
hair= MX.sym('hair');
hisol= MX.sym('hisol');
kwfl= MX.sym('kwfl');
params=[mCfloor;mCair;hair;hisol;kwfl];

rhs=[(1/(1+exp(-6*(twatersetp-5)))*kwfl*twatersetp+L1*mCfloor*tairmeas+(-1/(1+exp(-6*(twatersetp-5)))*kwfl-hair)*tfloor+(hair-L1*mCfloor)*tairmeas)/mCfloor;...
(hisol*toutside+L2*mCair*tairmeas+hair*tfloor+(-L2*mCair-hisol-hair)*tair)/mCair];

rhs = [1/mCfloor*(1/(1+exp(-6*(twatersetp-1.5)))*kwfl*(twatersetp-tfloor)+hair*(tair-tfloor));...
       1/mCair*(hair*(tfloor-tair)+hisol*(toutside-tair))];

% casadi stuff
ode1 = struct;
ode1.x=states;
ode1.u=controls;
ode1.ode=rhs;
ode1.p=params;

intg1 = integrator('intg1','cvodes',ode1,0,Ts); %integrator function, integrate over 1 sample

end
