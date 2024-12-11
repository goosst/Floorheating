function [ode,intg,ode1,intg1]=plantModelDefinition(Ts)

function out = merge_structs(s1, s2)
    out = s1;
    fields = fieldnames(s2);
    for i = 1:length(fields)
        out.(fields{i}) = s2.(fields{i});
    end
end


  function y = sigmoid(x)
    y=1/(1+exp(-6*(x-5)));
  end

% Add states
tfloor = MX.sym('tfloor');
tair = MX.sym('tair');
states = [tfloor;tair]; n_states = length(states);

% control inputs, measurement added for observer
twatersetp = MX.sym('twatersetp');
toutside = MX.sym('toutside');
tairmeas = MX.sym('tairmeas');
valve = MX.sym('valve');

controls = [twatersetp;toutside;tairmeas;valve]; n_controls = length(controls);

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

%rhs=[(1/(1+exp(-6*(twatersetp-5)))*kwfl*valve*twatersetp+L1*mCfloor*tairmeas+(-1/(1+exp(-6*(twatersetp-5)))*kwfl*valve-hair)*tfloor+(hair-L1*mCfloor)*tairmeas)/mCfloor;...
%(hisol*toutside+L2*mCair*tairmeas+hair*tfloor+(-L2*mCair-hisol-hair)*tair)/mCair];

rhs=[(sigmoid(twatersetp)*kwfl*valve*twatersetp+L1*mCfloor*tairmeas-(sigmoid(twatersetp)*kwfl*valve-hair)*tfloor+(hair-L1*mCfloor)*tairmeas)/mCfloor;...
(hisol*toutside+L2*mCair*tairmeas+hair*tfloor+(-L2*mCair-hisol-hair)*tair)/mCair];

% casadi stuff
ode = struct;
ode.x=states;
ode.u=controls;
ode.ode=rhs;
ode.p=params;

%intg = integrator('intg','rk',ode,0,Ts); %integrator function, integrate over 1 sample
%keyboard

opts = struct('simplify', true); % codegeneration stuff
integrator_opts = struct('tf', Ts);  % tf option
merged_opts = merge_structs(integrator_opts, opts);
intg = integrator('intg', 'rk', ode, merged_opts);

%%%%%%%%%%%%%%%%%%%% model without observer

% control inputs, measurement added for observer
twatersetp = MX.sym('twatersetp');
toutside = MX.sym('toutside');
valve = MX.sym('valve');
controls = [twatersetp;toutside;valve]; n_controls = length(controls);

% parameters
mCfloor = MX.sym('mCfloor');
mCair = MX.sym('mCair');
hair= MX.sym('hair');
hisol= MX.sym('hisol');
kwfl= MX.sym('kwfl');
params=[mCfloor;mCair;hair;hisol;kwfl];

%rhs=[(1/(1+exp(-6*(twatersetp-5)))*kwfl*twatersetp+L1*mCfloor*tairmeas+(-1/(1+exp(-6*(twatersetp-5)))*kwfl-hair)*tfloor+(hair-L1*mCfloor)*tairmeas)/mCfloor;...
%(hisol*toutside+L2*mCair*tairmeas+hair*tfloor+(-L2*mCair-hisol-hair)*tair)/mCair];

%rhs = [1/mCfloor*(sigmoid(twatersetp)*kwfl*valve*(twatersetp-tfloor)+hair*(tair-tfloor));...
%       1/mCair*(hair*(tfloor-tair)+hisol*(toutside-tair))];

rhs = [1/mCfloor*(kwfl*valve*(twatersetp-tfloor)+hair*(tair-tfloor));...
1/mCair*(hair*(tfloor-tair)+hisol*(toutside-tair))];

% casadi stuff
ode1 = struct;
ode1.x=states;
ode1.u=controls;
ode1.ode=rhs;
ode1.p=params;

%intg1 = integrator('intg1','cvodes',ode1,0,Ts); %integrator function, integrate over 1 sample

opts = struct('simplify', true); % codegeneration stuff
integrator_opts = struct('tf', Ts);  % tf option
merged_opts = merge_structs(integrator_opts, opts);
intg1 = integrator('intg1', 'rk', ode1, merged_opts);


    % Define RK4 integrator function
%    M=4;
%    X0 = MX.sym('X0', 2);
%    U = MX.sym('U', 3);
%    P = MX.sym('P', 5); % Parameters
%    X = X0;
%    DT = Ts / M;
%    for j = 1:M
%        k1 = substitute(ode1.ode, [ode1.x; ode1.u; ode1.p], [X; U; P]);
%        k2 = substitute(ode1.ode, [ode1.x; ode1.u; ode1.p], [X + DT/2 * k1; U; P]);
%        k3 = substitute(ode1.ode, [ode1.x; ode1.u; ode1.p], [X + DT/2 * k2; U; P]);
%        k4 = substitute(ode1.ode, [ode1.x; ode1.u; ode1.p], [X + DT * k3; U; P]);
%        X = X + DT/6 * (k1 + 2*k2 + 2*k3 + k4);
%    end
%    intg = Function('intg', {'x0', 'u', 'p'}, {X}, {'x0', 'u', 'p'}, {'xf'});
end
