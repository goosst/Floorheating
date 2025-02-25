function [room_configs, all_model_params_observer, all_model_params_no_observer] = Roomconfig(num_rooms)
    % map sensors, valves, setpoints in home assistant to different rooms + also define model parameters
    % the names in the left column cannot be changed
    % must have definitions for every room are: tairmeas, tairsetpoint, valve
    % optional arguments are 'tfloormeas' (temperature sensor put on the floor itself) and multiple identical valves e.g. 'valve2' (will just follow the same setpoint as 'valve')
    room_configs = {
    % Room 1 (Living)
                    struct('tairmeas', 'sensor.temperature_living', ...
                        'tairsetpoint', 'input_number.tgttemplivingroom', ...
                        'valve', 'switch.collector', ...
                        'valve2', 'switch.collector_4', ... %optional valve just following the identical setpoint as valve
                        'tfloormeas', 'sensor.floor_temperature') %circuit has second valve, expected to have same position
    % Room 2 (Bedroom)
                    struct('tairmeas', 'sensor.temperature_slaapkamer', ...
                        'tairsetpoint', 'input_number.tgttempbedroom', ...
                        'valve', 'switch.collector_3'),
    % Room 3 (Frontroom)
                    struct('tairmeas', 'sensor.room_temperature', ...
                        'tairsetpoint', 'input_number.tgttempfrontroom', ...
                        'valve', 'switch.collector_2')

    % common entities
                    struct('twatersetp', 'sensor.temperature_flow_radiator', ... % reported by heater
                        'toutside', 'sensor.temperature_buiten_vaillant', ...
                        'airtemp_tgt', 'input_number.NRsetpThermostat') %applied by controller to thermostat/heater
                    };
    designobserver = true;
    % add plant model parameters of different rooms
    % mCfloor = 3.1542e+06;
    % mCair = 6.1871e+06;
    % hair= 3.1215e+02;
    % hisol= 5.1943e+01;
    % kwfl= 4.0000e+02;
    % L1 = 5.4348e-03;
    % L2 = 2.0929e-03;


%    mCfloor = 15534487.4922
%    mCair = 5482.5855
%    hair = 933.0409
%    hisol = 84.2796
%    kwfl = 400

%
mCfloor=9036271.9999;
mCair=3097750.9515;
hair=597.6987;
hisol=59.5332;
kwfl=400;

    if designobserver
        pkg load control

        %observer correction parameters
        A = [(-kwfl - hair) / mCfloor, hair / mCfloor;
             hair / mCair, (-hisol - hair) / mCair];

        B = [kwfl / mCfloor, 0;
             0, hisol / mCair];

        C = [[1, 0]; [0, 1]]; %full state observer

        SYS = ss (A, B, C);
        PolesDyn = pole(SYS);

        OB = obsv (A, C);

        if rank(OB) < size(A, 1)
            error('System is not observable!')
        end

        rank(OB);

        pole1 = PolesDyn(1); % first state
        pole2 = PolesDyn(2); % second state

        pole1 = -1/800; % first state
        pole2 = -1/800; % second state

        P = [pole1, pole2];

        %    timepole=[0:7200];
        %    figure
        %    plot(timepole,exp(pole2*timepole))

        L = place (SYS.', P').';

        %    desired_poles=eig(A)*2
        %    L = place(A', C', desired_poles)'

        reshaped_L = reshape(L', [], 1);

%        reshaped_L=[10^(-3) 0 0 -10^(-3)]';

        twatersetp = 30;
        toutside = 5;
        xinf = [((hisol + hair) * kwfl * twatersetp + hair * hisol * toutside) / ((hisol + hair) * kwfl + hair * hisol);
              (hair * kwfl * twatersetp + (hisol * kwfl + hair * hisol) * toutside) / ((hisol + hair) * kwfl + hair * hisol)];

%        keyboard
    end

    %    keyboard
%    reshaped_L=zeros(4,1);

    model_params_no_obs = [mCfloor; mCair; hair; hisol; kwfl];
    model_params = [[mCfloor; mCair; hair; hisol; kwfl]; reshaped_L];

    room_configs{1}.modelparams_obs = model_params;
    room_configs{1}.modelparams_no_obs = model_params_no_obs;

    mCfloor = 3449651.4475;
    mCair = 14260323.4642;
    hair = 1584.1038;
    hisol = 72.0658;
    kwfl = 400;
    %observer correction parameters

    L1 = 5.9274e-03;
    L2 = 2.3102e-03;

    model_params_no_obs = [mCfloor; mCair; hair; hisol; kwfl];
    model_params = [[mCfloor; mCair; hair; hisol; kwfl]; [L1; L2]];

    room_configs{2}.modelparams_obs = model_params;
    room_configs{3}.modelparams_obs = model_params;

    room_configs{2}.modelparams_no_obs = model_params_no_obs;
    room_configs{3}.modelparams_no_obs = model_params_no_obs;

    %% generate for amount of rooms used
    %all_sensors = {};
    %
    %% common sensors
    %all_sensors{end+1} =room_configs{end}.twatersetp;
    %all_sensors{end+1} =room_configs{end}.toutside;
    %
    %% Add room-specific
    %for i = 1:num_rooms
    %    all_sensors{end+1} = room_configs{i}.tairmeas;
    %    all_sensors{end+1} = room_configs{i}.tairsetpoint;
    %    all_sensors{end+1} = room_configs{i}.valve;
    %end

    all_model_params_observer = [];

    for i = 1:num_rooms
        all_model_params_observer = [all_model_params_observer; room_configs{i}.modelparams_obs];
    end

    all_model_params_no_observer = [];

    for i = 1:num_rooms
        all_model_params_no_observer = [all_model_params_no_observer; room_configs{i}.modelparams_no_obs];
    end

    %room_config={};
    %for i=1:num_rooms
    %  room_config{end+1}=room_configs{i};
    %
    %end
    %room_config{end+1}=room_configs{end};

end
