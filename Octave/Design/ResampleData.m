function resampled_data=ResampleData(entity_data,Ts)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% resampling crap to get same timestamp for all

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
    if length(times)>=2
      resampled_values = interp1(times, data, common_times, 'previous', 'extrap');
    else
      % if nothing changed at all
      resampled_values=data*ones(size(common_times));
    end

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



end

