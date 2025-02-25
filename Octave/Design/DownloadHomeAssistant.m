function [entity_data,hass_time,most_recent_states]=DownloadHomeAssistant(address_hass,auth_token,entity_id)
% returns data from all home assistant
% over a time period from now till "hours_of_data" ago
% returns per entity_id a vector last_changed
%
% Example usage:
%address_hass='192.168.0.205:8123';
%entity_id='sensor.temperature_living,sensor.temperature_buiten_vaillant';
%DownloadHomeAssistant(address_hass,auth_token,entity_id);

hours_of_data=15;

  function encoded = urlencode(str)
    % Initialize an empty string for the encoded output
    encoded = "";
    for i = 1:length(str)
        c = str(i);
        if isalnum(c) || c == '-' || c == '_' || c == '.' || c == '~'
            % Safe characters
            encoded = [encoded, c];
        else
            % Encode as %XX where XX is the hex value of the ASCII code
            encoded = [encoded, sprintf('%%%02X', double(c))];
        end
    end
end

# get latest time from home assistant (to avoid timezone/utc crap)
url=strcat('http://',address_hass,'/api/states/sensor.date_time_iso');
options = weboptions('HeaderFields',{'Authorization' ['Bearer ' auth_token]});
full_url = [url, "&minimal_response"];

data = webread(url, options);
data = jsondecode (data);
hass_time = data.state;

% get actual data from entity
base_url = strcat("http://",address_hass,"/api/history/period/");

end_time = hass_time;%strftime("%Y-%m-%dT%H:%M:%S", localtime(hass_time));          % Current time
t = datenum(hass_time, "yyyy-mm-ddTHH:MM:SS");
start_time=datestr(t-hours_of_data/24, 'yyyy-mm-ddTHH:MM:SS');

%keyboard

%start_time='2025-02-15T05:29:00'
%end_time='2025-02-17T05:29:00'

%start_time=datestr(t-60/24, 'yyyy-mm-ddTHH:MM:SS');
%end_time=datestr(t-50/24, 'yyyy-mm-ddTHH:MM:SS');

encoded_end_time= urlencode(end_time);
encoded_start_time = urlencode(start_time);

full_url = [base_url, encoded_start_time,"?end_time=", encoded_end_time, "&filter_entity_id=", entity_id,"&minimal_response"];

options = weboptions('HeaderFields',{'Authorization' ['Bearer ' auth_token]});
data = webread(full_url, options);
data = jsondecode (data);

% Initialize a container for results
entity_data = struct(); % To store data for each entity_id


%keyboad

% Loop through each set of data
for i = 1:numel(data)
    current = data{i}; % Current data block
    last_entity_id = ''; % Initialize last entity ID tracker

    if iscell(current)
        % Loop through nested structures in a cell array
        for j = 1:numel(current)
            item = current{j};
            % Extract or reuse entity_id
            if isfield(item, 'entity_id')
                entity_id = item.entity_id;
                last_entity_id = entity_id; % Update tracker
            else
                entity_id = last_entity_id; % Use previous entity_id
            end

            % Extract state and last_changed
            if isfield(item, 'state') && isfield(item, 'last_changed')
                state = item.state;
                time = item.last_changed;

                % Append data to the corresponding entity_id
                if ~isfield(entity_data, entity_id)
                    entity_data.(entity_id) = []; % Initialize if not already exists
                end

                entity_data.(entity_id) = [entity_data.(entity_id); {time, state}];
            end
        end
    else
        % Handle non-nested structures
        item = current;
        if isfield(item, 'entity_id') && isfield(item, 'state') && isfield(item, 'last_changed')
            entity_id = item.entity_id;
            state = item.state;
            time = item.last_changed;

            % Append data
            if ~isfield(entity_data, entity_id)
                entity_data.(entity_id) = [];
            end
            entity_data.(entity_id) = [entity_data.(entity_id); {time, state}];
        end
    end
end

% only most recent state
entity_ids = fieldnames(entity_data);
for i = 1:numel(entity_ids)
    entity_id = entity_ids{i};
    data_array = entity_data.(entity_id); % Extract time-state pairs
    % Get the last entry (most recent)
    data_reformat=cell2mat(data_array(end, 2));
    if strcmp(data_reformat,'on')
      most_recent_states.(entity_id) = 1;
    elseif strcmp(data_reformat,'off')
      most_recent_states.(entity_id) = 0;
    else
      most_recent_states.(entity_id) = str2num(data_reformat);
    end

end


end

