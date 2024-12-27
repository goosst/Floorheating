function [entity_data,hass_time,most_recent_states]=DownloadHomeAssistant(address_hass,auth_token,entity_id)
% returns data from all home assistant
% over a time period from now till "hours_of_data" ago
% returns per entity_id a vector last_changed
%
% Example usage:
%address_hass='192.168.0.205:8123';
%entity_id='sensor.temperature_living,sensor.temperature_buiten_vaillant';
%DownloadHomeAssistant(address_hass,auth_token,entity_id);

hours_of_data=1;

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

  function parsed_time = parse_timestamp(timestamp)
    % Remove timezone information and microseconds for Octave compatibility
    if ~isempty(strfind(timestamp, '+'))
        timestamp = strsplit(timestamp, '+'){1};
    elseif ~isempty(strfind(timestamp, '-')) && length(timestamp) > 19  % Check if it's not just date separators
        parts = strsplit(timestamp, '-');
        if length(parts) > 3  % More splits than just the date parts
            timestamp = strcat(parts{1}, '-', parts{2}, '-', parts{3});  % Reconstruct just the date part
        end
    end
    % Remove any microseconds if present
    if ~isempty(strfind(timestamp, '.'))
        timestamp = strsplit(timestamp, '.'){1};
    end
    parsed_time = datenum(timestamp, 'yyyy-mm-ddTHH:MM:SS');
  end

  % get latest time from home assistant (to avoid timezone/utc crap)
  url=strcat('http://',address_hass,'/api/states/sensor.date_time_iso');
  options = weboptions('HeaderFields',{'Authorization' ['Bearer ' auth_token]});
  full_url = [url, "&minimal_response"];
  data = webread(url, options);
  if isempty(data)
    entity_data='error';
    hass_time='error';
    most_recent_states='unkown';
    return
  end
  data = jsondecode(data);
  hass_time = data.state;

  % get actual data from entity
  base_url = strcat("http://",address_hass,"/api/history/period/");
  end_time = hass_time;
  t = parse_timestamp(hass_time);
  start_time=datestr(t-hours_of_data/24, 'yyyy-mm-ddTHH:MM:SS');
  encoded_end_time= urlencode(end_time);
  encoded_start_time = urlencode(start_time);

  full_url = [base_url, encoded_start_time,"?end_time=", encoded_end_time, "&filter_entity_id=", entity_id,"&minimal_response"];

  options = weboptions('HeaderFields',{'Authorization' ['Bearer ' auth_token]});
  data = webread(full_url, options);

  if isempty(data)
    entity_data='error';
    most_recent_states='error';
    diary('debug_downloadhass.txt');
    display('empty data')
    display(full_url);
    display(data);
    diary off
    return
  end
  data = jsondecode(data);

  % Initialize a container for results
  entity_data = struct();

  % Function to add a single state entry
  function addStateEntry(entity_id_raw, state, time)
    entity_id = strrep(entity_id_raw, '.', '_');

    % Initialize if not already exists
    if ~isfield(entity_data, entity_id)
        entity_data.(entity_id) = {};
    end

    % Convert time to datenum for comparison
    current_time = parse_timestamp(time);

    % Get existing times if any
    if ~isempty(entity_data.(entity_id))
        existing_times = cellfun(@parse_timestamp, entity_data.(entity_id)(:,1));
        % Only add if this is a newer time
        if isempty(existing_times) || current_time > max(existing_times)
            entity_data.(entity_id){end+1, 1} = time;
            entity_data.(entity_id){end, 2} = state;
        end
    else
        % First entry for this entity
        entity_data.(entity_id){end+1, 1} = time;
        entity_data.(entity_id){end, 2} = state;
    end
  end

  % Process the data structure
  if iscell(data)
    % Process each entity's history array
    for i = 1:numel(data)
        if iscell(data{i}) && ~isempty(data{i})
            % Get the entity_id from the first entry
            if isstruct(data{i}{1}) && isfield(data{i}{1}, 'entity_id')
                current_entity_id = data{i}{1}.entity_id;

                % Process all entries for this entity
                for j = 1:numel(data{i})
                    if isstruct(data{i}{j})
                        state = data{i}{j}.state;
                        time = data{i}{j}.last_changed;
                        addStateEntry(current_entity_id, state, time);
                    end
                end
            end
        elseif isstruct(data{i}) && isfield(data{i}, 'entity_id')
            % Handle single entry arrays
            state = data{i}.state;
            time = data{i}.last_changed;
            addStateEntry(data{i}.entity_id, state, time);
        end
    end
  end

  % Process most recent states
  entity_ids = fieldnames(entity_data);
  most_recent_states = struct();

  for i = 1:numel(entity_ids)
      entity_id = entity_ids{i};
      data_array = entity_data.(entity_id);

      if ~isempty(data_array)
          % Find the most recent entry
          times = cellfun(@parse_timestamp, data_array(:,1));
          [~, latest_idx] = max(times);
          data_reformat = data_array{latest_idx, 2};

          % Convert the state to appropriate format
          if strcmp(data_reformat, 'on')
              most_recent_states.(entity_id) = 1;
          elseif strcmp(data_reformat, 'off')
              most_recent_states.(entity_id) = 0;
          else
              most_recent_states.(entity_id) = str2double(data_reformat);
          end
      end
  end
end
