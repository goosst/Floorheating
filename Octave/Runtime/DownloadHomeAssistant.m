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
  t = datenum(hass_time, "yyyy-mm-ddTHH:MM:SS");
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

  % Function to process a single entity's data
  function processEntityData(item)
    if isfield(item, 'entity_id') && isfield(item, 'state') && isfield(item, 'last_changed')
        orig_entity_id = item.entity_id;
        % Replace dots with underscores for valid field name
        entity_id = strrep(orig_entity_id, '.', '_');
        state = item.state;
        time = item.last_changed;

        % Initialize if not already exists
        if ~isfield(entity_data, entity_id)
            entity_data.(entity_id) = {};
        end

        % Append new data
        entity_data.(entity_id){end+1, 1} = time;
        entity_data.(entity_id){end, 2} = state;
    end
  end

  % Handle different data structures
  if iscell(data)
    % Handle cell array structure (data2 format)
    for i = 1:numel(data)
        current = data{i};
        if iscell(current)
            for j = 1:numel(current)
                processEntityData(current{j});
            end
        else
            processEntityData(current);
        end
    end
  else
    % Handle non-cell array structure (data1 format)
    for i = 1:numel(data)
        current = data(i);
        if isstruct(current) && numel(current) == 1
            % Process single entity data
            item = current(1);
            if isstruct(item)
                processEntityData(item);
            end
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
          data_reformat = data_array{end, 2};

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
