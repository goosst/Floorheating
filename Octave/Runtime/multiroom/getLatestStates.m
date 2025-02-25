function [most_recent_states,values_array]=getLatestStates(address_hass,auth_token,entity_ids)
% get latest data from entity_ids
% most_recent_states including entity information,
% values_array just containing plain values

base_url = strcat("http://", address_hass, "/api/states");


options = weboptions('HeaderFields',{'Authorization' ['Bearer ' auth_token]});

% Fetch all states from Home Assistant
%response = webread(base_url, options);


most_recent_states = struct();
values_array=[];
for i=1:size(entity_ids,2)
  full_url=strcat(base_url,'/',entity_ids{i});

%  keyboard
  try
    response = webread(full_url, options);
  catch
    diary('debug_getlateststates.txt');
    display(err.message)
    display(full_url)
    diary off
    continue
  end

  data = jsondecode(response);

    % Convert the state to appropriate format
  if strcmp(data.state, 'on')
      most_recent_states(i).(data.entity_id) = 1;
  elseif strcmp(data.state, 'off')
      most_recent_states(i).(data.entity_id) = 0;
  else
      most_recent_states(i).(data.entity_id) = str2double(data.state);
  end

    values_array=[values_array; most_recent_states(i).(data.entity_id)];
end


end

