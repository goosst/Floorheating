function [entity_data,hass_time,most_recent_states]=PostHomeAssistant(address_hass,auth_token,entity_id,value)
% I could not get it to work using webwrite in octav, so used curl

api_endpoint = sprintf('%s/api/states/%s', ['http://',address_hass], entity_id);

% Construct curl command
curl_cmd = sprintf(['curl "%s" ' ...
                   '-H "Authorization: Bearer %s" ' ...
                   '-H "Content-Type: application/json" ' ...
                   '-d ''{"state":"%s","attributes":{}}'' '], ...
                   api_endpoint, auth_token,num2str(value));

[status, response] = system(curl_cmd);

end

